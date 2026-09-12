import { measure, pathPoint, offsetPath } from './street-geometry.mjs';
import { createStreetLayout } from './street-layout.mjs';
import { findRoute, planRoadNetwork } from './traffic-planner.mjs';
import { makeTurn, isRoundabout, roundaboutHasGap, priorityHasGap, releaseJunction } from './junctions.mjs';
import { singleTrackPath, updateSingleTracks, releaseSingleTrack } from './single-track.mjs';
export { findRoute } from './traffic-planner.mjs';
export { makeTurn, isRoundabout } from './junctions.mjs';
export { pathPoint, offsetPath } from './street-geometry.mjs';

// The town and its traffic share a directed road graph. Distances are in world
// pixels; the renderer can scale it independently of the simulation clock.
export const ROAD = 44;
export const JUNCTION = 36;
export const LANE = 11;
export const GAP = 9;
export const ROUTES = [
    { number: '12', colour: '#cf5249', name: 'Park circular' },
    { number: '24', colour: '#258b8a', name: 'Town loop' },
    { number: '36', colour: '#cc933c', name: 'Neighbourhood line' },
];

export function randomSource(seed) {
    return () => {
        seed |= 0;
        seed = seed + 0x6D2B79F5 | 0;
        let n = Math.imul(seed ^ seed >>> 15, 1 | seed);
        n = n + Math.imul(n ^ n >>> 7, 61 | n) ^ n;
        return ((n ^ n >>> 14) >>> 0) / 4294967296;
    };
}

export function signalState(node, axis, time) {
    if (!node.signal) return 'green';
    const phase = (time + node.offset) % node.cycle;
    const half = node.cycle / 2;
    const local = axis === 'x' ? phase : (phase + half) % node.cycle;
    if (local < half - 2.6) return 'green';
    if (local < half - 1.1) return 'amber';
    return 'red'; // Includes an all-red clearance interval between directions.
}

export function lanePoint(lane, distance) {
    return pathPoint(lane.path, distance);
}

export function vehiclePoint(vehicle) {
    if (vehicle.phase === 'lane') return lanePoint(vehicle.lane, vehicle.distance);
    return pathPoint(vehicle.turn, vehicle.distance);
}

export function createTown(width, height, seed) {
    const random = randomSource(seed);
    const choose = items => items[Math.floor(random() * items.length)];
    const layout = createStreetLayout(width, height, random);
    const nodes = layout.nodes;
    const edges = layout.roads;
    const lanes = [];
    for (const node of nodes) Object.assign(node, { outgoing: [], owner: null,
        radius: JUNCTION, signal: false, offset: random() * 22, cycle: 19 + random() * 6 });
    for (const edge of edges) {
        edge.lanes = [];
        for (const [from, to, reverse] of [[edge.a, edge.b, false], [edge.b, edge.a, true]]) {
            const heading = reverse ? edge.path.points.at(-1).angle + Math.PI : edge.path.points[0].angle;
            const arrival = reverse ? edge.path.points[0].angle + Math.PI : edge.path.points.at(-1).angle;
            const lane = { id: lanes.length, edge, from, to, reverse,
                dx: Math.cos(heading), dy: Math.sin(heading), heading,
                axis: Math.abs(Math.cos(arrival)) > Math.abs(Math.sin(arrival)) ? 'x' : 'y', stop: null };
            lanes.push(lane);
            edge.lanes.push(lane);
            from.outgoing.push(lane);
        }
    }
    const network = planRoadNetwork(nodes, edges, width, height, random);
    for (const lane of lanes) {
        const center = lane.reverse ? measure([...lane.edge.path.points].reverse().map(p =>
            ({ x: p.x, y: p.y, angle: p.angle + Math.PI }))) : lane.edge.path;
        lane.path = lane.edge.singleTrack ? singleTrackPath(lane, center) :
            offsetPath(center, lane.edge.offset, lane.from.radius, center.length - lane.to.radius);
        lane.length = lane.path.length;
        lane.start = pathPoint(lane.path, 0);
        lane.end = pathPoint(lane.path, lane.length);
    }
    const blocks = layout.faces.map(face => {
        const polygon = face.path.points.slice(0, -1);
        // Area-weighted centroids keep the plots centred in irregular crescents.
        let cx = 0;
        let cy = 0;
        polygon.forEach((a, i) => {
            const b = polygon[(i + 1) % polygon.length];
            const cross = a.x * b.y - b.x * a.y;
            cx += (a.x + b.x) * cross;
            cy += (a.y + b.y) * cross;
        });
        const center = { x: cx / (6 * face.area), y: cy / (6 * face.area) };
        const local = polygon.map(p => ({ x: p.x - center.x, y: p.y - center.y, angle: p.angle }));
        const x = Math.min(...local.map(p => p.x));
        const y = Math.min(...local.map(p => p.y));
        return { x, y, width: Math.max(...local.map(p => p.x)) - x,
            height: Math.max(...local.map(p => p.y)) - y, center, angle: 0, polygon: local,
            boundary: measure([...local.map(p => ({ ...p })), { ...local[0] }]),
            kind: choose(['homes', 'homes', 'shops', 'park']), seed: random() * 4294967296 };
    });
    // Each line visits a different set of neighbourhoods on a closed circuit.
    const routes = ROUTES.map((style, i) => {
        const selected = new Set();
        const radius = i === 1 ? 0.24 : 0.4;
        const count = Math.min(5, nodes.length);
        const waypoints = Array.from({ length: count }, (_, j) => {
            const angle = j * Math.PI * 2 / count + i * 0.8;
            const target = { x: width / 2 + Math.cos(angle) * width * radius,
                y: height / 2 + Math.sin(angle) * height * radius };
            const node = nodes.filter(n => !selected.has(n)).sort((a, b) =>
                Math.hypot(a.x - target.x, a.y - target.y) - Math.hypot(b.x - target.x, b.y - target.y))[0];
            selected.add(node);
            return node;
        });
        if (i % 2) waypoints.reverse();
        const path = waypoints.flatMap((node, index) => findRoute(node, waypoints[(index + 1) % waypoints.length], { bus: true, live: false }).path);
        path.forEach((lane, index) => {
            if (index % 2 === 0 && lane.length > 105 && !lane.edge.singleTrack) {
                if (!lane.stop) lane.stop = { distance: lane.length * 0.48, routes: [] };
                if (!lane.stop.routes.includes(style.number)) lane.stop.routes.push(style.number);
            }
        });
        return { ...style, path };
    });
    const town = { width, height, seed, random, nodes, edges, lanes, blocks, routes, vehicles: [],
        time: 0, nextVehicleId: 0, trafficLevel: 1, network, metricsAt: 0,
        baseTraffic: Math.min(250, Math.max(12, Math.round(lanes.length * 0.3))) };
    town.spawnLanes = lanes.flatMap(lane => Array(lane.edge.roadType === 'arterial' ? 4 : lane.edge.roadType === 'collector' ? 2 : 1).fill(lane));
    routes.forEach(route => { route.baseBuses = Math.max(2, Math.round(route.path.length / 7)); });
    setTrafficLevel(town, 1);
    updateTrafficMetrics(town);
    return town;
}

const CAR_COLOURS = ['#efe9d9', '#537d9b', '#d68563', '#e4b94f', '#718978', '#a7b9bc', '#49556a'];

function addVehicle(town, route = null) {
    const { random, vehicles } = town;
    const lanes = town.spawnLanes;
    const bus = !!route;
    const routeIndex = route ? Math.floor(random() * route.path.length) : 0;
    const lane = route ? route.path[routeIndex] : lanes[Math.floor(random() * lanes.length)];
    const length = bus ? 34 : 21 + random() * 4;
    const distance = length / 2 + 12 + random() * Math.max(0, lane.length - length - 36);
    // Never insert a vehicle into an occupied junction or the gap reserved by a
    // turning vehicle. Generous insertion spacing allows existing traffic to brake.
    if (lane.from.occupants.size || lane.to.occupants.size || distance > lane.length - length / 2 - 4 ||
        lane.edge.singleTrack && distance > lane.singleEntry - length / 2 - 10 ||
        vehicles.some(v => v.phase === 'lane' && v.lane === lane &&
            Math.abs(v.distance - distance) < (v.length + length) / 2 + GAP + 5)) return false;
    const vehicle = { id: town.nextVehicleId++, bus, route, routeIndex, lane, length,
        width: bus ? 14 : 11, distance, phase: 'lane', speed: 0,
        maxSpeed: bus ? 35 + random() * 4 : 45 + random() * 12,
        colour: route ? route.colour : CAR_COLOURS[Math.floor(random() * CAR_COLOURS.length)],
        next: null, turn: null, reserved: null, dwell: 0, narrowPermit: null, destination: null, itinerary: [],
        served: !!lane.stop && distance > lane.stop.distance - 2,
        stopsVisited: 0, distanceTravelled: 0, braking: false };
    vehicles.push(vehicle);
    planNext(town, vehicle);
    return true;
}

export function setTrafficLevel(town, level) {
    if (!Number.isFinite(level)) return;
    town.trafficLevel = Math.max(0, Math.min(6, level));
    function setGroup(route, target) {
        const group = town.vehicles.filter(v => v.route === route);
        const removed = new Set(group.slice(target));
        for (const vehicle of removed) {
            releaseJunction(vehicle);
            releaseSingleTrack(vehicle);
        }
        town.vehicles = town.vehicles.filter(v => !removed.has(v));
        let needed = Math.max(0, target - group.length);
        for (let attempt = 0; needed && attempt < Math.min(8000, target * 20); attempt++) {
            if (addVehicle(town, route)) needed--;
        }
    }
    let buses = 0;
    for (const route of town.routes) {
        const target = Math.round(route.baseBuses * town.trafficLevel);
        buses += target;
        setGroup(route, target);
    }
    setGroup(null, Math.max(0, Math.round(town.baseTraffic * town.trafficLevel) - buses));
}

function planNext(town, vehicle) {
    if (vehicle.bus) {
        vehicle.next = vehicle.route.path[(vehicle.routeIndex + 1) % vehicle.route.path.length];
    } else {
        if (!vehicle.destination || vehicle.destination === vehicle.lane.to || !vehicle.itinerary.length ||
            vehicle.itinerary[0].from !== vehicle.lane.to) {
            if (!vehicle.destination || vehicle.destination === vehicle.lane.to) {
                const choices = town.random() < 0.65 ? town.network.hubs : town.nodes;
                vehicle.destination = choices[Math.floor(town.random() * choices.length)];
                if (vehicle.destination === vehicle.lane.to) vehicle.destination = town.nodes[(vehicle.destination.id + 3) % town.nodes.length];
            }
            vehicle.itinerary = findRoute(vehicle.lane.to, vehicle.destination).path;
        }
        vehicle.next = vehicle.itinerary.shift() || vehicle.lane.to.outgoing[0];
    }
}

function crossingSpeed(vehicle) {
    if (isRoundabout(vehicle.lane.to)) return 20;
    if (Math.cos(vehicle.next.start.angle - vehicle.lane.end.angle) > 0.85) {
        return Math.min(vehicle.maxSpeed, vehicle.lane.edge.speed, vehicle.next.edge.speed);
    }
    return 20;
}

export function updateTrafficMetrics(town) {
    let queued = 0;
    let speed = 0;
    for (const lane of town.lanes) { lane.queueLength = 0; lane.load = 0; }
    for (const vehicle of town.vehicles) {
        const waiting = vehicle.speed < 3 && vehicle.dwell <= 0;
        if (waiting) queued++;
        speed += vehicle.speed / Math.min(vehicle.maxSpeed, vehicle.lane.edge.speed);
        if (vehicle.phase === 'lane') {
            vehicle.lane.load += vehicle.length + GAP;
            if (waiting) vehicle.lane.queueLength++;
        }
    }
    for (const lane of town.lanes) lane.density = lane.load / Math.max(1, lane.length);
    const ratio = town.vehicles.length ? queued / town.vehicles.length : 0;
    town.metrics = { queued, ratio, flow: town.vehicles.length ? speed / town.vehicles.length : 1,
        status: !town.vehicles.length ? 'Empty streets' : ratio > 0.65 ? 'Traffic jam' : ratio > 0.4 ? 'Congested' : ratio > 0.2 ? 'Busy' : 'Flowing' };
}

export function updateTown(town, dt) {
    town.time += dt;
    if (town.time >= town.metricsAt) { updateTrafficMetrics(town); town.metricsAt = town.time + 0.75; }
    const occupied = new Map(town.lanes.map(lane => [lane, []]));
    for (const vehicle of town.vehicles) {
        if (vehicle.phase === 'lane') occupied.get(vehicle.lane).push(vehicle);
    }
    for (const queue of occupied.values()) queue.sort((a, b) => b.distance - a.distance);
    updateSingleTracks(town, occupied);
    const hasRoom = (vehicle, next) => {
        // Reserve real exit space, including other roundabout users who have
        // already chosen this exit but have not reached its lane yet.
        let space = Math.min(next.length, next.edge.singleTrack ? next.singleEntry : Infinity) - 4;
        for (const other of occupied.get(next)) space = Math.min(space, other.distance - other.length / 2);
        for (const other of next.from.occupants) {
            if (other !== vehicle && other.next === next && !(other.phase === 'lane' && other.lane === next)) space -= other.length + GAP;
        }
        return space > vehicle.length + GAP + 6;
    };
    const candidates = new Map();
    for (const [lane, queue] of occupied) {
        const vehicle = queue[0];
        if (!vehicle || vehicle.reserved) continue;
        const node = lane.to;
        const lookahead = !node.signal && !isRoundabout(node) && lane.hasPriority ? Math.max(6, Math.min(50, vehicle.speed * 1.3)) : 6;
        if (vehicle.distance < lane.length - vehicle.length / 2 - lookahead ||
            vehicle.bus && lane.stop && !vehicle.served) continue;
        vehicle.waitingSince ??= town.time;
        if (!vehicle.bus && town.time - vehicle.waitingSince > 12 && !hasRoom(vehicle, vehicle.next)) {
            const alternatives = lane.to.outgoing.filter(next => next !== vehicle.next && hasRoom(vehicle, next));
            alternatives.sort((a, b) => (a.to === lane.from) - (b.to === lane.from) ||
                (a.queueLength || 0) - (b.queueLength || 0));
            if (alternatives.length) { vehicle.next = alternatives[0]; vehicle.itinerary = []; }
        }
        if (node.owner || signalState(node, lane.axis, town.time) !== 'green' || !hasRoom(vehicle, vehicle.next)) continue;
        if (node.control === 'giveway' && !priorityHasGap(vehicle, occupied, hasRoom)) continue;
        if (!candidates.has(node)) candidates.set(node, []);
        candidates.get(node).push(vehicle);
    }
    for (const [node, arrivals] of candidates) {
        arrivals.sort((a, b) => a.waitingSince - b.waitingSince || a.id - b.id);
        for (const vehicle of arrivals) {
            if (node.owner || !hasRoom(vehicle, vehicle.next)) continue;
            if (isRoundabout(node) && !roundaboutHasGap(node, vehicle)) continue;
            if (!isRoundabout(node)) node.owner = vehicle;
            node.occupants.add(vehicle);
            vehicle.reserved = node;
            vehicle.turn = makeTurn(vehicle.lane, vehicle.next);
            vehicle.waitingSince = null;
        }
    }
    // Update leaders first. Followers can use their new positions without ever
    // jumping through a queue when the frame rate drops.
    const ordered = [...occupied.values()].flat();
    const turning = town.vehicles.filter(v => v.phase === 'turn');
    for (const vehicle of ordered) {
        const { lane } = vehicle;
        if (vehicle.reserved && vehicle.reserved !== lane.to && vehicle.distance > vehicle.length / 2 + 4) {
            releaseJunction(vehicle);
        }
        let limit = lane.length;
        let targetSpeed = Math.min(vehicle.maxSpeed, lane.edge.speed);
        const queue = occupied.get(lane);
        const index = queue.indexOf(vehicle);
        if (index > 0) {
            const leader = queue[index - 1];
            limit = Math.min(limit, leader.distance - (leader.length + vehicle.length) / 2 - GAP);
        }
        for (const crossing of lane.to.occupants) {
            if (crossing !== vehicle && crossing.phase === 'turn' && crossing.lane === lane) {
                limit = Math.min(limit, lane.length + crossing.distance - (crossing.length + vehicle.length) / 2 - GAP);
            }
        }
        if (lane.edge.singleTrack && !vehicle.narrowPermit && vehicle.distance < lane.singleExit) {
            limit = Math.min(limit, lane.singleEntry - vehicle.length / 2 - 4);
        }
        const stopLine = lane.length - vehicle.length / 2 - 4;
        if (vehicle.reserved !== lane.to) {
            limit = Math.min(limit, stopLine);
        }
        if (vehicle.bus && lane.stop && !vehicle.served) {
            limit = Math.min(limit, lane.stop.distance);
            if (vehicle.distance >= lane.stop.distance - 0.3) {
                if (!vehicle.dwell) vehicle.dwell = 2.2 + town.random() * 1.5;
                vehicle.dwell -= dt;
                targetSpeed = 0;
                if (vehicle.dwell <= 0) {
                    vehicle.served = true;
                    vehicle.dwell = 0;
                    vehicle.stopsVisited++;
                }
            }
        }
        const available = Math.max(0, limit - vehicle.distance);
        // A lane endpoint is a path transition, not a reason to brake when the
        // driver has already secured a clear junction.
        if (limit < lane.length || vehicle.reserved !== lane.to) targetSpeed = Math.min(targetSpeed, Math.sqrt(2 * 35 * available));
        if (vehicle.reserved === lane.to) targetSpeed = Math.min(targetSpeed, crossingSpeed(vehicle));
        const oldSpeed = vehicle.speed;
        vehicle.speed += Math.max(-65 * dt, Math.min(22 * dt, targetSpeed - vehicle.speed));
        const movement = Math.min(available, Math.max(0, vehicle.speed * dt));
        vehicle.distance += movement;
        vehicle.distanceTravelled += movement;
        if (available < 0.1 && (limit < lane.length || vehicle.reserved !== lane.to)) vehicle.speed = 0;
        vehicle.braking = vehicle.speed < oldSpeed - 0.05 || vehicle.speed < 2;
        if (vehicle.distance >= lane.length - 0.001 && vehicle.reserved === lane.to) {
            vehicle.phase = 'turn';
            vehicle.turn ||= makeTurn(lane, vehicle.next);
            vehicle.distance = 0;
            // Its reserved junction keeps the following vehicle behind its tail
            // until it has crossed far enough to clear this approach.
            queue.splice(queue.indexOf(vehicle), 1);
        }
    }
    for (const vehicle of turning) {
        vehicle.speed = Math.min(crossingSpeed(vehicle), vehicle.speed + dt * 22);
        const movement = Math.min(vehicle.speed * dt, vehicle.turn.length - vehicle.distance);
        vehicle.distance += movement;
        vehicle.distanceTravelled += movement;
        vehicle.braking = false;
        if (vehicle.distance >= vehicle.turn.length - 0.001) {
            vehicle.lane = vehicle.next;
            vehicle.phase = 'lane';
            vehicle.distance = 0;
            vehicle.turn = null;
            vehicle.served = false;
            if (vehicle.bus) vehicle.routeIndex = (vehicle.routeIndex + 1) % vehicle.route.path.length;
            planNext(town, vehicle);
        }
    }
}
