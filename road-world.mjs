import { measure, pathPoint, offsetPath } from './street-geometry.mjs';
import { createStreetLayout } from './street-layout.mjs';
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

function shortestPath(from, to) {
    const pending = new Set([from]);
    const distance = new Map([[from, 0]]);
    const previous = new Map([[from, null]]);
    while (pending.size) {
        let node;
        for (const candidate of pending) if (!node || distance.get(candidate) < distance.get(node)) node = candidate;
        pending.delete(node);
        if (node === to) break;
        for (const lane of node.outgoing) {
            const cost = distance.get(node) + lane.edge.path.length;
            if (!distance.has(lane.to) || cost < distance.get(lane.to)) {
                distance.set(lane.to, cost);
                previous.set(lane.to, lane);
                pending.add(lane.to);
            }
        }
    }
    if (!previous.has(to)) return [];
    const path = [];
    for (let node = to; node !== from;) {
        const lane = previous.get(node);
        path.unshift(lane);
        node = lane.from;
    }
    return path;
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



function makeTurn(incoming, outgoing) {
    const p0 = incoming.end;
    const p3 = outgoing.start;
    const reach = Math.hypot(p3.x - p0.x, p3.y - p0.y) * 0.55;
    const p1 = { x: p0.x + Math.cos(p0.angle) * reach, y: p0.y + Math.sin(p0.angle) * reach };
    const p2 = { x: p3.x - Math.cos(p3.angle) * reach, y: p3.y - Math.sin(p3.angle) * reach };
    const points = [];
    let length = 0;
    for (let i = 0; i <= 32; i++) {
        const t = i / 32;
        const u = 1 - t;
        const p = {
            x: u ** 3 * p0.x + 3 * u * u * t * p1.x + 3 * u * t * t * p2.x + t ** 3 * p3.x,
            y: u ** 3 * p0.y + 3 * u * u * t * p1.y + 3 * u * t * t * p2.y + t ** 3 * p3.y,
        };
        if (i) length += Math.hypot(p.x - points[i - 1].x, p.y - points[i - 1].y);
        points.push({ ...p, distance: length });
    }
    return { points, length };
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
    for (const node of nodes) {
        node.signal = node.outgoing.length >= 3;
        const angles = node.outgoing.map(lane => lane.heading).sort((a, b) => a - b);
        const smallest = Math.min(...angles.map((angle, i) =>
            (angles[(i + 1) % angles.length] - angle + Math.PI * 4) % (Math.PI * 2)));
        node.radius = Math.max(JUNCTION, 31 / Math.tan(smallest / 2));
    }
    for (const lane of lanes) {
        const center = lane.reverse ? measure([...lane.edge.path.points].reverse().map(p =>
            ({ x: p.x, y: p.y, angle: p.angle + Math.PI }))) : lane.edge.path;
        lane.path = offsetPath(center, LANE, lane.from.radius, center.length - lane.to.radius);
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
        const path = waypoints.flatMap((node, index) => shortestPath(node, waypoints[(index + 1) % waypoints.length]));
        path.forEach((lane, index) => {
            if (index % 2 === 0 && lane.length > 105) {
                if (!lane.stop) lane.stop = { distance: lane.length * 0.48, routes: [] };
                if (!lane.stop.routes.includes(style.number)) lane.stop.routes.push(style.number);
            }
        });
        return { ...style, path };
    });
    const town = { width, height, seed, random, nodes, edges, lanes, blocks, routes, vehicles: [],
        time: 0, nextVehicleId: 0, trafficLevel: 1,
        baseTraffic: Math.min(420, Math.max(18, Math.round(lanes.length * 0.65))) };
    routes.forEach(route => { route.baseBuses = Math.max(2, Math.round(route.path.length / 7)); });
    setTrafficLevel(town, 1);
    return town;
}

const CAR_COLOURS = ['#efe9d9', '#537d9b', '#d68563', '#e4b94f', '#718978', '#a7b9bc', '#49556a'];

function addVehicle(town, route = null) {
    const { random, vehicles, lanes } = town;
    const bus = !!route;
    const routeIndex = route ? Math.floor(random() * route.path.length) : 0;
    const lane = route ? route.path[routeIndex] : lanes[Math.floor(random() * lanes.length)];
    const length = bus ? 34 : 21 + random() * 4;
    const distance = length / 2 + 12 + random() * Math.max(0, lane.length - length - 36);
    // Never insert a vehicle into an occupied junction or the gap reserved by a
    // turning vehicle. Generous insertion spacing allows existing traffic to brake.
    if (lane.from.owner || lane.to.owner || distance > lane.length - length / 2 - 4 ||
        vehicles.some(v => v.phase === 'lane' && v.lane === lane &&
            Math.abs(v.distance - distance) < (v.length + length) / 2 + GAP + 18)) return false;
    const vehicle = { id: town.nextVehicleId++, bus, route, routeIndex, lane, length,
        width: bus ? 14 : 11, distance, phase: 'lane', speed: 0,
        maxSpeed: bus ? 29 + random() * 4 : 32 + random() * 13,
        colour: route ? route.colour : CAR_COLOURS[Math.floor(random() * CAR_COLOURS.length)],
        next: null, turn: null, reserved: null, dwell: 0,
        served: !!lane.stop && distance > lane.stop.distance - 2,
        stopsVisited: 0, distanceTravelled: 0, braking: false };
    vehicles.push(vehicle);
    planNext(town, vehicle);
    return true;
}

export function setTrafficLevel(town, level) {
    if (!Number.isFinite(level)) return;
    town.trafficLevel = Math.max(0, Math.min(2, level));
    function setGroup(route, target) {
        const group = town.vehicles.filter(v => v.route === route);
        const removed = new Set(group.slice(target));
        for (const vehicle of removed) {
            if (vehicle.reserved?.owner === vehicle) vehicle.reserved.owner = null;
        }
        town.vehicles = town.vehicles.filter(v => !removed.has(v));
        let needed = Math.max(0, target - group.length);
        for (let attempt = 0; needed && attempt < target * 25; attempt++) {
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
        const choices = vehicle.lane.to.outgoing.filter(l => l.to !== vehicle.lane.from);
        const heading = vehicle.lane.end.angle;
        const straight = choices.find(l => Math.cos(l.start.angle - heading) > 0.86);
        vehicle.next = straight && town.random() < 0.55 ? straight :
            choices[Math.floor(town.random() * choices.length)] || vehicle.lane.to.outgoing[0];
    }
}

export function updateTown(town, dt) {
    town.time += dt;
    const occupied = new Map(town.lanes.map(lane => [lane, []]));
    for (const vehicle of town.vehicles) {
        if (vehicle.phase === 'lane') occupied.get(vehicle.lane).push(vehicle);
    }
    for (const queue of occupied.values()) queue.sort((a, b) => b.distance - a.distance);
    const candidates = new Map();
    for (const [lane, queue] of occupied) {
        const vehicle = queue[0];
        if (!vehicle || vehicle.reserved || vehicle.distance < lane.length - vehicle.length / 2 - 6) continue;
        vehicle.waitingSince ??= town.time;
        // The entire vehicle must be able to clear the junction, even if the
        // outgoing queue does not move. Checking only its front can lock two
        // neighbouring junctions with vehicles whose tails still occupy them.
        const hasRoom = next => occupied.get(next).every(other =>
            other.distance - other.length / 2 > vehicle.length + GAP + 6);
        // Drivers can abandon a persistently blocked turn. Without a detour,
        // queues around a short residential loop can lock one another forever.
        if (!vehicle.bus && town.time - vehicle.waitingSince > 7 && !hasRoom(vehicle.next)) {
            const alternatives = lane.to.outgoing.filter(next => next !== vehicle.next && hasRoom(next));
            alternatives.sort((a, b) => (a.to === lane.from) - (b.to === lane.from) ||
                occupied.get(a).length - occupied.get(b).length);
            if (alternatives.length) vehicle.next = alternatives[0];
        }
        if (lane.to.owner || signalState(lane.to, lane.axis, town.time) !== 'green') continue;
        const room = hasRoom(vehicle.next);
        if (!room) continue;
        const candidate = candidates.get(lane.to);
        if (!candidate || vehicle.waitingSince < candidate.waitingSince ||
            (vehicle.waitingSince === candidate.waitingSince && vehicle.id < candidate.id)) candidates.set(lane.to, vehicle);
    }
    // Oldest eligible arrival wins. Fixed lane ordering would starve one arm of
    // a busy junction, particularly when diagonal streets share a signal phase.
    for (const [node, vehicle] of candidates) {
        node.owner = vehicle;
        vehicle.reserved = node;
        vehicle.waitingSince = null;
    }
    // Update leaders first. Followers can use their new positions without ever
    // jumping through a queue when the frame rate drops.
    const ordered = [...occupied.values()].flat();
    const turning = town.vehicles.filter(v => v.phase === 'turn');
    for (const vehicle of ordered) {
        const { lane } = vehicle;
        if (vehicle.reserved && vehicle.reserved !== lane.to && vehicle.distance > vehicle.length / 2 + 4) {
            vehicle.reserved.owner = null;
            vehicle.reserved = null;
        }
        let limit = lane.length;
        let targetSpeed = vehicle.maxSpeed;
        const queue = occupied.get(lane);
        const index = queue.indexOf(vehicle);
        if (index > 0) {
            const leader = queue[index - 1];
            limit = Math.min(limit, leader.distance - (leader.length + vehicle.length) / 2 - GAP);
        }
        const crossing = lane.to.owner;
        if (crossing && crossing !== vehicle && crossing.phase === 'turn' && crossing.lane === lane) {
            limit = Math.min(limit, lane.length + crossing.distance - (crossing.length + vehicle.length) / 2 - GAP);
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
        targetSpeed = Math.min(targetSpeed, Math.sqrt(2 * 35 * available));
        if (vehicle.reserved === lane.to) targetSpeed = Math.min(targetSpeed, 23);
        const oldSpeed = vehicle.speed;
        vehicle.speed += Math.max(-65 * dt, Math.min(22 * dt, targetSpeed - vehicle.speed));
        const movement = Math.min(available, Math.max(0, vehicle.speed * dt));
        vehicle.distance += movement;
        vehicle.distanceTravelled += movement;
        if (available < 0.1) vehicle.speed = 0;
        vehicle.braking = vehicle.speed < oldSpeed - 0.05 || vehicle.speed < 2;
        if (vehicle.distance >= lane.length - 0.001 && vehicle.reserved === lane.to) {
            vehicle.phase = 'turn';
            vehicle.turn = makeTurn(lane, vehicle.next);
            vehicle.distance = 0;
            // Its reserved junction keeps the following vehicle behind its tail
            // until it has crossed far enough to clear this approach.
            queue.splice(queue.indexOf(vehicle), 1);
        }
    }
    for (const vehicle of turning) {
        vehicle.speed = Math.min(23, vehicle.speed + dt * 22);
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
