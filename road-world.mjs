// The town and its traffic share a directed road graph. Distances are in world
// pixels; the renderer can scale it independently of the simulation clock.
export const ROAD = 44;
export const JUNCTION = 31;
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
    const queue = [from];
    const previous = new Map([[from, null]]);
    for (const node of queue) {
        if (node === to) break;
        for (const lane of node.outgoing) {
            if (!previous.has(lane.to)) {
                previous.set(lane.to, lane);
                queue.push(lane.to);
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
    return {
        x: lane.start.x + lane.dx * distance,
        y: lane.start.y + lane.dy * distance,
        angle: Math.atan2(lane.dy, lane.dx),
    };
}

function makeTurn(incoming, outgoing) {
    const p0 = incoming.end;
    const p3 = outgoing.start;
    const reach = Math.hypot(p3.x - p0.x, p3.y - p0.y) * 0.55;
    const p1 = { x: p0.x + incoming.dx * reach, y: p0.y + incoming.dy * reach };
    const p2 = { x: p3.x - outgoing.dx * reach, y: p3.y - outgoing.dy * reach };
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
    const points = vehicle.turn.points;
    let i = 1;
    while (i < points.length - 1 && points[i].distance < vehicle.distance) i++;
    const a = points[i - 1];
    const b = points[i];
    const t = Math.max(0, Math.min(1, (vehicle.distance - a.distance) / (b.distance - a.distance || 1)));
    return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, angle: Math.atan2(b.y - a.y, b.x - a.x) };
}

export function createTown(width, height, seed) {
    const random = randomSource(seed);
    const choose = items => items[Math.floor(random() * items.length)];
    function coordinates(size) {
        const values = [-90 - random() * 65];
        while (values.at(-1) < size + 90) values.push(values.at(-1) + 180 + random() * 95);
        return values;
    }
    const xs = coordinates(width);
    const ys = coordinates(height);
    const nodes = [];
    const edges = [];
    const lanes = [];
    const blocks = [];
    const grid = ys.map((y, row) => xs.map((x, col) => {
        const node = { id: nodes.length, x, y, row, col, outgoing: [], owner: null,
            signal: false, offset: random() * 22, cycle: 19 + random() * 6 };
        nodes.push(node);
        return node;
    }));
    function connect(a, b) {
        const distance = Math.hypot(b.x - a.x, b.y - a.y);
        const edge = { id: edges.length, a, b, axis: a.row === b.row ? 'x' : 'y', lanes: [] };
        edges.push(edge);
        for (const [from, to] of [[a, b], [b, a]]) {
            const dx = (to.x - from.x) / distance;
            const dy = (to.y - from.y) / distance;
            const lane = { id: lanes.length, edge, from, to, dx, dy, axis: edge.axis,
                length: distance - JUNCTION * 2,
                start: { x: from.x + dx * JUNCTION + dy * LANE, y: from.y + dy * JUNCTION - dx * LANE },
                end: { x: to.x - dx * JUNCTION + dy * LANE, y: to.y - dy * JUNCTION - dx * LANE },
                stop: null };
            lanes.push(lane);
            edge.lanes.push(lane);
            from.outgoing.push(lane);
        }
    }
    grid.forEach((row, r) => row.forEach((node, c) => {
        if (c < xs.length - 1) connect(node, row[c + 1]);
        if (r < ys.length - 1) connect(node, grid[r + 1][c]);
    }));
    // Remove a few streets to create T-junctions and larger neighbourhoods.
    // Check connectivity before accepting a removal, so every route stays usable.
    for (const edge of [...edges]) {
        if (random() > 0.16 || edge.a.outgoing.length < 3 || edge.b.outgoing.length < 3) continue;
        for (const lane of edge.lanes) lane.from.outgoing = lane.from.outgoing.filter(l => l !== lane);
        if (!shortestPath(edge.a, edge.b).length) {
            for (const lane of edge.lanes) lane.from.outgoing.push(lane);
        } else {
            edges.splice(edges.indexOf(edge), 1);
            for (const lane of edge.lanes) lanes.splice(lanes.indexOf(lane), 1);
        }
    }
    for (const node of nodes) node.signal = node.outgoing.length >= 3;
    for (let r = 0; r < ys.length - 1; r++) {
        for (let c = 0; c < xs.length - 1; c++) {
            blocks.push({ x: xs[c] + 35, y: ys[r] + 35, width: xs[c + 1] - xs[c] - 70,
                height: ys[r + 1] - ys[r] - 70, kind: choose(['homes', 'homes', 'shops', 'park']), seed: random() * 4294967296 });
        }
    }
    // Each numbered bus line is a repeatable circuit through four neighbourhoods.
    const routes = ROUTES.map((style, i) => {
        const left = i === 1 && xs.length > 4 ? 1 : 0;
        const right = xs.length - 1 - (i === 2 && xs.length > 4 ? 1 : 0);
        const top = i === 2 && ys.length > 4 ? 1 : 0;
        const bottom = ys.length - 1;
        let waypoints = [grid[top][left], grid[top][right], grid[bottom][right], grid[bottom][left]];
        // A central waypoint brings the lines through the visible town as well.
        waypoints.splice(2, 0, grid[Math.floor(ys.length / 2)][Math.floor(xs.length / 2)]);
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
    const vehicles = [];
    const carColours = ['#efe9d9', '#537d9b', '#d68563', '#e4b94f', '#718978', '#a7b9bc', '#49556a'];
    function addVehicle(bus, route = null, routeIndex = 0) {
        const lane = route ? route.path[routeIndex] : choose(lanes);
        const length = bus ? 34 : 21 + random() * 4;
        const distance = 24 + random() * Math.max(0, lane.length - 65);
        if (vehicles.some(v => v.lane === lane && Math.abs(v.distance - distance) < (v.length + length) / 2 + GAP + 6)) return false;
        vehicles.push({ id: vehicles.length, bus, route, routeIndex, lane, length,
            width: bus ? 14 : 11, distance, phase: 'lane', speed: 0,
            maxSpeed: bus ? 29 + random() * 4 : 32 + random() * 13,
            colour: route ? route.colour : choose(carColours), next: null, turn: null,
            reserved: null, dwell: 0, served: !!lane.stop && distance > lane.stop.distance - 2,
            stopsVisited: 0, distanceTravelled: 0, braking: false });
        return true;
    }
    for (const route of routes) {
        const count = Math.max(2, Math.round(route.path.length / 7));
        for (let i = 0; i < count; i++) {
            for (let attempt = 0; attempt < 12; attempt++) {
                if (addVehicle(true, route, (Math.floor(i * route.path.length / count) + attempt) % route.path.length)) break;
            }
        }
    }
    const target = Math.min(100, Math.max(18, Math.round(lanes.length * 0.65)));
    for (let attempt = 0; vehicles.length < target && attempt < target * 15; attempt++) addVehicle(false);
    const town = { width, height, seed, random, nodes, edges, lanes, blocks, routes, vehicles, time: 0 };
    vehicles.forEach(v => planNext(town, v));
    return town;
}

function planNext(town, vehicle) {
    if (vehicle.bus) {
        vehicle.next = vehicle.route.path[(vehicle.routeIndex + 1) % vehicle.route.path.length];
    } else {
        const choices = vehicle.lane.to.outgoing.filter(l => l.to !== vehicle.lane.from);
        const straight = choices.find(l => l.dx === vehicle.lane.dx && l.dy === vehicle.lane.dy);
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
            const outgoing = occupied.get(vehicle.next);
            const room = outgoing.every(other => other.distance - other.length / 2 > vehicle.length / 2 + GAP + 8);
            const green = signalState(lane.to, lane.axis, town.time) === 'green';
            if (green && !lane.to.owner && !vehicle.reserved && room && vehicle.distance >= stopLine - 2 && index === 0) {
                lane.to.owner = vehicle;
                vehicle.reserved = lane.to;
            } else {
                limit = Math.min(limit, stopLine);
            }
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
