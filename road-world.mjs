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
    return pathPoint(lane.path, distance);
}

function measure(points) {
    let length = 0;
    points.forEach((point, i) => {
        if (i) length += Math.hypot(point.x - points[i - 1].x, point.y - points[i - 1].y);
        point.distance = length;
    });
    return { points, length };
}

export function pathPoint(path, distance) {
    const points = path.points;
    distance = Math.max(0, Math.min(path.length, distance));
    let low = 1;
    let high = points.length - 1;
    while (low < high) {
        const mid = (low + high) >>> 1;
        if (points[mid].distance < distance) low = mid + 1;
        else high = mid;
    }
    const a = points[low - 1];
    const b = points[low];
    const t = (distance - a.distance) / (b.distance - a.distance || 1);
    const angle = a.angle === undefined ? Math.atan2(b.y - a.y, b.x - a.x) :
        a.angle + Math.atan2(Math.sin(b.angle - a.angle), Math.cos(b.angle - a.angle)) * t;
    return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, angle };
}

export function offsetPath(path, offset, start = 0, end = path.length) {
    const count = Math.max(2, Math.ceil((end - start) / 4));
    return measure(Array.from({ length: count + 1 }, (_, i) => {
        const p = pathPoint(path, start + (end - start) * i / count);
        return { x: p.x + Math.sin(p.angle) * offset, y: p.y - Math.cos(p.angle) * offset, angle: p.angle };
    }));
}

function curveBetween(a, b, bend) {
    const dx = b.x - a.x;
    const dy = b.y - a.y;
    const length = Math.hypot(dx, dy);
    return measure(Array.from({ length: 81 }, (_, i) => {
        const t = i / 80;
        // A smooth bow with straight approaches keeps the junction mouths clear.
        const u = Math.max(0, Math.min(1, (t - 0.25) / 0.5));
        const bulge = Math.sin(Math.PI * u) ** 2 * bend;
        const derivative = t > 0.25 && t < 0.75 ? Math.PI * Math.sin(2 * Math.PI * u) * bend / 0.5 : 0;
        return { x: a.x + dx * t - dy / length * bulge, y: a.y + dy * t + dx / length * bulge,
            angle: Math.atan2(dy + dx / length * derivative, dx - dy / length * derivative) };
    }));
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
    function coordinates(size) {
        const values = [-210 - random() * 60];
        while (values.at(-1) < size + 240) values.push(values.at(-1) + 245 + random() * 85);
        return values;
    }
    const xs = coordinates(width);
    const ys = coordinates(height);
    const nodes = [];
    const edges = [];
    const lanes = [];
    const blocks = [];
    const wave = random() * Math.PI * 2;
    const grid = ys.map((y, row) => xs.map((x, col) => {
        const node = { id: nodes.length,
            x: x + Math.sin(y / 430 + wave) * 90 + (random() - 0.5) * 42,
            y: y + Math.sin(x / 480 + wave) * 90 + (random() - 0.5) * 42,
            row, col, outgoing: [], owner: null, radius: JUNCTION,
            signal: false, offset: random() * 22, cycle: 19 + random() * 6 };
        nodes.push(node);
        return node;
    }));
    let nextEdgeId = 0;
    let nextLaneId = 0;
    const boundaries = new Map();
    const edgeKey = (a, b) => [a.id, b.id].sort((x, y) => x - y).join(':');
    function connect(a, b, diagonal = false) {
        const distance = Math.hypot(b.x - a.x, b.y - a.y);
        const edge = { id: nextEdgeId++, a, b, diagonal,
            axis: Math.abs(b.x - a.x) > Math.abs(b.y - a.y) ? 'x' : 'y', lanes: [],
            path: curveBetween(a, b, diagonal ? 0 : (random() - 0.5) * 34) };
        edges.push(edge);
        boundaries.set(edgeKey(a, b), edge);
        for (const [from, to] of [[a, b], [b, a]]) {
            const dx = (to.x - from.x) / distance;
            const dy = (to.y - from.y) / distance;
            const lane = { id: nextLaneId++, edge, from, to, dx, dy, axis: edge.axis, stop: null };
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
    function minimumAngle(node, extra) {
        const directions = node.outgoing.map(l => Math.atan2(l.dy, l.dx));
        if (extra) directions.push(Math.atan2(extra.y - node.y, extra.x - node.x));
        directions.sort((a, b) => a - b);
        return Math.min(...directions.map((angle, i) =>
            (directions[(i + 1) % directions.length] - angle + Math.PI * 2) % (Math.PI * 2)));
    }
    const diagonals = new Map();
    for (let r = 0; r < ys.length - 1; r++) {
        for (let c = 0; c < xs.length - 1; c++) {
            if (random() > 0.3) continue;
            const reverse = random() > 0.5;
            const a = grid[r][c + (reverse ? 1 : 0)];
            const b = grid[r + 1][c + (reverse ? 0 : 1)];
            if (a.outgoing.length >= 5 || b.outgoing.length >= 5 ||
                minimumAngle(a, b) < 0.72 || minimumAngle(b, a) < 0.72) continue;
            connect(a, b, true);
            diagonals.set(`${r}:${c}`, reverse);
        }
    }
    for (const node of nodes) {
        node.signal = node.outgoing.length >= 3;
        // Acute and five-way junctions need more clearance than square corners.
        node.radius = Math.max(JUNCTION, 31 / Math.tan(minimumAngle(node) / 2));
    }
    for (const lane of lanes) {
        const reverse = lane.from !== lane.edge.a;
        const center = reverse ? measure([...lane.edge.path.points].reverse().map(p =>
            ({ x: p.x, y: p.y, angle: p.angle + Math.PI }))) : lane.edge.path;
        lane.path = offsetPath(center, LANE, lane.from.radius, center.length - lane.to.radius);
        lane.length = lane.path.length;
        lane.start = pathPoint(lane.path, 0);
        lane.end = pathPoint(lane.path, lane.length);
    }
    function addBlock(corners) {
        const polygon = corners.flatMap((a, i) => {
            const b = corners[(i + 1) % corners.length];
            const edge = boundaries.get(edgeKey(a, b));
            const points = edge.a === a ? edge.path.points : [...edge.path.points].reverse();
            return points.slice(0, -1).map(p => ({ x: p.x, y: p.y }));
        });
        const center = { x: corners.reduce((sum, p) => sum + p.x, 0) / corners.length,
            y: corners.reduce((sum, p) => sum + p.y, 0) / corners.length };
        let angle = Math.atan2(corners[1].y - corners[0].y, corners[1].x - corners[0].x);
        while (angle > Math.PI / 4) angle -= Math.PI / 2;
        while (angle < -Math.PI / 4) angle += Math.PI / 2;
        const local = polygon.map(p => ({ x: (p.x - center.x) * Math.cos(angle) + (p.y - center.y) * Math.sin(angle),
            y: -(p.x - center.x) * Math.sin(angle) + (p.y - center.y) * Math.cos(angle) }));
        const x = Math.min(...local.map(p => p.x));
        const y = Math.min(...local.map(p => p.y));
        blocks.push({ x, y, width: Math.max(...local.map(p => p.x)) - x,
            height: Math.max(...local.map(p => p.y)) - y, center, angle, polygon: local,
            kind: corners.length === 3 ? 'park' : choose(['homes', 'homes', 'shops', 'park']), seed: random() * 4294967296 });
    }
    for (let r = 0; r < ys.length - 1; r++) {
        for (let c = 0; c < xs.length - 1; c++) {
            const [a, b, d, e] = [grid[r][c], grid[r][c + 1], grid[r + 1][c + 1], grid[r + 1][c]];
            const diagonal = diagonals.get(`${r}:${c}`);
            if (diagonal === false) { addBlock([a, b, d]); addBlock([a, d, e]); }
            else if (diagonal === true) { addBlock([a, b, e]); addBlock([b, d, e]); }
            else addBlock([a, b, d, e]);
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
        if (lane.to.owner || signalState(lane.to, lane.axis, town.time) !== 'green') continue;
        const room = occupied.get(vehicle.next).every(other =>
            other.distance - other.length / 2 > vehicle.length / 2 + GAP + 8);
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
