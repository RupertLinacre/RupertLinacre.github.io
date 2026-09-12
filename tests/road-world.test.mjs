import test from 'node:test';
import assert from 'node:assert/strict';
import { createTown, updateTown, setTrafficLevel, signalState, vehiclePoint, isRoundabout, GAP } from '../road-world.mjs';

const step = 1 / 60;
function advance(town, seconds) {
    for (let i = 0; i < seconds / step; i++) updateTown(town, step);
}

test('generated streets stay connected and numbered bus circuits close on desktop and mobile', () => {
    for (const [width, height] of [[1440, 1000], [500, 1080], [320, 500]]) {
        for (let seed = 1; seed <= 30; seed++) {
            const town = createTown(width, height, seed);
            const visited = new Set([town.nodes[0]]);
            for (const node of visited) for (const lane of node.outgoing) visited.add(lane.to);
            assert.equal(visited.size, town.nodes.length, `disconnected town ${seed}`);
            for (const route of town.routes) {
                assert.ok(route.path.length > 0);
                assert.ok(route.path.some(lane => lane.stop), 'route must have stops');
                route.path.forEach((lane, i) => {
                    assert.equal(lane.to, route.path[(i + 1) % route.path.length].from);
                    assert.ok(town.lanes.includes(lane));
                    assert.ok(lane.length > 0);
                });
            }
            assert.ok(town.vehicles.some(v => v.bus));
            assert.ok(town.vehicles.some(v => !v.bus));
        }
    }
});

test('a seed reproduces its town and different seeds change streets and neighbourhoods', () => {
    const describe = seed => {
        const town = createTown(1200, 900, seed);
        return JSON.stringify({
            nodes: town.nodes.map(n => [n.x, n.y]),
            edges: town.edges.map(e => [e.a.id, e.b.id]),
            blocks: town.blocks,
            routes: town.routes.map(route => route.path.map(lane => lane.id)),
        });
    };
    assert.equal(describe(42), describe(42));
    assert.notEqual(describe(42), describe(43));
});

test('signals have amber and all-red clearance and never release both axes together', () => {
    const node = { signal: true, offset: 0, cycle: 20 };
    let amber = false;
    let allRed = false;
    for (let time = 0; time < 40; time += step) {
        const x = signalState(node, 'x', time);
        const y = signalState(node, 'y', time);
        assert.ok(!(x === 'green' && y === 'green'));
        amber ||= x === 'amber';
        allRed ||= x === 'red' && y === 'red';
    }
    assert.ok(amber && allRed);
});

test('streets use genuine circular arcs and continue smoothly through junctions', () => {
    const town = createTown(2200, 1700, 42);
    const arcs = town.edges.filter(edge => edge.path.kind === 'arc');
    assert.ok(arcs.length > town.edges.length * 0.65);
    assert.ok(arcs.some(edge => Math.abs(edge.path.points.at(-1).angle - edge.path.points[0].angle) > Math.PI / 4),
        'the layout must have sweeping bends, not tiny deviations from straight roads');
    for (const { path } of arcs) {
        const sign = Math.sign(path.points.at(-1).angle - path.points[0].angle);
        path.points.forEach((point, i) => {
            assert.ok(Math.abs(Math.hypot(point.x - path.center.x, point.y - path.center.y) - path.radius) < 0.1,
                'each street arc must retain its circular radius after subdivision');
            if (i) assert.ok((point.angle - path.points[i - 1].angle) * sign >= -1e-8,
                'curvature must not reverse partway along an arc');
        });
    }
    for (const node of town.nodes) {
        assert.ok(node.outgoing.some(a => node.outgoing.some(b =>
            a !== b && Math.cos(a.heading - b.heading) < -0.99999)),
            'a main street must keep its tangent when a side street joins');
    }
    assert.equal(town.nodes.length - town.edges.length + town.blocks.length, 1,
        'every neighbourhood must be a face in the connected street network');
    assert.ok(town.blocks.every(block => block.polygon.length > 3));
    for (const lane of town.lanes) {
        assert.ok(lane.length > 50, 'junctions must leave usable lane space');
        assert.ok(lane.path.points.every(p => Number.isFinite(p.x) && Number.isFinite(p.y)));
    }
});

test('live traffic changes preserve the town, insert safely, and release reservations when removing vehicles', () => {
    const town = createTown(1800, 1200, 94);
    advance(town, 30);
    const edges = town.edges;
    const routes = town.routes;
    const time = town.time;
    const previous = new Map(town.vehicles.map(v => [v.id, vehiclePoint(v)]));
    const initialCount = town.vehicles.length;
    setTrafficLevel(town, 2);
    assert.ok(town.vehicles.length > initialCount);
    assert.equal(town.edges, edges);
    assert.equal(town.routes, routes);
    assert.equal(town.time, time);
    for (const vehicle of town.vehicles) {
        if (previous.has(vehicle.id)) assert.deepEqual(vehiclePoint(vehicle), previous.get(vehicle.id));
    }
    assert.equal(new Set(town.vehicles.map(v => v.id)).size, town.vehicles.length);
    advance(town, 20);
    setTrafficLevel(town, 0);
    assert.equal(town.vehicles.length, 0);
    assert.ok(town.nodes.every(node => node.owner === null));
    assert.ok(town.nodes.every(node => node.occupants.size === 0));
    assert.ok(town.edges.every(edge => !edge.singleTrack || edge.narrowVehicles.size === 0));
    advance(town, 1);
    setTrafficLevel(town, 1);
    assert.ok(town.vehicles.some(v => v.bus));
    assert.ok(town.vehicles.some(v => !v.bus));
    assert.ok(town.vehicles.every(v => !previous.has(v.id)), 'vehicle IDs must not be reused');
    advance(town, 60);
    assert.ok(town.vehicles.some(v => v.distanceTravelled > 150));
});

test('cars stop before a red light and depart when it turns green', () => {
    const town = createTown(1200, 800, 15);
    const car = town.vehicles.find(v => !v.bus && v.lane.to.signal);
    town.vehicles = [car];
    const lane = car.lane;
    lane.to.cycle = 20;
    lane.to.offset = lane.axis === 'x' ? 12 : 2;
    const stopLine = lane.length - car.length / 2 - 4;
    car.distance = stopLine - 8;
    advance(town, 5);
    assert.equal(car.phase, 'lane');
    assert.equal(car.lane, lane);
    assert.ok(car.distance <= stopLine);
    assert.equal(car.speed, 0);
    advance(town, 12);
    assert.ok(car.distanceTravelled > 80);
});

test('buses dwell at a stop before continuing their numbered route', () => {
    const town = createTown(1200, 800, 16);
    const bus = town.vehicles.find(v => v.bus);
    town.vehicles = [bus];
    bus.routeIndex = bus.route.path.findIndex(lane => lane.stop);
    bus.lane = bus.route.path[bus.routeIndex];
    bus.next = bus.route.path[(bus.routeIndex + 1) % bus.route.path.length];
    bus.distance = bus.lane.stop.distance;
    bus.served = false;
    advance(town, 1);
    assert.equal(bus.distance, bus.lane.stop.distance);
    assert.ok(bus.dwell > 0);
    assert.equal(bus.stopsVisited, 0);
    advance(town, 3);
    assert.equal(bus.stopsVisited, 1);
    assert.ok(bus.distance > bus.lane.stop.distance);
});

test('a bus waits until its whole length can clear the junction behind a stationary queue', () => {
    const town = createTown(1440, 1000, 1);
    const bus = town.vehicles.find(v => v.bus && v.next.length > 100);
    const leader = town.vehicles.find(v => !v.bus);
    town.vehicles = [bus, leader];
    const node = bus.lane.to;
    node.signal = false;
    node.control = 'bend';
    bus.served = true;
    bus.distance = bus.lane.length - bus.length / 2 - 4;
    leader.lane = bus.next;
    leader.next = leader.lane.to.outgoing[0];
    leader.distance = bus.length + GAP + 2 + leader.length / 2;
    leader.maxSpeed = 0;
    const stoppedAt = bus.distance;
    advance(town, 2);
    assert.equal(node.owner, null, 'front-bumper space alone must not reserve the junction');
    assert.equal(bus.distance, stoppedAt);
    leader.distance += 25;
    advance(town, 1);
    assert.ok(bus.distance > stoppedAt || bus.phase === 'turn');
});

test('short curved streets do not lock neighbouring junctions in a longer run', () => {
    const town = createTown(1440, 1000, 1);
    advance(town, 300);
    const distance = town.vehicles.map(v => v.distanceTravelled);
    const buses = town.vehicles.filter(v => v.bus);
    const stops = buses.map(v => v.stopsVisited);
    advance(town, 300);
    assert.ok(town.vehicles.every((v, i) => v.distanceTravelled > distance[i] + 200));
    assert.ok(buses.every((v, i) => v.stopsVisited > stops[i]));
});

test('five-minute light-traffic runs keep queues separated, turns continuous, and buses serving stops', () => {
    for (const [width, height, seed] of [[1440, 1000, 1], [1440, 1000, 42], [500, 1080, 73]]) {
        const town = createTown(width, height, seed);
        // Check bus service in free-flow conditions. Heavy demand deliberately
        // delays fixed bus circuits, and is covered by the rush-hour tests.
        setTrafficLevel(town, 0.65);
        let previous = town.vehicles.map(vehiclePoint);
        for (let tick = 0; tick < 300 / step; tick++) {
            updateTown(town, step);
            const queues = new Map();
            town.vehicles.forEach((vehicle, index) => {
                const p = vehiclePoint(vehicle);
                assert.ok(Number.isFinite(p.x) && Number.isFinite(p.y) && Number.isFinite(p.angle));
                assert.ok(Math.hypot(p.x - previous[index].x, p.y - previous[index].y) < 1.1, 'vehicle must not teleport at a turn');
                previous[index] = p;
                if (vehicle.phase === 'lane') {
                    if (!queues.has(vehicle.lane)) queues.set(vehicle.lane, []);
                    queues.get(vehicle.lane).push(vehicle);
                }
            });
            for (const queue of queues.values()) {
                queue.sort((a, b) => b.distance - a.distance);
                for (let i = 1; i < queue.length; i++) {
                    const front = queue[i - 1];
                    const rear = queue[i];
                    const gap = front.distance - rear.distance - (front.length + rear.length) / 2;
                    assert.ok(gap >= GAP - 0.01, `queue overlap in town ${seed}: ${gap}`);
                }
            }
            for (const node of town.nodes) {
                const crossing = town.vehicles.filter(v => v.phase === 'turn' && v.lane.to === node);
                if (!isRoundabout(node)) {
                    assert.ok(crossing.length <= 1, 'junction must not admit conflicting traffic');
                    if (crossing.length) assert.ok(node.owner === crossing[0]);
                }
                assert.ok(crossing.every(v => node.occupants.has(v)));
            }
        }
        assert.ok(town.vehicles.every(v => v.distanceTravelled > 250), 'no vehicle should remain stuck');
        assert.ok(town.vehicles.filter(v => v.bus).every(v => v.stopsVisited >= 2), 'each bus must still serve multiple stops with the new junction delays');
    }
});
