import test from 'node:test';
import assert from 'node:assert/strict';
import { createTown, setTrafficLevel, setCyclistCount, setPeopleCount, updateTown, vehiclePoint } from '../road-world.mjs';
import { measure } from '../street-geometry.mjs';
import { serveBus } from '../town-people.mjs';
const advance = (town, seconds) => { for (let i = 0; i < seconds * 60; i++) updateTown(town, 1 / 60); };
function busFixture(count) {
    const town = createTown(2400, 1700, 42);
    const bus = town.vehicles.find(v => v.bus);
    setTrafficLevel(town, 0);
    const lane = bus.route.path.find(l => l.stop);
    const destination = bus.route.path.find(l => l.stop && l !== lane);
    Object.assign(bus, { lane, distance: lane.stop.distance, speed: 0, served: false, phase: 'lane', reserved: null,
        next: bus.route.path[(bus.route.path.indexOf(lane) + 1) % bus.route.path.length], passengers: [] });
    town.vehicles = [bus];
    town.people = Array.from({ length: count }, (_, id) => ({ id, lane, destination, route: bus.route.number, state: 'queue', trips: 0, speed: 8 }));
    lane.stop.queue = [...town.people];
    return { town, bus, lane, destination };
}
test('boarding is sequential, keeps the bus stopped, and larger queues increase dwell', () => {
    const empty = busFixture(0), busy = busFixture(8);
    advance(empty.town, 4); advance(busy.town, 4);
    assert.ok(empty.bus.served);
    assert.ok(!busy.bus.served && busy.bus.passengers.length > 0 && busy.bus.passengers.length < 8);
    assert.equal(busy.bus.distance, busy.lane.stop.distance);
    advance(busy.town, 10);
    assert.equal(busy.bus.passengers.length, 8);
    assert.ok(busy.bus.lastDwell > empty.bus.lastDwell + 6);
    assert.equal(busy.lane.stop.queue.length, 0);
    assert.ok(busy.town.people.every(p => p.state === 'riding' && p.bus === busy.bus));
});
test('full buses leave a queue and passengers alight at their destination', () => {
    const { town, bus, lane, destination } = busFixture(30);
    advance(town, 20);
    assert.equal(bus.passengers.length, 18);
    assert.equal(lane.stop.queue.length, 12);
    bus.lane = destination; bus.served = false;
    for (let i = 0; i < 1000 && !bus.served; i++) serveBus(town, bus, 1 / 60);
    assert.equal(bus.passengers.length, 0);
    assert.equal(town.people.filter(p => p.state === 'leaving' && p.trips === 1).length, 18);
});
test('population controls are independent and never leave ghost queue members or passengers', () => {
    const { town, bus } = busFixture(8); advance(town, 12);
    setCyclistCount(town, 20); setTrafficLevel(town, 0);
    assert.equal(town.vehicles.filter(v => v.cyclist).length, 20);
    assert.equal(town.people.length, 8);
    assert.equal(bus.passengers.length, 0);
    assert.ok(town.people.every(p => p.state === 'queue' && !p.bus));
    setPeopleCount(town, 0); setCyclistCount(town, 0);
    assert.ok(town.lanes.every(l => !l.stop || l.stop.queue.length === 0));
    assert.equal(town.vehicles.length, 0);
    assert.ok(town.nodes.every(n => n.occupants.size === 0));
});
function passFixture(oncoming = false) {
    const town = createTown(2400, 1700, 42);
    const car = town.vehicles.find(v => !v.bus);
    setCyclistCount(town, 1);
    const bike = town.vehicles.find(v => v.cyclist);
    setTrafficLevel(town, 0); setCyclistCount(town, 0);
    const lane = town.lanes.find(l => !l.stop && !l.edge.singleTrack);
    // A straight, empty test road gives a controlled passing opportunity.
    lane.edge.speed = 48; lane.edge.crossings = [];
    for (const l of lane.edge.lanes) {
        l.path = measure(Array.from({ length: 101 }, (_, i) => ({ x: i * 10, y: l === lane ? 0 : 26, angle: 0 })));
        l.length = 1000; l.start = l.path.points[0]; l.end = l.path.points.at(-1);
    }
    for (const [v, distance, speed] of [[car, 65, 10], [bike, 100, 13]]) {
        Object.assign(v, { lane, distance, speed, maxSpeed: v.cyclist ? 13 : 48, phase: 'lane', reserved: null, next: lane.to.outgoing[0] });
    }
    town.vehicles = [bike, car];
    if (oncoming) town.vehicles.push({ ...car, id: 9999, lane: lane.edge.lanes.find(l => l !== lane), distance: 100, speed: 0, maxSpeed: 0,
        next: lane.from.outgoing[0], length: 24 });
    return { town, car, bike, lane };
}
test('cars physically move into the opposing lane to pass a slow cyclist and then merge back', () => {
    const { town, car, bike } = passFixture();
    let pulledOut = false;
    for (let i = 0; i < 15 * 60; i++) {
        updateTown(town, 1 / 60);
        if (car.overtake?.offset > 0.85) pulledOut = true;
        const a = vehiclePoint(car), b = vehiclePoint(bike);
        assert.ok(Math.hypot(a.x - b.x, a.y - b.y) > 9, 'passing bodies remain separate');
    }
    assert.ok(pulledOut);
    assert.ok(town.overtakes >= 1);
    assert.equal(car.overtake, null);
    assert.ok(car.distance > bike.distance);
});
test('oncoming traffic prevents a pass; a car follows at cycling speed', () => {
    const { town, car, bike } = passFixture(true);
    advance(town, 7);
    assert.ok(!car.overtake && !town.overtakes);
    assert.ok(car.distance < bike.distance - (car.length + bike.length) / 2);
    assert.ok(car.speed <= bike.maxSpeed + 1);
});


test('people keep walking on pavements throughout town as well as waiting for buses', () => {
    const town = createTown(2400, 1700, 42); setPeopleCount(town, 140);
    const walkers = town.people.filter(p => p.state === 'strolling');
    assert.equal(walkers.length, 70);
    assert.ok(walkers.some(p => !p.lane.stop), 'pavement walkers are not limited to bus-stop streets');
    assert.ok(new Set(walkers.map(p => p.lane.id)).size > 20);
    assert.ok(walkers.some(p => p.direction === -1) && walkers.some(p => p.direction === 1));
    const positions = walkers.map(p => p.distance);
    advance(town, 3);
    assert.ok(walkers.every((p, i) => Math.abs(p.distance - positions[i]) > 0.1));
    advance(town, 90);
    assert.ok(walkers.every(p => ['strolling', 'crossing', 'crossing_wait'].includes(p.state) && p.distance >= 0 && p.distance <= p.lane.length));
    assert.ok(town.people.some(p => p.state === 'queue' || p.state === 'riding'));
    setPeopleCount(town, 0); assert.equal(town.people.length, 0);
});
