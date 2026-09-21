import test from 'node:test';
import assert from 'node:assert/strict';
import { createTown, setTrafficLevel, setPeopleCount, updateTown, vehiclePoint } from '../road-world.mjs';
import { requestCrossing, crossingPersonPoint } from '../town-crossings.mjs';

function fixture(jaywalk = false, close = false) {
    const town = createTown(2400, 1700, 42);
    const template = town.vehicles.find(v => !v.bus);
    setTrafficLevel(town, 0);
    const crossing = town.crossings.find(c => c.edge.lanes.every(l => c.distances.get(l) > 100));
    const lane = crossing.edge.lanes[0];
    if (jaywalk) lane.edge.crossings = [];
    const person = { id: 999, lane, distance: crossing.distances.get(lane), speed: 9, direction: 1, state: 'strolling', crossCooldown: 0 };
    town.people = [person];
    town.vehicles = lane.edge.lanes.map((l, i) => ({ ...template, id: 1000 + i, lane: l,
        distance: crossing.distances.get(l) - (close && i === 0 ? 25 : 90),
        speed: 25, maxSpeed: 35, phase: 'lane', reserved: null, next: l.to.outgoing.find(out => out.to !== l.from) || l.to.outgoing[0],
        overtake: null }));
    assert.ok(requestCrossing(town, person, jaywalk ? null : crossing));
    return { town, person, crossing: person.crossing, original: lane };
}
for (const jaywalk of [false, true]) test(jaywalk ? 'jaywalking stops traffic away from markings' : 'zebras stop both lanes until the pedestrian reaches the pavement', () => {
    const { town, person, crossing, original } = fixture(jaywalk);
    const stopped = new Set();
    for (let tick = 0; tick < 900 && person.crossing; tick++) {
        updateTown(town, 1 / 60);
        for (const v of town.vehicles) {
            if (v.speed < 0.1) stopped.add(v.id);
            if (person.state === 'crossing') {
                const p = crossingPersonPoint(person), c = vehiclePoint(v);
                const dx = p.x - c.x, dy = p.y - c.y;
                const along = dx * Math.cos(c.angle) + dy * Math.sin(c.angle);
                const across = -dx * Math.sin(c.angle) + dy * Math.cos(c.angle);
                assert.ok(Math.abs(along) > v.length / 2 + 2 || Math.abs(across) > v.width / 2 + 2, 'vehicle cannot overlap a crossing pedestrian');
            }
        }
    }
    assert.equal(stopped.size, 2);
    assert.equal(person.state, 'strolling');
    assert.notEqual(person.lane, original);
    assert.equal(crossing.users.size, 0);
    for (let i = 0; i < 60; i++) updateTown(town, 1 / 60);
    assert.ok(town.vehicles.every(v => v.speed > 0));
    assert.equal(town[jaywalk ? 'jaywalkTrips' : 'zebraTrips'], 1);
});
test('pedestrians wait for a vehicle too close to stop, and removing people releases traffic', () => {
    const { town, person } = fixture(false, true);
    updateTown(town, 1 / 60);
    assert.equal(person.state, 'crossing_wait');
    assert.ok(town.vehicles[0].speed > 0);
    setPeopleCount(town, 0);
    assert.equal(town.activeCrossings.size, 0);
    assert.ok(town.crossings.every(c => !c.users.size));
});
