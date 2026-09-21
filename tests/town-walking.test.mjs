import test from 'node:test';
import assert from 'node:assert/strict';
import { createTown, setPeopleCount, setTrafficLevel, updateTown } from '../road-world.mjs';
import { planWalk, pavementPoint } from '../town-walking.mjs';
import { crossingPersonPoint } from '../town-crossings.mjs';
const pose = p => p.crossing ? crossingPersonPoint(p) : p.walkPose || pavementPoint(p.lane, p.distance);

test('pedestrians plan distant journeys and travel continuously through several streets', () => {
    const town = createTown(2400, 1700, 42);
    setTrafficLevel(town, 0); setPeopleCount(town, 16);
    town.people = town.people.filter(p => p.state === 'strolling');
    const walkers = town.people;
    const origins = walkers.map(p => pose(p));
    const lanes = walkers.map(p => new Set([p.lane]));
    for (const [i, p] of walkers.entries()) {
        assert.ok(p.walkRoute?.length > 2);
        assert.ok(Math.hypot(origins[i].x - p.walkDestination.x, origins[i].y - p.walkDestination.y) > 350);
    }
    let positions = walkers.map(pose);
    for (let tick = 0; tick < 1800 * 30; tick++) {
        updateTown(town, 1 / 30);
        walkers.forEach((p, i) => {
            const point = pose(p), previous = positions[i];
            assert.ok(Math.hypot(point.x - previous.x, point.y - previous.y) < 3, 'no jumps between streets or pavements');
            positions[i] = point; lanes[i].add(p.lane);
        });
    }
    assert.ok(walkers.every(p => p.walkTrips > 0), 'every walker reaches an actual destination');
    assert.ok(lanes.every(seen => seen.size >= 3), 'journeys span several streets');
    assert.ok(town.zebraTrips + town.jaywalkTrips > 0);
});

test('a passenger walks from their current location to the planned boarding stop', () => {
    const town = createTown(2400, 1700, 7);
    setTrafficLevel(town, 0); setPeopleCount(town, 2);
    const person = town.people[0];
    town.people = [person];
    const target = town.lanes.find(l => l.stop && l !== person.lane && planWalk(town, person, l));
    assert.ok(target);
    const origin = pose(person);
    person.state = 'walking';
    assert.deepEqual(pose(person), origin, 'planning does not teleport the person');
    for (let tick = 0; tick < 900 * 30 && person.state !== 'queue'; tick++) updateTown(town, 1 / 30);
    assert.equal(person.state, 'queue');
    assert.equal(person.lane, target);
    assert.equal(person.distance, target.stop.distance);
    assert.ok(target.stop.queue.includes(person));
});
