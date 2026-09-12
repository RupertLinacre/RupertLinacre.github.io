import test from 'node:test';
import assert from 'node:assert/strict';
import { createTown, updateTown, setTrafficLevel, findRoute, makeTurn, lanePoint } from '../road-world.mjs';
import { priorityHasGap, roundaboutHasGap } from '../junctions.mjs';
import { updateSingleTracks } from '../single-track.mjs';

const advance = (town, seconds) => { for (let i = 0; i < seconds * 60; i++) updateTown(town, 1 / 60); };

test('traffic corridors form a connected backbone with a mix of street and junction types', () => {
    for (const seed of [1, 42, 73, 384]) {
        const town = createTown(2200, 1700, seed);
        const main = town.edges.filter(e => e.roadType === 'arterial');
        const reached = new Set([main[0].a]);
        for (const node of reached) for (const lane of node.outgoing) if (lane.edge.roadType === 'arterial') reached.add(lane.to);
        assert.ok(main.every(e => reached.has(e.a) && reached.has(e.b)));
        for (const type of ['arterial', 'collector', 'residential', 'single']) assert.ok(town.edges.some(e => e.roadType === type), type);
        for (const control of ['roundabout', 'mini', 'signals', 'giveway']) assert.ok(town.nodes.some(n => n.control === control), control);
        assert.ok(town.nodes.filter(n => n.signal).length < town.nodes.filter(n => n.control === 'giveway').length);
    }
});

test('the route planner trades distance against main-road speed and live congestion', () => {
    const [a, b, c] = Array.from({ length: 3 }, () => ({ outgoing: [], control: 'bend' }));
    const lane = (from, to, roadType, length) => {
        const result = { from, to, hasPriority: true, edge: { roadType, path: { length } } };
        from.outgoing.push(result); return result;
    };
    const direct = lane(a, c, 'residential', 180);
    const main = lane(a, b, 'arterial', 130);
    const end = lane(b, c, 'arterial', 130);
    assert.deepEqual(findRoute(a, c, { distanceOnly: true }).path, [direct]);
    assert.deepEqual(findRoute(a, c, { live: false }).path, [main, end]);
    main.queueLength = 10;
    main.density = 0.8;
    assert.deepEqual(findRoute(a, c).path, [direct]);
    assert.equal(findRoute(a, a).seconds, 0);
});

test('roundabouts follow a clockwise orbit, yield to circulating traffic and admit clear gaps', () => {
    const town = createTown(2200, 1700, 42);
    for (const node of town.nodes.filter(n => ['roundabout', 'mini'].includes(n.control))) {
        const incoming = node.outgoing[0].edge.lanes.find(l => l.to === node);
        const outgoing = node.outgoing[1];
        const path = makeTurn(incoming, outgoing);
        assert.ok(path.exitAngle > path.entryAngle);
        assert.ok(path.points.every(p => Math.hypot(p.x - node.x, p.y - node.y) >= node.orbitRadius - 0.1));
        const candidate = { lane: incoming, next: outgoing, phase: 'lane', distance: incoming.length - 18, length: 24, speed: 0 };
        assert.ok(roundaboutHasGap(node, candidate));
        const circulating = { ...candidate, phase: 'turn', turn: path, distance: path.entryLength + 3, speed: 20 };
        node.occupants.add(circulating);
        assert.equal(roundaboutHasGap(node, candidate), false);
        node.occupants.clear();
    }
});

test('minor roads wait for an approaching main-road car, then take a clear gap', () => {
    const town = createTown(2200, 1700, 42);
    const node = town.nodes.find(n => n.control === 'giveway' && n.outgoing.length === 3);
    const incoming = node.outgoing.map(l => l.edge.lanes.find(v => v.to === node));
    const minor = incoming.find(l => !l.hasPriority);
    const main = incoming.find(l => l.hasPriority);
    const other = incoming.find(l => l !== main && l.hasPriority);
    const car = { lane: minor, next: node.outgoing.find(l => l.edge === main.edge), length: 24, speed: 0 };
    const through = { lane: main, next: node.outgoing.find(l => l.edge === other.edge), distance: main.length - 70, length: 24, speed: 35 };
    const occupied = new Map(town.lanes.map(l => [l, []]));
    occupied.get(main).push(through);
    assert.equal(priorityHasGap(car, occupied, () => true), false);
    occupied.set(main, []);
    assert.equal(priorityHasGap(car, occupied, () => true), true);
    // Two opposing U-turns must not yield to each other forever.
    const turnA = { ...through, next: main.edge.lanes.find(l => l.from === node), distance: main.length - 16, speed: 0 };
    const turnB = { ...through, lane: other, next: other.edge.lanes.find(l => l.from === node), distance: other.length - 16, speed: 0 };
    occupied.set(main, [turnA]); occupied.set(other, [turnB]);
    assert.equal(priorityHasGap(turnA, occupied, () => true), true);
});

test('single-track traffic shares the centreline and alternates opposing queues', () => {
    const town = createTown(2200, 1700, 42);
    const edge = town.edges.find(e => e.singleTrack);
    town.edges = [edge];
    const [forward, backward] = edge.lanes;
    const point = lanePoint(forward, (forward.singleEntry + forward.singleExit) / 2);
    const closest = Math.min(...backward.path.points.map(p => Math.hypot(p.x - point.x, p.y - point.y)));
    assert.ok(closest < 4, 'opposing vehicles really use the same physical space');
    const a = { lane: forward, phase: 'lane', distance: forward.singleEntry - 16, length: 24, narrowPermit: null };
    const b = { lane: backward, phase: 'lane', distance: backward.singleEntry - 16, length: 24, narrowPermit: null };
    const occupied = new Map([[forward, [a]], [backward, [b]]]);
    updateSingleTracks(town, occupied);
    assert.ok(a.narrowPermit === edge && !b.narrowPermit);
    town.time = 20;
    updateSingleTracks(town, occupied);
    assert.equal(b.narrowPermit, null, 'opposing traffic must wait for the rear to clear');
    a.distance = forward.singleExit + a.length / 2 + 5;
    updateSingleTracks(town, occupied);
    assert.ok(!a.narrowPermit && b.narrowPermit === edge, 'the waiting direction gets its turn');
});

test('rush-hour demand creates sustained congestion and removing traffic releases every reservation', () => {
    const quiet = createTown(2200, 1700, 42), busy = createTown(2200, 1700, 42);
    setTrafficLevel(quiet, 0.5); setTrafficLevel(busy, 6);
    advance(quiet, 120); advance(busy, 120);
    assert.ok(busy.vehicles.length > quiet.vehicles.length * 5);
    assert.ok(busy.metrics.ratio > 0.6 && busy.metrics.ratio > quiet.metrics.ratio + 0.25);
    assert.ok(busy.metrics.flow < quiet.metrics.flow * 0.65);
    assert.ok(busy.vehicles.some(v => v.waitingSince && busy.time - v.waitingSince > 30), 'queues persist beyond a single signal cycle');
    for (const edge of busy.edges.filter(e => e.singleTrack)) assert.ok(new Set([...edge.narrowVehicles].map(v => v.lane.reverse)).size <= 1);
    setTrafficLevel(busy, 0);
    assert.ok(busy.nodes.every(n => n.occupants.size === 0 && !n.owner));
    assert.ok(busy.edges.every(e => !e.singleTrack || e.narrowVehicles.size === 0 && e.narrowDirection === null));
    setTrafficLevel(busy, 0.5); advance(busy, 60);
    assert.ok(busy.vehicles.every(v => v.distanceTravelled > 20));
});
