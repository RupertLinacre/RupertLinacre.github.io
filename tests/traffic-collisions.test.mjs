import test from 'node:test';
import assert from 'node:assert/strict';
import { createTown, updateTown, vehiclePoint, setTrafficLevel, setPeopleCount, setCyclistCount } from '../road-world.mjs';

// Separating-axis check on the actual oriented vehicle bodies, with a small
// tolerance for antialiased body edges. This catches crossings across lanes too.
function overlaps(a, b) {
    const dx = b.p.x - a.p.x, dy = b.p.y - a.p.y;
    const reach = (Math.hypot(a.v.length, a.v.width) + Math.hypot(b.v.length, b.v.width)) / 2;
    if (Math.hypot(dx, dy) > reach) return false;
    for (const axis of [a.x, a.y, b.x, b.y]) {
        const center = Math.abs(dx * axis.x + dy * axis.y);
        const radius = o => (o.v.length / 2 - 0.4) * Math.abs(o.x.x * axis.x + o.x.y * axis.y) +
            (o.v.width / 2 - 0.4) * Math.abs(o.y.x * axis.x + o.y.y * axis.y);
        if (center >= radius(a) + radius(b)) return false;
    }
    return true;
}

test('vehicle bodies stay separated at roundabouts and single tracks, including rush hour', () => {
    let simultaneous = false;
    for (const seed of [1, 42]) {
        const town = createTown(2200, 1700, seed);
        for (let tick = 0; tick < 60 * 150; tick++) {
            if (tick === 2400) setTrafficLevel(town, 6);
            if (tick === 6600) setTrafficLevel(town, 0.5);
            updateTown(town, 1 / 60);
            if (tick % 6) continue;
            simultaneous ||= town.nodes.some(n => n.control === 'roundabout' && n.occupants.size > 1);
            for (const edge of town.edges.filter(e => e.singleTrack)) {
                assert.ok(new Set([...edge.narrowVehicles].map(v => v.lane.reverse)).size <= 1, 'opposing single-track permits');
            }
            const rectangles = town.vehicles.map(v => {
                const p = vehiclePoint(v);
                return { v, p, x: { x: Math.cos(p.angle), y: Math.sin(p.angle) },
                    y: { x: -Math.sin(p.angle), y: Math.cos(p.angle) } };
            });
            for (let i = 0; i < rectangles.length; i++) for (let j = i + 1; j < rectangles.length; j++) {
                assert.ok(!overlaps(rectangles[i], rectangles[j]),
                    `body overlap: seed ${seed}, time ${tick / 60}, vehicles ${rectangles[i].v.id}/${rectangles[j].v.id}`);
            }
        }
    }
    assert.ok(simultaneous, 'full roundabouts should safely admit more than one vehicle');
});


test('cyclists and overtaking cars remain separated in mixed traffic with passenger queues', () => {
    for (const seed of [1, 42]) {
        const town = createTown(2400, 1700, seed);
        setCyclistCount(town, 30); setPeopleCount(town, 180);
        for (let tick = 0; tick < 60 * 120; tick++) {
            if (tick === 2400) { setCyclistCount(town, 100); setPeopleCount(town, 400); setTrafficLevel(town, 3); }
            if (tick === 5400) { setCyclistCount(town, 0); setTrafficLevel(town, 0.5); setPeopleCount(town, 40); }
            updateTown(town, 1 / 60);
            if (tick % 12) continue;
            const rectangles = town.vehicles.map(v => {
                const p = vehiclePoint(v);
                return { v, p, x: { x: Math.cos(p.angle), y: Math.sin(p.angle) }, y: { x: -Math.sin(p.angle), y: Math.cos(p.angle) } };
            });
            for (let i = 0; i < rectangles.length; i++) for (let j = i + 1; j < rectangles.length; j++) {
                assert.ok(!overlaps(rectangles[i], rectangles[j]),
                    `mixed overlap seed ${seed}, time ${tick / 60}, vehicles ${rectangles[i].v.id}/${rectangles[j].v.id}`);
            }
            const queued = town.lanes.flatMap(l => l.stop?.queue || []);
            const riders = town.vehicles.flatMap(v => v.passengers || []);
            const walkers = town.people.filter(p => p.state === 'crossing' || p.state === 'crossing_wait' || p.state === 'strolling' || p.state === 'walking' || p.state === 'leaving');
            assert.equal(new Set([...queued, ...riders, ...walkers]).size, town.people.length);
            assert.equal(queued.length + riders.length + walkers.length, town.people.length);
        }
        assert.ok(town.edges.every(e => !e.passing || town.vehicles.includes(e.passing)));
    }
});
