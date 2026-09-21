import { pathPoint, measure } from './street-geometry.mjs';
import { requestCrossing } from './town-crossings.mjs';

export function pavementPoint(lane, distance) {
    const p = pathPoint(lane.path, distance);
    const normal = lane.edge.width / 2 - lane.edge.offset + 3;
    const nearStop = lane.stop ? Math.max(0, 1 - Math.abs(distance - lane.stop.distance) / 24) : 0;
    const offset = normal + (25 - normal) * nearStop;
    return { x: p.x + Math.sin(p.angle) * offset, y: p.y - Math.cos(p.angle) * offset, angle: p.angle };
}
export function walkingNetwork(town) {
    if (town.walkingNetwork) return town.walkingNetwork;
    const nodes = [], byLane = new Map();
    function add(lane, distance) {
        let n = byLane.get(lane)?.find(n => Math.abs(n.distance - distance) < 0.01);
        if (n) return n;
        n = { lane, distance, links: [], ...pavementPoint(lane, distance) };
        nodes.push(n);
        if (!byLane.has(lane)) byLane.set(lane, []);
        byLane.get(lane).push(n);
        return n;
    }
    function join(a, b, type, extra = {}) {
        const cost = type === 'walk' ? Math.abs(a.distance - b.distance) :
            type === 'corner' ? extra.path.length : a.lane.edge.width + (type === 'zebra' ? 15 : 110);
        a.links.push({ to: b, type, cost, ...extra });
        b.links.push({ to: a, type, cost, ...extra, backwards: true });
    }
    for (const lane of town.lanes) {
        if (lane.edge.singleTrack) continue;
        for (const distance of [0, lane.length / 2, lane.length]) add(lane, distance);
        if (lane.stop) add(lane, lane.stop.distance);
    }
    for (const edge of town.edges) {
        if (edge.singleTrack) continue;
        for (const c of edge.crossings.filter(c => c.zebra))
            join(...edge.lanes.map(l => add(l, c.distances.get(l))), 'zebra', { crossing: c });
        // An occasional unmarked crossing offers a shorter route, at a high cost.
        if (edge.lanes.every(l => l.length > 190 && (!l.stop || Math.abs(l.stop.distance - l.length / 2) > 65)) &&
            !edge.crossings.some(c => Math.abs(c.distances.get(edge.lanes[0]) - edge.lanes[0].length / 2) < 65))
            join(...edge.lanes.map(l => add(l, l.length / 2)), 'jaywalk');
    }
    for (const points of byLane.values()) {
        points.sort((a, b) => a.distance - b.distance);
        for (let i = 1; i < points.length; i++) join(points[i - 1], points[i], 'walk');
    }
    for (const junction of town.nodes) {
        // Adjacent street arms share a pavement around the outside corner.
        // Never connect across an intervening carriageway or a single-track arm.
        const arms = [...junction.outgoing].sort((a, b) => a.heading - b.heading);
        if (arms.length < 2) continue;
        for (let i = 0; i < arms.length; i++) {
            const arm = arms[i], next = arms[(i + 1) % arms.length];
            if (arm.edge.singleTrack || next.edge.singleTrack) continue;
            const incoming = arm.edge.lanes.find(l => l.to === junction);
            const a = add(incoming, incoming.length), b = add(next, 0);
            const start = Math.atan2(a.y - junction.y, a.x - junction.x);
            let sweep = Math.atan2(b.y - junction.y, b.x - junction.x) - start;
            sweep = Math.atan2(Math.sin(sweep), Math.cos(sweep));
            const r1 = Math.hypot(a.x - junction.x, a.y - junction.y), r2 = Math.hypot(b.x - junction.x, b.y - junction.y);
            const path = measure(Array.from({ length: 25 }, (_, j) => {
                const t = j / 24, angle = start + sweep * t, r = r1 + (r2 - r1) * t;
                return { x: junction.x + Math.cos(angle) * r, y: junction.y + Math.sin(angle) * r, angle: angle + Math.sign(sweep) * Math.PI / 2 };
            }));
            join(a, b, 'corner', { path });
        }
    }
    return town.walkingNetwork = { nodes, byLane };
}
export function planWalk(town, person, targetLane = null) {
    const graph = walkingNetwork(town), starts = graph.byLane.get(person.lane);
    if (!starts) return false;
    const distances = new Map(), previous = new Map(), pending = new Set();
    // Only the neighbouring waypoints may be reached without passing a turn.
    const before = [...starts].reverse().find(n => n.distance <= person.distance), after = starts.find(n => n.distance >= person.distance);
    for (const n of new Set([before, after].filter(Boolean))) {
        distances.set(n, Math.abs(n.distance - person.distance)); pending.add(n);
    }
    while (pending.size) {
        let current;
        for (const n of pending) if (!current || distances.get(n) < distances.get(current)) current = n;
        pending.delete(current);
        for (const link of current.links) {
            const d = distances.get(current) + link.cost;
            if (d >= (distances.get(link.to) ?? Infinity)) continue;
            distances.set(link.to, d); previous.set(link.to, { from: current, link }); pending.add(link.to);
        }
    }
    let destination;
    if (targetLane) destination = graph.byLane.get(targetLane)?.find(n => n.distance === targetLane.stop?.distance && distances.has(n));
    else {
        const reachable = [...distances.keys()].filter(n => n.lane !== person.lane && n.distance > 0 && n.distance < n.lane.length);
        const distant = reachable.filter(n => Math.hypot(n.x - pavementPoint(person.lane, person.distance).x, n.y - pavementPoint(person.lane, person.distance).y) > 350);
        const choices = distant.length ? distant : reachable;
        destination = choices[Math.floor(town.random() * choices.length)];
    }
    if (!destination) return false;
    const steps = [];
    let n = destination;
    while (previous.has(n)) {
        const { from, link } = previous.get(n);
        steps.unshift({ ...link, from }); n = from;
    }
    steps.unshift({ type: 'walk', to: n });
    person.walkRoute = steps; person.walkIndex = 0; person.walkDestination = destination;
    person.walkPose = null; person.cornerDistance = 0;
    return true;
}
export function followWalk(town, person, dt) {
    const step = person.walkRoute?.[person.walkIndex];
    if (!step) return true;
    if (step.type === 'walk') {
        const delta = step.to.distance - person.distance;
        person.direction = Math.sign(delta) || person.direction || 1;
        person.distance += Math.sign(delta) * Math.min(Math.abs(delta), person.speed * dt);
        if (Math.abs(delta) <= person.speed * dt) person.walkIndex++;
    } else if (step.type === 'corner') {
        person.cornerDistance += person.speed * dt;
        const d = Math.min(step.path.length, person.cornerDistance);
        person.walkPose = pathPoint(step.path, step.backwards ? step.path.length - d : d);
        if (step.backwards) person.walkPose.angle += Math.PI;
        if (d >= step.path.length) {
            person.lane = step.to.lane; person.distance = step.to.distance;
            person.walkPose = null; person.cornerDistance = 0; person.walkIndex++;
        }
    } else {
        const state = person.state;
        if (requestCrossing(town, person, step.crossing || null)) person.resumeWalkState = state;
    }
    return false;
}
