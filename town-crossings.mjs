import { pathPoint } from './street-geometry.mjs';

export const CROSSING_HALF = 9;
function project(path, point) {
    let best = { distance: 0, error: Infinity };
    for (let i = 1; i < path.points.length; i++) {
        const a = path.points[i - 1], b = path.points[i];
        const dx = b.x - a.x, dy = b.y - a.y;
        const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / (dx * dx + dy * dy || 1)));
        const error = Math.hypot(point.x - a.x - t * dx, point.y - a.y - t * dy);
        if (error < best.error) best = { error, distance: a.distance + t * (b.distance - a.distance) };
    }
    return best.distance;
}
function makeCrossing(edge, rawDistance, zebra) {
    const point = pathPoint(edge.path, rawDistance);
    return { edge, rawDistance, point, zebra, users: new Set(), exempt: new Set(),
        distances: new Map(edge.lanes.map(lane => [lane, project(lane.path, point)])) };
}
export function prepareCrossings(town) {
    town.crossings = []; town.activeCrossings = new Set();
    for (const edge of town.edges) {
        edge.crossings = [];
        if (edge.singleTrack || edge.lanes.some(l => l.length < 190)) continue;
        // A stable subset of streets, without consuming the traffic random stream.
        if ((Math.imul(edge.id + 1, 37) + town.seed) % 4 !== 0) continue;
        for (const fraction of [0.55, 0.32, 0.72]) {
            const c = makeCrossing(edge, edge.a.radius + (edge.path.length - edge.a.radius - edge.b.radius) * fraction, true);
            if (edge.lanes.some(l => c.distances.get(l) < 65 || c.distances.get(l) > l.length - 65 ||
                l.stop && Math.abs(c.distances.get(l) - l.stop.distance) < 65)) continue;
            edge.crossings.push(c); town.crossings.push(c); break;
        }
    }
}
function stopLine(crossing, vehicle) {
    return crossing.distances.get(vehicle.lane) - CROSSING_HALF - vehicle.length / 2 - 5;
}
export function requestCrossing(town, person, crossing = null) {
    const lane = person.lane, edge = lane.edge;
    if (edge.singleTrack || edge.passing || person.distance < 65 || person.distance > lane.length - 65) return false;
    if (!crossing) {
        if (edge.crossings.some(c => c.users.size || Math.abs(c.distances.get(lane) - person.distance) < 65) ||
            edge.lanes.some(l => l.stop && Math.abs(l.stop.distance - (l === lane ? person.distance : l.length - person.distance)) < 65)) return false;
        crossing = makeCrossing(edge, project(edge.path, pathPoint(lane.path, person.distance)), false);
        edge.crossings.push(crossing);
    }
    if (!crossing.users.size) {
        crossing.exempt.clear();
        for (const v of town.vehicles) if (v.phase === 'lane' && v.lane.edge === edge) {
            const available = stopLine(crossing, v) - v.distance;
            // Let a vehicle already too close clear before anyone steps out.
            if (available < v.speed * v.speed / 70 + v.speed * 0.2 + 3) crossing.exempt.add(v);
        }
    }
    crossing.users.add(person); town.activeCrossings.add(crossing);
    person.crossing = crossing; person.crossProgress = 0; person.state = 'crossing_wait';
    person.distance = crossing.distances.get(lane);
    person.pause = 0;
    return true;
}
export function crossingTrafficLimit(vehicle) {
    let limit = Infinity;
    for (const c of vehicle.lane.edge.crossings || []) {
        if (!c.users.size || c.exempt.has(vehicle)) continue;
        const d = c.distances.get(vehicle.lane);
        if (vehicle.distance - vehicle.length / 2 > d + CROSSING_HALF + 5) continue;
        limit = Math.min(limit, stopLine(c, vehicle));
    }
    return limit;
}
export function crossingPersonPoint(person) {
    const c = person.crossing, p = c.point;
    const side = person.lane.reverse ? -1 : 1;
    const offset = (c.edge.width / 2 + 3) * side * (1 - 2 * person.crossProgress);
    return { x: p.x + Math.sin(p.angle) * offset, y: p.y - Math.cos(p.angle) * offset,
        angle: p.angle + side * Math.PI / 2 };
}
function canEnter(town, crossing) {
    for (const v of town.vehicles) {
        if (v.phase !== 'lane' || v.lane.edge !== crossing.edge) continue;
        const distance = crossing.distances.get(v.lane) - v.distance;
        if (distance < -v.length / 2 - CROSSING_HALF - 6) continue;
        if (crossing.exempt.has(v) || Math.abs(distance) < v.length / 2 + CROSSING_HALF + 4) return false;
        if (distance < 100 && v.speed > 0.5) return false;
    }
    return !crossing.edge.passing;
}
export function clearUnusedCrossings(town) {
    const alive = new Set(town.people);
    for (const c of town.activeCrossings || []) {
        for (const person of c.users) if (!alive.has(person)) c.users.delete(person);
        if (c.users.size) continue;
        c.exempt.clear(); town.activeCrossings.delete(c);
        if (!c.zebra) c.edge.crossings = c.edge.crossings.filter(other => other !== c);
    }
}
export function updateCrossings(town, dt) {
    for (const c of town.activeCrossings) {
        const ready = canEnter(town, c);
        for (const p of [...c.users]) {
            if (p.state === 'crossing_wait' && ready) p.state = 'crossing';
            if (p.state !== 'crossing') continue;
            p.crossProgress = Math.min(1, p.crossProgress + p.speed * dt / (c.edge.width + 6));
            if (p.crossProgress < 1) continue;
            p.lane = c.edge.lanes.find(l => l !== p.lane);
            p.distance = c.distances.get(p.lane); p.direction *= -1;
            p.state = p.resumeWalkState || 'strolling'; p.resumeWalkState = null;
            if (p.walkRoute) p.walkIndex++;
            p.crossing = null; p.crossCooldown = 18 + town.random() * 25;
            c.users.delete(p);
            town[c.zebra ? 'zebraTrips' : 'jaywalkTrips'] = (town[c.zebra ? 'zebraTrips' : 'jaywalkTrips'] || 0) + 1;
        }
    }
    clearUnusedCrossings(town);
}
