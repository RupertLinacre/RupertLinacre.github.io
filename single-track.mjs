import { measure, pathPoint } from './street-geometry.mjs';

export function singleTrackPath(lane, center) {
    const edge = lane.edge;
    const count = Math.ceil((center.length - lane.from.radius - lane.to.radius) / 3);
    const points = Array.from({ length: count + 1 }, (_, i) => {
        const distance = lane.from.radius + (center.length - lane.from.radius - lane.to.radius) * i / count;
        const raw = lane.reverse ? center.length - distance : distance;
        // The two end bays retain normal left-hand lanes; the centre is shared.
        const outside = raw < edge.narrowStart ? Math.min(1, (edge.narrowStart - raw) / 24) :
            raw > edge.narrowEnd ? Math.min(1, (raw - edge.narrowEnd) / 24) : 0;
        const offset = edge.offset * (1 - Math.cos(Math.PI * outside)) / 2;
        const p = pathPoint(center, distance);
        return { x: p.x + Math.sin(p.angle) * offset, y: p.y - Math.cos(p.angle) * offset, raw };
    });
    // Heading follows the tapered lane itself, rather than the road centreline.
    points.forEach((p, i) => {
        const a = points[Math.max(0, i - 1)], b = points[Math.min(points.length - 1, i + 1)];
        p.angle = Math.atan2(b.y - a.y, b.x - a.x);
    });
    const path = measure(points);
    const inside = p => p.raw >= edge.narrowStart - 24 && p.raw <= edge.narrowEnd + 24;
    const shared = points.filter(inside);
    lane.singleEntry = shared[0].distance;
    lane.singleExit = shared.at(-1).distance;
    return path;
}

export function releaseSingleTrack(vehicle) {
    const edge = vehicle.narrowPermit;
    if (!edge) return;
    edge.narrowVehicles.delete(vehicle);
    vehicle.narrowPermit = null;
    if (!edge.narrowVehicles.size) {
        edge.lastDirection = edge.narrowDirection;
        edge.narrowDirection = null;
        edge.batch = 0;
    }
}

export function updateSingleTracks(town, occupied) {
    for (const edge of town.edges) {
        if (!edge.singleTrack) continue;
        for (const vehicle of [...edge.narrowVehicles]) {
            if (vehicle.phase !== 'lane' || vehicle.lane.edge !== edge ||
                vehicle.distance > vehicle.lane.singleExit + vehicle.length / 2 + 4) releaseSingleTrack(vehicle);
        }
        const waiting = edge.lanes.map(lane => {
            const vehicle = occupied.get(lane).find(v => !v.narrowPermit && v.distance < lane.singleExit);
            if (!vehicle || vehicle.distance < lane.singleEntry - vehicle.length / 2 - 7) return null;
            vehicle.narrowWaitingSince ??= town.time;
            return vehicle;
        }).filter(Boolean);
        if (!waiting.length) continue;
        if (edge.narrowDirection === null) {
            waiting.sort((a, b) => (a.lane.reverse === edge.lastDirection) - (b.lane.reverse === edge.lastDirection) ||
                a.narrowWaitingSince - b.narrowWaitingSince);
            edge.narrowDirection = waiting[0].lane.reverse;
        }
        const opposing = waiting.find(v => v.lane.reverse !== edge.narrowDirection);
        for (const vehicle of waiting) {
            if (vehicle.lane.reverse !== edge.narrowDirection || opposing && edge.narrowVehicles.size &&
                (edge.batch >= 3 || town.time - opposing.narrowWaitingSince > 8)) continue;
            edge.narrowVehicles.add(vehicle);
            vehicle.narrowPermit = edge;
            vehicle.narrowWaitingSince = null;
            edge.batch++;
        }
    }
}
