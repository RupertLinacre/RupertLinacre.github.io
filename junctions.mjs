import { measure, pathPoint } from './street-geometry.mjs';

const TAU = Math.PI * 2;
const clockwise = angle => (angle % TAU + TAU) % TAU;
export const isRoundabout = node => node.control === 'roundabout' || node.control === 'mini';

function bezier(a, b, reach = Math.hypot(b.x - a.x, b.y - a.y) * 0.55) {
    const c = { x: a.x + Math.cos(a.angle) * reach, y: a.y + Math.sin(a.angle) * reach };
    const d = { x: b.x - Math.cos(b.angle) * reach, y: b.y - Math.sin(b.angle) * reach };
    return Array.from({ length: 33 }, (_, i) => {
        const t = i / 32, u = 1 - t;
        return { x: u ** 3 * a.x + 3 * u * u * t * c.x + 3 * u * t * t * d.x + t ** 3 * b.x,
            y: u ** 3 * a.y + 3 * u * u * t * c.y + 3 * u * t * t * d.y + t ** 3 * b.y,
            angle: Math.atan2(3 * u * u * (c.y - a.y) + 6 * u * t * (d.y - c.y) + 3 * t * t * (b.y - d.y),
                3 * u * u * (c.x - a.x) + 6 * u * t * (d.x - c.x) + 3 * t * t * (b.x - d.x)) };
    });
}

export function makeTurn(incoming, outgoing) {
    const node = incoming.to;
    if (!isRoundabout(node)) return measure(bezier(incoming.end, outgoing.start));
    const radius = node.orbitRadius;
    const entryAngle = Math.atan2(incoming.end.y - node.y, incoming.end.x - node.x) + 0.28;
    const exitAngle = Math.atan2(outgoing.start.y - node.y, outgoing.start.x - node.x) - 0.28;
    const sweep = clockwise(exitAngle - entryAngle);
    const at = theta => ({ x: node.x + Math.cos(theta) * radius, y: node.y + Math.sin(theta) * radius, angle: theta + Math.PI / 2 });
    const approach = measure(bezier(incoming.end, at(entryAngle)));
    const count = Math.max(12, Math.ceil(sweep * radius / 2));
    const ring = measure(Array.from({ length: count + 1 }, (_, i) => at(entryAngle + sweep * i / count)));
    const exit = bezier(at(entryAngle + sweep), outgoing.start);
    return { ...measure([...approach.points, ...ring.points.slice(1), ...exit.slice(1)]),
        entryLength: approach.length, ringLength: ring.length, exitStart: approach.length + ring.length,
        entryAngle, exitAngle: entryAngle + sweep, radius };
}

export function roundaboutHasGap(node, vehicle) {
    if (vehicle.maxSpeed === 0) return false;
    const turn = makeTurn(vehicle.lane, vehicle.next);
    // A mini has room for one manoeuvre at a time. Full roundabouts can carry
    // several cars, but a gap must remain safe throughout the entry manoeuvre.
    if (node.control === 'mini') return node.occupants.size === 0;
    function projected(v, path, time) {
        const maximum = Math.min(20, v.maxSpeed ?? 20);
        const speed = Math.min(maximum, v.speed);
        const accelerating = Math.min(time, (maximum - speed) / 22);
        const movement = speed * accelerating + 11 * accelerating ** 2 + maximum * (time - accelerating);
        let distance = v.distance + movement;
        if (v.phase === 'lane') {
            if (distance <= v.lane.length) return pathPoint(v.lane.path, distance);
            distance -= v.lane.length;
        }
        return distance <= path.length ? pathPoint(path, distance) : pathPoint(v.next.path, distance - path.length);
    }
    for (const other of node.occupants) {
        if (other === vehicle || other.phase === 'lane' && other.lane.from === node) continue;
        const path = other.turn || makeTurn(other.lane, other.next);
        if (other.phase === 'turn' && other.distance > path.exitStart + other.length / 2 + 8) continue;
        const progress = other.phase === 'turn' ? Math.max(0, Math.min(path.ringLength, other.distance - path.entryLength)) : 0;
        const angle = path.entryAngle + progress / path.radius;
        const ahead = clockwise(angle - turn.entryAngle) * turn.radius;
        const behind = clockwise(turn.entryAngle - angle) * turn.radius;
        const gap = (vehicle.length + other.length) / 2 + 12;
        if (ahead < gap || behind < gap + 28) return false;
        if (other.lane === vehicle.lane && other.phase === 'lane') return false;
        const duration = (vehicle.lane.length - vehicle.distance + turn.length) / Math.min(20, vehicle.maxSpeed ?? 20) + 1;
        for (let t = 0; t <= duration; t += 0.2) {
            const a = projected(vehicle, turn, t), b = projected(other, path, t);
            if (Math.hypot(a.x - b.x, a.y - b.y) < (vehicle.length + other.length) / 2 + 5) return false;
        }
    }
    return true;
}

export function priorityHasGap(vehicle, occupied, hasRoom) {
    const lane = vehicle.lane;
    const node = lane.to;
    const rightTurn = Math.sin(vehicle.next.start.angle - lane.end.angle) > 0.35 ||
        Math.cos(vehicle.next.start.angle - lane.end.angle) < -0.7;
    if (lane.hasPriority && !rightTurn) return true;
    for (const outgoing of node.outgoing) {
        const incoming = outgoing.edge.lanes.find(l => l.to === node);
        if (incoming === lane || !incoming.hasPriority) continue;
        const other = occupied.get(incoming)[0];
        if (!other || other.reserved && other.reserved !== node) continue;
        if (lane.hasPriority) {
            if (Math.cos(incoming.end.angle - lane.end.angle) > -0.7) continue;
            const otherTurn = other.next.start.angle - incoming.end.angle;
            if (Math.sin(otherTurn) > 0.35 || Math.cos(otherTurn) < -0.7) continue;
        }
        const approaching = incoming.length - other.distance - other.length / 2;
        if (approaching < Math.max(45, other.speed * 3.2) && (other.speed > 3 || hasRoom(other, other.next))) return false;
    }
    return true;
}

export function releaseJunction(vehicle) {
    const node = vehicle.reserved;
    if (!node) return;
    node.occupants.delete(vehicle);
    if (node.owner === vehicle) node.owner = null;
    vehicle.reserved = null;
}
