import { measure, pathPoint, slicePath, reversePath, circularArc, polygonArea, insidePolygon, boundaryDistance } from './street-geometry.mjs';

const MIN_ROAD = 195;
const MIN_ANGLE = 0.93;
const MIN_BLOCK = 32000;

// Streets subdivide real curved neighbourhoods. Splitting a parent street keeps
// its exact path and tangent, so a boulevard continues smoothly past side roads.
export function createStreetLayout(width, height, random) {
    const nodes = [];
    const roads = [];
    const faces = [];
    let nextRoadId = 0;
    const nodeAt = point => {
        const node = { id: nodes.length, x: point.x, y: point.y };
        nodes.push(node);
        return node;
    };
    function roadBetween(a, b, path) {
        const road = { id: nextRoadId++, a, b, path };
        roads.push(road);
        return road;
    }
    const startNode = half => half.forward ? half.road.a : half.road.b;
    const halfPath = half => half.forward ? half.road.path : reversePath(half.road.path);
    function refresh(face) {
        const points = face.boundary.flatMap(half => halfPath(half).points.slice(0, -1));
        face.path = measure([...points.map(p => ({ x: p.x, y: p.y, angle: p.angle })), { ...points[0] }]);
        face.area = polygonArea(points);
    }
    const cx = width / 2;
    const cy = height / 2;
    const rx = Math.max(width * 0.82, width / 2 + 240);
    const ry = Math.max(height * 0.82, height / 2 + 240);
    const phase = random() * Math.PI / 2;
    const ellipse = angle => ({ x: cx + Math.cos(angle) * rx, y: cy + Math.sin(angle) * ry,
        angle: Math.atan2(Math.cos(angle) * ry, -Math.sin(angle) * rx) });
    const corners = Array.from({ length: 4 }, (_, i) => nodeAt(ellipse(phase + i * Math.PI / 2)));
    const boundary = corners.map((a, i) => {
        const count = Math.ceil(Math.max(rx, ry) * Math.PI / 2 / 5);
        const points = Array.from({ length: count + 1 }, (_, j) => ellipse(phase + (i + j / count) * Math.PI / 2));
        const road = roadBetween(a, corners[(i + 1) % 4], { ...measure(points), kind: 'ellipse' });
        return { road, forward: true };
    });
    const first = { boundary, finished: false };
    refresh(first);
    faces.push(first);

    function incident(node) {
        return roads.filter(road => road.a === node || road.b === node);
    }
    function directionAt(road, node) {
        return road.a === node ? road.path.points[0].angle : road.path.points.at(-1).angle + Math.PI;
    }
    function location(face) {
        const eligible = face.boundary.filter(half => half.road.path.length > MIN_ROAD * 2 + 20);
        let candidate;
        if (eligible.length && random() > 0.2) {
            const total = eligible.reduce((sum, half) => sum + half.road.path.length - MIN_ROAD * 2, 0);
            let choice = random() * total;
            const half = eligible.find(h => (choice -= h.road.path.length - MIN_ROAD * 2) <= 0) || eligible.at(-1);
            const distance = MIN_ROAD + random() * (half.road.path.length - MIN_ROAD * 2);
            candidate = { road: half.road, distance, point: pathPoint(half.road.path, distance) };
        } else {
            const possible = face.boundary.map(startNode).filter(node => incident(node).length < 4);
            if (!possible.length) return null;
            const node = possible[Math.floor(random() * possible.length)];
            candidate = { node, point: node };
        }
        let position = 0;
        for (const half of face.boundary) {
            if (candidate.node === startNode(half)) break;
            if (candidate.road === half.road) {
                position += half.forward ? candidate.distance : half.road.path.length - candidate.distance;
                break;
            }
            position += half.road.path.length;
        }
        candidate.position = position;
        return candidate;
    }
    function validAngle(candidate, heading) {
        const directions = candidate.node ? incident(candidate.node).map(road => directionAt(road, candidate.node)) :
            [candidate.point.angle, candidate.point.angle + Math.PI];
        return directions.every(angle => Math.acos(Math.max(-1, Math.min(1, Math.cos(angle - heading)))) >= MIN_ANGLE);
    }
    function walk(face, from, to) {
        if (from < to) return slicePath(face.path, from, to).points;
        return [...slicePath(face.path, from, face.path.length).points, ...slicePath(face.path, 0, to).points.slice(1)];
    }
    function splitRoad(candidate, other) {
        if (candidate.node) return candidate.node;
        const { road, distance, point } = candidate;
        const node = nodeAt(point);
        const left = roadBetween(road.a, node, slicePath(road.path, 0, distance));
        const right = roadBetween(node, road.b, slicePath(road.path, distance, road.path.length));
        roads.splice(roads.indexOf(road), 1);
        for (const face of faces) {
            face.boundary = face.boundary.flatMap(half => half.road !== road ? [half] : half.forward ?
                [{ road: left, forward: true }, { road: right, forward: true }] :
                [{ road: right, forward: false }, { road: left, forward: false }]);
        }
        if (other?.road === road) {
            other.road = other.distance < distance ? left : right;
            if (other.distance > distance) other.distance -= distance;
        }
        return node;
    }
    function divide(face) {
        for (let attempt = 0; attempt < 85; attempt++) {
            const a = location(face);
            const b = location(face);
            if (!a || !b || a.node && a.node === b.node) continue;
            const chord = Math.hypot(b.point.x - a.point.x, b.point.y - a.point.y);
            if (chord < MIN_ROAD) continue;
            if (a.road && a.road === b.road && Math.abs(a.distance - b.distance) < MIN_ROAD) continue;
            const sameStreet = a.road && a.road === b.road;
            const bend = chord * (sameStreet ? 0.27 + random() * 0.17 : 0.055 + random() * 0.18);
            const arc = circularArc(a.point, b.point, bend * (random() < 0.5 ? -1 : 1));
            if (arc.radius < 135 || !validAngle(a, arc.points[0].angle) ||
                !validAngle(b, arc.points.at(-1).angle + Math.PI)) continue;
            // The complete curve must fit its neighbourhood, with enough room
            // between streets for pavements and buildings. No accidental crossings.
            let fits = true;
            for (let i = 3; i < arc.points.length - 3; i += 3) {
                const p = arc.points[i];
                if (!insidePolygon(p, face.path.points) || (p.distance > 100 && p.distance < arc.length - 100 &&
                    boundaryDistance(p, face.path.points) < 57)) { fits = false; break; }
            }
            if (!fits) continue;
            const area = polygonArea([...walk(face, a.position, b.position), ...[...arc.points].reverse()]);
            if (area < MIN_BLOCK || face.area - area < MIN_BLOCK) continue;
            const na = splitRoad(a, b);
            const nb = splitRoad(b);
            const road = roadBetween(na, nb, arc);
            const start = face.boundary.findIndex(half => startNode(half) === na);
            const rotated = [...face.boundary.slice(start), ...face.boundary.slice(0, start)];
            const end = rotated.findIndex(half => startNode(half) === nb);
            const one = { boundary: [...rotated.slice(0, end), { road, forward: false }], finished: false };
            const two = { boundary: [...rotated.slice(end), { road, forward: true }], finished: false };
            refresh(one);
            refresh(two);
            faces.splice(faces.indexOf(face), 1, one, two);
            return true;
        }
        return false;
    }
    const limit = Math.min(170, Math.max(12, Math.round(width * height / 42000)));
    while (faces.length < limit) {
        const face = faces.filter(f => !f.finished && f.area > 95000).sort((a, b) => b.area - a.area)[0];
        if (!face) break;
        if (!divide(face)) face.finished = true;
    }
    faces.forEach(refresh);
    return { nodes, roads, faces };
}
