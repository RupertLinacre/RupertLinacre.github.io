export const ROAD_TYPES = {
    arterial: { name: 'Arterial', width: 54, offset: 13, speed: 52, colour: '#5d6c72', mapColour: '#d0a34e' },
    collector: { name: 'Local main road', width: 44, offset: 11, speed: 38, colour: '#68787a', mapColour: '#8eb4ad' },
    residential: { name: 'Residential', width: 38, offset: 9.5, speed: 27, colour: '#82908c', mapColour: '#d7decb' },
    single: { name: 'Single track', width: 38, offset: 9.5, speed: 17, colour: '#93988a', mapColour: '#c58e72' },
};

// One travel-time planner serves cars, scheduled bus circuits, and the map UI.
export function findRoute(from, to, { distanceOnly = false, live = true, bus = false } = {}) {
    const pending = new Set([from]);
    const cost = new Map([[from, 0]]);
    const previous = new Map([[from, null]]);
    while (pending.size) {
        let node;
        for (const candidate of pending) if (!node || cost.get(candidate) < cost.get(node)) node = candidate;
        pending.delete(node);
        if (node === to) break;
        for (const lane of node.outgoing) {
            const edge = lane.edge;
            const road = ROAD_TYPES[edge.roadType] || ROAD_TYPES.residential;
            let travel = distanceOnly ? edge.path.length : edge.path.length / Math.min(road.speed, bus ? 38 : 52);
            if (!distanceOnly) {
                travel += lane.to.control === 'signals' ? 4.5 : lane.to.control === 'roundabout' ? 1.2 : lane.hasPriority ? 0.3 : 2.5;
                if (edge.singleTrack) travel += bus ? 24 : 9;
                if (bus && edge.roadType === 'residential') travel *= 1.35;
                if (live) travel += (lane.queueLength || 0) * 2.2 + (lane.density || 0) ** 2 * travel;
            }
            const nextCost = cost.get(node) + travel;
            if (!cost.has(lane.to) || nextCost < cost.get(lane.to)) {
                cost.set(lane.to, nextCost);
                previous.set(lane.to, lane);
                pending.add(lane.to);
            }
        }
    }
    if (!previous.has(to)) return { path: [], seconds: Infinity };
    const path = [];
    for (let node = to; node !== from;) {
        const lane = previous.get(node);
        path.unshift(lane);
        node = lane.from;
    }
    return { path, seconds: cost.get(to) };
}

export function planRoadNetwork(nodes, edges, width, height, random) {
    const nearest = (x, y) => nodes.reduce((best, node) =>
        Math.hypot(node.x - x, node.y - y) < Math.hypot(best.x - x, best.y - y) ? node : best);
    const center = nearest(width / 2, height / 2);
    const hubs = Array.from({ length: 8 }, (_, i) => nearest(
        width / 2 + Math.cos(i * Math.PI / 4) * width * 0.43,
        height / 2 + Math.sin(i * Math.PI / 4) * height * 0.43));
    for (const edge of edges) { edge.demand = 0; edge.roadType = 'residential'; }
    // The arterial backbone is connected, rather than a scattering of randomly
    // coloured streets. Sampled cross-town journeys identify the busy corridors.
    for (const hub of hubs) for (const lane of findRoute(center, hub, { distanceOnly: true }).path) lane.edge.roadType = 'arterial';
    for (let i = 0; i < hubs.length; i++) {
        for (let j = i + 1; j < hubs.length; j++) {
            for (const lane of findRoute(hubs[i], hubs[j], { distanceOnly: true }).path) lane.edge.demand++;
        }
    }
    for (const edge of edges) if (edge.roadType !== 'arterial' && edge.demand >= 2) edge.roadType = 'collector';
    const narrowCandidates = edges.filter(edge => edge.roadType === 'residential' &&
        edge.path.length > 390 && edge.path.length < 800 &&
        edge.a.x > 0 && edge.a.x < width && edge.a.y > 0 && edge.a.y < height);
    narrowCandidates.sort((a, b) => a.demand - b.demand || a.path.length - b.path.length);
    const count = Math.max(1, Math.round(edges.length * 0.035));
    const narrowNodes = new Set();
    for (const edge of narrowCandidates) {
        if (edges.filter(e => e.singleTrack).length >= count) break;
        if (narrowNodes.has(edge.a) || narrowNodes.has(edge.b)) continue;
        edge.roadType = 'single';
        edge.singleTrack = true;
        edge.narrowVehicles = new Set();
        edge.narrowDirection = null;
        edge.lastDirection = null;
        edge.batch = 0;
        narrowNodes.add(edge.a);
        narrowNodes.add(edge.b);
    }
    for (const edge of edges) Object.assign(edge, ROAD_TYPES[edge.roadType]);
    for (const node of nodes) {
        node.occupants = new Set();
        node.control = node.outgoing.length < 3 ? 'bend' : 'giveway';
        node.priorityEdges = new Set();
        let bestPair = [];
        let bestScore = -Infinity;
        for (const a of node.outgoing) for (const b of node.outgoing) {
            if (a === b) continue;
            const rank = lane => lane.edge.roadType === 'arterial' ? 5 : lane.edge.roadType === 'collector' ? 2 : 0;
            const score = rank(a) + rank(b) - 4 * Math.cos(a.heading - b.heading) + (a.edge.demand + b.edge.demand) * 0.05;
            if (score > bestScore) { bestScore = score; bestPair = [a, b]; }
        }
        bestPair.forEach(lane => node.priorityEdges.add(lane.edge.id));
        const angles = node.outgoing.map(l => l.heading).sort((a, b) => a - b);
        const smallest = Math.min(...angles.map((a, i) => (angles[(i + 1) % angles.length] - a + Math.PI * 4) % (Math.PI * 2)));
        const kerb = Math.max(...node.outgoing.map(l => l.edge.width)) / 2 + 10;
        node.radius = Math.max(36, kerb / Math.tan(smallest / 2));
    }
    const junctions = nodes.filter(n => n.outgoing.length >= 3 && n.x > 80 && n.x < width - 80 && n.y > 80 && n.y < height - 80);
    const busy = n => n.outgoing.reduce((sum, lane) => sum + lane.edge.demand + (lane.edge.roadType === 'arterial' ? 8 : 0), 0);
    const selected = [];
    for (const node of [...junctions].sort((a, b) => busy(b) - busy(a))) {
        if (selected.length >= Math.max(1, Math.round(junctions.length * 0.075))) break;
        if (selected.some(other => Math.hypot(node.x - other.x, node.y - other.y) < 340) ||
            node.outgoing.some(l => l.edge.singleTrack || l.edge.path.length - 80 - l.to.radius < 65)) continue;
        node.control = 'roundabout';
        node.radius = Math.max(node.radius, 80);
        node.orbitRadius = 41;
        selected.push(node);
    }
    let minis = 0;
    for (const node of junctions) {
        if (node.control === 'roundabout') continue;
        if ((random() < 0.19 || minis === 0) && !node.outgoing.some(l => l.edge.singleTrack) &&
            node.outgoing.every(l => l.edge.path.length - Math.max(node.radius, 55) - l.to.radius > 60)) {
            node.control = 'mini';
            node.radius = Math.max(node.radius, 55);
            node.orbitRadius = 23;
            minis++;
        }
    }
    for (const node of nodes) {
        const mainRoads = node.outgoing.filter(l => ['arterial', 'collector'].includes(l.edge.roadType)).length;
        if (node.control === 'giveway' && mainRoads >= 2 && random() < 0.43) node.control = 'signals';
        node.signal = node.control === 'signals';
        for (const lane of node.outgoing) lane.hasPriority = lane.to.priorityEdges.has(lane.edge.id);
    }
    // Keep at least one signal-controlled junction when the town has room for it.
    if (!nodes.some(n => n.signal)) {
        const node = [...junctions].filter(n => n.control === 'giveway').sort((a, b) => busy(b) - busy(a))[0];
        if (node) { node.control = 'signals'; node.signal = true; }
    }
    for (const edge of edges.filter(e => e.singleTrack)) {
        edge.narrowStart = Math.max(edge.a.radius + 90, edge.path.length * 0.23);
        edge.narrowEnd = Math.min(edge.path.length - edge.b.radius - 90, edge.path.length * 0.77);
    }
    return { hubs: [...new Set([center, ...hubs])] };
}
