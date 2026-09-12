import { createTown, updateTown, setTrafficLevel, vehiclePoint, lanePoint, signalState, randomSource, offsetPath, ROAD } from './road-world.mjs?v=3';

const canvas = document.getElementById('road-canvas');
const ctx = canvas?.getContext('2d');

if (ctx) {
    const scenery = document.createElement('canvas');
    const background = scenery.getContext('2d');
    const pauseButton = document.getElementById('pause-town');
    const exploreButton = document.getElementById('explore-town');
    const newButton = document.getElementById('new-town');
    const status = document.getElementById('town-status');
    const content = document.getElementById('page-content');
    const speedControl = document.getElementById('simulation-speed');
    const trafficControl = document.getElementById('traffic-level');
    const zoomControl = document.getElementById('town-zoom');
    const motionPreference = window.matchMedia('(prefers-reduced-motion: reduce)');
    let paused = motionPreference.matches;
    let exploring = false;
    let town;
    let scale = 1;
    let cameraX = 0;
    let cameraY = 0;
    let simulationSpeed = 1;
    let trafficLevel = 1;
    let dpr = 1;
    let frame = null;
    let lastTime = 0;
    let accumulator = 0;
    let savedScroll = 0;
    let seed = newSeed();
    let previousSize = '';
    let resizeTimer;
    const FIXED_STEP = 1 / 60;

    function newSeed() {
        return crypto.getRandomValues(new Uint32Array(1))[0];
    }

    function rounded(g, x, y, width, height, radius, fill, stroke) {
        g.beginPath();
        g.roundRect(x, y, width, height, radius);
        if (fill) { g.fillStyle = fill; g.fill(); }
        if (stroke) { g.strokeStyle = stroke; g.lineWidth = 1; g.stroke(); }
    }

    function line(g, x1, y1, x2, y2, colour, width = 1) {
        g.strokeStyle = colour;
        g.lineWidth = width;
        g.beginPath();
        g.moveTo(x1, y1);
        g.lineTo(x2, y2);
        g.stroke();
    }

    function circle(g, x, y, radius, colour) {
        g.fillStyle = colour;
        g.beginPath();
        g.arc(x, y, radius, 0, Math.PI * 2);
        g.fill();
    }

    function trace(g, points) {
        g.moveTo(points[0].x, points[0].y);
        for (let i = 1; i < points.length; i++) g.lineTo(points[i].x, points[i].y);
    }

    function strokePath(g, path, colour, width) {
        g.beginPath();
        trace(g, path.points);
        g.strokeStyle = colour;
        g.lineWidth = width;
        g.stroke();
    }

    function worldTransform(g) {
        g.setTransform(dpr * scale, 0, 0, dpr * scale, -cameraX * dpr * scale, -cameraY * dpr * scale);
    }

    function visible(p, margin = 50) {
        return p.x > cameraX - margin && p.x < cameraX + window.innerWidth / scale + margin &&
            p.y > cameraY - margin && p.y < cameraY + window.innerHeight / scale + margin;
    }

    function tree(g, x, y, radius, random) {
        circle(g, x + 3, y + 5, radius + 1, '#5f77562a');
        const colour = ['#729967', '#81a674', '#609077', '#99ad6d'][Math.floor(random() * 4)];
        circle(g, x, y, radius, colour);
        circle(g, x - radius * 0.3, y - radius * 0.25, radius * 0.66, '#ffffff16');
        circle(g, x + radius * 0.3, y + radius * 0.3, radius * 0.35, '#365a3820');
    }

    function building(g, x, y, width, height, random, shop = false) {
        const roofs = ['#c88870', '#b9b3a1', '#7f9b9e', '#ceac79', '#aa938f', '#9baba2'];
        const roof = roofs[Math.floor(random() * roofs.length)];
        rounded(g, x + 4, y + 6, width + 1, height, 3, '#57655327');
        rounded(g, x - 2, y - 2, width + 4, height + 5, 3, '#f4edde');
        rounded(g, x, y, width, height, 2, roof);
        g.fillStyle = '#ffffff23';
        g.fillRect(x + 2, y + 2, width - 4, height / 2 - 2);
        line(g, x + 2, y + height / 2, x + width - 2, y + height / 2, '#58615e40', 2);
        line(g, x + 1, y + 1, x + 10, y + height / 2, '#ffffff38');
        line(g, x + width - 1, y + 1, x + width - 10, y + height / 2, '#ffffff38');
        rounded(g, x + width * 0.65, y + 5, 6, 8, 1, '#677574', '#d5d4c4');
        if (shop) {
            for (let i = 0; i < Math.floor(width / 7); i++) {
                g.fillStyle = i % 2 ? '#eee9d8' : '#6b9390';
                g.fillRect(x + i * 7, y + height - 1, 7, 7);
            }
            rounded(g, x + width / 2 - 8, y + height + 8, 16, 3, 1, '#a3ac9b');
        } else if (random() > 0.5) {
            rounded(g, x + 7, y + height / 2 + 4, 14, 8, 1, '#536e79', '#b2c2c0');
            line(g, x + 14, y + height / 2 + 4, x + 14, y + height / 2 + 12, '#a6b8ba');
        }
    }

    function bench(g, x, y, vertical = false) {
        g.save();
        g.translate(x, y);
        if (vertical) g.rotate(Math.PI / 2);
        rounded(g, -8, -3, 16, 6, 1, '#b39368');
        line(g, -8, 0, 8, 0, '#ead7ad');
        line(g, -5, -4, -5, 4, '#626d5d', 1.5);
        line(g, 5, -4, 5, 4, '#626d5d', 1.5);
        g.restore();
    }

    function drawBlock(g, block) {
        const random = randomSource(block.seed);
        const x = block.x + 35;
        const y = block.y + 35;
        const w = block.width - 70;
        const h = block.height - 70;
        const polygon = block.polygon;
        function clearPlot(px, py, radius = 0) {
            let inside = false;
            let clearance = Infinity;
            for (let i = 0, j = polygon.length - 1; i < polygon.length; j = i++) {
                const a = polygon[j];
                const b = polygon[i];
                if ((a.y > py) !== (b.y > py) && px < (b.x - a.x) * (py - a.y) / (b.y - a.y) + a.x) inside = !inside;
                const dx = b.x - a.x;
                const dy = b.y - a.y;
                const t = Math.max(0, Math.min(1, ((px - a.x) * dx + (py - a.y) * dy) / (dx * dx + dy * dy || 1)));
                clearance = Math.min(clearance, Math.hypot(px - a.x - dx * t, py - a.y - dy * t));
            }
            return inside && clearance > 32 + radius;
        }
        g.save();
        g.translate(block.center.x, block.center.y);
        g.rotate(block.angle);
        g.beginPath();
        trace(g, polygon);
        g.closePath();
        g.clip();
        g.fillStyle = block.kind === 'park' ? '#bed2a6' : '#d3dfbe';
        g.fillRect(block.x, block.y, block.width, block.height);
        rounded(g, x, y, w, h, 9, block.kind === 'park' ? '#bed2a6' : '#d3dfbe');
        if (block.kind === 'park') {
            // Footpaths, a little pond, benches, and groves make each park unique.
            line(g, x + 8, y + h * 0.7, x + w - 8, y + h * 0.3, '#e8e2c8', 9);
            line(g, x + w * 0.33, y + 5, x + w * 0.68, y + h - 5, '#e8e2c8', 7);
            if (random() > 0.3 && clearPlot(x + w * 0.68, y + h * 0.64, Math.max(w * 0.2, h * 0.16))) {
                g.save();
                g.translate(x + w * 0.68, y + h * 0.64);
                g.rotate(-0.35);
                g.beginPath();
                g.ellipse(0, 0, w * 0.2, h * 0.16, 0, 0, Math.PI * 2);
                g.fillStyle = '#8abcbc';
                g.fill();
                g.strokeStyle = '#aecda3';
                g.lineWidth = 5;
                g.stroke();
                line(g, -10, -4, 6, -4, '#c9dfd3', 1.5);
                line(g, 2, 5, 15, 5, '#c9dfd3', 1.5);
                g.restore();
            }
            for (let i = 0; i < 12; i++) {
                const tx = x + 13 + random() * (w - 26);
                const ty = y + 12 + random() * h * 0.28;
                if (clearPlot(tx, ty, 12)) tree(g, tx, ty, 7 + random() * 6, random);
            }
            for (let i = 0; i < 10; i++) {
                const tx = x + random() * w;
                const ty = y + random() * h;
                if (clearPlot(tx, ty, 11)) tree(g, tx, ty, 7 + random() * 5, random);
            }
            if (clearPlot(x + w * 0.43, y + h * 0.53, 9)) bench(g, x + w * 0.43, y + h * 0.53);
            if (clearPlot(x + w * 0.2, y + h * 0.63, 9)) bench(g, x + w * 0.2, y + h * 0.63);
            g.fillStyle = '#668268';
            g.font = '600 8px system-ui, sans-serif';
            g.textAlign = 'center';
            if (clearPlot(x + w / 2, y + h - 13, 32)) g.fillText(['THE GREEN', 'WILLOW PARK', 'TOWN GARDENS', 'OAK MEADOW'][Math.floor(random() * 4)], x + w / 2, y + h - 13);
        } else {
            const columns = Math.max(2, Math.floor(w / 65));
            const rows = Math.max(2, Math.floor(h / 66));
            const cellW = w / columns;
            const cellH = h / rows;
            for (let r = 0; r < rows; r++) {
                for (let c = 0; c < columns; c++) {
                    const bx = x + c * cellW;
                    const by = y + r * cellH;
                    if (random() < 0.15) {
                        if (clearPlot(bx + cellW / 2, by + cellH / 2, 13)) tree(g, bx + cellW / 2, by + cellH / 2, 10 + random() * 4, random);
                        continue;
                    }
                    const bw = cellW * (0.58 + random() * 0.17);
                    const bh = cellH * (0.4 + random() * 0.15);
                    if (!clearPlot(bx + cellW / 2, by + 10 + bh / 2, Math.hypot(bw, bh) / 2 + 6)) continue;
                    line(g, bx + cellW / 2, by + cellH / 2, bx + cellW / 2, by + cellH - 1, '#e8e2cc', 6);
                    building(g, bx + (cellW - bw) / 2, by + 10, bw, bh, random, block.kind === 'shops');
                    if (random() > 0.3 && clearPlot(bx + 9, by + cellH - 13, 8)) tree(g, bx + 9, by + cellH - 13, 5 + random() * 3, random);
                    line(g, bx + 3, by + cellH - 2, bx + cellW - 3, by + cellH - 2, '#b3c698', 2);
                }
            }
        }
        g.restore();
    }

    function drawScenery() {
        const g = background;
        g.setTransform(1, 0, 0, 1, 0, 0);
        g.fillStyle = '#cbdab8';
        g.fillRect(0, 0, scenery.width, scenery.height);
        worldTransform(g);
        town.blocks.forEach(block => { if (visible(block.center, 400)) drawBlock(g, block); });
        g.lineCap = 'round';
        g.lineJoin = 'round';
        // Stroke the whole network once per layer so junctions join cleanly.
        for (const [width, colour] of [[ROAD + 15, '#a8b89c'], [ROAD + 12, '#e9e3d4'], [ROAD, '#637477']]) {
            g.beginPath();
            for (const edge of town.edges) {
                trace(g, edge.path.points);
            }
            g.strokeStyle = colour;
            g.lineWidth = width;
            g.stroke();
        }
        g.lineCap = 'butt';
        for (const edge of town.edges) {
            const start = edge.a.radius + 8;
            const end = edge.path.length - edge.b.radius - 8;
            g.setLineDash([9, 11]);
            strokePath(g, offsetPath(edge.path, 0, start, end), '#dde0cd', 1.5);
            g.setLineDash([]);
            // Fine kerb lines sit just inside the asphalt.
            for (const side of [-1, 1]) {
                strokePath(g, offsetPath(edge.path, 20 * side, start, end), '#b7bdaa', 0.65);
            }
        }
        for (const node of town.nodes) {
            if (node.outgoing.length < 3) continue;
            for (const outgoing of node.outgoing) {
                const incoming = outgoing.edge.lanes.find(l => l.to === node);
                g.save();
                g.translate(incoming.end.x, incoming.end.y);
                g.rotate(incoming.end.angle);
                // Crossings are inside the stop lines, clear of queued traffic.
                for (let stripe = -17; stripe <= 17; stripe += 6) {
                    g.fillStyle = '#e6e6d5';
                    g.fillRect(6, stripe + 11, 7, 3);
                }
                line(g, -3, -8, -3, 8, '#f1edda', 2);
                g.restore();
            }
        }
        for (const lane of town.lanes) {
            if (!lane.stop) continue;
            const p = lanePoint(lane, lane.stop.distance);
            g.save();
            g.translate(p.x, p.y);
            g.rotate(p.angle);
            g.strokeStyle = '#e5c26a';
            g.lineWidth = 1;
            g.setLineDash([4, 3]);
            g.strokeRect(-22, -8, 44, 16);
            g.setLineDash([]);
            // A glass shelter and roundel, on the left-hand pavement.
            rounded(g, -15, -21, 30, 8, 2, '#577f8055', '#628484');
            line(g, -12, -16, 11, -16, '#e5d8ab', 2);
            line(g, 21, -12, 21, -24, '#657974', 1.5);
            circle(g, 21, -24, 4, '#ce6254');
            line(g, 18, -24, 24, -24, '#fbecd9', 1.8);
            g.restore();
        }
        // Street trees use their own random stream so repainting is deterministic.
        const random = randomSource(town.seed ^ 0x739bc1);
        for (const edge of town.edges) {
            const lane = edge.lanes[0];
            for (let d = 52; d < lane.length - 20; d += 45 + random() * 25) {
                if (edge.lanes.some(l => l.stop && Math.abs((l === lane ? d : lane.length - d) - l.stop.distance) < 38)) continue;
                const p = lanePoint(lane, d);
                tree(g, p.x + Math.sin(p.angle) * 25, p.y - Math.cos(p.angle) * 25, 5 + random() * 3, random);
            }
        }
    }

    function drawSignal(lane) {
        const state = signalState(lane.to, lane.axis, town.time);
        const g = ctx;
        const p = lane.end;
        if (!visible(p)) return;
        g.save();
        g.translate(p.x - Math.cos(p.angle) * 7 + Math.sin(p.angle) * 18,
            p.y - Math.sin(p.angle) * 7 - Math.cos(p.angle) * 18);
        // Upright, legible signal heads, regardless of the approach direction.
        rounded(g, -5, -11, 10, 23, 3, '#35484a', '#8c9a87');
        ['red', 'amber', 'green'].forEach((colour, i) => {
            const active = state === colour;
            const fill = { red: '#f47b65', amber: '#f6ca5b', green: '#9ad896' }[colour];
            if (active) circle(g, 0, -6 + i * 7, 4.8, `${fill}25`);
            circle(g, 0, -6 + i * 7, 2.5, active ? fill : '#53605a');
        });
        g.restore();
    }

    function drawVehicle(vehicle) {
        const p = vehiclePoint(vehicle);
        if (!visible(p)) return;
        const g = ctx;
        const length = vehicle.length;
        const width = vehicle.width;
        g.save();
        g.translate(p.x, p.y);
        g.rotate(p.angle);
        rounded(g, -length / 2 + 2, -width / 2 + 3, length, width, 3, '#263e3c35');
        // Tyres remain visible either side of the body.
        for (const axle of [-length * 0.3, length * 0.29]) {
            rounded(g, axle - 2, -width / 2 - 1, 4, width + 2, 1, '#34494a');
        }
        rounded(g, -length / 2, -width / 2, length, width, vehicle.bus ? 3 : 4, vehicle.colour);
        if (vehicle.bus) {
            rounded(g, -length / 2 + 4, -width / 2 + 2, length - 10, width - 4, 2, '#f3e8c9');
            rounded(g, length / 2 - 6, -width / 2 + 1, 4, width - 2, 1, '#34575c');
            rounded(g, -length / 2 + 3, -width / 2 + 1, 3, width - 2, 1, '#567574');
            rounded(g, -7, -3, 6, 6, 1, '#ddd5bc');
            for (let x = -8; x < 9; x += 5) {
                g.fillStyle = '#3f6666';
                g.fillRect(x, -width / 2, 3.5, 1.5);
                g.fillRect(x, width / 2 - 1.5, 3.5, 1.5);
            }
            // Route numbers on the roof are easy to follow from above.
            g.fillStyle = '#485a53';
            g.font = 'bold 7px system-ui, sans-serif';
            g.textAlign = 'center';
            g.textBaseline = 'middle';
            g.fillText(vehicle.route.number, 5, 0.4);
        } else {
            rounded(g, -length * 0.19, -width / 2 + 1, length * 0.48, width - 2, 2, '#3e626d');
            rounded(g, -length * 0.12, -width / 2 + 1, length * 0.26, width - 2, 1, vehicle.colour);
            line(g, -length * 0.07, -width / 2 + 2, length * 0.11, -width / 2 + 2, '#ffffff50');
        }
        for (const side of [-1, 1]) {
            rounded(g, length / 2 - 1.5, side * (width / 2 - 2) - 1, 1.5, 2, 0.5, '#fff1c1');
            rounded(g, -length / 2, side * (width / 2 - 2) - 1, 1.5, 2, 0.5, vehicle.braking ? '#ff9a75' : '#a94b40');
        }
        if (vehicle.dwell > 0) {
            g.fillStyle = '#f6d679';
            g.fillRect(4, -width / 2 - 1, 5, 2);
        }
        // Indicate turns on the approach and while crossing a junction.
        const cross = Math.sin(vehicle.next.start.angle - vehicle.lane.end.angle);
        if (Math.abs(cross) > 0.4 && (vehicle.phase === 'turn' || vehicle.lane.length - vehicle.distance < 50) && Math.floor(town.time * 2.8) % 2 === 0) {
            circle(g, length / 2 - 3, Math.sign(cross) * width / 2, 1.8, '#ffe08b');
        }
        g.restore();
    }

    function render() {
        ctx.setTransform(1, 0, 0, 1, 0, 0);
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.drawImage(scenery, 0, 0);
        worldTransform(ctx);
        town.vehicles.forEach(drawVehicle);
        for (const lane of town.lanes) if (lane.to.signal) drawSignal(lane);
    }

    function updateLabels() {
        pauseButton.textContent = paused ? 'Resume' : 'Pause';
        pauseButton.setAttribute('aria-label', paused ? 'Resume town animation' : 'Pause town animation');
        pauseButton.setAttribute('aria-pressed', String(paused));
        document.getElementById('town-state').textContent = paused ? 'Town paused' : 'A little town, alive';
        document.body.classList.toggle('town-paused', paused);
    }

    function updateTrafficLabels() {
        const buses = town.vehicles.filter(v => v.bus).length;
        const cars = town.vehicles.length - buses;
        document.getElementById('bus-count').textContent = `${buses} buses`;
        document.getElementById('car-count').textContent = `${cars} cars`;
        document.getElementById('traffic-value').textContent = `${Math.round(trafficLevel * 100)}%`;
        trafficControl.setAttribute('aria-valuetext', `${buses} buses and ${cars} cars`);
    }

    function updateView() {
        const zoom = Number(zoomControl.value) / 100;
        const baseScale = window.innerWidth < 600 ? 0.78 : 1;
        // The lower end always fits the whole town, including after a resize.
        const fitScale = Math.min(window.innerWidth / town.width, window.innerHeight / town.height);
        scale = zoom < 1 ? fitScale + (baseScale - fitScale) * (zoom - 0.5) / 0.5 : baseScale * zoom;
        cameraX = (town.width - window.innerWidth / scale) / 2;
        cameraY = (town.height - window.innerHeight / scale) / 2;
        const percent = Math.round(scale / baseScale * 100);
        document.getElementById('zoom-value').textContent = zoom === 0.5 ? 'Whole town' : `${percent}%`;
        zoomControl.setAttribute('aria-valuetext', zoom === 0.5 ? 'Whole town' : `${percent} percent`);
        drawScenery();
        render();
    }

    function resize(force = false) {
        const width = window.innerWidth;
        const height = window.innerHeight;
        const nextDpr = Math.min(window.devicePixelRatio || 1, 2);
        const size = `${width}:${height}:${nextDpr}`;
        if (!force && size === previousSize) return;
        previousSize = size;
        dpr = nextDpr;
        canvas.width = scenery.width = Math.round(width * dpr);
        canvas.height = scenery.height = Math.round(height * dpr);
        if (!town || force) {
            town = createTown(Math.max(width < 600 ? 1000 : 2200, width * 2), Math.max(1700, height * 2), seed);
            setTrafficLevel(town, trafficLevel);
            // Start with moving traffic, spread naturally along its lanes.
            for (let i = 0; i < 180; i++) updateTown(town, FIXED_STEP);
        }
        updateView();
        document.getElementById('town-number').textContent = `Town ${seed.toString(36).slice(-4).toUpperCase().padStart(4, '0')}`;
        updateTrafficLabels();
        accumulator = 0;
        lastTime = 0;
    }

    function animate(timestamp) {
        frame = null;
        if (paused || document.hidden) return;
        if (lastTime) accumulator += Math.min((timestamp - lastTime) / 1000, 0.1) * simulationSpeed;
        lastTime = timestamp;
        while (accumulator >= FIXED_STEP) {
            updateTown(town, FIXED_STEP);
            accumulator -= FIXED_STEP;
        }
        render();
        frame = requestAnimationFrame(animate);
    }

    function syncAnimation() {
        if (frame !== null) cancelAnimationFrame(frame);
        frame = null;
        lastTime = 0;
        accumulator = 0;
        if (!paused && !document.hidden) frame = requestAnimationFrame(animate);
        updateLabels();
    }

    function toggleExplore() {
        exploring = !exploring;
        if (exploring) savedScroll = window.scrollY;
        document.body.classList.toggle('exploring-town', exploring);
        content.inert = exploring;
        content.setAttribute('aria-hidden', String(exploring));
        exploreButton.textContent = exploring ? 'Back to links' : 'Watch the town';
        exploreButton.setAttribute('aria-pressed', String(exploring));
        if (!exploring) window.scrollTo(0, savedScroll);
    }

    pauseButton.addEventListener('click', () => {
        paused = !paused;
        syncAnimation();
    });
    speedControl.addEventListener('change', () => {
        simulationSpeed = Number(speedControl.value);
    });
    trafficControl.addEventListener('input', () => {
        trafficLevel = Number(trafficControl.value) / 100;
        setTrafficLevel(town, trafficLevel);
        updateTrafficLabels();
        render();
    });
    zoomControl.addEventListener('input', updateView);
    newButton.addEventListener('click', () => {
        seed = newSeed();
        resize(true);
        status.textContent = 'A new town is ready. New streets, neighbourhoods and bus routes.';
    });
    exploreButton.addEventListener('click', toggleExplore);
    document.addEventListener('keydown', event => {
        if (event.key === 'Escape' && exploring) {
            toggleExplore();
            exploreButton.focus();
        }
    });
    document.addEventListener('visibilitychange', syncAnimation);
    motionPreference.addEventListener('change', event => {
        paused = event.matches;
        syncAnimation();
    });
    window.addEventListener('resize', () => {
        clearTimeout(resizeTimer);
        resizeTimer = setTimeout(() => resize(), 180);
    });
    resize();
    document.getElementById('town-controls').hidden = false;
    syncAnimation();
}
