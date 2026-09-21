import { TownView, bindTownGestures } from './town-view.mjs';
import { createTown, updateTown, setTrafficLevel, updateTrafficMetrics, vehiclePoint, lanePoint, signalState, randomSource, pathPoint, offsetPath, isRoundabout } from './road-world.mjs?v=5';

const canvas = document.getElementById('road-canvas');
const ctx = canvas?.getContext('2d');

if (ctx) {
    const scenery = document.createElement('canvas');
    const background = scenery.getContext('2d');
    const pauseButton = document.getElementById('pause-town');
    const showTownButton = document.getElementById('show-town');
    const controls = document.getElementById('town-controls');
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
    const view = new TownView();
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
        g.setTransform(dpr * view.scale, 0, 0, dpr * view.scale, -view.x * dpr * view.scale, -view.y * dpr * view.scale);
    }

    function visible(p, margin = 50) {
        return p.x > view.x - margin && p.x < view.x + window.innerWidth / view.scale + margin &&
            p.y > view.y - margin && p.y < view.y + window.innerHeight / view.scale + margin;
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
            const plots = [];
            // Front gardens and houses follow the actual curb, including around
            // crescents, rather than filling each neighbourhood with a small grid.
            for (let distance = 30 + random() * 30; distance < block.boundary.length; distance += 62 + random() * 12) {
                const curb = pathPoint(block.boundary, distance);
                const setback = 69 + random() * 7;
                const px = curb.x - Math.sin(curb.angle) * setback;
                const py = curb.y + Math.cos(curb.angle) * setback;
                const bw = 35 + random() * 12;
                const bh = 27 + random() * 9;
                const radius = Math.hypot(bw, bh) / 2 + 5;
                if (!clearPlot(px, py, radius) || plots.some(p => Math.hypot(p.x - px, p.y - py) < p.radius + radius + 6)) continue;
                plots.push({ x: px, y: py, radius });
                line(g, curb.x - Math.sin(curb.angle) * 30, curb.y + Math.cos(curb.angle) * 30,
                    px, py, '#e8e2cc', 5);
                g.save();
                g.translate(px, py);
                g.rotate(curb.angle + Math.PI);
                building(g, -bw / 2, -bh / 2, bw, bh, random, block.kind === 'shops');
                g.restore();
            }
            for (let i = 0; i < 20; i++) {
                const tx = x + random() * w;
                const ty = y + random() * h;
                if (clearPlot(tx, ty, 10) && plots.every(p => Math.hypot(p.x - tx, p.y - ty) > p.radius + 13)) {
                    tree(g, tx, ty, 6 + random() * 5, random);
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
        g.lineCap = 'butt';
        g.lineJoin = 'round';
        g.setLineDash([]);
        town.blocks.forEach(block => { if (visible(block.center, Math.hypot(block.width, block.height) / 2 + 50)) drawBlock(g, block); });
        g.lineCap = 'round';
        g.lineJoin = 'round';
        // Wider, darker corridors carry through traffic. Narrow lanes taper
        // into one shared carriageway, matching the paths driven by the cars.
        const roadsByWidth = [...town.edges].sort((a, b) => a.width - b.width);
        for (const extra of [15, 12, 0]) {
            for (const edge of roadsByWidth) {
                const colour = extra === 15 ? '#a8b89c' : extra === 12 ? '#e9e3d4' : edge.colour;
                if (!edge.singleTrack) strokePath(g, edge.path, colour, edge.width + extra);
                else for (let d = 0; d < edge.path.length; d += 4) {
                    const outside = d < edge.narrowStart ? Math.min(1, (edge.narrowStart - d) / 24) :
                        d > edge.narrowEnd ? Math.min(1, (d - edge.narrowEnd) / 24) : 0;
                    const width = 20 + (edge.width - 20) * (1 - Math.cos(Math.PI * outside)) / 2;
                    const a = pathPoint(edge.path, d), b = pathPoint(edge.path, d + 4);
                    line(g, a.x, a.y, b.x, b.y, colour, width + extra);
                }
            }
            for (const node of town.nodes) if (isRoundabout(node)) {
                circle(g, node.x, node.y, node.orbitRadius + 15 + extra / 2,
                    extra === 15 ? '#a8b89c' : extra === 12 ? '#e9e3d4' : '#637477');
            }
        }
        g.lineCap = 'butt';
        for (const edge of town.edges) {
            const start = edge.a.radius + 8;
            const end = edge.path.length - edge.b.radius - 8;
            const sections = edge.singleTrack ? [[start, edge.narrowStart - 34], [edge.narrowEnd + 34, end]] : [[start, end]];
            for (const [a, b] of sections) {
                if (b <= a) continue;
                if (edge.roadType !== 'residential') {
                    g.setLineDash([9, 11]);
                    strokePath(g, offsetPath(edge.path, 0, a, b), '#dde0cd', 1.5);
                    g.setLineDash([]);
                }
                for (const side of [-1, 1]) strokePath(g, offsetPath(edge.path, (edge.width / 2 - 2) * side, a, b), '#c6ccba', 0.65);
            }
            if (edge.singleTrack) {
                for (const side of [-1, 1]) {
                    strokePath(g, offsetPath(edge.path, side * 13, edge.narrowStart + 8, edge.narrowEnd - 8), '#747f73', 2.5);
                    strokePath(g, offsetPath(edge.path, side * 13, edge.narrowStart + 8, edge.narrowEnd - 8), '#e5debd', 1);
                }
                for (const lane of edge.lanes) {
                    const p = lanePoint(lane, lane.singleEntry - 15);
                    g.save(); g.translate(p.x, p.y); g.rotate(p.angle);
                    // Roadside narrowing sign; drivers take turns through here.
                    line(g, 0, -11, 0, -26, '#667567', 2);
                    g.beginPath(); g.moveTo(-7, -20); g.lineTo(7, -20); g.lineTo(0, -32); g.closePath();
                    g.fillStyle = '#fbf2d6'; g.fill(); g.strokeStyle = '#b9614d'; g.lineWidth = 2; g.stroke();
                    line(g, -2, -23, 0, -28, '#5a685e', 1.3);
                    line(g, 2, -23, 0, -28, '#5a685e', 1.3);
                    g.restore();
                }
            }
        }
        for (const node of town.nodes) {
            if (isRoundabout(node)) {
                const full = node.control === 'roundabout';
                circle(g, node.x, node.y, full ? 27 : 8, '#e9e3d4');
                circle(g, node.x, node.y, full ? 24 : 6, full ? '#9ebc83' : '#f4edda');
                if (full) {
                    circle(g, node.x + 2, node.y + 3, 13, '#709366');
                    circle(g, node.x - 3, node.y - 3, 10, '#8ead73');
                }
                for (let i = 0; i < 3; i++) {
                    const angle = i * Math.PI * 2 / 3;
                    g.save(); g.translate(node.x + Math.cos(angle) * node.orbitRadius, node.y + Math.sin(angle) * node.orbitRadius);
                    g.rotate(angle + Math.PI / 2);
                    line(g, -5, 0, 5, 0, '#e9e6cf', 1.6);
                    line(g, 2, -3, 5, 0, '#e9e6cf', 1.6);
                    line(g, 2, 3, 5, 0, '#e9e6cf', 1.6);
                    g.restore();
                }
            }
            if (node.outgoing.length < 3) continue;
            for (const outgoing of node.outgoing) {
                const incoming = outgoing.edge.lanes.find(l => l.to === node);
                if (!node.signal && !isRoundabout(node) && incoming.hasPriority) continue;
                g.save();
                g.translate(incoming.end.x, incoming.end.y);
                g.rotate(incoming.end.angle);
                if (node.signal) {
                    for (let stripe = -17; stripe <= 17; stripe += 6) {
                        g.fillStyle = '#e6e6d5';
                        g.fillRect(6, stripe + incoming.edge.offset, 7, 3);
                    }
                    line(g, -3, -8, -3, 8, '#f1edda', 2);
                } else {
                    g.setLineDash([3, 3]);
                    for (const x of [-3, -7]) line(g, x, -7, x, 7, '#f1edda', 1.6);
                    g.setLineDash([]);
                    g.beginPath(); g.moveTo(-15, 0); g.lineTo(-25, -5); g.lineTo(-25, 5); g.closePath();
                    g.strokeStyle = '#edead5'; g.lineWidth = 1.2; g.stroke();
                }
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
    }

    function updateTrafficLabels() {
        const buses = town.vehicles.filter(v => v.bus).length;
        const cars = town.vehicles.length - buses;
        document.getElementById('traffic-value').textContent = `${Math.round(trafficLevel * 100)}%`;
        trafficControl.setAttribute('aria-valuetext', `${buses} buses and ${cars} cars`);
    }

    function repaintView() {
        zoomControl.value = String(view.value);
        const label = view.value === 50 ? 'Whole town' : `${Math.round(view.scale / view.base * 100)}%`;
        document.getElementById('zoom-value').textContent = label;
        zoomControl.setAttribute('aria-valuetext', label === 'Whole town' ? label : `${Math.round(view.scale / view.base * 100)} percent`);
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
        const resetView = !town || force;
        if (resetView) {
            town = createTown(Math.max(width < 600 ? 1000 : 2200, width * 2), Math.max(1700, height * 2), seed);
            setTrafficLevel(town, trafficLevel);
            // Start with moving traffic, spread naturally along its lanes.
            for (let i = 0; i < 180; i++) updateTown(town, FIXED_STEP);
        }
        view.configure(width, height, town.width, town.height, resetView);
        repaintView();
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
        controls.hidden = !exploring;
        content.inert = exploring;
        content.setAttribute('aria-hidden', String(exploring));
        exploreButton.textContent = exploring ? 'Back to links' : 'Watch the town';
        exploreButton.setAttribute('aria-pressed', String(exploring));
        if (!exploring) {
            showTownButton.focus({ preventScroll: true });
            window.scrollTo(0, savedScroll);
        } else {
            exploreButton.focus({ preventScroll: true });
        }
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
        updateTrafficMetrics(town);
        updateTrafficLabels();
        render();
    });
    zoomControl.addEventListener('input', () => { view.setZoom(Number(zoomControl.value)); repaintView(); });
    bindTownGestures(canvas, view, repaintView);
    newButton.addEventListener('click', () => {
        seed = newSeed();
        resize(true);
        status.textContent = 'A new town is ready. New streets, neighbourhoods and bus routes.';
    });
    exploreButton.addEventListener('click', toggleExplore);
    showTownButton.addEventListener('click', toggleExplore);
    document.addEventListener('keydown', event => {
        if (event.key === 'Escape' && exploring) {
            toggleExplore();
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
    showTownButton.hidden = false;
    syncAnimation();
}
