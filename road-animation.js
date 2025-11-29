// Road Animation with Buses
// Procedurally generated curved roads with animated bus images
// Uses Poisson disk sampling, Delaunay triangulation, and intersection avoidance

(function () {
    const canvas = document.getElementById('road-canvas');
    if (!canvas) return;

    const ctx = canvas.getContext('2d');

    let roadSegments = [];
    let buses = [];

    // Load bus images
    const busImages = [];
    const busImageSrcs = ['bus1.png', 'bus2.png'];
    let imagesLoaded = 0;

    busImageSrcs.forEach((src, index) => {
        const img = new Image();
        img.onload = () => {
            imagesLoaded++;
            if (imagesLoaded === busImageSrcs.length) {
                // All images loaded, start animation
                resizeCanvas();
                animate();
            }
        };
        img.src = src;
        busImages[index] = img;
    });

    function resizeCanvas() {
        canvas.width = window.innerWidth;
        canvas.height = window.innerHeight;
        generateRoads();
    }

    // Get point and tangent angle along a road segment at progress t (0-1)
    function getPointOnSegment(segment, t) {
        if (segment.type === 'line') {
            const x = segment.x1 + (segment.x2 - segment.x1) * t;
            const y = segment.y1 + (segment.y2 - segment.y1) * t;
            const angle = Math.atan2(segment.y2 - segment.y1, segment.x2 - segment.x1);
            return { x, y, angle };
        } else if (segment.type === 'arc') {
            const angle = segment.startAngle + (segment.endAngle - segment.startAngle) * t;
            const x = segment.cx + segment.radius * Math.cos(angle);
            const y = segment.cy + segment.radius * Math.sin(angle);
            // Tangent is perpendicular to radius
            const tangentAngle = angle + (segment.endAngle > segment.startAngle ? Math.PI / 2 : -Math.PI / 2);
            return { x, y, angle: tangentAngle };
        } else if (segment.type === 'bezier') {
            // Cubic bezier
            const t2 = t * t;
            const t3 = t2 * t;
            const mt = 1 - t;
            const mt2 = mt * mt;
            const mt3 = mt2 * mt;

            const x = mt3 * segment.x1 + 3 * mt2 * t * segment.cx1 + 3 * mt * t2 * segment.cx2 + t3 * segment.x2;
            const y = mt3 * segment.y1 + 3 * mt2 * t * segment.cy1 + 3 * mt * t2 * segment.cy2 + t3 * segment.y2;

            // Derivative for tangent
            const dx = 3 * mt2 * (segment.cx1 - segment.x1) + 6 * mt * t * (segment.cx2 - segment.cx1) + 3 * t2 * (segment.x2 - segment.cx2);
            const dy = 3 * mt2 * (segment.cy1 - segment.y1) + 6 * mt * t * (segment.cy2 - segment.cy1) + 3 * t2 * (segment.y2 - segment.cy2);
            const angle = Math.atan2(dy, dx);

            return { x, y, angle };
        }
    }

    // Poisson disk sampling for organic node distribution
    function poissonDiskSampling(width, height, minDist, maxAttempts = 30) {
        const cellSize = minDist / Math.sqrt(2);
        const gridWidth = Math.ceil(width / cellSize);
        const gridHeight = Math.ceil(height / cellSize);
        const grid = new Array(gridWidth * gridHeight).fill(null);
        const points = [];
        const active = [];

        function gridIndex(x, y) {
            return Math.floor(x / cellSize) + Math.floor(y / cellSize) * gridWidth;
        }

        function isValid(x, y) {
            if (x < 0 || x >= width || y < 0 || y >= height) return false;

            const cellX = Math.floor(x / cellSize);
            const cellY = Math.floor(y / cellSize);

            for (let dy = -2; dy <= 2; dy++) {
                for (let dx = -2; dx <= 2; dx++) {
                    const nx = cellX + dx;
                    const ny = cellY + dy;
                    if (nx >= 0 && nx < gridWidth && ny >= 0 && ny < gridHeight) {
                        const neighbor = grid[nx + ny * gridWidth];
                        if (neighbor !== null) {
                            const dist = Math.hypot(x - neighbor.x, y - neighbor.y);
                            if (dist < minDist) return false;
                        }
                    }
                }
            }
            return true;
        }

        // Start with a random point
        const startX = width * 0.5;
        const startY = height * 0.5;
        const startPoint = { x: startX, y: startY };
        points.push(startPoint);
        active.push(startPoint);
        grid[gridIndex(startX, startY)] = startPoint;

        while (active.length > 0) {
            const randIndex = Math.floor(Math.random() * active.length);
            const point = active[randIndex];
            let found = false;

            for (let i = 0; i < maxAttempts; i++) {
                const angle = Math.random() * Math.PI * 2;
                const dist = minDist + Math.random() * minDist;
                const newX = point.x + Math.cos(angle) * dist;
                const newY = point.y + Math.sin(angle) * dist;

                if (isValid(newX, newY)) {
                    const newPoint = { x: newX, y: newY };
                    points.push(newPoint);
                    active.push(newPoint);
                    grid[gridIndex(newX, newY)] = newPoint;
                    found = true;
                    break;
                }
            }

            if (!found) {
                active.splice(randIndex, 1);
            }
        }

        return points;
    }

    // Simple Delaunay triangulation using Bowyer-Watson algorithm
    function delaunayTriangulation(points) {
        if (points.length < 3) return [];

        // Create super triangle
        const minX = Math.min(...points.map(p => p.x)) - 100;
        const maxX = Math.max(...points.map(p => p.x)) + 100;
        const minY = Math.min(...points.map(p => p.y)) - 100;
        const maxY = Math.max(...points.map(p => p.y)) + 100;

        const dx = maxX - minX;
        const dy = maxY - minY;
        const dmax = Math.max(dx, dy) * 2;

        const p1 = { x: minX - dmax, y: minY - dmax, superTriangle: true };
        const p2 = { x: minX + dmax * 2, y: minY - dmax, superTriangle: true };
        const p3 = { x: minX + dx / 2, y: maxY + dmax, superTriangle: true };

        let triangles = [{ a: p1, b: p2, c: p3 }];

        function circumcircle(t) {
            const ax = t.a.x, ay = t.a.y;
            const bx = t.b.x, by = t.b.y;
            const cx = t.c.x, cy = t.c.y;

            const d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
            if (Math.abs(d) < 1e-10) return { x: 0, y: 0, r: Infinity };

            const ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
            const uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;

            return { x: ux, y: uy, r: Math.hypot(ax - ux, ay - uy) };
        }

        for (const point of points) {
            const badTriangles = [];

            for (const tri of triangles) {
                const cc = circumcircle(tri);
                if (Math.hypot(point.x - cc.x, point.y - cc.y) < cc.r) {
                    badTriangles.push(tri);
                }
            }

            const polygon = [];
            for (const tri of badTriangles) {
                const edges = [
                    [tri.a, tri.b],
                    [tri.b, tri.c],
                    [tri.c, tri.a]
                ];

                for (const edge of edges) {
                    let shared = false;
                    for (const other of badTriangles) {
                        if (other === tri) continue;
                        const otherEdges = [
                            [other.a, other.b],
                            [other.b, other.c],
                            [other.c, other.a]
                        ];
                        for (const oe of otherEdges) {
                            if ((edge[0] === oe[0] && edge[1] === oe[1]) ||
                                (edge[0] === oe[1] && edge[1] === oe[0])) {
                                shared = true;
                                break;
                            }
                        }
                        if (shared) break;
                    }
                    if (!shared) polygon.push(edge);
                }
            }

            triangles = triangles.filter(t => !badTriangles.includes(t));

            for (const edge of polygon) {
                triangles.push({ a: edge[0], b: edge[1], c: point });
            }
        }

        // Remove triangles with super triangle vertices
        triangles = triangles.filter(t =>
            !t.a.superTriangle && !t.b.superTriangle && !t.c.superTriangle
        );

        return triangles;
    }

    // Extract unique edges from triangulation
    function extractEdges(triangles, points) {
        const edgeSet = new Set();
        const edges = [];

        for (const tri of triangles) {
            const triEdges = [
                [points.indexOf(tri.a), points.indexOf(tri.b)],
                [points.indexOf(tri.b), points.indexOf(tri.c)],
                [points.indexOf(tri.c), points.indexOf(tri.a)]
            ];

            for (const [a, b] of triEdges) {
                if (a === -1 || b === -1) continue;
                const key = a < b ? `${a}-${b}` : `${b}-${a}`;
                if (!edgeSet.has(key)) {
                    edgeSet.add(key);
                    edges.push({ a, b, dist: Math.hypot(points[a].x - points[b].x, points[a].y - points[b].y) });
                }
            }
        }

        return edges;
    }

    // Union-Find for MST
    function createUnionFind(n) {
        const parent = Array.from({ length: n }, (_, i) => i);
        const rank = new Array(n).fill(0);

        function find(x) {
            if (parent[x] !== x) parent[x] = find(parent[x]);
            return parent[x];
        }

        function union(x, y) {
            const px = find(x), py = find(y);
            if (px === py) return false;
            if (rank[px] < rank[py]) parent[px] = py;
            else if (rank[px] > rank[py]) parent[py] = px;
            else { parent[py] = px; rank[px]++; }
            return true;
        }

        return { find, union };
    }

    // Check if two line segments intersect (excluding endpoints)
    function segmentsIntersect(p1, p2, p3, p4) {
        const ccw = (A, B, C) => (C.y - A.y) * (B.x - A.x) > (B.y - A.y) * (C.x - A.x);

        // Check if they share an endpoint
        if ((p1.x === p3.x && p1.y === p3.y) || (p1.x === p4.x && p1.y === p4.y) ||
            (p2.x === p3.x && p2.y === p3.y) || (p2.x === p4.x && p2.y === p4.y)) {
            return false;
        }

        return ccw(p1, p3, p4) !== ccw(p2, p3, p4) && ccw(p1, p2, p3) !== ccw(p1, p2, p4);
    }

    // Check if a new edge would intersect existing segments
    function wouldIntersect(nodes, edges, newEdge, segments) {
        const p1 = nodes[newEdge.a];
        const p2 = nodes[newEdge.b];

        // Check against existing straight-line approximations of segments
        for (const seg of segments) {
            const s1 = nodes[seg.startNode];
            const s2 = nodes[seg.endNode];
            if (segmentsIntersect(p1, p2, s1, s2)) {
                return true;
            }
        }

        return false;
    }

    function generateRoads() {
        roadSegments = [];
        buses = [];

        const minDist = 100;
        const padding = 20;

        // Generate nodes using Poisson disk sampling
        let nodes = poissonDiskSampling(
            canvas.width - padding * 2,
            canvas.height - padding * 2,
            minDist
        ).map(p => ({
            x: p.x + padding,
            y: p.y + padding,
            connections: []
        }));

        if (nodes.length < 3) {
            // Fallback if not enough nodes
            nodes = [
                { x: canvas.width * 0.25, y: canvas.height * 0.5, connections: [] },
                { x: canvas.width * 0.75, y: canvas.height * 0.5, connections: [] },
                { x: canvas.width * 0.5, y: canvas.height * 0.25, connections: [] }
            ];
        }

        // Delaunay triangulation
        const triangles = delaunayTriangulation(nodes);
        const allEdges = extractEdges(triangles, nodes);

        // Sort edges by distance for MST
        allEdges.sort((a, b) => a.dist - b.dist);

        // Build MST using Kruskal's algorithm
        const uf = createUnionFind(nodes.length);
        const mstEdges = [];
        const extraEdges = [];

        for (const edge of allEdges) {
            if (uf.union(edge.a, edge.b)) {
                mstEdges.push(edge);
            } else {
                extraEdges.push(edge);
            }
        }

        // Add some extra edges for loops (makes it more interesting)
        // But avoid edges that would create too many crossings
        const selectedEdges = [...mstEdges];
        const maxExtra = Math.floor(mstEdges.length * 0.4);

        // Shuffle extra edges and add some that don't cause intersections
        extraEdges.sort(() => Math.random() - 0.5);

        let addedExtra = 0;
        for (const edge of extraEdges) {
            if (addedExtra >= maxExtra) break;

            // Prefer shorter edges and avoid ones that intersect
            if (edge.dist < minDist * 2.5) {
                // Simple intersection check with existing edges
                let intersects = false;
                for (const existing of selectedEdges) {
                    if (edge.a === existing.a || edge.a === existing.b ||
                        edge.b === existing.a || edge.b === existing.b) continue;

                    if (segmentsIntersect(
                        nodes[edge.a], nodes[edge.b],
                        nodes[existing.a], nodes[existing.b]
                    )) {
                        intersects = true;
                        break;
                    }
                }

                if (!intersects) {
                    selectedEdges.push(edge);
                    addedExtra++;
                }
            }
        }

        // Create road segments from selected edges
        for (const edge of selectedEdges) {
            const node = nodes[edge.a];
            const other = nodes[edge.b];
            const dist = edge.dist;

            // Choose curve type based on distance and randomness
            const segmentType = Math.random();
            let segment;

            // Prefer straighter roads for longer distances
            const straightBias = Math.min(dist / (minDist * 2), 1) * 0.3;

            if (segmentType < 0.35 + straightBias) {
                // Straight line
                segment = {
                    type: 'line',
                    x1: node.x,
                    y1: node.y,
                    x2: other.x,
                    y2: other.y
                };
            } else if (segmentType < 0.7) {
                // Bezier curve with gentler curvature
                const midX = (node.x + other.x) / 2;
                const midY = (node.y + other.y) / 2;
                const perpX = -(other.y - node.y);
                const perpY = other.x - node.x;
                const len = Math.hypot(perpX, perpY);
                const curve = (Math.random() - 0.5) * 0.5; // Gentler curves

                segment = {
                    type: 'bezier',
                    x1: node.x,
                    y1: node.y,
                    cx1: midX + perpX / len * dist * curve * 0.4,
                    cy1: midY + perpY / len * dist * curve * 0.4,
                    cx2: midX + perpX / len * dist * curve * 0.4,
                    cy2: midY + perpY / len * dist * curve * 0.4,
                    x2: other.x,
                    y2: other.y
                };
            } else {
                // Arc with controlled curvature
                const midX = (node.x + other.x) / 2;
                const midY = (node.y + other.y) / 2;
                const perpX = -(other.y - node.y);
                const perpY = other.x - node.x;
                const len = Math.hypot(perpX, perpY);
                const bulge = (Math.random() - 0.5) * 0.8; // Gentler arcs

                const cx = midX + perpX / len * dist * bulge;
                const cy = midY + perpY / len * dist * bulge;
                const radius = Math.hypot(node.x - cx, node.y - cy);

                let startAngle = Math.atan2(node.y - cy, node.x - cx);
                let endAngle = Math.atan2(other.y - cy, other.x - cx);

                if (Math.abs(endAngle - startAngle) > Math.PI) {
                    if (endAngle > startAngle) {
                        startAngle += Math.PI * 2;
                    } else {
                        endAngle += Math.PI * 2;
                    }
                }

                segment = {
                    type: 'arc',
                    cx: cx,
                    cy: cy,
                    radius: radius,
                    startAngle: startAngle,
                    endAngle: endAngle
                };
            }

            segment.startNode = edge.a;
            segment.endNode = edge.b;
            roadSegments.push(segment);
            node.connections.push(roadSegments.length - 1);
            other.connections.push(roadSegments.length - 1);
        }

        // Store nodes for bus navigation
        roadSegments.nodes = nodes;

        // Create buses
        const numBuses = Math.max(3, Math.floor((canvas.width * canvas.height) / 70000));
        for (let i = 0; i < numBuses; i++) {
            createBus();
        }
    }

    function createBus() {
        if (roadSegments.length === 0) return;

        const segmentIndex = Math.floor(Math.random() * roadSegments.length);
        const bus = {
            segmentIndex: segmentIndex,
            progress: Math.random(),
            speed: 0.002 + Math.random() * 0.003,
            direction: Math.random() > 0.5 ? 1 : -1,
            imageIndex: Math.floor(Math.random() * busImages.length),
            facingRight: true // Track which way the bus is facing
        };

        buses.push(bus);
    }

    function drawRoads() {
        ctx.strokeStyle = '#ddd';
        ctx.lineWidth = 18;
        ctx.lineCap = 'round';
        ctx.lineJoin = 'round';

        roadSegments.forEach(segment => {
            ctx.beginPath();
            if (segment.type === 'line') {
                ctx.moveTo(segment.x1, segment.y1);
                ctx.lineTo(segment.x2, segment.y2);
            } else if (segment.type === 'arc') {
                const counterClockwise = segment.endAngle < segment.startAngle;
                ctx.arc(segment.cx, segment.cy, segment.radius, segment.startAngle, segment.endAngle, counterClockwise);
            } else if (segment.type === 'bezier') {
                ctx.moveTo(segment.x1, segment.y1);
                ctx.bezierCurveTo(segment.cx1, segment.cy1, segment.cx2, segment.cy2, segment.x2, segment.y2);
            }
            ctx.stroke();
        });

        // Draw road markings (dashed center line)
        ctx.strokeStyle = '#fff';
        ctx.lineWidth = 2;
        ctx.setLineDash([8, 8]);

        roadSegments.forEach(segment => {
            ctx.beginPath();
            if (segment.type === 'line') {
                ctx.moveTo(segment.x1, segment.y1);
                ctx.lineTo(segment.x2, segment.y2);
            } else if (segment.type === 'arc') {
                const counterClockwise = segment.endAngle < segment.startAngle;
                ctx.arc(segment.cx, segment.cy, segment.radius, segment.startAngle, segment.endAngle, counterClockwise);
            } else if (segment.type === 'bezier') {
                ctx.moveTo(segment.x1, segment.y1);
                ctx.bezierCurveTo(segment.cx1, segment.cy1, segment.cx2, segment.cy2, segment.x2, segment.y2);
            }
            ctx.stroke();
        });

        ctx.setLineDash([]);
    }

    function updateBuses() {
        const nodes = roadSegments.nodes;
        if (!nodes) return;

        buses.forEach(bus => {
            bus.progress += bus.speed * bus.direction;

            // Check if bus reached end of segment
            if (bus.progress >= 1 || bus.progress <= 0) {
                const currentSegment = roadSegments[bus.segmentIndex];
                const nodeIndex = bus.direction === 1 ? currentSegment.endNode : currentSegment.startNode;
                const node = nodes[nodeIndex];

                if (node && node.connections.length > 0) {
                    // Pick a random connected segment (prefer different ones)
                    const otherConnections = node.connections.filter(c => c !== bus.segmentIndex);
                    const connections = otherConnections.length > 0 ? otherConnections : node.connections;
                    const newSegmentIndex = connections[Math.floor(Math.random() * connections.length)];
                    const newSegment = roadSegments[newSegmentIndex];

                    // Determine direction on new segment
                    if (newSegment.startNode === nodeIndex) {
                        bus.direction = 1;
                        bus.progress = 0;
                    } else {
                        bus.direction = -1;
                        bus.progress = 1;
                    }

                    bus.segmentIndex = newSegmentIndex;
                } else {
                    // Reverse if stuck
                    bus.direction *= -1;
                    bus.progress = Math.max(0, Math.min(1, bus.progress));
                }
            }
        });
    }

    function drawBuses() {
        buses.forEach(bus => {
            const segment = roadSegments[bus.segmentIndex];
            if (!segment) return;

            const point = getPointOnSegment(segment, bus.progress);
            let angle = point.angle;

            // Adjust angle based on direction of travel
            if (bus.direction === -1) {
                angle += Math.PI;
            }

            // Normalize angle to -PI to PI
            while (angle > Math.PI) angle -= Math.PI * 2;
            while (angle < -Math.PI) angle += Math.PI * 2;

            // Determine if bus should face right or left based on horizontal movement
            // Bus images face right by default
            // Mirror horizontally if traveling rightward (angle between -PI/2 and PI/2, i.e., |angle| <= PI/2)
            const facingRight = Math.abs(angle) <= Math.PI / 2;

            const img = busImages[bus.imageIndex];
            if (!img || !img.complete) return;

            // Scale bus to 35 pixels high
            const targetHeight = 35;
            const scale = targetHeight / img.height;
            const width = img.width * scale;
            const height = targetHeight;

            ctx.save();
            ctx.translate(point.x, point.y);

            if (facingRight) {
                // Mirror horizontally for rightward travel (images face left by default)
                ctx.scale(-1, 1);
            }

            // Draw bus centered at position
            ctx.drawImage(img, -width / 2, -height / 2, width, height);
            ctx.restore();
        });
    }

    function animate() {
        ctx.fillStyle = '#f0f4f0';
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        drawRoads();
        updateBuses();
        drawBuses();

        requestAnimationFrame(animate);
    }

    window.addEventListener('resize', resizeCanvas);

    // Only start if images aren't loaded yet (they'll trigger resizeCanvas and animate when ready)
    // If images are already cached and loaded synchronously, start now
    if (imagesLoaded === busImageSrcs.length) {
        resizeCanvas();
        animate();
    }
})();
