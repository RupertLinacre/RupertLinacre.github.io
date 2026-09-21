// Screen coordinates and the zoom slider share one camera, in both page modes.
export class TownView {
    constructor() { this.x = 0; this.y = 0; this.scale = 1; this.value = 100; }
    configure(width, height, townWidth, townHeight, reset = false) {
        const centre = this.width ? this.world(this.width / 2, this.height / 2) : null;
        Object.assign(this, { width, height, townWidth, townHeight });
        this.base = width < 600 ? 0.78 : 1;
        this.fit = Math.min(width / townWidth, height / townHeight);
        this.scale = this.scaleAt(this.value);
        const p = reset || !centre ? { x: townWidth / 2, y: townHeight / 2 } : centre;
        this.x = p.x - width / this.scale / 2;
        this.y = p.y - height / this.scale / 2;
    }
    scaleAt(value) {
        return value < 100 ? this.fit + (this.base - this.fit) * (value - 50) / 50 : this.base * value / 100;
    }
    world(x, y) { return { x: this.x + x / this.scale, y: this.y + y / this.scale }; }
    setZoom(value, anchor = { x: this.width / 2, y: this.height / 2 }, destination = anchor) {
        const world = this.world(anchor.x, anchor.y);
        this.value = Math.max(50, Math.min(150, value));
        this.scale = this.scaleAt(this.value);
        this.x = world.x - destination.x / this.scale;
        this.y = world.y - destination.y / this.scale;
    }
    zoomBy(factor, anchor, destination = anchor) {
        const scale = this.scale * factor;
        const value = scale < this.base ? 50 + 50 * (scale - this.fit) / (this.base - this.fit) : 100 * scale / this.base;
        this.setZoom(value, anchor, destination);
    }
    pan(dx, dy) { this.x -= dx / this.scale; this.y -= dy / this.scale; }
}

export function bindTownGestures(canvas, view, redraw) {
    const pointers = new Map();
    const point = event => ({ x: event.clientX, y: event.clientY });
    const centre = pair => ({ x: (pair[0].x + pair[1].x) / 2, y: (pair[0].y + pair[1].y) / 2 });
    const distance = pair => Math.hypot(pair[1].x - pair[0].x, pair[1].y - pair[0].y);
    canvas.addEventListener('pointerdown', event => {
        if (event.button > 0 || pointers.size >= 2) return;
        pointers.set(event.pointerId, point(event));
        canvas.setPointerCapture(event.pointerId);
    });
    canvas.addEventListener('pointermove', event => {
        const previous = pointers.get(event.pointerId);
        if (!previous) return;
        const before = [...pointers.values()];
        pointers.set(event.pointerId, point(event));
        if (pointers.size === 1) view.pan(event.clientX - previous.x, event.clientY - previous.y);
        else {
            const after = [...pointers.values()];
            view.zoomBy(distance(after) / Math.max(1, distance(before)), centre(before), centre(after));
        }
        redraw();
    });
    const release = event => {
        pointers.delete(event.pointerId);
        if (canvas.hasPointerCapture(event.pointerId)) canvas.releasePointerCapture(event.pointerId);
    };
    for (const type of ['pointerup', 'pointercancel', 'lostpointercapture']) canvas.addEventListener(type, release);
    canvas.addEventListener('wheel', event => {
        event.preventDefault();
        const unit = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? view.height : 1;
        // Trackpad pinch is delivered as Ctrl+wheel; ordinary two-finger scrolling pans.
        if (event.ctrlKey || event.metaKey) view.zoomBy(Math.exp(-event.deltaY * unit * 0.01), point(event));
        else view.pan(-event.deltaX * unit, -event.deltaY * unit);
        redraw();
    }, { passive: false });
}
