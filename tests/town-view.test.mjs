import test from 'node:test';
import assert from 'node:assert/strict';
import { TownView, bindTownGestures } from '../town-view.mjs';
const close = (a, b) => assert.ok(Math.abs(a - b) < 1e-8, `${a} != ${b}`);
const same = (a, b) => { close(a.x, b.x); close(a.y, b.y); };
function setup() {
    const view = new TownView(); view.configure(1200, 800, 2400, 1700);
    const canvas = new EventTarget(), captured = new Set();
    canvas.setPointerCapture = id => captured.add(id);
    canvas.hasPointerCapture = id => captured.has(id);
    canvas.releasePointerCapture = id => captured.delete(id);
    let slider;
    bindTownGestures(canvas, view, () => { slider = view.value; });
    const send = (type, properties) => {
        const event = new Event(type, { cancelable: true });
        Object.assign(event, { button: 0, deltaMode: 0, deltaX: 0, deltaY: 0 }, properties);
        canvas.dispatchEvent(event); return event;
    };
    return { view, send, captured, slider: () => slider };
}
test('pinching anchors the town to the moving midpoint and updates slider state', () => {
    const { view, send, slider } = setup();
    const anchor = view.world(200, 200);
    send('pointerdown', { pointerId: 1, clientX: 100, clientY: 200 });
    send('pointerdown', { pointerId: 2, clientX: 300, clientY: 200 });
    send('pointermove', { pointerId: 2, clientX: 380, clientY: 200 });
    same(view.world(240, 200), anchor);
    close(view.scale, 1.4); close(slider(), 140);
});
test('lifting one finger continues panning without a jump; cancellation releases it', () => {
    const { view, send, captured } = setup();
    send('pointerdown', { pointerId: 1, clientX: 100, clientY: 200 });
    send('pointerdown', { pointerId: 2, clientX: 300, clientY: 200 });
    send('pointerup', { pointerId: 2 });
    const before = view.world(100, 200);
    send('pointermove', { pointerId: 1, clientX: 130, clientY: 220 });
    same(view.world(130, 220), before);
    send('pointercancel', { pointerId: 1 });
    const stopped = { x: view.x, y: view.y };
    send('pointermove', { pointerId: 1, clientX: 500, clientY: 600 });
    same(view, stopped); assert.equal(captured.size, 0);
});
test('slider zoom after a pan and viewport resizing preserve the chosen centre', () => {
    const { view } = setup(); view.pan(200, -80);
    const centre = view.world(600, 400);
    view.setZoom(65); same(view.world(600, 400), centre);
    view.configure(390, 844, 2400, 1700); same(view.world(195, 422), centre);
    view.setZoom(100); same(view.world(195, 422), centre);
    close(view.scale, 0.78);
});
test('trackpad scrolling pans; pinch zoom respects slider limits and its focal point', () => {
    const { view, send, slider } = setup();
    const oldX = view.x;
    assert.ok(send('wheel', { deltaX: 40, deltaY: 20 }).defaultPrevented);
    close(view.x, oldX + 40);
    const anchor = view.world(80, 100);
    send('wheel', { ctrlKey: true, deltaY: -1000, clientX: 80, clientY: 100 });
    close(slider(), 150); same(view.world(80, 100), anchor);
    send('wheel', { ctrlKey: true, deltaY: 1000, clientX: 80, clientY: 100 });
    close(slider(), 50); close(view.scale, view.fit); same(view.world(80, 100), anchor);
});
