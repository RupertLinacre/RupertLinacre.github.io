# RupertLinacre.github.io
Rupert's Website

The background is a procedural miniature town with left-hand traffic, three
numbered bus circuits, bus stops, and working traffic lights. Every visit (or
**New town** click) generates fresh streets and neighbourhoods. **Watch the town**
hides the links; **Escape** brings them back. **Pause** freezes the simulation.
Reduced-motion preferences pause it automatically, and hidden tabs stop rendering.

Run locally with `python3 -m http.server 4173`, then open
<http://localhost:4173>. There is no build step or external runtime dependency.

`road-world.mjs` generates the connected road graph and advances traffic in fixed
time steps. `road-animation.js` draws the town on a high-DPI canvas, caching the
static scenery separately from moving vehicles and lights. Rendering uses native
Canvas 2D shapes and does not need image downloads.

Run the simulation checks with `node --test tests/road-world.test.mjs`.
