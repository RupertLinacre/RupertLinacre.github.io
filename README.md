# RupertLinacre.github.io
Rupert's Website

The background is a procedural miniature town with sweeping circular arcs,
crescents, irregular neighbourhoods, left-hand traffic, three
numbered bus circuits, bus stops, and working traffic lights. Every visit (or
**New town** click) generates fresh streets and neighbourhoods. **Watch the town**
hides the links; **Escape** brings them back. **Pause** freezes the simulation.
Reduced-motion preferences pause it automatically, and hidden tabs stop rendering.

The controls adjust speed from **0.5× to 8×**, add or remove traffic from **0% to
200%**, and zoom from the **whole town** to **150%**. Zooming and resizing preserve
the current town and its traffic. Traffic changes insert vehicles into safe gaps
and release junction reservations when vehicles are removed.

Run locally with `python3 -m http.server 4173`, then open
<http://localhost:4173>. There is no build step or external runtime dependency.

`street-layout.mjs` divides neighbourhoods with circular-arc streets. Existing
roads retain their exact curve and tangent as side roads join them; there is no
underlying rectangular grid. `street-geometry.mjs` provides measured arcs and
lane offsets. Houses follow the curbs around each neighbourhood.

`road-world.mjs` advances traffic in fixed time steps, even at higher speeds.
Vehicles follow measured lane paths; busy junctions admit eligible vehicles in
arrival order and require enough space for the whole vehicle to clear. Cars can
choose a detour around a persistently blocked turn. `road-animation.js` draws the
town on a high-DPI canvas, caching the
static scenery separately from moving vehicles and lights. Rendering uses native
Canvas 2D shapes and does not need image downloads.

Run the simulation checks with `node --test tests/road-world.test.mjs`.
