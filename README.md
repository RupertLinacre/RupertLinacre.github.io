# RupertLinacre.github.io
Rupert's Website

The background is a procedural miniature town with winding roads, diagonal
shortcuts, irregular neighbourhoods, left-hand traffic, three
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

`road-world.mjs` generates the connected road graph and advances traffic in fixed
time steps, even at higher speeds. Vehicles follow measured, curved lane paths;
busy junctions admit eligible vehicles in arrival order. `road-animation.js`
draws the town on a high-DPI canvas, caching the
static scenery separately from moving vehicles and lights. Rendering uses native
Canvas 2D shapes and does not need image downloads.

Run the simulation checks with `node --test tests/road-world.test.mjs`.
