# RupertLinacre.github.io
Rupert's Website

The background is a procedural miniature town with sweeping circular arcs,
crescents, irregular neighbourhoods, left-hand traffic, three
numbered bus circuits, bus stops, roundabouts, mini-roundabouts, give-way
junctions, single-track bottlenecks, and working traffic lights. Every visit (or
**New town** click) generates fresh streets and neighbourhoods. **Watch the town**
hides the links; **Escape** brings them back. **Pause** freezes the simulation.
Reduced-motion preferences pause it automatically, and hidden tabs stop rendering.

The controls adjust speed from **0.5× to 8×**, add or remove traffic from **0% to
600%**, and zoom from the **whole town** to **150%**. Zooming and resizing preserve
the current town and its traffic. Traffic changes insert vehicles into safe gaps
and release junction reservations when vehicles are removed.

**Plan a trip** reveals the street hierarchy. Choose two places on the map or
use the From/To menus for a route that accounts for road speeds and current
queues. Blue marks the planned route; gold marks arterial corridors. The
estimated time uses simulation seconds. Turn traffic up to create queues that
spill back from junctions, and watch the live waiting count and congestion status.

Run locally with `python3 -m http.server 4173`, then open
<http://localhost:4173>. There is no build step or external runtime dependency.

`street-layout.mjs` divides neighbourhoods with circular-arc streets. Existing
roads retain their exact curve and tangent as side roads join them; there is no
underlying rectangular grid. `street-geometry.mjs` provides measured arcs and
lane offsets. Houses follow the curbs around each neighbourhood.

`traffic-planner.mjs` builds a connected arterial backbone between neighbourhood
hubs, identifies local main roads from cross-town demand, and keeps quieter
roads residential. Cars and bus circuits use travel-time routing; buses favour
main roads and avoid narrow shortcuts when practical.

`road-world.mjs` advances traffic in fixed time steps, even at higher speeds.
Vehicles follow measured lane paths and require space for their whole length
before entering a junction. `junctions.mjs` handles priority-road gap acceptance,
clockwise roundabout paths and safe entry gaps; compact mini-roundabouts admit
one manoeuvre at a time. `single-track.mjs` tapers both directions into a shared
carriageway and alternates waiting traffic once the previous vehicles clear.
Cars can choose a detour around a persistently blocked turn; high demand can
still overwhelm road capacity. Traffic changes are bounded by safe insertion
space, so a full network may admit fewer vehicles than requested.

`road-animation.js` draws the town on a high-DPI canvas, caching the
static scenery separately from moving vehicles and lights. Rendering uses native
Canvas 2D shapes and does not need image downloads.

Run the simulation checks with `node --test tests/*.test.mjs`.
