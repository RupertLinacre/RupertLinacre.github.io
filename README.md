# RupertLinacre.github.io
Rupert's Website

The background is a procedural miniature town with sweeping circular arcs,
crescents, irregular neighbourhoods, left-hand traffic, three
numbered bus circuits, bus stops, roundabouts, mini-roundabouts, give-way
junctions, single-track bottlenecks, and working traffic lights. Every visit (or
**New town** click) generates fresh streets and neighbourhoods. The top-right **×** hides the original-style listing and reveals the town controls.
**Back to links** or **Escape** restores the listing and hides the controls.
Drag the visible town to pan and pinch to zoom in either view. Trackpad scrolling
pans; trackpad pinch zooms. The zoom slider stays in sync and zooms around the
current view. Gestures on the links panel retain normal page scrolling. **Pause** freezes the simulation.
Reduced-motion preferences pause it automatically, and hidden tabs stop rendering.

The controls adjust speed from **0.5× to 8×**, add or remove traffic from **0% to
600%**, and zoom from the **whole town** to **150%**. Zooming and resizing preserve
the current town and its traffic. Traffic changes insert vehicles into safe gaps
and release junction reservations when vehicles are removed.

**Cyclists** sets 0–150 riders independently of motor traffic. Cyclists occupy
lanes, obey junctions and single-track priority, and ride more slowly than cars.
Cars pull out to pass only on a clear, gentle section away from bus stops and
junctions. Oncoming traffic prevents passing; drivers merge back behind the
cyclist if they cannot finish a pass. Roundabout gap checks account for cyclists'
lower speeds.

**People** sets 0–400 residents, including people walking, queuing and riding.
Half keep strolling along pavements throughout town, including streets without
bus stops, walking in both directions and turning back before junctions. The
others walk to a stop, queue for their route, board one at a time, and alight at a
later stop. Each boarding adds 0.85 simulated seconds and each alighting adds
0.65 seconds to the stop. Buses hold 18 passengers; full buses leave remaining
people waiting. A small count appears above a stopped bus. Population changes
preserve the street layout and remove passengers from queues and buses cleanly.
Removing a bus returns its passengers to stops. These are illustrative journeys.

Zebra crossings have flashing amber Belisha beacons. Pavement walkers cross
there and occasionally jaywalk elsewhere. Cars, buses and cyclists stop in both
directions until the crossing is clear; pedestrians wait for vehicles already
too close to stop. More people therefore also means more interruptions to traffic.
Overtaking is disabled on zebra-crossing streets and during a jaywalk.

`town-crossings.mjs` manages crossing requests, clearance and vehicle stop lines.
`town-people.mjs` manages passenger journeys and boarding;
`town-cyclists.mjs` manages opposing-lane reservations for overtaking.

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
