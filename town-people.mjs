import { planWalk, followWalk } from './town-walking.mjs';
import { updateCrossings, clearUnusedCrossings } from './town-crossings.mjs';
// People have a persistent journey: pavement -> queue -> bus -> pavement.
const COLOURS = ['#c66a52', '#478994', '#d5a23f', '#775e87', '#577553', '#436387'];
export function startJourney(town, person, queued = false) {
    const routes = town.routes.filter(r => r.path.filter(l => l.stop).length > 1);
    if (!routes.length) return;
    const route = routes[Math.floor(town.random() * routes.length)];
    const stops = [...new Set(route.path.filter(l => l.stop))];
    const index = Math.floor(town.random() * stops.length);
    const boardingLane = stops[index];
    if (person.lane && !queued && planWalk(town, person, boardingLane)) {
        person.destination = stops[(index + 1 + Math.floor(town.random() * (stops.length - 1))) % stops.length];
        person.route = route.number; person.state = 'walking'; person.bus = null;
        return;
    }
    if (person.lane && !queued) {
        person.state = 'strolling'; planWalk(town, person); return;
    }
    person.lane = boardingLane;
    person.destination = stops[(index + 1 + Math.floor(town.random() * (stops.length - 1))) % stops.length];
    person.route = route.number;
    person.state = queued ? 'queue' : 'walking';
    person.distance = queued ? person.lane.stop.distance : Math.max(5, person.lane.stop.distance * town.random());
    person.bus = null; person.walkRoute = null;
    if (queued) person.lane.stop.queue.push(person);
}
function startStroll(town, person) {
    const lanes = town.lanes.filter(l => !l.edge.singleTrack && l.length > 70);
    const lane = lanes[Math.floor(town.random() * lanes.length)];
    if (!lane) { startJourney(town, person); return; }
    Object.assign(person, { state: 'strolling', lane, distance: 10 + town.random() * (lane.length - 20),
        direction: town.random() < 0.5 ? -1 : 1, pause: 0, bus: null, crossCooldown: 3 + town.random() * 12 });
    planWalk(town, person);
}

export function setPeopleCount(town, count) {
    count = Math.max(0, Math.min(400, Math.round(Number(count) || 0)));
    town.people ||= []; town.nextPersonId ||= 0;
    for (const lane of town.lanes) if (lane.stop) lane.stop.queue ||= [];
    town.people = town.people.slice(0, count);
    const keep = new Set(town.people);
    for (const lane of town.lanes) if (lane.stop) lane.stop.queue = lane.stop.queue.filter(p => keep.has(p));
    for (const bus of town.vehicles) if (bus.passengers) bus.passengers = bus.passengers.filter(p => keep.has(p));
    clearUnusedCrossings(town);
    while (town.people.length < count) {
        const id = town.nextPersonId++;
        const person = { id, colour: COLOURS[id % COLOURS.length], speed: 7 + town.random() * 4, trips: 0 };
        town.people.push(person);
        if (id % 2 === 0) startStroll(town, person);
        else startJourney(town, person, town.random() < 0.35);
    }
}
export function updatePeople(town, dt) {
    updateCrossings(town, dt);
    for (const person of town.people || []) {
        if (person.crossing) continue;
        if (person.state === 'strolling') {
            if (person.pause > 0) { person.pause -= dt; continue; }
            if (!person.walkRoute && !planWalk(town, person)) continue;
            if (followWalk(town, person, dt)) {
                person.walkTrips = (person.walkTrips || 0) + 1;
                person.pause = 2 + town.random() * 5;
                planWalk(town, person);
            }
        } else if (person.state === 'walking') {
            const arrived = person.walkRoute ? followWalk(town, person, dt) :
                (person.distance = Math.min(person.lane.stop.distance, person.distance + person.speed * dt)) === person.lane.stop.distance;
            if (arrived) {
                person.walkRoute = null; person.state = 'queue'; person.lane.stop.queue.push(person);
            }
        } else if (person.state === 'leaving') {
            if (!person.walkRoute) planWalk(town, person);
            if (person.walkRoute && followWalk(town, person, dt)) {
                person.walkTrips = (person.walkTrips || 0) + 1;
                person.walkRoute = null; startJourney(town, person);
            }
        }
    }
}
export function unloadRemovedBus(town, bus) {
    for (const person of bus.passengers || []) startJourney(town, person, true);
    bus.passengers = [];
}
export function serveBus(town, bus, dt) {
    bus.passengers ||= [];
    bus.capacity ||= 18;
    if (!bus.service) bus.service = { clock: 2.2 + town.random() * 1.5, elapsed: 0 };
    const service = bus.service;
    service.elapsed += dt; service.clock -= dt;
    bus.dwell = Math.max(0.01, service.clock);
    if (service.clock > 0) return;
    const departing = bus.passengers.find(p => p.destination === bus.lane);
    if (departing) {
        bus.passengers.splice(bus.passengers.indexOf(departing), 1);
        Object.assign(departing, { state: 'leaving', walkRoute: null, walkPose: null, lane: bus.lane, distance: bus.lane.stop.distance + 12, bus: null, trips: departing.trips + 1 });
        service.clock += 0.65;
        return;
    }
    const queue = bus.lane.stop.queue || [];
    const index = queue.findIndex(p => p.route === bus.route.number);
    if (index >= 0 && bus.passengers.length < bus.capacity) {
        const person = queue.splice(index, 1)[0];
        person.state = 'riding'; person.bus = bus;
        bus.passengers.push(person); bus.boarded = (bus.boarded || 0) + 1;
        service.clock += 0.85; // Boarding is sequential: longer queues mean longer dwell.
    } else {
        bus.lastDwell = service.elapsed;
        bus.service = null; bus.served = true; bus.dwell = 0; bus.stopsVisited++;
    }
}
