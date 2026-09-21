// People have a persistent journey: pavement -> queue -> bus -> pavement.
const COLOURS = ['#c66a52', '#478994', '#d5a23f', '#775e87', '#577553', '#436387'];
export function startJourney(town, person, queued = false) {
    const routes = town.routes.filter(r => r.path.filter(l => l.stop).length > 1);
    if (!routes.length) return;
    const route = routes[Math.floor(town.random() * routes.length)];
    const stops = [...new Set(route.path.filter(l => l.stop))];
    const index = Math.floor(town.random() * stops.length);
    person.lane = stops[index];
    person.destination = stops[(index + 1 + Math.floor(town.random() * (stops.length - 1))) % stops.length];
    person.route = route.number;
    person.state = queued ? 'queue' : 'walking';
    person.distance = queued ? person.lane.stop.distance : Math.max(5, person.lane.stop.distance * town.random());
    person.bus = null;
    if (queued) person.lane.stop.queue.push(person);
}
function startStroll(town, person) {
    const lanes = town.lanes.filter(l => !l.edge.singleTrack && l.length > 70);
    const lane = lanes[Math.floor(town.random() * lanes.length)];
    if (!lane) { startJourney(town, person); return; }
    Object.assign(person, { state: 'strolling', lane, distance: 10 + town.random() * (lane.length - 20),
        direction: town.random() < 0.5 ? -1 : 1, pause: 0, bus: null });
}

export function setPeopleCount(town, count) {
    count = Math.max(0, Math.min(400, Math.round(Number(count) || 0)));
    town.people ||= []; town.nextPersonId ||= 0;
    for (const lane of town.lanes) if (lane.stop) lane.stop.queue ||= [];
    town.people = town.people.slice(0, count);
    const keep = new Set(town.people);
    for (const lane of town.lanes) if (lane.stop) lane.stop.queue = lane.stop.queue.filter(p => keep.has(p));
    for (const bus of town.vehicles) if (bus.passengers) bus.passengers = bus.passengers.filter(p => keep.has(p));
    while (town.people.length < count) {
        const id = town.nextPersonId++;
        const person = { id, colour: COLOURS[id % COLOURS.length], speed: 7 + town.random() * 4, trips: 0 };
        town.people.push(person);
        if (id % 2 === 0) startStroll(town, person);
        else startJourney(town, person, town.random() < 0.35);
    }
}
export function updatePeople(town, dt) {
    for (const person of town.people || []) {
        if (person.state === 'strolling') {
            // Walk the length of the pavement in both directions, staying clear
            // of junctions. Keep an ongoing population out walking, even when
            // every bus passenger has reached their stop.
            if (person.pause > 0) { person.pause -= dt; continue; }
            person.distance += person.direction * person.speed * dt;
            if (person.distance >= person.lane.length - 8 || person.distance <= 8) {
                person.distance = Math.max(8, Math.min(person.lane.length - 8, person.distance));
                person.direction *= -1;
                person.pause = 0.6 + town.random() * 1.5;
            }
        } else if (person.state === 'walking') {
            person.distance = Math.min(person.lane.stop.distance, person.distance + person.speed * dt);
            if (person.distance === person.lane.stop.distance) {
                person.state = 'queue'; person.lane.stop.queue.push(person);
            }
        } else if (person.state === 'leaving') {
            person.distance += person.speed * dt;
            if (person.distance > Math.min(person.lane.length - 6, person.lane.stop.distance + 75)) startJourney(town, person);
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
        Object.assign(departing, { state: 'leaving', lane: bus.lane, distance: bus.lane.stop.distance + 12, bus: null, trips: departing.trips + 1 });
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
