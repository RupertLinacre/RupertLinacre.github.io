// A pass owns the opposing lane until the driver has merged back in. No passing
// at stops, tight bends or single tracks, and never into oncoming traffic.
export function clearOvertake(vehicle) {
    if (vehicle.lane.edge.passing === vehicle) vehicle.lane.edge.passing = null;
    vehicle.overtake = null;
}
export function updateOvertakes(town, occupied, dt) {
    for (const car of town.vehicles) {
        if (!car.overtake) continue;
        const pass = car.overtake;
        pass.elapsed = (pass.elapsed || 0) + dt;
        if (pass.phase === 'out') {
            pass.offset = Math.min(1, pass.offset + dt);
            if (pass.offset === 1) pass.phase = 'along';
        } else if (pass.phase === 'along') {
            if (!town.vehicles.includes(pass.bike) || pass.bike.lane !== car.lane || pass.bike.phase !== 'lane' || car.distance - pass.bike.distance > (car.length + pass.bike.length) / 2 + 30) pass.phase = 'in';
            else if (car.distance > car.lane.length - 90 || pass.elapsed > 12) { pass.phase = 'abort'; pass.aborting = true; }
        } else if (pass.phase === 'abort') {
            if (!town.vehicles.includes(pass.bike) || pass.bike.lane !== car.lane || pass.bike.phase !== 'lane' || pass.bike.distance - car.distance > (car.length + pass.bike.length) / 2 + 18) pass.phase = 'in';
        } else {
            pass.offset = Math.max(0, pass.offset - dt);
            if (!pass.offset) { clearOvertake(car); if (!pass.aborting) town.overtakes = (town.overtakes || 0) + 1; }
        }
    }
    for (const [lane, queue] of occupied) {
        if (lane.edge.singleTrack || lane.edge.passing || lane.stop || lane.length < 230) continue;
        for (let i = 1; i < queue.length; i++) {
            const car = queue[i], bike = queue[i - 1];
            if (car.bus || car.cyclist || car.overtake || !bike.cyclist || car.reserved ||
                car.distance < 35 || bike.distance - car.distance > 60 ||
                lane.length - bike.distance < 190 || Math.min(car.maxSpeed, lane.edge.speed) < bike.maxSpeed + 9) continue;
            // Keep the whole manoeuvre clear, including the space to merge ahead.
            if (queue.some(v => v !== bike && v !== car && v.distance > car.distance && v.distance < bike.distance + 180)) continue;
            if (town.vehicles.some(v => v !== car && (v.phase === 'lane' && v.lane.edge === lane.edge && v.lane !== lane ||
                (v.phase === 'turn' || v.reserved) && v.next.edge === lane.edge))) continue;
            const section = lane.path.points.filter(p => p.distance >= car.distance && p.distance < bike.distance + 180);
            if (section.some(p => Math.cos(p.angle - section[0].angle) < 0.9)) continue;
            car.overtake = { bike, phase: 'out', offset: 0 };
            lane.edge.passing = car;
            break;
        }
    }
}
export function passingPair(a, b) {
    const pass = a.overtake?.bike === b ? a.overtake : b.overtake?.bike === a ? b.overtake : null;
    return pass && pass.offset >= 0.85;
}
