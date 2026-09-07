// Waste transport model: truck types, loading rules, chico attachments,
// operation types and the translation of a planner's day (company +
// operations + fleet) into a VROOM request using the `capacities` and
// `vehicle_groups` extensions of this fork.
//
// The rules themselves live in docs/waste_rules.json (single source of
// truth), the planner's starting values in docs/waste_defaults.json and
// the forbidden areas in docs/no_go_zones.json, all three next to each
// other. This file only interprets them: build a model with
//   WasteModel.create(rules, defaults, zones)
// where every argument is the parsed JSON (`defaults` and `zones` are
// optional and fall back to the built-in values below). In the browser
// the server exposes them as window.WASTE_RULES (/rules.js),
// window.WASTE_DEFAULTS (/defaults.js) and window.WASTE_ZONES
// (/zones.js); under node use
//   require("./waste_model").create(require("../../docs/waste_rules.json"),
//                                   require("../../docs/waste_defaults.json"),
//                                   require("../../docs/no_go_zones.json"))
//
// Amount components are container kinds, one-hot per task:
//   e2 e6 ...  empty containers by size (m3), one per size in rules.sizes
//   f2 f6 ...  full containers by size
// Materials are sold in a container of a standard size that leaves the
// company full, so for the loading rules they are a full container.
//
// Chicos are trailers that attach to a truck type. A truck with a chico
// carries one allowed load per bed (its own plus `extra_loads` on the
// chico), any mix. The company has a limited number of chicos and the
// solver decides which trucks take one: every truck of a type is
// offered both with and without a chico, and a VROOM vehicle group caps
// the number of vehicles used at the number of physical trucks.
//
// No-go zones (docs/no_go_zones.json, third argument of create, exposed
// in the browser as window.WASTE_ZONES) are areas some vehicles may not
// drive through. VROOM only ever sees travel times, so a zone is not a
// solver constraint: it is enforced in the road graph, by giving the
// restricted vehicles a routing `profile` served by an OSRM instance in
// which the roads inside the zones are prohibitively slow. This file
// only decides which vehicle uses which profile, and keeps operations
// inside a zone away from the vehicles that cannot reach them, through
// one skill per zone.
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.WasteModel = factory();
})(typeof self !== "undefined" ? self : this, function () {
  // Planner-facing operation types.
  const OPERATION_TYPES = {
    deliver_empty: { label: "Deliver empty container", short: "deliver empty", needsSize: true },
    pickup_full: { label: "Pick up full container", short: "pick up full", needsSize: true },
    exchange: { label: "Exchange empty for full", short: "exchange", needsSize: true },
    sell_materials: { label: "Sell materials (full container)", short: "materials", needsSize: true },
  };

  // Fallbacks for every value docs/waste_defaults.json may set, so the
  // model still works when it is called without them.
  const BUILTIN_DEFAULTS = {
    company: { lat: 37.030558, lng: -7.976093 },
    fleet: {},
    chicos: {},
    working_day: { start: "08:00", end: "17:00", lunch_start: "12:00", lunch_end: "13:00" },
    operation_times_min: { client_per_container: 10, company_per_visit: 5, company_per_container: 5 },
    solver: { geometry: true, show_request: false },
  };

  // Fallback when no zone file is given: a single unrestricted profile,
  // which is exactly the behaviour before no-go zones existed.
  const BUILTIN_ZONES = {
    profiles: { car: { description: "default profile: no area restriction" } },
    vehicle_profiles: { rules: [], default: "car" },
    zones: [],
  };

  // "08:30" -> 30600. Also accepts a plain number of seconds.
  function clockToSeconds(value, fallback) {
    if (typeof value === "number" && isFinite(value)) return value;
    const m = /^(\d{1,2}):(\d{2})$/.exec(String(value || "").trim());
    if (!m) return fallback;
    return Number(m[1]) * 3600 + Number(m[2]) * 60;
  }

  function minutesToSeconds(value, fallback) {
    const n = Number(value);
    return isFinite(n) && n >= 0 ? Math.round(n * 60) : fallback;
  }

  function nonNegativeInt(value, fallback) {
    const n = Number(value);
    return isFinite(n) && n >= 0 ? Math.floor(n) : fallback;
  }

  function create(rules, defaults, zoneConfig) {
    if (!rules || !rules.trucks || !rules.sizes) {
      throw new Error("waste rules missing: expected {sizes, trucks}");
    }
    const D = defaults || {};
    const day = { ...BUILTIN_DEFAULTS.working_day, ...(D.working_day || {}) };
    const svc = { ...BUILTIN_DEFAULTS.operation_times_min, ...(D.operation_times_min || {}) };
    const SOLVER_DEFAULTS = { ...BUILTIN_DEFAULTS.solver, ...(D.solver || {}) };
    const SIZES = rules.sizes.slice();
    const KINDS = [...SIZES.map((s) => `e${s}`), ...SIZES.map((s) => `f${s}`)];
    const TRUCK_TYPES = {};
    for (const [key, t] of Object.entries(rules.trucks)) {
      TRUCK_TYPES[key] = { label: t.label || key, rules: t.rules.slice() };
    }
    const TYPE_ORDER = Object.keys(TRUCK_TYPES);

    // Chico attachments: which truck type each one fits and how many
    // extra loads it adds.
    const CHICO_TYPES = {};
    for (const [key, c] of Object.entries(rules.chicos || {})) {
      if (key.startsWith("_")) continue;
      if (!TRUCK_TYPES[c.attaches_to]) {
        throw new Error(`chico ${key}: unknown truck type ${c.attaches_to}`);
      }
      CHICO_TYPES[key] = {
        label: c.label || key,
        attachesTo: c.attaches_to,
        extraLoads: nonNegativeInt(c.extra_loads, 2),
      };
    }
    const CHICO_ORDER = Object.keys(CHICO_TYPES);

    // ---------- no-go zones and routing profiles ----------
    // A zone is an area a routing profile may not enter. It is enforced
    // outside the solver, in the OSRM dataset that profile is served
    // from (scripts/build_zone_graphs.sh); here it decides two things:
    // which profile a vehicle uses, and which vehicles may serve an
    // operation that sits inside a zone.
    const Z = { ...BUILTIN_ZONES, ...(zoneConfig || {}) };
    const PROFILES = {};
    for (const [key, def] of Object.entries(Z.profiles || {})) {
      if (key.startsWith("_")) continue;
      PROFILES[key] = {
        description: (def && def.description) || key,
        hostPort: def && def.host_port,
      };
    }
    if (!Object.keys(PROFILES).length) {
      throw new Error("no_go_zones.json: at least one routing profile is needed");
    }

    const PROFILE_RULES = ((Z.vehicle_profiles || {}).rules || []).filter((r) => r && r.match);
    const DEFAULT_PROFILE = (Z.vehicle_profiles || {}).default || Object.keys(PROFILES)[0];
    for (const name of [DEFAULT_PROFILE, ...PROFILE_RULES.map((r) => r.profile)]) {
      if (!PROFILES[name]) {
        throw new Error(`no_go_zones.json: vehicle_profiles names unknown profile ${name}`);
      }
    }

    // Zones that actually forbid something. A zone blocking no profile,
    // or only profiles nothing uses, is inert and simply ignored.
    const ZONES = (Z.zones || [])
      .filter((z) => z && Array.isArray(z.polygon) && (z.blocked_profiles || []).length)
      .map((z) => ({
        id: Number(z.id),
        name: String(z.name || `zone ${z.id}`),
        blockedProfiles: z.blocked_profiles.slice(),
        polygon: z.polygon.map((pt) => [Number(pt[0]), Number(pt[1])]),
      }));
    for (const z of ZONES) {
      for (const name of z.blockedProfiles) {
        if (!PROFILES[name]) {
          throw new Error(`no_go_zones.json: zone ${z.id} blocks unknown profile ${name}`);
        }
      }
    }

    // Which routing profile a vehicle configuration uses: the first
    // matching rule wins. Selectors are "chico" (any chico),
    // "chico:<key>" and "truck:<type>"; adding a selector here is all it
    // takes to restrict another kind of vehicle.
    function matchesSelector(selector, vehicle) {
      if (selector === "chico") return !!vehicle.chico;
      if (selector.startsWith("chico:")) return vehicle.chico === selector.slice(6);
      if (selector.startsWith("truck:")) return vehicle.type === selector.slice(6);
      throw new Error(`no_go_zones.json: unknown vehicle selector ${selector}`);
    }

    // Profiles that some zone actually forbids something to. A profile
    // nothing is blocked from is the default profile in disguise, and
    // using it would mean asking for a routing server that need not even
    // be running: with no zone drawn, every vehicle stays on the default
    // one and the extra OSRM instance is unnecessary.
    const RESTRICTED_PROFILES = new Set(ZONES.flatMap((z) => z.blockedProfiles));

    function profileFor(vehicle) {
      for (const rule of PROFILE_RULES) {
        if (matchesSelector(rule.match, vehicle)) {
          return RESTRICTED_PROFILES.has(rule.profile) ? rule.profile : DEFAULT_PROFILE;
        }
      }
      return DEFAULT_PROFILE;
    }

    // Ray casting on the [lng, lat] ring; a point on the very edge may
    // fall either way, which is irrelevant at the scale of a drawn area.
    function pointInZone(zone, lng, lat) {
      const ring = zone.polygon;
      let inside = false;
      for (let i = 0, j = ring.length - 1; i < ring.length; j = i++) {
        const [xi, yi] = ring[i];
        const [xj, yj] = ring[j];
        if ((yi > lat) !== (yj > lat) &&
            lng < ((xj - xi) * (lat - yi)) / (yj - yi) + xi) {
          inside = !inside;
        }
      }
      return inside;
    }

    function zonesAt(lng, lat) {
      return ZONES.filter((z) => pointInZone(z, lng, lat));
    }

    // Profiles that cannot reach a point, i.e. that are blocked by at
    // least one of the zones covering it.
    function blockedProfilesAt(lng, lat) {
      const blocked = new Set();
      for (const z of zonesAt(lng, lat)) {
        for (const p of z.blockedProfiles) blocked.add(p);
      }
      return [...blocked];
    }

    // A zone becomes a mandatory skill: the operations inside it require
    // it, and only the vehicles whose profile may enter it hold it. So a
    // truck that cannot reach a client is never even considered for it,
    // instead of being merely discouraged by the penalised travel times.
    function zoneSkillsAt(lng, lat) {
      return zonesAt(lng, lat).map((z) => z.id);
    }

    function zoneSkillsOfProfile(profile) {
      return ZONES.filter((z) => !z.blockedProfiles.includes(profile)).map((z) => z.id);
    }

    // The company site is a planner setting, not a loading rule: it comes
    // from waste_defaults.json (rules.company is still honoured so older
    // rule files keep working).
    const site = D.company || rules.company || BUILTIN_DEFAULTS.company;
    const COMPANY = { lat: Number(site.lat), lng: Number(site.lng) };
    if (!isFinite(COMPANY.lat) || !isFinite(COMPANY.lng)) {
      throw new Error("waste defaults: company must have numeric lat and lng");
    }

    function zeros() {
      return KINDS.map(() => 0);
    }

    function oneHot(kind) {
      const v = zeros();
      const i = KINDS.indexOf(kind);
      if (i < 0) throw new Error(`unknown container kind ${kind}`);
      v[i] = 1;
      return v;
    }

    // "2f6 + 1e2" -> count vector over KINDS.
    function parseLoad(spec) {
      const v = zeros();
      for (const part of spec.replace(/\s+/g, "").split("+")) {
        if (!part) continue;
        const m = /^(\d*)([a-z]+\d*)$/.exec(part);
        if (!m) throw new Error(`bad load spec ${spec}`);
        const i = KINDS.indexOf(m[2]);
        if (i < 0) throw new Error(`unknown container kind ${m[2]} in ${spec}`);
        v[i] += m[1] ? Number(m[1]) : 1;
      }
      return v;
    }

    // Validate every rule once so a typo in the JSON fails loudly.
    for (const t of Object.values(TRUCK_TYPES)) t.rules.forEach(parseLoad);

    // Capacity vectors of a truck type, one per rule.
    function capacitiesFor(type) {
      return TRUCK_TYPES[type].rules.map(parseLoad);
    }

    const leq = (a, b) => a.every((x, i) => x <= b[i]);
    const addVec = (a, b) => a.map((x, i) => x + b[i]);

    // Drop duplicates and vectors dominated by another one (the solver
    // does it too; this keeps the request small).
    function maximal(vectors) {
      const out = [];
      for (const v of vectors) {
        if (out.some((o) => leq(v, o))) continue;
        for (let i = out.length - 1; i >= 0; i--) if (leq(out[i], v)) out.splice(i, 1);
        out.push(v);
      }
      return out;
    }

    // Capacity vectors of a truck type with a chico: any (extraLoads + 1)
    // allowed loads together, one per bed.
    function chicoCapacitiesFor(chicoKey) {
      const c = CHICO_TYPES[chicoKey];
      const base = capacitiesFor(c.attachesTo);
      let sums = base.map((v) => v.slice());
      for (let k = 0; k < c.extraLoads; k++) {
        const next = [];
        for (const s of sums) for (const b of base) next.push(addVec(s, b));
        sums = maximal(next);
      }
      return sums;
    }

    // Which container sizes a truck type can carry at all.
    function sizesFor(type) {
      const caps = capacitiesFor(type);
      return SIZES.filter((s) => caps.some((c) => c[KINDS.indexOf(`e${s}`)] > 0 || c[KINDS.indexOf(`f${s}`)] > 0));
    }

    function defaultFleet() {
      const fleet = {};
      for (const t of TYPE_ORDER) fleet[t] = nonNegativeInt((D.fleet || {})[t], 1);
      return fleet;
    }

    // Number of chicos of each type available today.
    function defaultChicos() {
      const chicos = {};
      for (const k of CHICO_ORDER) {
        const entry = (D.chicos || {})[k];
        chicos[k] = nonNegativeInt(entry && typeof entry === "object" ? entry.count : entry, 0);
      }
      return chicos;
    }

    // Fixed cost charged once when a truck goes out with this chico
    // (VROOM's costs.fixed, same unit as the other costs: one hour of
    // driving is 3600). Set in docs/waste_defaults.json; 0 lets the
    // solver take a chico whenever it helps.
    function chicoCost(chicoKey) {
      const entry = (D.chicos || {})[chicoKey];
      return nonNegativeInt(entry && typeof entry === "object" ? entry.cost : 0, 0);
    }

    function defaultTimes() {
      return {
        dayStart: clockToSeconds(day.start, 8 * 3600),
        dayEnd: clockToSeconds(day.end, 17 * 3600),
        // Lunch: a fixed break from lunchStart to lunchEnd. Equal values
        // mean no lunch.
        lunchStart: clockToSeconds(day.lunch_start, 12 * 3600),
        lunchEnd: clockToSeconds(day.lunch_end, 13 * 3600),
        clientService: minutesToSeconds(svc.client_per_container, 600),
        companySetup: minutesToSeconds(svc.company_per_visit, 300),
        companyService: minutesToSeconds(svc.company_per_container, 300),
      };
    }

    // Step ids are derived from the operation id so that solution steps
    // can be mapped back: operation k uses ids 10k+1 .. 10k+4.
    function opIdOfStep(stepId) {
      return Math.floor(stepId / 10);
    }

    function describe(op) {
      const t = OPERATION_TYPES[op.type];
      return t.needsSize ? `${t.short} ${op.size} m³` : t.short;
    }

    // Chicos offered for a truck type: never more than the trucks.
    function chicosOffered(type, fleet, chicos) {
      const offered = {};
      for (const k of CHICO_ORDER) {
        if (CHICO_TYPES[k].attachesTo !== type) continue;
        offered[k] = Math.min(chicos[k] || 0, fleet[type] || 0);
      }
      return offered;
    }

    // Build the VROOM request.
    //   depot        {lat, lng}                     the company
    //   operations   [{id, lat, lng, type, size, priority}]
    //   fleet        {small: n, multiban: n, poliban: n}
    //   chicos       {multiban: n, poliban: n}      chicos available
    //   times        see defaultTimes()
    //   geometry     bool
    // Returns {request, stepInfo, vehicleInfo}: stepInfo maps step ids
    // to a description, vehicleInfo maps vehicle ids to {type, chico}.
    function buildRequest({ depot, operations, fleet, chicos, times, geometry }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      chicos = Object.assign(defaultChicos(), chicos || {});
      times = Object.assign(defaultTimes(), times || {});
      const company = [depot.lng, depot.lat];

      const vehicles = [];
      const vehicleGroups = [];
      const vehicleInfo = {};
      let vId = 1;
      let gId = 1;

      const addVehicle = (type, description, capacities, chico, extra) => {
        // The profile is what enforces the no-go zones: it decides which
        // OSRM instance answers for this vehicle, and therefore whether
        // its travel times and its drawn route go through the areas it
        // is not allowed in.
        const profile = profileFor({ type, chico });
        const v = {
          id: vId++,
          description,
          type,
          profile,
          start: company,
          end: company,
          capacities,
          time_window: [times.dayStart, times.dayEnd],
          costs: { per_hour: 3600, per_task_hour: 3600 },
          ...extra,
        };
        // With no zone at all, no vehicle and no task carries a skill.
        const zoneSkills = zoneSkillsOfProfile(profile);
        if (ZONES.length) v.skills = zoneSkills;
        if (times.lunchEnd > times.lunchStart) {
          // VROOM break time windows bound the break start: a single
          // instant makes the lunch fixed.
          v.breaks = [{
            id: 1,
            description: "lunch",
            time_windows: [[times.lunchStart, times.lunchStart]],
            service: times.lunchEnd - times.lunchStart,
          }];
        }
        vehicles.push(v);
        return v;
      };

      for (const type of TYPE_ORDER) {
        const n = fleet[type] || 0;
        if (n === 0) continue;
        const label = TRUCK_TYPES[type].label.toLowerCase();
        const offered = chicosOffered(type, fleet, chicos);
        const anyChico = Object.values(offered).some((c) => c > 0);

        // One group per truck type: plain and chico versions of the
        // trucks together may not exceed the number of physical trucks.
        let groups;
        if (anyChico) {
          vehicleGroups.push({ id: gId, max_vehicles: n, description: `${label} trucks` });
          groups = [gId++];
        }

        for (let i = 1; i <= n; i++) {
          const v = addVehicle(type, `${label} ${i}`, capacitiesFor(type), null,
                               groups ? { groups } : {});
          vehicleInfo[v.id] = { type, chico: null, profile: v.profile };
        }
        for (const [chicoKey, count] of Object.entries(offered)) {
          const caps = chicoCapacitiesFor(chicoKey);
          const cost = chicoCost(chicoKey);
          for (let i = 1; i <= count; i++) {
            const v = addVehicle(type, `${label} + chico ${i}`, caps, chicoKey, {
              groups,
              costs: { fixed: cost, per_hour: 3600, per_task_hour: 3600 },
            });
            vehicleInfo[v.id] = { type, chico: chicoKey, profile: v.profile };
          }
        }
      }

      const shipments = [];
      const stepInfo = {}; // step id -> text

      const atCompany = (id, text) => {
        stepInfo[id] = text;
        return { id, location: company, setup: times.companySetup, service: times.companyService, description: text };
      };
      const atClient = (id, op, text) => {
        stepInfo[id] = text;
        return { id, location: [op.lng, op.lat], service: times.clientService, description: text };
      };

      for (const op of operations) {
        const base = op.id * 10;
        const priority = Math.max(0, Math.min(100, Number(op.priority) || 0));
        const size = op.size;
        const tag = `op ${op.id}`;
        // An operation inside a zone requires that zone's skill, which
        // only the vehicles allowed in carry: the ones that cannot get
        // there are excluded outright rather than merely discouraged by
        // the travel times. Shipment skills cover both of its steps, and
        // the company end of a shipment is checked separately (a company
        // inside a zone is a configuration error, see validate).
        const skills = zoneSkillsAt(op.lng, op.lat);
        const push = (s) => {
          s.priority = priority;
          if (skills.length) s.skills = skills;
          shipments.push(s);
        };

        switch (op.type) {
          case "deliver_empty":
            push({
              amount: oneHot(`e${size}`),
              pickup: atCompany(base + 1, `${tag}: load empty ${size} m³ at company`),
              delivery: atClient(base + 2, op, `${tag}: deliver empty ${size} m³`),
            });
            break;
          case "pickup_full":
            push({
              amount: oneHot(`f${size}`),
              pickup: atClient(base + 1, op, `${tag}: pick up full ${size} m³`),
              delivery: atCompany(base + 2, `${tag}: empty full ${size} m³ at company`),
            });
            break;
          case "exchange":
            push({
              amount: oneHot(`e${size}`),
              pickup: atCompany(base + 1, `${tag}: load empty ${size} m³ at company`),
              delivery: atClient(base + 2, op, `${tag}: leave empty ${size} m³ (exchange)`),
            });
            push({
              amount: oneHot(`f${size}`),
              pickup: atClient(base + 3, op, `${tag}: pick up full ${size} m³ (exchange)`),
              delivery: atCompany(base + 4, `${tag}: empty full ${size} m³ at company`),
            });
            break;
          case "sell_materials":
            // A container of materials leaves the company full and stays
            // at the client: a full container for the loading rules.
            push({
              amount: oneHot(`f${size}`),
              pickup: atCompany(base + 1, `${tag}: load materials (${size} m³ container) at company`),
              delivery: atClient(base + 2, op, `${tag}: deliver materials (${size} m³ container)`),
            });
            break;
          default:
            throw new Error(`unknown operation type ${op.type}`);
        }
      }

      const request = { vehicles, shipments, options: { g: !!geometry } };
      if (vehicleGroups.length) request.vehicle_groups = vehicleGroups;
      return { request, stepInfo, vehicleInfo };
    }

    // Sanity checks on a planner's day before building the request.
    // Returns {level, text} entries: an "error" makes the day
    // unplannable as it stands, a "warning" is something the planner
    // should see but that the solver can live with.
    function validate({ depot, operations, fleet, chicos, times }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      chicos = Object.assign(defaultChicos(), chicos || {});
      times = Object.assign(defaultTimes(), times || {});
      const found = [];
      const error = (text) => found.push({ level: "error", text });
      const warning = (text) => found.push({ level: "warning", text });

      if (times.dayEnd <= times.dayStart) {
        error("Working day: the end must be after the start.");
      }
      if (times.lunchEnd < times.lunchStart) {
        error("Lunch: the end must not be before the start.");
      } else if (times.lunchEnd > times.lunchStart &&
                 (times.lunchStart < times.dayStart || times.lunchEnd > times.dayEnd)) {
        error("Lunch: must be inside the working day.");
      }
      const total = TYPE_ORDER.reduce((n, t) => n + (fleet[t] || 0), 0);
      if (total === 0) error("The fleet is empty: set at least one truck in Config.");

      // The company is the start, the end and every unloading stop of
      // every route, so a restricted profile whose depot is inside its
      // own no-go zone can do nothing sensible at all.
      if (depot) {
        for (const z of zonesAt(depot.lng, depot.lat)) {
          error(`The company site is inside the no-go zone "${z.name}": ` +
                `${describeProfiles(z.blockedProfiles)} could not leave it. ` +
                "Move the company or the zone.");
        }
      }

      for (const op of operations) {
        // Vehicle configurations of today's fleet that could carry this
        // container, as routing profiles.
        const able = [];
        for (const t of TYPE_ORDER) {
          if (!fleet[t] || !sizesFor(t).includes(op.size)) continue;
          able.push(profileFor({ type: t, chico: null }));
          for (const [key, count] of Object.entries(chicosOffered(t, fleet, chicos))) {
            if (count > 0) able.push(profileFor({ type: t, chico: key }));
          }
        }
        if (!able.length) {
          error(`Operation ${op.id}: no truck in the fleet can carry a ${op.size} m³ container.`);
          continue;
        }
        const here = zonesAt(op.lng, op.lat);
        if (!here.length) continue;
        const blocked = blockedProfilesAt(op.lng, op.lat);
        const names = here.map((z) => `"${z.name}"`).join(", ");
        if (able.every((profile) => blocked.includes(profile))) {
          error(`Operation ${op.id} is inside the no-go zone ${names} and no truck ` +
                `of the fleet that can carry a ${op.size} m³ container is allowed in.`);
        } else if (able.some((profile) => blocked.includes(profile))) {
          warning(`Operation ${op.id} is inside the no-go zone ${names}: ` +
                  `${describeProfiles(blocked)} cannot serve it, so it is left to the others.`);
        }
      }
      return found;
    }

    // "trucks going out with a chico" rather than "chico": the
    // descriptions come from no_go_zones.json.
    function describeProfiles(names) {
      const labels = names.map((n) => (PROFILES[n] ? PROFILES[n].description : n));
      if (labels.length <= 1) return labels[0] || "no vehicle";
      return `${labels.slice(0, -1).join(", ")} and ${labels[labels.length - 1]}`;
    }

    return {
      SIZES, KINDS, TRUCK_TYPES, TYPE_ORDER, CHICO_TYPES, CHICO_ORDER,
      OPERATION_TYPES, COMPANY, SOLVER_DEFAULTS,
      ZONES, PROFILES, DEFAULT_PROFILE,
      parseLoad, oneHot, capacitiesFor, chicoCapacitiesFor, sizesFor,
      defaultFleet, defaultChicos, chicoCost, chicosOffered, defaultTimes,
      profileFor, pointInZone, zonesAt, blockedProfilesAt, describeProfiles,
      opIdOfStep, describe, buildRequest, validate,
    };
  }

  return { create, OPERATION_TYPES };
});
