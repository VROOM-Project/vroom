// Waste transport model: truck types, loading rules, chico attachments,
// operation types and the translation of a planner's day (company +
// operations + fleet) into a VROOM request using the `capacities`,
// `vehicle_groups` and `task_groups` extensions of this fork.
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
// The containers the company has in the yard (docs/waste_defaults.json,
// "container_stock" block, edited in the planner's Config tab) cap how
// many operations may take one out in a day: one `task_groups` entry
// per size, holding the company-outbound shipment of every operation of
// that size. The solver then decides which operations are left for
// another day, priority first, instead of the planner choosing
// beforehand. A size left blank is not tracked and gets no group.
//
// Lunch (docs/waste_defaults.json, "working_day" block) is spent at the
// company: every truck is back and unloaded by the lunch start and
// loads nothing before the lunch end. VROOM cannot pin a break to a
// place, so a truck's day is two shifts instead, morning and
// afternoon, each a VROOM vehicle of its own that starts and ends at
// the company (see shiftsOf and buildRequest). One solve still plans
// the whole day, choosing what goes before and after lunch.
//
// Chicos are trailers that attach to a truck type. A truck with a chico
// carries one allowed load per bed (its own plus `extra_loads` on the
// chico), any mix. The company has a limited number of chicos and the
// solver decides which trucks take one: every truck of a type is
// offered both with and without a chico, and a VROOM vehicle group caps
// the number of vehicles used at the number of physical trucks. The
// group counts per shift, so a chico can go on or come off at the
// company over lunch.
//
// Costs (docs/waste_defaults.json, "costs" block) are what the solver
// minimises once it has decided what gets done. In money, a route costs
// the road it burns and nothing else; see COST_MODEL below for why, and
// solverPerKm for how money turns into what VROOM is actually given.
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
  // `takesContainerOut` marks the operations that need a container from
  // the company yard to start with, and so use up the stock of that
  // size: an exchange leaves an empty behind, materials leave in a
  // container of that size. Picking up a full container does not, it
  // brings one in.
  const OPERATION_TYPES = {
    deliver_empty: { label: "Deliver empty container", short: "deliver empty", needsSize: true, takesContainerOut: true },
    pickup_full: { label: "Pick up full container", short: "pick up full", needsSize: true, takesContainerOut: false },
    exchange: { label: "Exchange empty for full", short: "exchange", needsSize: true, takesContainerOut: true },
    sell_materials: { label: "Sell materials (full container)", short: "materials", needsSize: true, takesContainerOut: true },
  };

  // ---------- the cost model ----------
  //
  //   cost(route) = chico multiplier x cost per km of the truck type x km
  //                 + REFERENCE_PER_HOUR x driving hours
  //
  // The first term is the company's real cost: the drivers are salaried
  // by the month, so their time is spent whether a truck goes out or
  // not and pricing it would trade fuel against money already gone.
  // Time is a hard limit (the working day, lunch at the company, any
  // cap in `limits`), not a price. Service time is not priced either
  // (per_task_hour is 0), and no vehicle carries a fixed cost.
  //
  // The second term is not a business cost and is not editable. It has
  // to be there: VROOM derives its internal "unreachable" sentinel from
  // per_hour alone (`_cost_upper_bound`, see Input::set_matrices in
  // src/structures/vroom/input/input.cpp), so with per_hour at 0 an
  // impossible job/vehicle pair evaluates cheaper than a possible one
  // and both the regret heuristic and insertion ranking go blind. It
  // never biases which truck is used, being identical on all of them;
  // it only mildly prefers less driving between otherwise equal plans.
  const REFERENCE_PER_HOUR = 3600;

  // Money never reaches the solver. Only the ratios between the
  // configurations decide which truck drives, while the absolute level
  // decides something else entirely: how completely the term above is
  // drowned out. So the money figures are normalised, the dearest
  // configuration landing here. Measured on a 20-operation day, the
  // plan found is identical for a dearest-configuration value anywhere
  // between roughly 700 and 10000 and degrades outside that, this
  // sitting in the middle of the safe range.
  const COST_SCALE = 3000;

  // Fallbacks for every value docs/waste_defaults.json may set, so the
  // model still works when it is called without them.
  const BUILTIN_DEFAULTS = {
    company: { lat: 37.030558, lng: -7.976093 },
    fleet: {},
    chicos: {},
    container_stock: {},
    working_day: { start: "08:00", end: "17:00", lunch_start: "12:00", lunch_end: "13:00" },
    operation_times_min: { client_per_container: 10, company_per_visit: 5, company_per_container: 5 },
    costs: { currency: "\u20ac", per_km: {}, chico_multiplier: {} },
    limits: { max_travel_time_min: 0, max_distance_km: 0, max_tasks: 0 },
    solver: { geometry: true, exploration_level: 5, threads: 4, show_request: false },
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

  function nonNegativeNumber(value, fallback) {
    const n = Number(value);
    return isFinite(n) && n >= 0 ? n : fallback;
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

    // Every vehicle configuration the planner can put on the road: a
    // truck type on its own, and the same type with each chico that
    // fits it. Keys use the selector vocabulary of no_go_zones.json
    // ("truck:<type>", "chico:<key>") and address a configuration in
    // the costs of waste_defaults.json. A chico attaches to exactly one
    // truck type, so "chico:<key>" names the pair on its own.
    const VEHICLE_CONFIGS = [];
    for (const type of TYPE_ORDER) {
      VEHICLE_CONFIGS.push({
        key: `truck:${type}`,
        label: TRUCK_TYPES[type].label,
        type,
        chico: null,
      });
      for (const k of CHICO_ORDER) {
        if (CHICO_TYPES[k].attachesTo !== type) continue;
        VEHICLE_CONFIGS.push({
          key: `chico:${k}`,
          label: `${TRUCK_TYPES[type].label} + chico`,
          type,
          chico: k,
        });
      }
    }

    function configKeyOf(type, chico) {
      return chico ? `chico:${chico}` : `truck:${type}`;
    }

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

    // Containers of each size the company has in the yard today, as
    // {size: count}. A null value means the stock is not tracked for
    // that size, i.e. there is always one available.
    function defaultContainerStock() {
      const stock = {};
      const src = D.container_stock || {};
      for (const size of SIZES) {
        const raw = src[size];
        stock[size] = (raw === undefined || raw === null || raw === "")
          ? null
          : nonNegativeInt(raw, 0);
      }
      return stock;
    }

    // How many containers of that size the yard holds, null when the
    // stock is not tracked (no limit).
    function stockFor(stock, size) {
      const raw = (stock || {})[size];
      if (raw === undefined || raw === null || raw === "") return null;
      const n = Number(raw);
      return isFinite(n) && n >= 0 ? Math.floor(n) : null;
    }

    // Whether an operation takes a container out of the company yard,
    // and therefore uses up the stock of its size.
    function takesContainerOut(op) {
      const t = OPERATION_TYPES[op.type];
      return !!(t && t.takesContainerOut);
    }

    // Operations that use up the stock of a given size.
    function stockUsers(operations, size) {
      return (operations || []).filter(
        (op) => takesContainerOut(op) && Number(op.size) === Number(size));
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

    // ---------- costs ----------
    // What a route costs the company, in money:
    //   {currency, per_km: {<truck type>: money},
    //             chico_multiplier: {<chico key>: factor}}
    // A truck type missing a price costs 1 a kilometre and a chico
    // without a multiplier costs its truck nothing extra, so a defaults
    // file that says nothing about costs behaves as it did before they
    // existed.
    function defaultCosts() {
      const src = D.costs || {};
      const perKm = {};
      for (const type of TYPE_ORDER) {
        perKm[type] = nonNegativeNumber((src.per_km || {})[type], 1);
      }
      const multiplier = {};
      for (const key of CHICO_ORDER) {
        multiplier[key] = nonNegativeNumber((src.chico_multiplier || {})[key], 1);
      }
      return {
        currency: src.currency || BUILTIN_DEFAULTS.costs.currency,
        per_km: perKm,
        chico_multiplier: multiplier,
      };
    }

    // What one kilometre costs for one vehicle configuration, in money:
    // its truck type's price, times its chico's multiplier if it has
    // one. This is the number to report a plan's cost with.
    function moneyPerKm(configKey, costs) {
      const c = costs || defaultCosts();
      const cfg = VEHICLE_CONFIGS.find((v) => v.key === configKey);
      if (!cfg) return 0;
      const base = nonNegativeNumber((c.per_km || {})[cfg.type], 0);
      const factor = cfg.chico
        ? nonNegativeNumber((c.chico_multiplier || {})[cfg.chico], 1)
        : 1;
      return base * factor;
    }

    // The money figures as VROOM wants them: one non-negative integer
    // per configuration, normalised so the dearest lands on COST_SCALE
    // and the rest keep their ratios to it. A configuration that costs
    // something is never rounded down to 0, which would make it free to
    // drive; if nothing costs anything, every per_km is 0 and the plan
    // is decided by the reference term alone.
    function solverPerKm(costs) {
      const money = {};
      let dearest = 0;
      for (const cfg of VEHICLE_CONFIGS) {
        money[cfg.key] = moneyPerKm(cfg.key, costs);
        dearest = Math.max(dearest, money[cfg.key]);
      }

      const out = {};
      for (const cfg of VEHICLE_CONFIGS) {
        out[cfg.key] = dearest > 0 && money[cfg.key] > 0
          ? Math.max(1, Math.round((money[cfg.key] / dearest) * COST_SCALE))
          : 0;
      }
      return out;
    }

    // Hard caps per truck, unlike the costs above: a route breaking one
    // of them is not a plan at all. 0 means no limit and the key is
    // then left out of the request. Returned in VROOM's units
    // (seconds, metres) rather than the file's (minutes, km).
    function defaultLimits() {
      const src = D.limits || {};
      return {
        maxTravelTime: minutesToSeconds(src.max_travel_time_min, 0),
        maxDistance: Math.round(nonNegativeNumber(src.max_distance_km, 0) * 1000),
        maxTasks: nonNegativeInt(src.max_tasks, 0),
      };
    }

    function defaultTimes() {
      return {
        dayStart: clockToSeconds(day.start, 8 * 3600),
        dayEnd: clockToSeconds(day.end, 17 * 3600),
        // Lunch, from lunchStart to lunchEnd, is spent at the company:
        // every truck is back and unloaded by lunchStart and loads
        // nothing before lunchEnd. Equal values mean no lunch.
        lunchStart: clockToSeconds(day.lunch_start, 12 * 3600),
        lunchEnd: clockToSeconds(day.lunch_end, 13 * 3600),
        clientService: minutesToSeconds(svc.client_per_container, 600),
        companySetup: minutesToSeconds(svc.company_per_visit, 300),
        companyService: minutesToSeconds(svc.company_per_container, 300),
      };
    }

    // The shifts of a truck's day, each starting and ending at the
    // company: the morning up to lunch and the afternoon from lunch on,
    // or the whole day when there is no lunch. A lunch starting at the
    // day's start (or ending at its end) simply leaves one shift. Lunch
    // is a hard rule, and VROOM has no "break at a location", so this
    // is how it is expressed: one VROOM vehicle per truck configuration
    // and shift, with the shift as its time_window. Being a route end,
    // the lunch start is also when the last unloading must be finished,
    // and being a route start, the lunch end is when loading may begin.
    function shiftsOf(times) {
      const { dayStart, dayEnd, lunchStart, lunchEnd } = times;
      if (!(lunchEnd > lunchStart)) return [{ key: "day", label: "day", start: dayStart, end: dayEnd }];
      const shifts = [];
      if (lunchStart > dayStart) shifts.push({ key: "morning", label: "morning", start: dayStart, end: lunchStart });
      if (dayEnd > lunchEnd) shifts.push({ key: "afternoon", label: "afternoon", start: lunchEnd, end: dayEnd });
      return shifts;
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
    //   stock        {2: n, 6: n, ...}              containers in the yard,
    //                                               null for no limit
    //   times        see defaultTimes()
    //   costs        see defaultCosts()
    //   limits       see defaultLimits()
    //   geometry     bool                          vroom's -g
    //   exploration  0..5                          vroom's -x
    //   threads      int                           vroom's -t
    // Returns {request, stepInfo, vehicleInfo}: stepInfo maps step ids
    // to a description, vehicleInfo maps vehicle ids to {type, chico,
    // profile, shift}, shift being a key of shiftsOf (a physical truck
    // is one vehicle per shift).
    function buildRequest({ depot, operations, fleet, chicos, stock, times, costs, limits,
                            geometry, exploration, threads }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      chicos = Object.assign(defaultChicos(), chicos || {});
      stock = Object.assign(defaultContainerStock(), stock || {});
      times = Object.assign(defaultTimes(), times || {});
      costs = costs || defaultCosts();
      limits = Object.assign(defaultLimits(), limits || {});
      const perKm = solverPerKm(costs);
      const company = [depot.lng, depot.lat];

      const vehicles = [];
      const vehicleGroups = [];
      const vehicleInfo = {};
      let vId = 1;
      let gId = 1;

      // Lunch is spent at the company, so a truck's day is one or two
      // shifts that both start and end there (see shiftsOf). Every
      // configuration of every truck becomes one VROOM vehicle per
      // shift, and VROOM has no break to place any more.
      const shifts = shiftsOf(times);

      const addVehicle = (type, description, capacities, chico, shift, extra) => {
        // The profile is what enforces the no-go zones: it decides which
        // OSRM instance answers for this vehicle, and therefore whether
        // its travel times and its drawn route go through the areas it
        // is not allowed in.
        const profile = profileFor({ type, chico });
        // The whole cost of this configuration is its road: the
        // reference per_hour (see REFERENCE_PER_HOUR, it is the same on
        // every vehicle and is not a business cost), no task cost, no
        // fixed cost, and the normalised price of a kilometre. A
        // configuration priced at nothing sends no per_km at all, which
        // also spares VROOM the distance matrix.
        const perKmForVehicle = perKm[configKeyOf(type, chico)];
        const vehicleCosts = { per_hour: REFERENCE_PER_HOUR, per_task_hour: 0 };
        if (perKmForVehicle > 0) vehicleCosts.per_km = perKmForVehicle;
        const v = {
          id: vId++,
          description: shifts.length > 1 ? `${description}, ${shift.label}` : description,
          type,
          profile,
          start: company,
          end: company,
          capacities,
          // The shift: back at the company, unloaded, by its end, and
          // not loading anything before its start.
          time_window: [shift.start, shift.end],
          costs: vehicleCosts,
          ...extra,
        };
        // The caps in `limits` are per VROOM vehicle, hence per shift.
        if (limits.maxTravelTime > 0) v.max_travel_time = limits.maxTravelTime;
        if (limits.maxDistance > 0) v.max_distance = limits.maxDistance;
        if (limits.maxTasks > 0) v.max_tasks = limits.maxTasks;
        // With no zone at all, no vehicle and no task carries a skill.
        const zoneSkills = zoneSkillsOfProfile(profile);
        if (ZONES.length) v.skills = zoneSkills;
        vehicles.push(v);
        vehicleInfo[v.id] = { type, chico, profile, shift: shift.key };
        return v;
      };

      for (const type of TYPE_ORDER) {
        const n = fleet[type] || 0;
        if (n === 0) continue;
        const label = TRUCK_TYPES[type].label.toLowerCase();
        const offered = chicosOffered(type, fleet, chicos);
        const anyChico = Object.values(offered).some((c) => c > 0);

        for (const shift of shifts) {
          // One group per truck type and shift: plain and chico versions
          // of the trucks together may not exceed the number of physical
          // trucks. Counting per shift is what lets a truck put its chico
          // on or take it off at the company over lunch.
          let groups;
          if (anyChico) {
            const suffix = shifts.length > 1 ? `, ${shift.label}` : "";
            vehicleGroups.push({ id: gId, max_vehicles: n, description: `${label} trucks${suffix}` });
            groups = [gId++];
          }

          for (let i = 1; i <= n; i++) {
            addVehicle(type, `${label} ${i}`, capacitiesFor(type), null, shift,
                       groups ? { groups } : {});
          }
          for (const [chicoKey, count] of Object.entries(offered)) {
            const caps = chicoCapacitiesFor(chicoKey);
            for (let i = 1; i <= count; i++) {
              // What taking this chico costs is its "chico:<key>" entry in
              // the costs, applied by addVehicle like any other override.
              addVehicle(type, `${label} + chico ${i}`, caps, chicoKey, shift, { groups });
            }
          }
        }
      }

      // A container only leaves the company if there is one in the yard.
      // The stock is a plain cap on how many operations of a size can
      // take one out today, which is a `task_groups` entry of this fork
      // (see docs/API.md#task-groups): the company-outbound shipment of
      // every operation of that size joins one group, whose `max_tasks`
      // is the stock. The solver then leaves the ones that do not fit
      // unassigned, choosing which by the usual ranking (priority
      // first, then what the plan costs), instead of the planner
      // picking beforehand.
      const taskGroups = [];
      const stockGroupOfSize = {};
      let tgId = 1;
      for (const size of SIZES) {
        const available = stockFor(stock, size);
        if (available === null || !stockUsers(operations, size).length) continue;
        stockGroupOfSize[size] = tgId;
        taskGroups.push({
          id: tgId++,
          max_tasks: available,
          description: `${size} m³ containers in the yard`,
        });
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
        // The shipment that takes the container out of the yard carries
        // the stock group of its size; the other half of an exchange
        // brings a container in, so it does not.
        const stockGroup = takesContainerOut(op) ? stockGroupOfSize[size] : undefined;
        const push = (s, fromStock) => {
          s.priority = priority;
          if (skills.length) s.skills = skills;
          if (fromStock && stockGroup !== undefined) s.groups = [stockGroup];
          shipments.push(s);
        };

        switch (op.type) {
          case "deliver_empty":
            push({
              amount: oneHot(`e${size}`),
              pickup: atCompany(base + 1, `${tag}: load empty ${size} m³ at company`),
              delivery: atClient(base + 2, op, `${tag}: deliver empty ${size} m³`),
            }, true);
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
            }, true);
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
            }, true);
            break;
          default:
            throw new Error(`unknown operation type ${op.type}`);
        }
      }

      // vroom-express turns these into command-line flags, for the ones
      // its config.yml allows to be overridden (vroom-conf/config.yml,
      // "override"). A distance cap needs distances to exist at all,
      // which -g guarantees; a non-zero per_km asks for them on its own
      // (Input::_profiles_requiring_distances), a max_distance does not.
      const options = { g: !!geometry || limits.maxDistance > 0 };
      if (exploration !== undefined && exploration !== null) {
        options.x = Math.max(0, Math.min(5, nonNegativeInt(exploration, 5)));
      }
      if (threads !== undefined && threads !== null) {
        options.t = Math.max(1, nonNegativeInt(threads, 4));
      }

      const request = { vehicles, shipments, options };
      if (vehicleGroups.length) request.vehicle_groups = vehicleGroups;
      if (taskGroups.length) request.task_groups = taskGroups;
      return { request, stepInfo, vehicleInfo };
    }

    // Sanity checks on a planner's day before building the request.
    // Returns {level, text} entries: an "error" makes the day
    // unplannable as it stands, a "warning" is something the planner
    // should see but that the solver can live with.
    function validate({ depot, operations, fleet, chicos, stock, times }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      chicos = Object.assign(defaultChicos(), chicos || {});
      stock = Object.assign(defaultContainerStock(), stock || {});
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
      } else if (times.dayEnd > times.dayStart && !shiftsOf(times).length) {
        error("Lunch: it covers the whole working day, so no truck could go out.");
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

      // The yard cannot hand out more containers than it holds, so the
      // extra operations are left for another day whatever the fleet
      // does. Worth saying before the plan comes back short.
      for (const size of SIZES) {
        const available = stockFor(stock, size);
        if (available === null) continue;
        const needed = stockUsers(operations, size).length;
        if (needed <= available) continue;
        warning(`${needed} operations need a ${size} m³ container from the yard ` +
                `and only ${available} ${available === 1 ? "is" : "are"} in stock: ` +
                `the solver will leave ${needed - available} of them for another day.`);
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
      REFERENCE_PER_HOUR, COST_SCALE, VEHICLE_CONFIGS,
      parseLoad, oneHot, capacitiesFor, chicoCapacitiesFor, sizesFor,
      defaultFleet, defaultChicos, defaultContainerStock, stockFor,
      takesContainerOut, stockUsers, chicosOffered, defaultTimes, shiftsOf,
      defaultCosts, moneyPerKm, solverPerKm, defaultLimits, configKeyOf,
      profileFor, pointInZone, zonesAt, blockedProfilesAt, describeProfiles,
      opIdOfStep, describe, buildRequest, validate,
    };
  }

  return { create, OPERATION_TYPES };
});
