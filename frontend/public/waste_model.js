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
// An early start (docs/waste_defaults.json, "working_day" block) lets a
// truck go out before the working day begins. It extends the day at the
// front and never moves its end, so it is time worked on top of the day
// and it is priced (costs.early_start_per_hour, one price for the whole
// fleet) rather than merely allowed. How much earlier is per truck
// type, since which drivers can come in early is not a fleet-wide
// fact; a type allowed none is offered none and costs nothing in
// solving time. Only the first shift can start early: the afternoon
// begins when lunch ends. Which trucks come in early is the solver's to
// decide, the same way it decides chicos — every truck is offered its
// normal start and two earlier ones (see earlyStartsOf), each a vehicle
// of its own carrying the price of its earliness as a VROOM `fixed`
// cost, and the vehicle group caps them all at the number of physical
// trucks. Jobs carry no time window of their own, so a vehicle offered
// an earlier window really does leave at it: the smallest early start
// that does the work is also the cheapest, so that is the one taken.
//
// What it will not do, today, is buy an operation with an early start.
// A vehicle group decides which of a truck's versions is used in the
// heuristic that starts a search and never revisits it: once the group
// is full the local search cannot open another member, and going from
// "this truck, normal start" to "this truck, an hour early and four
// more tasks" would have to pass through "this truck, an hour early
// and the same tasks", which costs more and is refused. So an early
// start is taken when it saves kilometres and not when it would only
// let more work be done, even though more work assigned outranks any
// cost. The same blind spot decides which trucks take a chico, so it
// is the vehicle group mechanism rather than anything here; fixing it
// is solver work.
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
//
// The day's operations also read and write as a CSV
// (parseOperationsCsv, operationsToCsv), which is how a list already
// held in the office gets in without being clicked onto the map one
// point at a time. A row carries only what an operation is — lat, lng,
// type, size, and a priority it may leave out — because there is no
// geocoder anywhere in the planner and everything else about a day is
// configuration rather than a property of one client. The reading is
// deliberately forgiving of what spreadsheets do to a file (any of
// three delimiters, decimal commas, the columns in any order and named
// in either language, quoted fields, comments, a byte order mark) and
// unforgiving of what a row says, since a wrong container size sends
// the wrong truck: a row that cannot be understood is left out and
// reported by its line rather than guessed at.
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

  // How many early starts a truck is offered on top of not starting
  // early at all: the maximum earliness cut into this many steps, so
  // two means half of it and all of it. Every step is one more VROOM
  // vehicle per truck configuration on the first shift, so this buys
  // granularity with solving time.
  const EARLY_STEPS = 2;

  // Fallbacks for every value docs/waste_defaults.json may set, so the
  // model still works when it is called without them. No early start
  // and no price for one, so a model built without a defaults file
  // behaves exactly as it did before early starts existed.
  const BUILTIN_DEFAULTS = {
    company: { lat: 37.030558, lng: -7.976093 },
    fleet: {},
    chicos: {},
    container_stock: {},
    working_day: { start: "08:00", end: "17:00", lunch_start: "12:00", lunch_end: "13:00", early_start_max_min: 0 },
    operation_times_min: { client_per_container: 10, company_per_visit: 5, company_per_container: 5 },
    costs: { currency: "\u20ac", per_km: {}, chico_multiplier: {}, early_start_per_hour: 0 },
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

  // 30600 -> "08:30", the other way round from clockToSeconds.
  function clockOf(seconds) {
    const s = Math.max(0, Math.round(seconds));
    const h = Math.floor(s / 3600);
    const m = Math.floor((s % 3600) / 60);
    return `${String(h).padStart(2, "0")}:${String(m).padStart(2, "0")}`;
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
    //             chico_multiplier: {<chico key>: factor},
    //             early_start_per_hour: money}
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
        early_start_per_hour: nonNegativeNumber(src.early_start_per_hour, 0),
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

    // What the dearest configuration of all costs per kilometre, in
    // money: the yardstick everything sent to the solver is measured
    // against. 0 when nothing is priced at all.
    function dearestMoneyPerKm(costs) {
      let dearest = 0;
      for (const cfg of VEHICLE_CONFIGS) {
        dearest = Math.max(dearest, moneyPerKm(cfg.key, costs));
      }
      return dearest;
    }

    // The money figures as VROOM wants them: one non-negative integer
    // per configuration, normalised so the dearest lands on COST_SCALE
    // and the rest keep their ratios to it. A configuration that costs
    // something is never rounded down to 0, which would make it free to
    // drive; if nothing costs anything, every per_km is 0 and the plan
    // is decided by the reference term alone.
    function solverPerKm(costs) {
      const dearest = dearestMoneyPerKm(costs);
      const out = {};
      for (const cfg of VEHICLE_CONFIGS) {
        const money = moneyPerKm(cfg.key, costs);
        out[cfg.key] = dearest > 0 && money > 0
          ? Math.max(1, Math.round((money / dearest) * COST_SCALE))
          : 0;
      }
      return out;
    }

    // What going out `seconds` before the working day begins costs, in
    // money: the hourly price of an early start for the time it buys.
    // This is the number to report a plan's early starts with.
    function moneyForEarlyStart(seconds, costs) {
      const c = costs || defaultCosts();
      if (!(seconds > 0)) return 0;
      return nonNegativeNumber(c.early_start_per_hour, 0) * (seconds / 3600);
    }

    // The same figure as VROOM wants it, as a `fixed` cost on the
    // vehicle that starts early. It has to land on the scale solverPerKm
    // puts kilometres on, or the trade the solver is being asked to make
    // is not the company's: VROOM weighs a fixed cost and a kilometre
    // driven at per_km 1 exactly alike, so one unit here is one such
    // kilometre and dividing by the dearest price per kilometre before
    // scaling by COST_SCALE puts money and road in the same units.
    // Never 0 while an early start is on offer: at 0 an early vehicle
    // and its normal twin would cost precisely the same and the solver
    // would send trucks out early for nothing.
    function solverEarlyStart(seconds, costs) {
      if (!(seconds > 0)) return 0;
      const dearest = dearestMoneyPerKm(costs);
      // Nothing is priced per kilometre, so there is no money scale to
      // put this on. What is left is the reference charge on driving
      // time (see REFERENCE_PER_HOUR), which comes to one unit a
      // second, so an hour early is worth an hour more on the road.
      if (dearest <= 0) return Math.round(seconds);
      return Math.max(1, Math.round((moneyForEarlyStart(seconds, costs) / dearest) * COST_SCALE));
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

    // How much earlier than the working day each truck type may go out,
    // in seconds: the "early_start_max_min" block of the defaults, which
    // may also be a plain number meaning the same for every type. A type
    // it says nothing about gets no early start.
    function defaultEarlyStartMax() {
      const src = day.early_start_max_min;
      const shared = (src !== null && typeof src === "object") ? undefined : src;
      const out = {};
      for (const type of TYPE_ORDER) {
        out[type] = minutesToSeconds(shared === undefined ? (src || {})[type] : shared, 0);
      }
      return out;
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
        // How much earlier than dayStart a truck may go out, per truck
        // type. The day is extended at the front and its end does not
        // move, so this is time worked on top of it;
        // costs.early_start_per_hour is what that is worth. 0 for a
        // type offers it no early start at all.
        earlyStartMax: defaultEarlyStartMax(),
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

    // How much earlier than its shift a truck of this type may go out,
    // in seconds. The times object holds one value per truck type; a
    // plain number is taken to mean the same for every type, which is
    // what a caller passing its own times most likely means.
    function earlyStartMaxOf(times, type) {
      const src = (times || {}).earlyStartMax;
      const value = (src !== null && typeof src === "object") ? src[type] : src;
      return isFinite(value) ? Math.max(0, Math.round(value)) : 0;
    }

    // The early starts on offer to a truck type for the first shift of
    // the day, in seconds before its normal start and cheapest first:
    // the maximum earliness cut into EARLY_STEPS steps, so an hour
    // allowed gives half an hour and an hour. Rounded to whole minutes,
    // and empty when the type is allowed none. Only the first shift
    // gets these: the afternoon begins when lunch ends and nothing may
    // move that.
    function earlyStartsOf(times, type) {
      const max = earlyStartMaxOf(times, type);
      const out = [];
      for (let i = 1; i <= EARLY_STEPS; ++i) {
        const seconds = Math.round((max * i) / EARLY_STEPS / 60) * 60;
        if (seconds > 0 && !out.includes(seconds)) out.push(seconds);
      }
      return out;
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
    // profile, shift, early}, shift being a key of shiftsOf (a physical
    // truck is one vehicle per shift) and early the seconds this
    // vehicle goes out before its shift normally starts, 0 for most.
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

      const addVehicle = (type, description, capacities, chico, shift, early, extra) => {
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
        // An early start is the one thing besides the road that a plan
        // pays for, and it is paid per truck that goes out early rather
        // than per kilometre: VROOM's `fixed` cost, charged exactly when
        // this vehicle is used. Nothing else on any vehicle is fixed, so
        // it is the whole price of the earliness.
        if (early > 0) vehicleCosts.fixed = solverEarlyStart(early, costs);
        const parts = [description];
        if (shifts.length > 1) parts.push(shift.label);
        if (early > 0) parts.push(`from ${clockOf(shift.start - early)}`);
        const v = {
          id: vId++,
          description: parts.join(", "),
          type,
          profile,
          start: company,
          end: company,
          capacities,
          // The shift: back at the company, unloaded, by its end, and
          // not loading anything before its start — brought forward by
          // `early` when this vehicle is one of the early starts. The
          // end never moves: an early start lengthens the day, it does
          // not shift it.
          time_window: [shift.start - early, shift.end],
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
        vehicleInfo[v.id] = { type, chico, profile, shift: shift.key, early };
        return v;
      };

      for (const type of TYPE_ORDER) {
        const n = fleet[type] || 0;
        if (n === 0) continue;
        const label = TRUCK_TYPES[type].label.toLowerCase();
        const offered = chicosOffered(type, fleet, chicos);
        const anyChico = Object.values(offered).some((c) => c > 0);
        // One more vehicle per early start this type is offered, on the
        // first shift only, see earlyStartsOf. A type allowed none adds
        // no vehicles at all.
        const earlyStarts = earlyStartsOf(times, type);

        shifts.forEach((shift, shiftRank) => {
          // Only the first shift of the day can start early: the
          // afternoon begins when lunch ends and nothing may move that.
          const earlyForShift = shiftRank === 0 ? earlyStarts : [];
          // The starts this truck type is offered on this shift, its
          // normal one first so the free option is the one the solver
          // finds before any it has to pay for.
          const starts = [0, ...earlyForShift];

          // One group per truck type and shift: every version of the
          // trucks together — plain, with a chico, starting early —
          // may not exceed the number of physical trucks, so the group
          // is what turns those versions into alternatives rather than
          // extra trucks. Counting per shift is what lets a truck put
          // its chico on or take it off at the company over lunch.
          let groups;
          if (anyChico || earlyForShift.length) {
            const suffix = shifts.length > 1 ? `, ${shift.label}` : "";
            vehicleGroups.push({ id: gId, max_vehicles: n, description: `${label} trucks${suffix}` });
            groups = [gId++];
          }
          const extra = groups ? { groups } : {};

          for (const early of starts) {
            for (let i = 1; i <= n; i++) {
              addVehicle(type, `${label} ${i}`, capacitiesFor(type), null, shift, early, extra);
            }
            for (const [chicoKey, count] of Object.entries(offered)) {
              const caps = chicoCapacitiesFor(chicoKey);
              for (let i = 1; i <= count; i++) {
                // What taking this chico costs is its "chico:<key>" entry in
                // the costs, applied by addVehicle like any other override.
                addVehicle(type, `${label} + chico ${i}`, caps, chicoKey, shift, early, extra);
              }
            }
          }
        });
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
    function validate({ depot, operations, fleet, chicos, stock, times, costs }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      chicos = Object.assign(defaultChicos(), chicos || {});
      stock = Object.assign(defaultContainerStock(), stock || {});
      times = Object.assign(defaultTimes(), times || {});
      costs = costs || defaultCosts();
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
      // Only the types with trucks today can start early, so only they
      // are worth complaining about.
      const earlyTypes = TYPE_ORDER.filter((t) => (fleet[t] || 0) > 0 &&
                                                  earlyStartsOf(times, t).length);
      if (earlyTypes.length) {
        const earliest = shiftsOf(times)[0];
        for (const type of earlyTypes) {
          const most = earlyStartMaxOf(times, type);
          if (earliest && earliest.start - most < 0) {
            error(`Starting early: ${TRUCK_TYPES[type].label.toLowerCase()} trucks would be ` +
                  "on the road before midnight.");
          }
        }
        if (!(nonNegativeNumber(costs.early_start_per_hour, 0) > 0)) {
          warning("Starting early costs nothing, so trucks may go out early with nothing " +
                  "to gain by it. Price an hour of early start in the Costs tab, or set " +
                  "the early start to 0 minutes in Config.");
        }
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

    // ---------- the day's operations as a CSV ----------
    //
    // A third way in, next to clicking the map and typing a pair of
    // coordinates: the list as the office already has it. A row carries
    // exactly what an operation is and nothing else — where it is, what
    // is done there, in which container, and how badly it is wanted —
    // because everything else about a day (the fleet, the working
    // hours, the prices) belongs to the configuration and not to one
    // client. There is no geocoder anywhere in the planner, so a row
    // has to bring its own coordinates.
    const CSV_COLUMNS = ["lat", "lng", "type", "size", "priority"];
    const CSV_REQUIRED = ["lat", "lng", "type", "size"];

    // No two spreadsheets name these columns the same way, and none of
    // the differences mean anything: case, spaces, underscores and
    // accents are dropped before a header cell is looked up here.
    const CSV_ALIASES = {
      lat: "lat", latitude: "lat", y: "lat",
      lng: "lng", lon: "lng", long: "lng", longitude: "lng", x: "lng",
      type: "type", operation: "type", op: "type", kind: "type", service: "type",
      size: "size", container: "size", containersize: "size", volume: "size", m3: "size",
      priority: "priority", prio: "priority",
      // The Portuguese for the column names, which is what the office
      // spreadsheets are written in. Only the headings: what an
      // operation is called in a cell stays the planner's own wording,
      // since guessing at that would be guessing at the business.
      operacao: "type", tipo: "type",
      tamanho: "size", contentor: "size", dimensao: "size",
      prioridade: "priority",
    };

    function csvKey(s) {
      return String(s == null ? "" : s).toLowerCase()
        .replace(/³/g, "3")
        .normalize("NFD").replace(/[̀-ͯ]/g, "")
        .replace(/[^a-z0-9]+/g, "");
    }

    // Every spelling of an operation the planner itself shows, plus the
    // short forms someone typing a list by hand reaches for.
    const CSV_TYPES = (function () {
      const map = {};
      for (const [key, t] of Object.entries(OPERATION_TYPES)) {
        for (const spelling of [key, t.short, t.label]) map[csvKey(spelling)] = key;
      }
      return Object.assign(map, {
        deliver: "deliver_empty", delivery: "deliver_empty", drop: "deliver_empty",
        dropempty: "deliver_empty", empty: "deliver_empty",
        pickup: "pickup_full", pick: "pickup_full", collect: "pickup_full",
        collection: "pickup_full", full: "pickup_full",
        swap: "exchange", change: "exchange",
        material: "sell_materials", sell: "sell_materials",
      });
    })();

    // A number as a spreadsheet writes it. Where the decimal separator
    // is a comma the delimiter has to be something else, so a comma
    // left inside a field is a decimal point and a dot beside it is a
    // thousands separator: "41,14961" and "1.234,5" both read.
    function csvNumber(raw) {
      let s = String(raw == null ? "" : raw).trim().replace(/\s/g, "");
      if (!s) return NaN;
      if (s.includes(",")) s = s.replace(/\./g, "").replace(",", ".");
      return /^[-+]?(?:\d+\.?\d*|\.\d+)$/.test(s) ? Number(s) : NaN;
    }

    // The delimiter is whatever the header uses most: a comma, a
    // semicolon (what a spreadsheet writes where the decimal separator
    // is a comma) or a tab (what a block copied out of one carries).
    function csvDelimiter(text) {
      const first = text.split(/\r?\n/)
        .find((l) => l.trim() !== "" && !l.trim().startsWith("#")) || "";
      let best = ",";
      let most = 0;
      for (const d of [",", ";", "\t"]) {
        const n = first.split(d).length - 1;
        if (n > most) { best = d; most = n; }
      }
      return best;
    }

    // RFC 4180 rows, each with the line it started on so that a
    // complaint can name it. A quoted field may hold the delimiter and
    // even a newline, and "" inside one is a single quote.
    function csvRows(text, delim) {
      const rows = [];
      let cells = [];
      let field = "";
      let line = 1;
      let rowLine = 1;
      let quoted = false;
      let started = false;
      const endRow = () => {
        cells.push(field);
        rows.push({ line: rowLine, cells });
        cells = [];
        field = "";
        started = false;
      };
      for (let i = 0; i < text.length; i++) {
        const c = text[i];
        if (!started) { rowLine = line; started = true; }
        if (quoted) {
          if (c === '"' && text[i + 1] === '"') { field += '"'; i++; }
          else if (c === '"') quoted = false;
          else { field += c; if (c === "\n") line++; }
          continue;
        }
        if (c === '"' && field === "") quoted = true;
        else if (c === delim) { cells.push(field); field = ""; }
        else if (c === "\n") { endRow(); line++; }
        else if (c !== "\r") field += c;
      }
      if (started) endRow();
      return rows;
    }

    // A file is read as far as it can be: a row that cannot be
    // understood is left out and reported by line, and the rest still
    // load, because one typo in two hundred lines should not cost a
    // day's list. Returns {operations, problems, rows}, the operations
    // carrying no id — the planner numbers them — and a problem being
    // {line, text}.
    const CSV_MAX_ROWS = 2000;

    function parseOperationsCsv(text) {
      const operations = [];
      const problems = [];
      let body = String(text == null ? "" : text);
      if (body.charCodeAt(0) === 0xfeff) body = body.slice(1);

      const rows = csvRows(body, csvDelimiter(body)).filter((r) =>
        r.cells.some((c) => c.trim() !== "") && !r.cells[0].trim().startsWith("#"));
      if (!rows.length) {
        problems.push({ line: 0, text: "The file holds no rows." });
        return { operations, problems, rows: 0 };
      }

      const column = {};
      rows[0].cells.forEach((cell, i) => {
        const field = CSV_ALIASES[csvKey(cell)];
        if (field && !(field in column)) column[field] = i;
      });
      const missing = CSV_REQUIRED.filter((f) => !(f in column));
      if (missing.length) {
        problems.push({ line: rows[0].line, text:
          `The first row must name the columns and this one has no ${missing.join(", ")}. ` +
          `A header reads "${CSV_COLUMNS.join(",")}"; priority may be left out.` });
        return { operations, problems, rows: 0 };
      }

      let data = rows.slice(1);
      if (data.length > CSV_MAX_ROWS) {
        problems.push({ line: data[CSV_MAX_ROWS].line, text:
          `Only the first ${CSV_MAX_ROWS} rows were read, of ${data.length} in the file.` });
        data = data.slice(0, CSV_MAX_ROWS);
      }

      for (const row of data) {
        const cell = (field) => String(row.cells[column[field]] || "").trim();
        const drop = (text) => problems.push({ line: row.line, text });

        const lat = csvNumber(cell("lat"));
        const lng = csvNumber(cell("lng"));
        if (!isFinite(lat) || !isFinite(lng)) {
          drop(`lat and lng must be numbers, not "${cell("lat")}" and "${cell("lng")}".`);
          continue;
        }
        if (Math.abs(lat) > 90 || Math.abs(lng) > 180) {
          drop(`lat must be -90..90 and lng -180..180, not ${lat} and ${lng}.`);
          continue;
        }

        const type = CSV_TYPES[csvKey(cell("type"))];
        if (!type) {
          drop(`"${cell("type")}" is not an operation. One of ` +
               `${Object.keys(OPERATION_TYPES).join(", ")}.`);
          continue;
        }

        let size = SIZES[0];
        if (OPERATION_TYPES[type].needsSize) {
          size = csvNumber(cell("size"));
          if (!SIZES.includes(size)) {
            drop(`"${cell("size")}" is not a container size. One of ${SIZES.join(", ")} m³.`);
            continue;
          }
        }

        let priority = 0;
        const wanted = "priority" in column ? cell("priority") : "";
        if (wanted !== "") {
          priority = csvNumber(wanted);
          if (!isFinite(priority) || priority < 0 || priority > 100 || priority % 1 !== 0) {
            drop(`"${wanted}" is not a priority. A whole number 0..100, or nothing at all.`);
            continue;
          }
        }

        operations.push({ lat, lng, type, size, priority });
      }
      return { operations, problems, rows: data.length };
    }

    // The same shape written back, so a day can go out to a
    // spreadsheet and come home. An empty list gives the template,
    // which is the only description of the format anyone reads.
    function operationsToCsv(operations) {
      const lines = [CSV_COLUMNS.join(",")];
      for (const op of operations || []) {
        lines.push([
          Number(op.lat).toFixed(6),
          Number(op.lng).toFixed(6),
          op.type,
          (OPERATION_TYPES[op.type] || {}).needsSize ? op.size : "",
          op.priority || 0,
        ].join(","));
      }
      if (!(operations || []).length) {
        lines.push(
          "# One line per operation. Replace the example below and delete these notes.",
          `# lat and lng are degrees, as ${COMPANY.lat.toFixed(5)},${COMPANY.lng.toFixed(5)} — the company.`,
          `# type is one of ${Object.keys(OPERATION_TYPES).join(", ")}; size is ${SIZES.join(", ")} (m3).`,
          "# priority is 0..100, higher is dropped last, and may be left empty.",
          `${COMPANY.lat.toFixed(6)},${COMPANY.lng.toFixed(6)},exchange,${SIZES.includes(6) ? 6 : SIZES[0]},50`);
      }
      return lines.join("\r\n") + "\r\n";
    }

    return {
      SIZES, KINDS, TRUCK_TYPES, TYPE_ORDER, CHICO_TYPES, CHICO_ORDER,
      OPERATION_TYPES, COMPANY, SOLVER_DEFAULTS,
      ZONES, PROFILES, DEFAULT_PROFILE,
      REFERENCE_PER_HOUR, COST_SCALE, EARLY_STEPS, VEHICLE_CONFIGS,
      parseLoad, oneHot, capacitiesFor, chicoCapacitiesFor, sizesFor,
      defaultFleet, defaultChicos, defaultContainerStock, stockFor,
      takesContainerOut, stockUsers, chicosOffered, defaultTimes, shiftsOf,
      earlyStartsOf, earlyStartMaxOf, clockOf,
      defaultCosts, moneyPerKm, dearestMoneyPerKm, solverPerKm,
      moneyForEarlyStart, solverEarlyStart, defaultLimits, configKeyOf,
      profileFor, pointInZone, zonesAt, blockedProfilesAt, describeProfiles,
      opIdOfStep, describe, buildRequest, validate,
      CSV_COLUMNS, parseOperationsCsv, operationsToCsv,
    };
  }

  return { create, OPERATION_TYPES };
});
