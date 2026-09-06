// Waste transport model: truck types, loading rules, operation types and
// the translation of a planner's day (company + operations + fleet) into
// a VROOM request using the `capacities` extension of this fork.
//
// The rules themselves live in docs/waste_rules.json (single source of
// truth) and the planner's starting values in docs/waste_defaults.json
// next to it. This file only interprets them: build a model with
//   WasteModel.create(rules, defaults)
// where both arguments are the parsed JSON (`defaults` is optional and
// falls back to the built-in values below). In the browser the server
// exposes them as window.WASTE_RULES (/rules.js) and
// window.WASTE_DEFAULTS (/defaults.js); under node use
//   require("./waste_model").create(require("../../docs/waste_rules.json"),
//                                   require("../../docs/waste_defaults.json"))
//
// Amount components are container kinds, one-hot per task:
//   e2 e6 ...  empty containers by size (m3), one per size in rules.sizes
//   f2 f6 ...  full containers by size
//   mat        a full truck of materials (nothing else on board)
(function (root, factory) {
  if (typeof module === "object" && module.exports) module.exports = factory();
  else root.WasteModel = factory();
})(typeof self !== "undefined" ? self : this, function () {
  // Planner-facing operation types.
  const OPERATION_TYPES = {
    deliver_empty: { label: "Deliver empty container", short: "deliver empty", needsSize: true },
    pickup_empty: { label: "Pick up empty container", short: "pick up empty", needsSize: true },
    exchange: { label: "Exchange empty for full", short: "exchange", needsSize: true },
    sell_materials: { label: "Sell materials (full truck)", short: "materials", needsSize: false },
  };

  // Fallbacks for every value docs/waste_defaults.json may set, so the
  // model still works when it is called without them.
  const BUILTIN_DEFAULTS = {
    company: { lat: 37.030558, lng: -7.976093 },
    fleet: {},
    working_day: { start: "08:00", end: "17:00", lunch_start: "12:00", lunch_end: "13:00" },
    operation_times_min: { client_per_container: 10, company_per_visit: 5, company_per_container: 5 },
    solver: { geometry: true, show_request: false },
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

  function create(rules, defaults) {
    if (!rules || !rules.trucks || !rules.sizes) {
      throw new Error("waste rules missing: expected {sizes, trucks}");
    }
    const D = defaults || {};
    const day = { ...BUILTIN_DEFAULTS.working_day, ...(D.working_day || {}) };
    const svc = { ...BUILTIN_DEFAULTS.operation_times_min, ...(D.operation_times_min || {}) };
    const SOLVER_DEFAULTS = { ...BUILTIN_DEFAULTS.solver, ...(D.solver || {}) };
    const SIZES = rules.sizes.slice();
    const KINDS = [...SIZES.map((s) => `e${s}`), ...SIZES.map((s) => `f${s}`), "mat"];
    const TRUCK_TYPES = {};
    for (const [key, t] of Object.entries(rules.trucks)) {
      TRUCK_TYPES[key] = { label: t.label || key, rules: t.rules.slice() };
    }
    const TYPE_ORDER = Object.keys(TRUCK_TYPES);
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

    // Capacity vectors of a truck type. Every truck may alternatively
    // carry a full load of materials and nothing else.
    function capacitiesFor(type) {
      const caps = TRUCK_TYPES[type].rules.map(parseLoad);
      caps.push(oneHot("mat"));
      return caps;
    }

    // Which container sizes a truck type can carry at all.
    function sizesFor(type) {
      const caps = capacitiesFor(type);
      return SIZES.filter((s) => caps.some((c) => c[KINDS.indexOf(`e${s}`)] > 0 || c[KINDS.indexOf(`f${s}`)] > 0));
    }

    function defaultFleet() {
      const fleet = {};
      for (const t of TYPE_ORDER) {
        const n = Number((D.fleet || {})[t]);
        fleet[t] = isFinite(n) && n >= 0 ? Math.floor(n) : 1;
      }
      return fleet;
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

    // Build the VROOM request.
    //   depot        {lat, lng}                     the company
    //   operations   [{id, lat, lng, type, size, priority}]
    //   fleet        {small: n, multiban: n, poliban: n}
    //   times        see defaultTimes()
    //   geometry     bool
    function buildRequest({ depot, operations, fleet, times, geometry }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      times = Object.assign(defaultTimes(), times || {});
      const company = [depot.lng, depot.lat];

      const vehicles = [];
      let vId = 1;
      for (const type of TYPE_ORDER) {
        for (let i = 1; i <= (fleet[type] || 0); i++) {
          const v = {
            id: vId++,
            description: `${TRUCK_TYPES[type].label.toLowerCase()} ${i}`,
            type,
            profile: "car",
            start: company,
            end: company,
            capacities: capacitiesFor(type),
            time_window: [times.dayStart, times.dayEnd],
            costs: { per_hour: 3600, per_task_hour: 3600 },
          };
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
        const push = (s) => { s.priority = priority; shipments.push(s); };

        switch (op.type) {
          case "deliver_empty":
            push({
              amount: oneHot(`e${size}`),
              pickup: atCompany(base + 1, `${tag}: load empty ${size} m³ at company`),
              delivery: atClient(base + 2, op, `${tag}: deliver empty ${size} m³`),
            });
            break;
          case "pickup_empty":
            push({
              amount: oneHot(`e${size}`),
              pickup: atClient(base + 1, op, `${tag}: pick up empty ${size} m³`),
              delivery: atCompany(base + 2, `${tag}: unload empty ${size} m³ at company`),
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
            push({
              amount: oneHot("mat"),
              pickup: atCompany(base + 1, `${tag}: load materials at company (full truck)`),
              delivery: atClient(base + 2, op, `${tag}: deliver materials`),
            });
            break;
          default:
            throw new Error(`unknown operation type ${op.type}`);
        }
      }

      const request = { vehicles, shipments, options: { g: !!geometry } };
      return { request, stepInfo };
    }

    // Sanity checks on a planner's day before building the request.
    function validate({ operations, fleet, times }) {
      fleet = Object.assign(defaultFleet(), fleet || {});
      times = Object.assign(defaultTimes(), times || {});
      const problems = [];
      if (times.dayEnd <= times.dayStart) {
        problems.push("Working day: the end must be after the start.");
      }
      if (times.lunchEnd < times.lunchStart) {
        problems.push("Lunch: the end must not be before the start.");
      } else if (times.lunchEnd > times.lunchStart &&
                 (times.lunchStart < times.dayStart || times.lunchEnd > times.dayEnd)) {
        problems.push("Lunch: must be inside the working day.");
      }
      const total = TYPE_ORDER.reduce((n, t) => n + (fleet[t] || 0), 0);
      if (total === 0) problems.push("The fleet is empty: set at least one truck in Config.");
      const carriers = {};
      for (const t of TYPE_ORDER) {
        if (!fleet[t]) continue;
        for (const s of sizesFor(t)) carriers[s] = true;
        carriers.mat = true;
      }
      for (const op of operations) {
        const t = OPERATION_TYPES[op.type];
        if (t.needsSize && !carriers[op.size]) {
          problems.push(`Operation ${op.id}: no truck in the fleet can carry a ${op.size} m³ container.`);
        }
        if (!t.needsSize && !carriers.mat) {
          problems.push(`Operation ${op.id}: no truck in the fleet is allowed to carry materials.`);
        }
      }
      return problems;
    }

    return {
      SIZES, KINDS, TRUCK_TYPES, TYPE_ORDER, OPERATION_TYPES, COMPANY,
      SOLVER_DEFAULTS,
      parseLoad, oneHot, capacitiesFor, sizesFor,
      defaultFleet, defaultTimes,
      opIdOfStep, describe, buildRequest, validate,
    };
  }

  return { create, OPERATION_TYPES };
});
