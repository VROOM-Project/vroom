#!/usr/bin/env node

/*
 Simple Node.js test runner for VROOM CLI.
 Mirrors tests from scripts/trexity-tests.sh including pinned semantics.
*/

const fs = require('fs');
const os = require('os');
const path = require('path');
const { spawnSync } = require('child_process');

function findBinary() {
  const root = path.resolve(__dirname, '..');
  const envBin = process.env.VROOM_BIN;
  if (envBin && fs.existsSync(envBin)) return envBin;
  const macos = path.join(root, 'bin', 'vroom-macos');
  const linux = path.join(root, 'bin', 'vroom');
  if (fs.existsSync(macos)) return macos;
  if (fs.existsSync(linux)) return linux;
  throw new Error('FATAL: No vroom binary found. Set VROOM_BIN or build first.');
}

const BIN = findBinary();
process.stdout.write(`[BIN] ${BIN}\n`);

function runVroom(inputPath, options = {}) {
  let args;
  if (Array.isArray(options)) {
    args = options;
  } else {
    const { explore = 5, threads = 4, extraArgs = [] } = options;
    args = [...extraArgs, '-t', String(threads), '-x', String(explore)];
  }
  const res = spawnSync(BIN, [...args, '-i', inputPath], { encoding: 'utf8' });
  const stdout = res.stdout || '';
  const code = res.status ?? 1;
  let json;
  try {
    json = JSON.parse(stdout || '{}');
  } catch (_) {
    json = {};
  }
  return { code, json, stdout };
}

function tmpDir() {
  // Ensure base tmp exists even if process.env.TMPDIR points to a removed dir
  let base = os.tmpdir();
  try {
    if (!fs.existsSync(base)) {
      fs.mkdirSync(base, { recursive: true });
    }
  } catch (_) {
    base = '/tmp';
  }
  if (!fs.existsSync(base)) base = '/tmp';
  return fs.mkdtempSync(path.join(base, 'trexity-vroom-node-tests.'));
}

function writeJSON(tmp, name, obj) {
  const p = path.join(tmp, name);
  fs.writeFileSync(p, JSON.stringify(obj));
  return p;
}

function matrix2() {
  return { car: { durations: [[0, 1000], [1000, 0]] } };
}
function matrix2_500() {
  return { car: { durations: [[0, 500], [500, 0]] } };
}
function matrix3_200() {
  return { car: { durations: [[0, 200, 200], [200, 0, 200], [200, 200, 0]] } };
}

// Skewed 3x3 matrix to make the optimal order Start -> Job2 -> Job1
// cheaper than Start -> Job1 -> Job2.
function matrix3_skewed() {
  // indices: 0=start, 1=job1, 2=job2
  // 0->2 and 2->1 are cheap; 0->1 is very expensive, so solver should place job2 before job1
  return { car: { durations: [[0, 1000, 100], [1000, 0, 100], [100, 100, 0]] } };
}

// Make job2 clearly cheaper (lower travel cost) while both single-job routes are feasible
// indices: 0=start, 1=job1, 2=job2
// 0->1=300, 0->2=100, 2->1=200, 1->2=200
function matrix3_cheaper_job2() {
  return { car: { durations: [[0, 300, 100], [300, 0, 200], [100, 200, 0]] } };
}

function matrix2_100() {
  return { car: { durations: [[0, 100], [100, 0]] } };
}

function matrix3_chain_50() {
  // 0->1=50, 1->2=50, 0->2=999 (not used)
  return { car: { durations: [[0, 50, 999], [50, 0, 50], [999, 50, 0]] } };
}

function matrix3_chain_50_both() {
  const durations = [[0, 50, 999], [50, 0, 50], [999, 50, 0]];
  const distances = [[0, 500, 9990], [500, 0, 500], [9990, 500, 0]];
  return { car: { durations, distances } };
}

// Helper to build a symmetric 2x2 matrix with both durations and distances
function matrix2_both(d01) {
  return { car: { durations: [[0, d01], [d01, 0]], distances: [[0, d01], [d01, 0]] } };
}

// Helper to build a symmetric 3x3 matrix with both durations and distances
function matrix3_both(d01, d02, d12) {
  return {
    car: {
      durations: [
        [0,   d01, d02],
        [d01, 0,   d12],
        [d02, d12, 0  ]
      ],
      distances: [
        [0,   d01, d02],
        [d01, 0,   d12],
        [d02, d12, 0  ]
      ]
    }
  };
}

function assertExit(exp, got) {
  if (exp !== got) throw new Error(`Expected exit ${exp}, got ${got}`);
}
function assertJsonEq(obj, jqPath, expected) {
  const parts = jqPath.replace(/^\./, '').split('.');
  let cur = obj;
  for (const p of parts) {
    if (p === '') continue;
    if (!(p in cur)) throw new Error(`JSON path not found: ${jqPath}`);
    cur = cur[p];
  }
  if (String(cur) !== String(expected)) {
    throw new Error(`Assert ${jqPath} == ${expected} failed (got ${cur})`);
  }
}

async function run(name, fn) {
  process.stdout.write(`[TEST] ${name}\n`);
  try {
    await fn();
    process.stdout.write('  PASS\n');
    return true;
  } catch (e) {
    process.stdout.write(`  FAIL: ${e.message}\n`);
    return false;
  }
}

// ----------------------- Tests -----------------------

const tests = {
  async budget_single_ok() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: true,
      vehicles: [{ id: 101, start_index: 0 }],
      jobs: [{ id: 1, location_index: 1, budget: 100 }], // travel 100s -> cost 100
      matrices: matrix2_100()
    };
    const f = writeJSON(t, 'budget_single_ok.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async budget_single_insufficient() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: true,
      vehicles: [{ id: 101, start_index: 0 }],
      jobs: [{ id: 1, location_index: 1, budget: 99 }], // need 100
      matrices: matrix2_100()
    };
    const f = writeJSON(t, 'budget_single_insufficient.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async preference_positive_penalty_discourages_vehicle() {
    const t = tmpDir();
    // Two vehicles with different starts, one job. Vehicle 102 is closer but
    // gets a positive penalty, so vehicle 101 should be chosen.
    const input = {
      vehicles: [
        { id: 101, start_index: 0 },
        { id: 102, start_index: 1 }
      ],
      jobs: [{
        id: 1,
        location_index: 2,
        vehicle_penalties: { '102': 150 } // discourage 102
      }],
      matrices: matrix3_both(0, 200, 100) // 0->job=200, 1->job=100
    };
    const f = writeJSON(t, 'pref_pos_penalty.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async preference_negative_penalty_biases_vehicle() {
    const t = tmpDir();
    // Negative penalties are not supported (use positive penalties on other
    // vehicles instead).
    const input = {
      vehicles: [
        { id: 101, start_index: 0 },
        { id: 102, start_index: 1 }
      ],
      jobs: [{
        id: 1,
        location_index: 2,
        vehicle_penalties: { '101': -150 }
      }],
      matrices: matrix3_both(0, 200, 100) // 0->job=200, 1->job=100
    };
    const f = writeJSON(t, 'pref_neg_penalty.json', input);
    const { code, json } = runVroom(f);
    assertExit(2, code);
    if (!json.error.includes('Negative vehicle_penalties are not supported')) {
      throw new Error(`Unexpected error: ${json.error}`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async preference_shipment_penalty_applied_once_on_pickup() {
    const t = tmpDir();
    // Shipment with pickup+delivery. We force assignment to vehicle 101 and
    // assert the penalty is applied once (not twice).
    const input = {
      vehicles: [
        { id: 101, start_index: 0, capacity: [1] },
        { id: 102, start_index: 1, capacity: [1] }
      ],
      shipments: [{
        amount: [1],
        allowed_vehicles: [101],
        vehicle_penalties: { '101': 150 },
        pickup: { id: 11, location_index: 2 },
        delivery: { id: 12, location_index: 3 }
      }],
      matrices: {
        car: {
          durations: [
            // 0=v101 start, 1=v102 start, 2=pickup, 3=delivery
            [0,   0,   200, 0],
            [0,   0,   100, 0],
            [200, 100, 0,   10],
            [0,   0,   10,  0]
          ]
        }
      }
    };
    const f = writeJSON(t, 'pref_shipment_penalty_once.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    // Cost should be start->pickup (200) + pickup->delivery (10) + penalty (150) = 360
    // (penalty counted once on pickup).
    assertJsonEq(json, '.routes.0.cost', 360);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async budget_ignores_vehicle_penalties() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: true,
      vehicles: [{ id: 101, start_index: 0 }],
      jobs: [{
        id: 1,
        location_index: 1,
        budget: 99, // travel 100s -> cost 100, still insufficient
        vehicle_penalties: { '101': 1000 } // should NOT make budget feasible
      }],
      matrices: matrix2_100()
    };
    const f = writeJSON(t, 'budget_ignores_penalties.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async budget_shipment_ok() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: true,
      vehicles: [{
        id: 101,
        start_index: 0,
        capacity: [1],
        steps: [
          { type: 'start' },
          { type: 'pickup', id: 11 },
          { type: 'delivery', id: 12 },
          { type: 'end' }
        ]
      }],
      shipments: [{
        amount: [1],
        budget: 101,
        pickup: { id: 11, location_index: 1 },
        delivery: { id: 12, location_index: 2 }
      }],
      matrices: matrix3_chain_50()
    };
    const f = writeJSON(t, 'budget_shipment_ok.json', input);
    const { code, json, stdout } = runVroom(f);
    if (code !== 0) {
      process.stdout.write(`  DEBUG budget_shipment_ok stdout:\n${stdout}\n`);
    }
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async budget_counts_service_and_setup() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: true,
      vehicles: [{ id: 101, start_index: 0 }],
      // travel 50 + service 30 => 80 budget required
      jobs: [{ id: 1, location_index: 1, service: 30, budget: 79 }],
      matrices: { car: { durations: [[0, 50], [50, 0]] } }
    };
    // First, insufficient
    let f = writeJSON(t, 'budget_counts_action_insufficient.json', input);
    let r = runVroom(f);
    assertExit(0, r.code);
    assertJsonEq(r.json, '.summary.unassigned', 1);
    // Then sufficient
    input.jobs[0].budget = 80;
    f = writeJSON(t, 'budget_counts_action_sufficient.json', input);
    r = runVroom(f);
    assertExit(0, r.code);
    assertJsonEq(r.json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },
  // Route-level budget: over-budget pinned route with no candidates gets dropped entirely
  async budget_postpass_drops_over_budget_route_no_densify() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: false,
      vehicles: [{
        id: 201,
        start_index: 0,
        end_index: 0,
        steps: [
          { type: 'start' },
          { type: 'job', id: 1 },
          { type: 'job', id: 2 },
          { type: 'end' }
        ]
      }],
      jobs: [
        { id: 1, location_index: 1, budget: 200, pinned: true, allowed_vehicles: [201] },
        { id: 2, location_index: 2, budget: 200, pinned: true, allowed_vehicles: [201] }
      ],
      // Travel roughly 600 total: 0->1=200, 1->2=200, 2->0=200
      matrices: { car: { durations: [[0,200,200],[200,0,200],[200,200,0]] } }
    };
    const f = writeJSON(t, 'budget_postpass_drop.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.routes', 0);
    assertJsonEq(json, '.summary.unassigned', 2);
    fs.rmSync(t, { recursive: true, force: true });
  },
  // Route-level budget: densify salvages over-budget route by adding a high-yield unassigned job
  async budget_postpass_densify_salvages_route_with_high_yield_job() {
    const t = tmpDir();
    const input = {
      include_action_time_in_budget: false,
      budget_densify_candidates_k: 1,
      vehicles: [{
        id: 202,
        start_index: 0,
        steps: [
          { type: 'start' },
          { type: 'job', id: 1 },
          { type: 'job', id: 2 },
          { type: 'end' }
        ]
      }],
      jobs: [
        { id: 1, location_index: 1, budget: 200, pinned: true, allowed_vehicles: [202] },
        { id: 2, location_index: 2, budget: 200, pinned: true, allowed_vehicles: [202] },
        // High budget candidate left unassigned initially, used by densify
        { id: 3, location_index: 3, budget: 500 }
      ],
      // Base route ~600 (0->1=200, 1->2=200, 2->0=200).
      // Inserting job 3 adds <= 300 so 200+200+500 >= new cost; densify can salvage.
      matrices: {
        car: { durations: [
          // 0,   1,   2,   3
          [  0, 200, 200, 100 ],
          [200,   0, 200, 150 ],
          [200, 200,   0, 150 ],
          [100, 150, 150,   0 ]
        ] }
      }
    };
    const f = writeJSON(t, 'budget_postpass_densify_salvage.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.routes', 1);
    assertJsonEq(json, '.summary.unassigned', 0);
    // Ensure job 3 was inserted by densify
    const steps = json.routes[0].steps.filter(s => s.type === 'job');
    const ids = steps.map(s => s.id).sort((a,b)=>a-b);
    if (String(ids) !== String([1,2,3])) {
      throw new Error(`Expected route to include jobs [1,2,3], got [${ids}]`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },
  async job_allowed_unassigned() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0 }],
      jobs: [{ id: 1, location_index: 1, allowed_vehicles: [999] }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'job_allowed_unassigned.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async job_pinned_allowed_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [{ id: 1, location_index: 1, pinned: true, allowed_vehicles: [101] }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'job_pinned_allowed_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async shipment_allowed_unassigned() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1] }],
      shipments: [{ amount: [1], allowed_vehicles: [999],
        pickup: { id: 9001, location_index: 0 },
        delivery: { id: 9002, location_index: 1 }
      }],
      matrices: matrix2_500()
    };
    const f = writeJSON(t, 'shipment_allowed_unassigned.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 2);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async shipment_pinned_allowed_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'delivery', id: 9002 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true, allowed_vehicles: [101],
        pickup: { id: 9001, location_index: 0 },
        delivery: { id: 9002, location_index: 1 }
      }],
      matrices: matrix2_500()
    };
    const f = writeJSON(t, 'shipment_pinned_allowed_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async skills_and_allowed_ok() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, skills: [1], steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [{ id: 1, location_index: 1, skills: [1], pinned: true, allowed_vehicles: [101] }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'skills_allowed_ok.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async skills_and_allowed_fail() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, skills: [1], steps: [
        { type: 'start' }, { type: 'job', id: 2 }, { type: 'end' }
      ] }],
      jobs: [{ id: 2, location_index: 1, skills: [2], pinned: true, allowed_vehicles: [101] }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'skills_allowed_fail.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_stays_same_vehicle() {
    const t = tmpDir();
    const input = {
      vehicles: [
        { id: 101, start_index: 0, steps: [ { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' } ] },
        { id: 102, start_index: 0 }
      ],
      jobs: [ { id: 1, location_index: 1, pinned: true }, { id: 2, location_index: 2 } ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_job_stays_same_vehicle.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.length', 1);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_missing_in_steps_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0 }],
      jobs: [{ id: 1, location_index: 1, pinned: true }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pinned_job_missing_in_steps_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_in_two_vehicles_err() {
    const t = tmpDir();
    const input = {
      vehicles: [
        { id: 101, start_index: 0, steps: [ { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' } ] },
        { id: 102, start_index: 0, steps: [ { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' } ] }
      ],
      jobs: [ { id: 1, location_index: 1, pinned: true } ],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pinned_job_in_two_vehicles_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_allowed_conflict_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [ { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' } ] }],
      jobs: [{ id: 1, location_index: 1, pinned: true, allowed_vehicles: [999] }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pinned_job_allowed_conflict_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_shipment_same_vehicle() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'delivery', id: 9002 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true,
        pickup: { id: 9001, location_index: 0 },
        delivery: { id: 9002, location_index: 1 }
      }],
      matrices: matrix2_500()
    };
    const f = writeJSON(t, 'pinned_shipment_same_vehicle.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_shipment_split_err() {
    const t = tmpDir();
    const input = {
      vehicles: [
        { id: 101, start_index: 0, capacity: [1], steps: [ { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'end' } ] },
        { id: 102, start_index: 0, capacity: [1], steps: [ { type: 'start' }, { type: 'delivery', id: 9002 }, { type: 'end' } ] }
      ],
      shipments: [{ amount: [1], pinned: true,
        pickup: { id: 9001, location_index: 0 },
        delivery: { id: 9002, location_index: 1 }
      }],
      matrices: matrix2_500()
    };
    const f = writeJSON(t, 'pinned_shipment_split_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_infeasible_capacity_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [0], steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [{ id: 1, location_index: 1, pinned: true, pickup: [1] }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pinned_infeasible_capacity_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // ---------------- Additional Edge Case Tests ----------------
  async pinned_job_reorder_with_added_task_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [
        { id: 1, location_index: 1, pinned: true },
        { id: 2, location_index: 2 }
      ],
      matrices: matrix3_skewed()
    };
    const f = writeJSON(t, 'pinned_job_reorder_with_added_task_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.length', 1);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    // Expect the solver to reorder due to strong skew: Job2 before Job1
    const steps = json.routes[0].steps;
    const jobSteps = steps.filter(s => s.type === 'job');
    if (jobSteps.length < 2) throw new Error('Expected 2 job steps in route');
    if (jobSteps[0].id !== 2 || jobSteps[1].id !== 1) {
      throw new Error(`Expected order [2,1], got [${jobSteps.map(s => s.id)}]`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_vs_unpinned_cheaper_selection() {
    const t = tmpDir();
    // Two vehicles, one job. Steps seed job 1 on vehicle 101.
    // Matrix favors vehicle 102 -> job (much cheaper). Only difference between runs is pinned flag.
    const base = {
      vehicles: [
        { id: 101, start_index: 0, steps: [ { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' } ] },
        { id: 102, start_index: 1 }
      ],
      jobs: [ { id: 1, location_index: 2 } ],
      matrices: {
        car: {
          durations: [
            // 0 (v101 start)  1 (v102 start)  2 (job)
            [ 0,               0,              1000 ],
            [ 0,               0,              100  ],
            [ 1000,           100,              0   ]
          ]
        }
      }
    };

    // With pinned=true, job must remain on vehicle 101 as seeded
    const withPinned = JSON.parse(JSON.stringify(base));
    withPinned.jobs[0].pinned = true;
    const f1 = writeJSON(t, 'pinned_vs_unpinned_1.json', withPinned);
    const r1 = runVroom(f1);
    assertExit(0, r1.code);
    // Find the route containing job 1
    const routeJob1Pinned = r1.json.routes.find(rt => rt.steps.some(s => s.type === 'job' && s.id === 1));
    if (!routeJob1Pinned || routeJob1Pinned.vehicle !== 101) {
      throw new Error(`Pinned run: expected job 1 on vehicle 101, got ${routeJob1Pinned && routeJob1Pinned.vehicle}`);
    }

    // With pinned=false, optimizer should migrate job 1 to cheaper vehicle 102
    const withoutPinned = JSON.parse(JSON.stringify(base));
    withoutPinned.jobs[0].pinned = false;
    const f2 = writeJSON(t, 'pinned_vs_unpinned_2.json', withoutPinned);
    const r2 = runVroom(f2);
    assertExit(0, r2.code);
    const routeJob1Unpinned = r2.json.routes.find(rt => rt.steps.some(s => s.type === 'job' && s.id === 1));
    if (!routeJob1Unpinned || routeJob1Unpinned.vehicle !== 102) {
      throw new Error(`Unpinned run: expected job 1 on vehicle 102, got ${routeJob1Unpinned && routeJob1Unpinned.vehicle}`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_two_jobs_same_vehicle_success() {
    const t = tmpDir();
    const input = {
      vehicles: [
        { id: 101, start_index: 0, steps: [ { type: 'start' }, { type: 'job', id: 1 }, { type: 'job', id: 2 }, { type: 'end' } ] },
        { id: 102, start_index: 0 }
      ],
      jobs: [
        { id: 1, location_index: 1, pinned: true },
        { id: 2, location_index: 2, pinned: true }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_two_jobs_same_vehicle_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.length', 1);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    const steps = json.routes[0].steps;
    const jobIds = steps.filter(s => s.type === 'job').map(s => s.id).sort((a,b)=>a-b);
    if (String(jobIds) !== String([1,2])) {
      throw new Error(`Expected job steps [1,2] on vehicle 101, got [${jobIds}]`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_shipment_partial_steps_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true,
        pickup: { id: 9001, location_index: 0 },
        delivery: { id: 9002, location_index: 1 }
      }],
      matrices: matrix2_500()
    };
    const f = writeJSON(t, 'pinned_shipment_partial_steps_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_plus_unpinned_assigned_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [
        { id: 1, location_index: 1, pinned: true },
        { id: 2, location_index: 2 }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_job_plus_unpinned_assigned_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.length', 1);
    assertJsonEq(json, '.routes.0.vehicle', 101);
    const steps = json.routes[0].steps;
    const jobIds = steps.filter(s => s.type === 'job').map(s => s.id).sort((a,b)=>a-b);
    if (String(jobIds) !== String([1,2])) {
      throw new Error(`Expected both jobs [1,2] on vehicle 101, got [${jobIds}]`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  // ---------------- pinned_position tests ----------------
  async pinned_job_first_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [
        { id: 1, location_index: 1, pinned: true, pinned_position: 'first' },
        { id: 2, location_index: 2 }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_job_first_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    const steps = json.routes[0].steps.filter(s => s.type === 'job');
    if (steps[0].id !== 1) throw new Error('Job 1 should be first');
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_first_survives_repair_depths() {
    const t = tmpDir();
    const input = {
      vehicles: [{
        id: 0,
        capacity: [2000],
        costs: { fixed: 2000, per_hour: 1440, per_km: 30 },
        initial_pickup_cost_multiplier: 1,
        non_initial_pickup_cost_multiplier: 8,
        start_index: 2,
        max_tasks: 30,
        steps: [
          { type: 'job', id: 5 },
          { type: 'job', id: 6 }
        ]
      }],
      jobs: [
        {
          id: 5,
          location_index: 0,
          service: 270,
          pickup: [0],
          pinned: true,
          pinned_position: 'first',
          allowed_vehicles: [0],
          priority: 0
        },
        {
          id: 6,
          location_index: 1,
          service: 285,
          delivery: [0],
          pinned: true,
          allowed_vehicles: [0],
          time_windows: [[0, 33453]],
          priority: 0
        }
      ],
      shipments: [],
      exclusive_tags_allow_pinned_conflicts: true,
      pinned_soft_timing: true,
      pinned_lateness_limit_sec: 3600,
      include_action_time_in_budget: true,
      matrices: {
        car: {
          durations: [
            [0, 1206, 899, 531, 1295, 486, 974],
            [1175, 0, 613, 942, 1444, 1271, 613],
            [825, 653, 0, 329, 831, 659, 633],
            [545, 947, 396, 0, 792, 337, 837],
            [1363, 1512, 973, 867, 0, 1008, 1386],
            [498, 1279, 729, 338, 938, 0, 1132],
            [948, 620, 590, 834, 1326, 1126, 0]
          ],
          distances: [
            [0, 10339, 7380, 4200, 13827, 3839, 8156],
            [10231, 0, 5768, 8962, 15987, 11627, 5162],
            [7332, 5931, 0, 3194, 10219, 5858, 5620],
            [4362, 8985, 3367, 0, 9814, 2660, 7387],
            [14059, 16033, 10619, 9921, 0, 9983, 14572],
            [3808, 11659, 6041, 2660, 9909, 0, 9251],
            [7711, 5193, 5419, 7396, 14544, 9251, 0]
          ]
        }
      }
    };
    const f = writeJSON(t, 'pinned_job_first_survives_repair_depths.json', input);
    const r0 = runVroom(f, { explore: 0 });
    const r1 = runVroom(f, { explore: 5 });
    assertExit(0, r0.code);
    assertExit(0, r1.code);

    const jobs0 = r0.json.routes[0].steps.filter(s => s.type === 'job').map(s => s.id);
    const jobs1 = r1.json.routes[0].steps.filter(s => s.type === 'job').map(s => s.id);
    if (String(jobs0) !== String([5, 6])) {
      throw new Error(`Expected -x 0 job order [5,6], got [${jobs0}]`);
    }
    if (String(jobs1) !== String([5, 6])) {
      throw new Error(`Expected -x 1 job order [5,6], got [${jobs1}]`);
    }

    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_job_last_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [
        { id: 1, location_index: 2, pinned: true, pinned_position: 'last' },
        { id: 2, location_index: 1 }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_job_last_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    const steps = json.routes[0].steps.filter(s => s.type === 'job');
    if (steps[steps.length - 1].id !== 1) throw new Error('Job 1 should be last');
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_shipment_first_contiguous_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'delivery', id: 9002 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true, pinned_position: 'first',
        pickup: { id: 9001, location_index: 1 },
        delivery: { id: 9002, location_index: 2 }
      }],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_shipment_first_contiguous_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    const steps = json.routes[0].steps.filter(s => s.type === 'pickup' || s.type === 'delivery');
    if (!(steps[0].type === 'pickup' && steps[0].id === 9001 && steps[1].type === 'delivery' && steps[1].id === 9002)) {
      throw new Error('Shipment should be contiguous at start');
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_shipment_last_contiguous_success() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'delivery', id: 9002 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true, pinned_position: 'last',
        pickup: { id: 9001, location_index: 1 },
        delivery: { id: 9002, location_index: 2 }
      }],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_shipment_last_contiguous_success.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    const steps = json.routes[0].steps.filter(s => s.type === 'pickup' || s.type === 'delivery');
    const n = steps.length;
    if (!(steps[n-2].type === 'pickup' && steps[n-2].id === 9001 && steps[n-1].type === 'delivery' && steps[n-1].id === 9002)) {
      throw new Error('Shipment should be contiguous at end');
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_position_without_pinned_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [ { id: 1, location_index: 1, pinned_position: 'first' } ],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pinned_position_without_pinned_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // Bias so that without anchor, optimizer puts unpinned job at end (making pinned job not last)
  async pinned_job_last_discriminating() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      jobs: [
        { id: 1, location_index: 1, pinned: true, pinned_position: 'last' },
        { id: 2, location_index: 2 }
      ],
      matrices: { car: { durations: [
        // 0 start, 1 job1, 2 job2
        [0, 200, 10],
        [200, 0, 10],
        [10, 10, 0]
      ]}}
    };
    const f = writeJSON(t, 'pinned_job_last_discriminating.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    const jobSteps = json.routes[0].steps.filter(s => s.type === 'job');
    if (jobSteps[jobSteps.length - 1].id !== 1) {
      throw new Error('Pinned job 1 must be last with anchor');
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  // Shipment first under pressure: extra job would like to be first, but the
  // pinned shipment must remain anchored at the start.
  async pinned_shipment_first_under_pressure() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'delivery', id: 9002 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true, pinned_position: 'first',
        pickup: { id: 9001, location_index: 1 },
        delivery: { id: 9002, location_index: 2 }
      }],
      jobs: [ { id: 3, location_index: 3 } ],
      matrices: { car: { durations: [
        // 0 start, 1 pickup, 2 delivery, 3 job3
        [0, 5, 5, 1],
        [5, 0, 5, 5],
        [5, 5, 0, 5],
        [1, 5, 5, 0]
      ]}}
    };
    const f = writeJSON(t, 'pinned_shipment_first_under_pressure.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    const steps = json.routes[0].steps.filter(s => s.type === 'pickup' || s.type === 'delivery' || s.type === 'job');
    if (!(steps[0].type === 'pickup' && steps[0].id === 9001 &&
          steps[1].type === 'delivery' && steps[1].id === 9002)) {
      throw new Error('Pinned shipment should remain contiguous at start');
    }
    if (!steps.some(s => s.type === 'job' && s.id === 3)) {
      throw new Error('Expected extra job 3 to remain assigned');
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  // Conflicts and mismatched shipment positions
  async pinned_position_conflict_same_vehicle_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, steps: [
        { type: 'start' }, { type: 'job', id: 1 }, { type: 'job', id: 2 }, { type: 'end' }
      ] }],
      jobs: [
        { id: 1, location_index: 1, pinned: true, pinned_position: 'first' },
        { id: 2, location_index: 2, pinned: true, pinned_position: 'first' }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pinned_position_conflict_same_vehicle_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pinned_shipment_conflict_with_job_first_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 101, start_index: 0, capacity: [1], steps: [
        { type: 'start' }, { type: 'pickup', id: 9001 }, { type: 'delivery', id: 9002 }, { type: 'job', id: 1 }, { type: 'end' }
      ] }],
      shipments: [{ amount: [1], pinned: true, pinned_position: 'first',
        pickup: { id: 9001, location_index: 1 },
        delivery: { id: 9002, location_index: 2 }
      }],
      jobs: [ { id: 1, location_index: 0, pinned: true, pinned_position: 'first' } ],
      matrices: { car: { durations: [[0,10,10,10],[10,0,10,10],[10,10,0,10],[10,10,10,0]] } }
    };
    const f = writeJSON(t, 'pinned_shipment_conflict_with_job_first_err.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

// ---------------- pinned_soft_timing tests ----------------
  // With pinned_soft_timing=true and budget=0, inserting extra work before a pinned step
  // should be blocked (no additional delay allowed). We expect unassigned=1.
  async pinned_soft_timing_blocks_pre_insertion_budget0() {
    const t = tmpDir();
    const input = {
      pinned_soft_timing: true,
      pinned_lateness_limit_sec: 0,
      vehicles: [
        { id: 101, start_index: 0, capacity: [1], steps: [
          { type: 'start' },
          { type: 'pickup', id: 9001 },
          { type: 'delivery', id: 9002 },
          { type: 'end' }
        ] }
      ],
      shipments: [
        { amount: [1], pinned: true, allowed_vehicles: [101],
          pickup: { id: 9001, location_index: 1, time_windows: [[0, 1000]], service: 0 },
          delivery: { id: 9002, location_index: 2, time_windows: [[0, 5000]], service: 0 }
        }
      ],
      jobs: [ { id: 3, location_index: 3, service: 0, time_windows: [[0, 2]] } ],
      matrices: {
        car: { durations: [
          // 0 start, 1 pickup, 2 delivery, 3 extra job
          [0, 5, 5, 1],
          [5, 0, 5, 5],
          [5, 5, 0, 5],
          [3, 5, 5, 0]
        ]}
      }
    };
    const f = writeJSON(t, 'pinned_soft_timing_blocks_pre_insertion_budget0.json', input);
    const { code, json } = runVroom(f);
    // EXPECTED AFTER IMPLEMENTATION: exit 0 with extra job unassigned
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // With pinned_soft_timing=true and small budget, allow small added delay before pinned step
  // We expect the extra job to be assigned when budget >= delta.
  async pinned_violation_budget_allows_small_delay() {
    const t = tmpDir();
    const base = {
      vehicles: [
        { id: 101, start_index: 0, capacity: [1], steps: [
          { type: 'start' },
          { type: 'pickup', id: 9001 },
          { type: 'delivery', id: 9002 },
          { type: 'end' }
        ] }
      ],
      shipments: [
        { amount: [1], pinned: true, allowed_vehicles: [101],
          pickup: { id: 9001, location_index: 1, time_windows: [[0, 1000]], service: 0 },
          delivery: { id: 9002, location_index: 2, time_windows: [[0, 5000]], service: 0 }
        }
      ],
      jobs: [ { id: 3, location_index: 3, service: 0, time_windows: [[0, 2]] } ],
      matrices: {
        car: { durations: [
          // 0 start, 1 pickup, 2 delivery, 3 extra job
          // Going 0->3->1 adds +1s vs 0->1 direct
          [0, 5, 5, 1],
          [5, 0, 5, 5],
          [5, 5, 0, 5],
          [3, 5, 5, 0]
        ]}
      }
    };

    // Case A: budget too small (0) -> expect extra job unassigned
    const a = JSON.parse(JSON.stringify(base));
    a.pinned_soft_timing = true;
    a.pinned_lateness_limit_sec = 0;
    let f = writeJSON(t, 'pinned_budget_small.json', a);
    let r = runVroom(f);
    // EXPECTED AFTER IMPLEMENTATION: exit 0 and unassigned=1
    assertExit(0, r.code);
    assertJsonEq(r.json, '.summary.unassigned', 1);

    // Case B: budget sufficient (>=1) -> expect extra job assigned
    const b = JSON.parse(JSON.stringify(base));
    b.pinned_soft_timing = true;
    b.pinned_lateness_limit_sec = 5;
    f = writeJSON(t, 'pinned_budget_large.json', b);
    r = runVroom(f);
    // EXPECTED AFTER IMPLEMENTATION: exit 0 and unassigned=0
    assertExit(0, r.code);
    assertJsonEq(r.json, '.summary.unassigned', 0);
    const steps = r.json.routes[0].steps;
    const found = steps.some(s => s.type === 'job' && s.id === 3);
    if (!found) throw new Error('Expected extra job 3 assigned on vehicle 101');

    fs.rmSync(t, { recursive: true, force: true });
  },

  // Minimal infeasible seed: pinned job with unreachable TW under provided matrix
  // With pinned_soft_timing=true, solver should not fail; current behavior fails.
  async pinned_soft_timing_saves_infeasible_seed() {
    const t = tmpDir();
    const input = {
      pinned_soft_timing: true,
      pinned_lateness_limit_sec: 0,
      vehicles: [
        { id: 101, start_index: 0, steps: [
          { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
        ] }
      ],
      jobs: [ { id: 1, location_index: 1, pinned: true, service: 0, time_windows: [[0, 10]] } ],
      matrices: { car: { durations: [
        // 0 start, 1 job; travel = 20 > latest 10
        [0, 20],
        [20, 0]
      ] } }
    };
    const f = writeJSON(t, 'pinned_soft_timing_saves_infeasible_seed.json', input);
    const { code } = runVroom(f);
    assertExit(0, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // Regression for debug builds: the output formatter used to assert when the
  // seeded pinned job remained late. Regular solve leaves violations arrays
  // empty, so assert on the late arrival itself.
  async pinned_soft_timing_outputs_late_seeded_arrival() {
    const t = tmpDir();
    const input = {
      pinned_soft_timing: true,
      pinned_lateness_limit_sec: 0,
      vehicles: [
        { id: 101, start_index: 0, steps: [
          { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
        ] }
      ],
      jobs: [ { id: 1, location_index: 1, pinned: true, service: 0, time_windows: [[0, 10]] } ],
      matrices: { car: { durations: [
        [0, 20],
        [20, 0]
      ] } }
    };
    const f = writeJSON(t, 'pinned_soft_timing_outputs_late_seeded_arrival.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    assertJsonEq(json, '.routes.length', 1);
    assertJsonEq(json, '.routes.0.steps.1.id', 1);
    assertJsonEq(json, '.routes.0.steps.1.arrival', 20);
    assertJsonEq(json, '.routes.0.steps.1.waiting_time', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // Control: with pinned_soft_timing=false, infeasible seed should fail (current behavior)
  async pinned_soft_timing_off_infeasible_seed_fails() {
    const t = tmpDir();
    const input = {
      vehicles: [
        { id: 101, start_index: 0, steps: [
          { type: 'start' }, { type: 'job', id: 1 }, { type: 'end' }
        ] }
      ],
      jobs: [ { id: 1, location_index: 1, pinned: true, service: 0, time_windows: [[0, 10]] } ],
      matrices: { car: { durations: [
        [0, 20],
        [20, 0]
      ] } }
    };
    const f = writeJSON(t, 'pinned_soft_timing_off_infeasible_seed_fails.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // ---------------- max_first_leg_distance tests ----------------
  async first_leg_blocks_unseeded_far_job() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 301, start_index: 0, max_first_leg_distance: 300 }],
      jobs: [{ id: 1, location_index: 1 }],
      // start(0)->job(1) = 500 > 300 => cannot seed, unassigned=1
      matrices: matrix2_both(500)
    };
    const f = writeJSON(t, 'first_leg_blocks_unseeded_far_job.json', input);
    const { code, json, stdout } = runVroom(f);
    if (code !== 0) {
      process.stdout.write(`  DEBUG first_leg_blocks_unseeded_far_job stdout:\n${stdout}\n`);
    }
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async first_leg_allows_later_insertion_after_near_seed() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 302, start_index: 0, max_first_leg_distance: 300 }],
      jobs: [
        { id: 1, location_index: 1 }, // near, 0->1 = 100
        { id: 2, location_index: 2 }  // far as seed, 0->2 = 500, but allowed after 1
      ],
      // Distances: 0->1=100 (within limit), 0->2=500 (exceeds), 1<->2=100
      matrices: matrix3_both(100, 500, 100)
    };
    const f = writeJSON(t, 'first_leg_allows_later_insertion_after_near_seed.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    // Ensure first job on the route is 1 (the near one)
    const route = json.routes.find(rt => rt.vehicle === 302);
    if (!route) throw new Error('Missing route for vehicle 302');
    const jobSteps = route.steps.filter(s => s.type === 'job');
    if (!jobSteps.length) throw new Error('Expected at least one job step');
    if (jobSteps[0].id !== 1) throw new Error(`Expected first job id=1, got ${jobSteps[0].id}`);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async first_leg_blocks_shipment_pickup() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 303, start_index: 0, capacity: [1], max_first_leg_distance: 300 }],
      shipments: [{
        amount: [1],
        pickup: { id: 11, location_index: 1 },   // 0->1 = 500 > 300
        delivery: { id: 12, location_index: 2 }
      }],
      matrices: matrix3_both(500, 200, 100)
    };
    const f = writeJSON(t, 'first_leg_blocks_shipment_pickup.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    // both shipment steps remain unassigned
    assertJsonEq(json, '.summary.unassigned', 2);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async first_leg_ignored_for_vehicles_with_steps() {
    const t = tmpDir();
    const input = {
      vehicles: [{
        id: 304,
        start_index: 0,
        max_first_leg_distance: 300, // would block 0->2=500, but steps exist so ignore
        steps: [
          { type: 'start' },
          { type: 'job', id: 2 },
          { type: 'end' }
        ]
      }],
      jobs: [{ id: 2, location_index: 1, pinned: true, allowed_vehicles: [304] }],
      matrices: {
        car: {
          durations: [
            // 0 start, 2 job
            [0, 500],
            [500, 0]
          ],
          distances: [
            [0, 500],
            [500, 0]
          ]
        }
      }
    };
    const f = writeJSON(t, 'first_leg_ignored_for_vehicles_with_steps.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  // ---------------- exclusive_tags tests ----------------
  async exclusive_tags_single_vehicle_conflict_unassigns_one() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 401, start_index: 0 }],
      jobs: [
        { id: 1, location_index: 1, exclusive_tags: [1] },
        { id: 2, location_index: 2, exclusive_tags: [1] }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'exclusive_tags_single_vehicle_conflict.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async exclusive_tags_two_vehicles_all_assigned() {
    const t = tmpDir();
    const input = {
      vehicles: [
        { id: 402, start_index: 0 },
        { id: 403, start_index: 1 }
      ],
      jobs: [
        { id: 1, location_index: 2, exclusive_tags: [1] },
        { id: 2, location_index: 3, exclusive_tags: [1] }
      ],
      matrices: {
        car: {
          durations: [
            // 0=v402 start, 1=v403 start, 2=job1, 3=job2
            [0,   0,  10, 100],
            [0,   0, 100,  10],
            [10, 100,  0,  50],
            [100, 10, 50,   0]
          ]
        }
      }
    };
    const f = writeJSON(t, 'exclusive_tags_two_vehicles_all_assigned.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    // Each job must be on a different route due to the exclusive tag.
    assertJsonEq(json, '.summary.routes', 2);
    const assigned = new Set();
    for (const rt of json.routes) {
      for (const st of rt.steps) {
        if (st.type === 'job') assigned.add(st.id);
      }
    }
    if (!(assigned.has(1) && assigned.has(2))) {
      throw new Error(`Expected both jobs assigned, got [${Array.from(assigned)}]`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  async exclusive_tags_shipment_conflict_unassigns_one_shipment() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 404, start_index: 0, capacity: [1] }],
      shipments: [
        {
          amount: [1],
          exclusive_tags: [1],
          pickup: { id: 11, location_index: 1 },
          delivery: { id: 12, location_index: 2 }
        },
        {
          amount: [1],
          exclusive_tags: [1],
          pickup: { id: 21, location_index: 3 },
          delivery: { id: 22, location_index: 4 }
        }
      ],
      matrices: {
        car: {
          durations: [
            // 0 start, 1 p1, 2 d1, 3 p2, 4 d2
            [0, 10, 10, 10, 10],
            [10, 0,  10, 10, 10],
            [10, 10, 0,  10, 10],
            [10, 10, 10, 0,  10],
            [10, 10, 10, 10, 0]
          ]
        }
      }
    };
    const f = writeJSON(t, 'exclusive_tags_shipment_conflict.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    // One shipment (pickup+delivery) must remain unassigned.
    assertJsonEq(json, '.summary.unassigned', 2);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async exclusive_tags_pinned_conflict_err() {
    const t = tmpDir();
    const input = {
      vehicles: [{
        id: 405,
        start_index: 0,
        steps: [
          { type: 'start' },
          { type: 'job', id: 1 },
          { type: 'job', id: 2 },
          { type: 'end' }
        ]
      }],
      jobs: [
        { id: 1, location_index: 1, pinned: true, exclusive_tags: [1] },
        { id: 2, location_index: 2, pinned: true, exclusive_tags: [1] }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'exclusive_tags_pinned_conflict_err.json', input);
    const { code, json } = runVroom(f);
    assertExit(2, code);
    if (!json.error.includes('Pinned tasks')) {
      throw new Error(`Unexpected error: ${json.error}`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  },

  // ---------------- pickup approach cost multiplier tests ----------------
  async pickup_multiplier_splits_distant_pickups() {
    const t = tmpDir();
    // 3 merchants with distant pickups and realistic fixed cost (500).
    // Without multiplier: all interleaved on 1 vehicle (cheaper than 3
    // fixed costs). With multiplier 10000: PDShift splits into 3 separate
    // single-pickup routes because the non-initial penalty dwarfs the
    // fixed cost savings.
    // 7 locations: 0=start, 1/2=merchant A pickup/delivery,
    //              3/4=merchant B, 5/6=merchant C
    const matrix7 = { car: { durations: [
      [  0, 100, 200, 100, 200, 100, 200],
      [100,   0, 100, 400, 300, 500, 400],
      [200, 100,   0, 300, 200, 400, 300],
      [100, 400, 300,   0, 100, 400, 300],
      [200, 300, 200, 100,   0, 300, 200],
      [100, 500, 400, 400, 300,   0, 100],
      [200, 400, 300, 300, 200, 100,   0]
    ]}};

    const makeInput = (mult) => ({
      vehicles: [
        { id: 1, start_index: 0, capacity: [1], costs: { fixed: 500 },
          non_initial_pickup_cost_multiplier: mult },
        { id: 2, start_index: 0, capacity: [1], costs: { fixed: 500 },
          non_initial_pickup_cost_multiplier: mult },
        { id: 3, start_index: 0, capacity: [1], costs: { fixed: 500 },
          non_initial_pickup_cost_multiplier: mult }
      ],
      shipments: [
        { amount: [1], pickup: { id: 11, location_index: 1 },
          delivery: { id: 12, location_index: 2 } },
        { amount: [1], pickup: { id: 21, location_index: 3 },
          delivery: { id: 22, location_index: 4 } },
        { amount: [1], pickup: { id: 31, location_index: 5 },
          delivery: { id: 32, location_index: 6 } }
      ],
      matrices: matrix7
    });

    // Control: no multiplier -> interleaved on 1 vehicle
    const f1 = writeJSON(t, 'pm_control.json', makeInput(1.0));
    const r1 = runVroom(f1);
    assertExit(0, r1.code);
    assertJsonEq(r1.json, '.summary.unassigned', 0);
    assertJsonEq(r1.json, '.summary.routes', 1);

    // With high multiplier -> split into 3 single-pickup routes
    const f2 = writeJSON(t, 'pm_split.json', makeInput(10000));
    const r2 = runVroom(f2);
    assertExit(0, r2.code);
    assertJsonEq(r2.json, '.summary.unassigned', 0);
    assertJsonEq(r2.json, '.summary.routes', 3);

    // Each route should have exactly 1 pickup step.
    for (const rt of r2.json.routes) {
      const pickups = rt.steps.filter(s => s.type === 'pickup');
      if (pickups.length !== 1) {
        throw new Error(`Route V${rt.vehicle}: expected 1 pickup, got ${pickups.length}`);
      }
    }

    fs.rmSync(t, { recursive: true, force: true });
  },

  async pickup_multiplier_allows_colocated_pickups() {
    const t = tmpDir();
    // Two shipments with pickups at the SAME location. Even with high
    // non_initial multiplier, penalty is (N-1)*0 = 0 for co-located pickups
    // placed consecutively, so interleaving on 1 vehicle stays optimal.
    const input = {
      vehicles: [
        { id: 1, start_index: 0, capacity: [2], costs: { fixed: 500 },
          non_initial_pickup_cost_multiplier: 10 },
        { id: 2, start_index: 0, capacity: [2], costs: { fixed: 500 },
          non_initial_pickup_cost_multiplier: 10 }
      ],
      shipments: [
        { amount: [1],
          pickup: { id: 11, location_index: 1 },
          delivery: { id: 12, location_index: 2 } },
        { amount: [1],
          pickup: { id: 21, location_index: 1 },
          delivery: { id: 22, location_index: 3 } }
      ],
      matrices: { car: { durations: [
        //  0    1    2    3
        [  0, 100, 300, 300],
        [100,   0, 200, 200],
        [300, 200,   0, 100],
        [300, 200, 100,   0]
      ]}}
    };
    const f = writeJSON(t, 'pm_colocated.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.routes', 1);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pickup_multiplier_no_effect_on_jobs() {
    const t = tmpDir();
    // Jobs only (not shipments) — multiplier should have no effect since
    // jobs are JOB_TYPE::SINGLE, not PICKUP.
    const input = {
      vehicles: [{ id: 1, start_index: 0, non_initial_pickup_cost_multiplier: 100 }],
      jobs: [
        { id: 1, location_index: 1 },
        { id: 2, location_index: 2 }
      ],
      matrices: matrix3_200()
    };
    const f = writeJSON(t, 'pm_no_effect_jobs.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    assertJsonEq(json, '.summary.unassigned', 0);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pickup_multiplier_validation_rejects_zero() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 1, start_index: 0, non_initial_pickup_cost_multiplier: 0 }],
      jobs: [{ id: 1, location_index: 1 }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pm_validation_zero.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pickup_multiplier_validation_rejects_negative() {
    const t = tmpDir();
    const input = {
      vehicles: [{ id: 1, start_index: 0, initial_pickup_cost_multiplier: -1 }],
      jobs: [{ id: 1, location_index: 1 }],
      matrices: matrix2()
    };
    const f = writeJSON(t, 'pm_validation_negative.json', input);
    const { code } = runVroom(f);
    assertExit(2, code);
    fs.rmSync(t, { recursive: true, force: true });
  },

  async pickup_multiplier_output_costs_unaffected() {
    const t = tmpDir();
    // With and without multiplier, the output cost/duration/distance should
    // be identical for the same route structure. The penalty is optimization-only.
    const makeInput = (mult) => ({
      vehicles: [
        { id: 1, start_index: 0, capacity: [1],
          non_initial_pickup_cost_multiplier: mult }
      ],
      shipments: [
        { amount: [1],
          pickup: { id: 11, location_index: 1 },
          delivery: { id: 12, location_index: 2 } }
      ],
      matrices: { car: { durations: [
        [0, 100, 200],
        [100, 0, 150],
        [200, 150, 0]
      ]}}
    });

    const f1 = writeJSON(t, 'pm_output_control.json', makeInput(1.0));
    const r1 = runVroom(f1);
    assertExit(0, r1.code);

    const f2 = writeJSON(t, 'pm_output_mult.json', makeInput(10.0));
    const r2 = runVroom(f2);
    assertExit(0, r2.code);

    // Same route structure, so output cost/duration/distance should match
    const route1 = r1.json.routes[0];
    const route2 = r2.json.routes[0];
    if (route1.cost !== route2.cost) {
      throw new Error(`Output cost changed with multiplier: ${route1.cost} vs ${route2.cost}`);
    }
    if (route1.duration !== route2.duration) {
      throw new Error(`Output duration changed: ${route1.duration} vs ${route2.duration}`);
    }

    fs.rmSync(t, { recursive: true, force: true });
  },

  async exclusive_tags_pinned_conflict_allowed_blocks_third() {
    const t = tmpDir();
    const input = {
      exclusive_tags_allow_pinned_conflicts: true,
      vehicles: [{
        id: 406,
        start_index: 0,
        steps: [
          { type: 'start' },
          { type: 'job', id: 1 },
          { type: 'job', id: 2 },
          { type: 'end' }
        ]
      }],
      jobs: [
        { id: 1, location_index: 1, pinned: true, exclusive_tags: [1] },
        { id: 2, location_index: 2, pinned: true, exclusive_tags: [1] },
        { id: 3, location_index: 3, exclusive_tags: [1] }
      ],
      matrices: {
        car: { durations: [
          // 0 start, 1 job1, 2 job2, 3 job3
          [0, 10, 10, 10],
          [10, 0, 10, 10],
          [10, 10, 0, 10],
          [10, 10, 10, 0]
        ] }
      }
    };
    const f = writeJSON(t, 'exclusive_tags_pinned_conflict_allowed_blocks_third.json', input);
    const { code, json } = runVroom(f);
    assertExit(0, code);
    // Job 3 must remain unassigned (tag already used twice by pinned jobs).
    assertJsonEq(json, '.summary.unassigned', 1);
    const assigned = new Set();
    for (const rt of json.routes) {
      for (const st of rt.steps) {
        if (st.type === 'job') assigned.add(st.id);
      }
    }
    if (!(assigned.has(1) && assigned.has(2)) || assigned.has(3)) {
      throw new Error(`Expected jobs 1&2 assigned and 3 unassigned, got assigned [${Array.from(assigned)}]`);
    }
    fs.rmSync(t, { recursive: true, force: true });
  }
};

async function main() {
  const order = [
    'budget_single_ok',
    'budget_single_insufficient',
    // New tests: vehicle_penalties (objective-only)
    'preference_positive_penalty_discourages_vehicle',
    'preference_negative_penalty_biases_vehicle',
    'preference_shipment_penalty_applied_once_on_pickup',
    'budget_ignores_vehicle_penalties',
    'budget_shipment_ok',
    'budget_counts_service_and_setup',
    // New route-level budget repair tests
    'budget_postpass_drops_over_budget_route_no_densify',
    'budget_postpass_densify_salvages_route_with_high_yield_job',
    'job_allowed_unassigned',
    'job_pinned_allowed_success',
    'shipment_allowed_unassigned',
    'shipment_pinned_allowed_success',
    'skills_and_allowed_ok',
    'skills_and_allowed_fail',
    'pinned_job_stays_same_vehicle',
    'pinned_job_missing_in_steps_err',
    'pinned_job_in_two_vehicles_err',
    'pinned_job_allowed_conflict_err',
    'pinned_shipment_same_vehicle',
    'pinned_shipment_split_err',
    'pinned_infeasible_capacity_err',
    // Additional edge cases
    'pinned_job_reorder_with_added_task_success',
    'pinned_two_jobs_same_vehicle_success',
    'pinned_shipment_partial_steps_err',
    'pinned_job_plus_unpinned_assigned_success',
    'pinned_vs_unpinned_cheaper_selection',
    // pinned_position
    'pinned_job_first_success',
    'pinned_job_first_survives_repair_depths',
    'pinned_job_last_success',
    'pinned_shipment_first_contiguous_success',
    'pinned_shipment_last_contiguous_success',
    'pinned_position_without_pinned_err',
    'pinned_job_last_discriminating',
    'pinned_shipment_first_under_pressure',
    'pinned_position_conflict_same_vehicle_err',
    'pinned_shipment_conflict_with_job_first_err',
    // New tests for pinned_soft_timing semantics
    'pinned_soft_timing_blocks_pre_insertion_budget0',
    'pinned_violation_budget_allows_small_delay',
    'pinned_soft_timing_saves_infeasible_seed',
    'pinned_soft_timing_outputs_late_seeded_arrival',
    'pinned_soft_timing_off_infeasible_seed_fails',
    // New tests: max_first_leg_distance behavior
    'first_leg_blocks_unseeded_far_job',
    'first_leg_allows_later_insertion_after_near_seed',
    'first_leg_blocks_shipment_pickup',
    'first_leg_ignored_for_vehicles_with_steps',
    // pickup approach cost multiplier
    'pickup_multiplier_splits_distant_pickups',
    'pickup_multiplier_allows_colocated_pickups',
    'pickup_multiplier_no_effect_on_jobs',
    'pickup_multiplier_validation_rejects_zero',
    'pickup_multiplier_validation_rejects_negative',
    'pickup_multiplier_output_costs_unaffected',
    // exclusive_tags
    'exclusive_tags_single_vehicle_conflict_unassigns_one',
    'exclusive_tags_two_vehicles_all_assigned',
    'exclusive_tags_shipment_conflict_unassigns_one_shipment',
    'exclusive_tags_pinned_conflict_err',
    'exclusive_tags_pinned_conflict_allowed_blocks_third'
  ];

  const requested = process.argv.slice(2);
  const selected = requested.length === 0
    ? order
    : order.filter((name) => requested.includes(name));
  if (requested.length !== 0 && selected.length !== requested.length) {
    const missing = requested.filter((name) => !order.includes(name));
    throw new Error(`Unknown test name(s): ${missing.join(', ')}`);
  }

  let pass = 0, fail = 0;
  for (const name of selected) {
    // eslint-disable-next-line no-await-in-loop
    const ok = await run(name, tests[name]);
    if (ok) pass++; else fail++;
  }
  process.stdout.write(`\nSummary: ${pass} passed, ${fail} failed\n`);
  process.exit(fail === 0 ? 0 : 1);
}

main().catch((e) => {
  console.error(e);
  process.exit(1);
});


