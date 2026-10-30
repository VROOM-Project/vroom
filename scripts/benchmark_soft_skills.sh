#!/usr/bin/env bash
# Benchmark runner for the soft-skills feature.
#
# For each synthetic instance (see benchmarks/generate_synthetic.py) and
# each variant (baseline = no soft-skills fields, soft = full
# soft-skills input), runs ./bin/vroom -t 1 with a fixed solving limit
# so that the baseline/soft comparison uses an identical time budget,
# and records:
#
#   - wall time per run (median/p95 aggregated later)
#   - peak resident set size (KiB) via /usr/bin/time
#   - the full solution JSON (cost, distance, preference counters,
#     unassigned count)
#
# Results land in benchmarks/results/: results.tsv plus one solution
# JSON per run. Aggregate them into markdown tables with
# benchmarks/aggregate_results.py.
#
# All runs are single-threaded (-t 1) for reproducible, comparable
# timing. Total wall time of a full sweep is ~25-30 minutes.

set -o errexit
set -o nounset
export LC_ALL=C

cd "$(dirname "$0")/.."

RUNS=${RUNS:-5}
OUTDIR=${OUTDIR:-benchmarks/results}
VROOM=${VROOM:-./bin/vroom}

# Fixed solving budget per instance size (seconds passed to -l) and
# exploration level passed to -x. With the default -x 5 (32 searches)
# and -t 1 the searches run sequentially and the sweep takes hours; -x
# 0 keeps 4 searches per run, which already saturates the budget on
# the soft-skills variant at both sizes.
LIMIT=20
EXPLORATION=0

if [[ ! -x "$VROOM" ]]; then
    echo "vroom binary not found at $VROOM" >&2
    exit 1
fi

if ! ls benchmarks/instances/synth_*.json >/dev/null 2>&1; then
    echo "generating instances..."
    python3 benchmarks/generate_synthetic.py
fi

mkdir -p "$OUTDIR"
TSV="$OUTDIR/results.tsv"
printf 'size\tcoverage\tvariant\trun\twall_s\tmax_rss_kib\texit_code\tfile\n' > "$TSV"

for size in 300x20 600x40; do
    for cov in 0 25 50 100; do
        for variant in baseline soft; do
            instance="benchmarks/instances/synth_${size}_cov${cov}_${variant}.json"
            for run in $(seq 1 "$RUNS"); do
                sol="$OUTDIR/sol_${size}_cov${cov}_${variant}_run${run}.json"
                rss_file="$OUTDIR/.rss.tmp"
                t0=$(date +%s.%N)
                set +o errexit
                /usr/bin/time -f "%M" -o "$rss_file" \
                    "$VROOM" -i "$instance" -t 1 -x "$EXPLORATION" \
                    -l "$LIMIT" -o "$sol" \
                    > /dev/null 2>&1
                ec=$?
                set -o errexit
                t1=$(date +%s.%N)
                wall=$(awk -v a="$t0" -v b="$t1" 'BEGIN { printf "%.2f", b - a }')
                rss=$(cat "$rss_file")
                printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
                    "$size" "$cov" "$variant" "$run" "$wall" "$rss" "$ec" \
                    "$(basename "$sol")" >> "$TSV"
                echo "[$(date +%T)] $size cov$cov $variant run$run: ${wall}s rss=${rss}KiB exit=$ec"
            done
        done
    done
done

echo "done: results in $TSV"
echo "aggregate with: python3 benchmarks/aggregate_results.py $TSV"
