#!/usr/bin/env bash
set -u
SP="$(cd "$(dirname "$0")" && pwd)"
BIN=/home/lukel/GridKit/build/application/PhasorDynamics/DynamicSimulation
CPU=${CPU:-4}; ROUNDS=${ROUNDS:-5}
CASES="hawaii newengland illinois wecc texas activsg10k"
OUT="$SP/raw_s10"; mkdir -p "$OUT"
for r in $(seq 1 "$ROUNDS"); do
  for c in $CASES; do
    taskset -c "$CPU" /usr/bin/time -f 'wall_seconds=%e\nmaxrss_kb=%M' \
      -o "$OUT/$c.trial-$r.time.txt" \
      "$BIN" "$SP/$c.s10.solver.json" > "$OUT/$c.trial-$r.stdout.txt" 2>/dev/null
    echo "r$r $c $(grep wall_seconds "$OUT/$c.trial-$r.time.txt")"
  done
done
echo BATCH_COMPLETE
