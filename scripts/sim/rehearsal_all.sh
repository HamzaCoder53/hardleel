#!/bin/bash
# Run the whole mission rehearsal matrix (each case = fresh sim, ~2-3 min).  Results: docs/sim_demo/rehearsal_*.log
# Worker positions are MAP coordinates; (-1.0,-2.27) = bottom corridor, (-3.4,0.08) = gated lane in front of the gate.
cd "$(dirname "$0")/../.."
export EXTRA_MISSION="${EXTRA_MISSION:-gate_align_heading:=true}"
run() { bash scripts/sim/kill_sim.sh; sleep 3; bash scripts/sim/mission_demo.sh "$@" > "docs/sim_demo/$1.out" 2>&1; }
run rehearsal_gate_open        gate=open
run rehearsal_gate_closed      gate=closed
run rehearsal_gate_timed2      gate=timed:2
run rehearsal_open_worker      gate=open   worker=-1.0,-2.27,40
run rehearsal_closed_worker    gate=closed worker=-1.0,-2.27,40
bash scripts/sim/kill_sim.sh
