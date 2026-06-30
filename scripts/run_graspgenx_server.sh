#!/bin/bash
# Start the GraspGenX ZMQ inference server for the MAGPIE gripper.
#
# GraspGenX (NVlabs, ICRA'26) is a CROSS-EMBODIMENT grasp model: one model, any gripper,
# conditioned on the gripper's swept volume. It runs the MAGPIE gripper zero-shot using
# the Correll-lab gripper definition at ~/GraspGenX/assets/x_grippers/magpie/.
#
# The model lives in its own uv venv (~/GraspGenX/.venv); the magpie pickup talks to it
# over ZMQ via scripts/grasp_detectors/graspgenx_zmq.py (GRASP_METHOD='graspgenx').
#
# Usage:   bash scripts/run_graspgenx_server.sh   [PORT]   [GRIPPER]
#   PORT     default 5557
#   GRIPPER  default magpie  (any name under ~/GraspGenX/assets/x_grippers/)
#
# VRAM (8 GB card): GraspGenX needs ~3 GB. SAM3 needs ~4 GB. They DON'T both fit with
# everything else — so when collecting with GraspGenX, expect to free SAM3 between the
# detect step and grasp planning (cold-start), or run them sequentially. See GRASPGENX.md.

set -e
PORT="${1:-5557}"
GRIPPER="${2:-magpie}"
GGX_DIR="$HOME/GraspGenX"
UV="$(command -v uv || echo "$HOME/.local/bin/uv")"
[ -x "$UV" ] || UV="$HOME/snap/code/247/.local/bin/uv"

cd "$GGX_DIR"
echo "Starting GraspGenX server: gripper=$GRIPPER port=$PORT"
echo "  (first call per gripper lazily builds its caches; checkpoints already in ext/)"
exec "$UV" run python client-server/graspgenx_server.py \
    --config ext/graspgenx_checkpoints/release \
    --assets_dir assets \
    --default_gripper "$GRIPPER" \
    --port "$PORT"
