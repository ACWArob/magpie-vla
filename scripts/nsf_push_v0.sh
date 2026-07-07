#!/usr/bin/env bash
# Push the V0 LeRobot dataset to NSF ACCESS storage.
# EDIT the two lines below, then:  bash scripts/nsf_push_v0.sh
set -euo pipefail

NSF_USER="aabid"
NSF_HOST="dtai-login.delta.ncsa.illinois.edu"   # DeltaAI login node

# Destination: your scratch space (fast, big; fine for datasets)
DEST="~/lerobot_v0"

echo "Pushing data/lerobot_v0 (387MB) -> ${NSF_USER}@${NSF_HOST}:${DEST}"
rsync -avz --progress \
  ~/magpie_control/data/lerobot_v0/ \
  "${NSF_USER}@${NSF_HOST}:${DEST}/"

echo
echo "Done. Verify on the cluster with:"
echo "  ssh ${NSF_USER}@${NSF_HOST} 'ls ${DEST}/meta && cat ${DEST}/meta/info.json | head -5'"
