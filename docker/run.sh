#!/usr/bin/env bash
# Launch the ROS 2 Humble + Gazebo Classic dev container with GPU + WSLg GUI.
# Mounts ~/ros2 as /workspace inside the container.
set -euo pipefail

IMAGE="dqn-humble:latest"
NAME="dqn-humble"
WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
    echo "[run.sh] Image $IMAGE not found — building..."
    docker build -t "$IMAGE" "$(dirname "$0")"
fi

# Reuse an existing exited container if it's there; otherwise create a new one.
if docker ps -a --format '{{.Names}}' | grep -q "^${NAME}$"; then
    if [ "$(docker inspect -f '{{.State.Running}}' "$NAME")" = "true" ]; then
        echo "[run.sh] Attaching to running container $NAME"
        exec docker exec -it "$NAME" bash
    else
        echo "[run.sh] Starting existing container $NAME"
        docker start "$NAME" >/dev/null
        exec docker exec -it "$NAME" bash
    fi
fi

GPU_FLAG=""
if docker info 2>/dev/null | grep -qi 'Runtimes:.*nvidia'; then
    GPU_FLAG="--gpus all"
fi

echo "[run.sh] Creating new container $NAME (workspace: $WORKSPACE)"
exec docker run -it \
    --name "$NAME" \
    $GPU_FLAG \
    --network host \
    --ipc host \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
    -e XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}" \
    -e PULSE_SERVER="${PULSE_SERVER:-/mnt/wslg/PulseServer}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -v "${WORKSPACE}:/workspace" \
    "$IMAGE" \
    bash