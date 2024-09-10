#!/bin/bash
xhost +

VOLUMES=(
    "${PWD}:/workspace/omnilrs:rw"
    "${HOME}/docker/omnilrs/isaac-sim/cache/kit:/home/spaceros-user/isaac-sim/kit/cache:rw"
    "${HOME}/docker/omnilrs/isaac-sim/cache/ov:/home/spaceros-user/.cache/ov:rw"
    "${HOME}/docker/omnilrs/isaac-sim/cache/pip:/home/spaceros-user/.cache/pip:rw"
    "${HOME}/docker/omnilrs/isaac-sim/cache/glcache:/home/spaceros-user/.cache/nvidia/GLCache:rw"
    "${HOME}/docker/omnilrs/isaac-sim/cache/computecache:/home/spaceros-user/.nv/ComputeCache:rw"
    "${HOME}/docker/omnilrs/isaac-sim/logs:/home/spaceros-user/.nvidia-omniverse/logs:rw"
    "${HOME}/docker/omnilrs/isaac-sim/data:/home/spaceros-user/.local/share/ov/data:rw"
)
# Create cache directories with appropriate permissions for the user inside the container
for VOLUME in "${VOLUMES[@]}"; do
    HOST_DIR="${VOLUME%%:*}"
    if [[ "${HOST_DIR}" == *cache* ]]; then
        mkdir -p "${HOST_DIR}"
        chmod 777 "${HOST_DIR}"
    fi
done
find "${DEMO_DIR}" -exec bash -c 'p=$(stat -c "%a" "$0"); op=${p:0:1}; chmod "${op}${op}${op}" "$0" 2>/dev/null' {} \; 2>/dev/null || :

docker run --name isaac-sim-omnilrs-container -it --gpus all -e "ACCEPT_EULA=Y" --rm --network=host --ipc=host \
-v $HOME/.Xauthority:/root/.Xauthority \
-e DISPLAY \
-e "PRIVACY_CONSENT=Y" \
"${VOLUMES[@]/#/"-v="}" \
isaac-sim-omnilrs:latest
