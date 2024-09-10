#!/bin/bash

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
TAG="isaac-sim-omnilrs"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

# Check if Isaac Sim can be pulled to determine whether to bundle Isaac Sim or rely on a local installation
ISAAC_SIM_IMAGE_NAME="${ISAAC_SIM_IMAGE_NAME:-"nvcr.io/nvidia/isaac-sim"}"
ISAAC_SIM_IMAGE_TAG="${ISAAC_SIM_IMAGE_TAG:-"2023.1.1"}"
if ${WITH_SUDO} docker pull "${ISAAC_SIM_IMAGE_NAME}:${ISAAC_SIM_IMAGE_TAG}" &>/dev/null; then
    DOCKERFILE+=".bundle_isaac_sim"
else
    >&2 echo -e "\033[1;33m[WARN] This system is not logged into the NVIDIA NGC registry to pull '${ISAAC_SIM_IMAGE_NAME}:${ISAAC_SIM_IMAGE_TAG}' image"
    >&2 echo -e "[WARN] Local installation of Isaac Sim will have to be mounted as a volume when running this demo image\033[0m"
fi

DOCKER_BUILD_CMD=(docker build "${SCRIPT_DIR}" --tag ${TAG} --file "${DOCKERFILE}")

echo -e "\033[0;32m${DOCKER_BUILD_CMD[*]}\033[0m" | xargs

# shellcheck disable=SC2068
exec ${DOCKER_BUILD_CMD[*]}
