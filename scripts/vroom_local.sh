#!/usr/bin/env bash
# Build and run the VROOM binary from this repository inside the
# "deps" stage of vroom-custom/Dockerfile.local, without rebuilding an
# image on every change. Sources and object files live in a named
# Docker volume so rebuilds are incremental.
#
#   scripts/vroom_local.sh image             # build the vroom-deps image (once)
#   scripts/vroom_local.sh build [make args] # compile bin/vroom in the volume
#   scripts/vroom_local.sh run <vroom args>  # run the binary, repo mounted at /work
#   scripts/vroom_local.sh shell             # interactive shell in the build env
#
# Examples:
#   scripts/vroom_local.sh build -j8
#   scripts/vroom_local.sh run -i /work/docs/waste_example.json -o /work/out.json
#
# Environment: VROOM_BUILD_VOLUME (default vroom-build), VROOM_DEPS_IMAGE
# (default vroom-deps), VROOM_APPLY_PATCH=1 to also apply the
# route-duration-quadratic patch before compiling.

set -euo pipefail

cd "$(dirname "$0")/.."

IMAGE="${VROOM_DEPS_IMAGE:-vroom-deps}"
VOLUME="${VROOM_BUILD_VOLUME:-vroom-build}"

# Docker Desktop on Windows needs a Windows-style host path and no
# MSYS path mangling of the container-side path.
if command -v cygpath > /dev/null 2>&1; then
  HOST_DIR="$(cygpath -w "$PWD")"
  export MSYS_NO_PATHCONV=1
else
  HOST_DIR="$PWD"
fi

run_in_env() {
  docker run --rm -i \
    -v "${HOST_DIR}:/work" \
    -v "${VOLUME}:/build" \
    -w /build \
    "$IMAGE" bash -c "$1"
}

# Copy a source tree into the volume with LF line endings, replacing
# only files whose content changed so that make stays incremental.
SYNC='
sync_tree() {
  cd /work
  find "$1" -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" -o -name makefile \) |
  while read -r f; do
    tmp=$(mktemp)
    sed "s/\r$//" "$f" > "$tmp"
    if ! cmp -s "$tmp" "/build/$f"; then
      mkdir -p "$(dirname "/build/$f")"
      mv "$tmp" "/build/$f"
    else
      rm -f "$tmp"
    fi
  done
}
sync_tree src
sync_tree include
'

cmd="${1:-}"
shift || true

case "$cmd" in
  image)
    docker build --target deps -t "$IMAGE" -f vroom-custom/Dockerfile.local .
    ;;
  build)
    patch_cmd="true"
    if [ "${VROOM_APPLY_PATCH:-0}" = "1" ]; then
      patch_cmd="cd /build && patch -p1 -N < /work/vroom-custom/route-duration-quadratic.patch"
    fi
    run_in_env "$SYNC
      $patch_cmd &&
      make -C /build/src $*"
    ;;
  run)
    run_in_env "/build/bin/vroom $*"
    ;;
  shell)
    docker run --rm -it -v "${HOST_DIR}:/work" -v "${VOLUME}:/build" -w /build "$IMAGE" bash
    ;;
  *)
    sed -n '2,19p' "$0"
    exit 1
    ;;
esac
