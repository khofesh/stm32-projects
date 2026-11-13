#!/usr/bin/env bash
# AC-10: build the same tree twice from different absolute paths and compare
# the artifacts. Uses the working tree, so run it after committing a release.
set -euo pipefail

proj=$(cd "$(dirname "$0")/.." && pwd)
root=$(git -C "${proj}" rev-parse --show-toplevel)
rel=$(git -C "${proj}" rev-parse --show-prefix)
strip=$(awk -F/ '{print NF}' <<< "${rel%/}")
tmp=$(mktemp -d "${TMPDIR:-/tmp}/repro.XXXXXX")
trap 'rm -rf "${tmp}"' EXIT

for tree in a deeper/second/tree; do
  dest="${tmp}/${tree}/project"
  mkdir -p "${dest}"
  tar -C "${root}" --exclude=build --exclude=temp -cf - "${rel}" \
    | tar -x -C "${dest}" --strip-components="${strip}"
  (
    cd "${dest}"
    cmake --preset ci >/dev/null
    cmake --build --preset ci >/dev/null
  )
done

a="${tmp}/a/project/build/ci/010_9dof_bmp280_cmake"
b="${tmp}/deeper/second/tree/project/build/ci/010_9dof_bmp280_cmake"

for ext in elf hex bin; do
  ha=$(sha256sum "${a}.${ext}" | cut -d' ' -f1)
  hb=$(sha256sum "${b}.${ext}" | cut -d' ' -f1)
  printf '%-4s %s\n' "${ext}" "${ha}"
  if [ "${ha}" != "${hb}" ]; then
    echo "MISMATCH: ${ext} differs between the two build paths" >&2
    exit 1
  fi
done

echo "Reproducible: artifacts are byte-identical across build paths"
