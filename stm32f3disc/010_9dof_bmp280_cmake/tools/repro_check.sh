#!/usr/bin/env bash
# AC-10: build a commit (default HEAD) twice from different absolute paths and
# compare the artifacts. Git worktrees keep .git reachable, so both builds embed
# the real `git describe` (the same BUILD_GIT_DESC as a checkout of that commit).
# Uncommitted changes are not included. Usage: repro_check.sh [<commit>]
set -euo pipefail

ref=${1:-HEAD}

proj=$(cd "$(dirname "$0")/.." && pwd)
root=$(git -C "${proj}" rev-parse --show-toplevel)
rel=$(git -C "${proj}" rev-parse --show-prefix)
tmp=$(mktemp -d "${TMPDIR:-/tmp}/repro.XXXXXX")
trees=("${tmp}/a" "${tmp}/deeper/second/tree")

cleanup() {
  for t in "${trees[@]}"; do
    git -C "${root}" worktree remove --force "${t}" 2>/dev/null || true
  done
  git -C "${root}" worktree prune
  rm -rf "${tmp}"
}
trap cleanup EXIT

if [ "${ref}" = "HEAD" ] && [ -n "$(git -C "${proj}" status --porcelain -- .)" ]; then
  echo "warning: uncommitted changes are not part of the check (building HEAD)" >&2
fi

for t in "${trees[@]}"; do
  mkdir -p "$(dirname "${t}")"
  git -C "${root}" worktree add --detach --quiet "${t}" "${ref}"
  (
    cd "${t}/${rel}"
    cmake --preset ci >/dev/null
    cmake --build --preset ci >/dev/null
  )
done

a="${trees[0]}/${rel}build/ci/010_9dof_bmp280_cmake"
b="${trees[1]}/${rel}build/ci/010_9dof_bmp280_cmake"

desc=$(sed -n 's/^#define BUILD_GIT_DESC "\(.*\)"$/\1/p' "${trees[0]}/${rel}build/ci/gen/build_info.h")
echo "BUILD_GIT_DESC ${desc}"
if [ "${desc}" = "unknown" ]; then
  echo "FAIL: git describe did not run in the worktree build" >&2
  exit 1
fi

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
