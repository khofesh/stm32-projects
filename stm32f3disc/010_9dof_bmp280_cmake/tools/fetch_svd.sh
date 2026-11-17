#!/usr/bin/env bash
# Fetch the STM32F303 SVD used by the cortex-debug peripheral view.
# The file is not vendored (1.3 MB); the URL is pinned to a cmsis-svd-data commit.
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
out=${root}/.vscode/STM32F303.svd
url=https://raw.githubusercontent.com/cmsis-svd/cmsis-svd-data/76387bff2f3b0da4b3752ed069a522367f465156/data/STMicro/STM32F303.svd

if [ -s "${out}" ]; then
  exit 0
fi

mkdir -p "$(dirname "${out}")"
curl -fsSL "${url}" -o "${out}.tmp"
mv "${out}.tmp" "${out}"
echo "SVD: ${out}"
