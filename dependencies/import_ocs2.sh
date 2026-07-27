#!/usr/bin/env bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repository_root=$(cd -- "$script_dir/.." && pwd)
source_root=${1:-"$repository_root/third_party"}

mkdir -p "$source_root"
vcs import "$source_root" < "$script_dir/ocs2_humble.repos"

for patch in "$script_dir"/patches/ocs2/*.patch; do
  git -C "$source_root/ocs2" apply "$patch"
done

test "$(git -C "$source_root/ocs2" rev-parse HEAD)" = \
  "243b80b8de5a427d7237136786821723aa4e792a"
