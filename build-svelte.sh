#! /bin/sh
# Copyright (c) 2026 Chris Lee and contributors.
# Licensed under the MIT license. See LICENSE file in the project root for details.

set -e
here="$(readlink -f "$(dirname "$0")")"
cd "$here/svelte"
npm run build
npx --yes svelteesp32 -e psychic -s dist -o ../src/svelteesp32async.h --etag=true
