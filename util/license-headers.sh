#! /bin/sh
# Copyright (c) 2026 Chris Lee and contributors.
# Licensed under the MIT license. See LICENSE file in the project root for details.

set -e
cd "$(dirname "$0")"/..
license-header-checker -r -i .git,.github,.pio,venv \
	-a ./util/license_header.txt . h cpp scad proto
license-header-checker -r -i .git,.github,.pio,venv \
	-a ./util/license_header_py.txt . py
