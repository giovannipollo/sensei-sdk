#!/bin/bash
# Copyright (c) 2022 Philip Schilk.
# SPDX-License-Identifier: Apache-2.0

mkdir -p $(dirname $0)/../.git/hooks
cp $(dirname $0)/pre-commit $(dirname $0)/../.git/hooks/pre-commit
