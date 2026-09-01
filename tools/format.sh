#!/bin/bash

BASE_DIR=$(dirname "$(realpath "$0")")/..

echo clang-format --style=google -i "$BASE_DIR/"*.{cc,h}

