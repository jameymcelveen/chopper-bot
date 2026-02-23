#!/bin/bash
REPO_ROOT=$(git rev-parse --show-toplevel)
cd "$REPO_ROOT"

# Source the utils
source scripts/utils.sh

# Run the function
commit_with_clipboard