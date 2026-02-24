#!/bin/bash
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$SCRIPT_DIR/.." && pwd )"
cd "$REPO_ROOT"

source "$REPO_ROOT/scripts/utils.sh"

# ... rest of your script logic ...

COMMIT_MSG="fix: implement file-based commit message cache to prevent clipboard overwrites"
copy_to_clipboard "$COMMIT_MSG"