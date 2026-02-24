#!/bin/bash

# Use the physical path of the script to find root
UTILS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
REPO_ROOT="$( cd "$UTILS_DIR/.." && pwd )"
MSG_CACHE="$REPO_ROOT/.gemini_msg"

copy_to_clipboard() {
    local CONTENT="$1"
    echo -n "$CONTENT" > "$MSG_CACHE"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo -n "$CONTENT" | pbcopy
    fi
}

commit_with_clipboard() {
    # Move to root explicitly
    cd "$REPO_ROOT" || exit 1
    
    # Stage EVERYTHING: modified, deleted, and untracked (like .gemini_msg)
    git add -A 
    
    local MSG=""
    if [ -f "$MSG_CACHE" ]; then
        MSG=$(cat "$MSG_CACHE")
    else
        MSG="chore: automated sync $(date +%Y%m%d_%H%M%S)"
    fi

    echo "💾 Staging and committing: $MSG"
    
    # Commit and Push
    if git commit -m "$MSG"; then
        echo "🚀 Pushing to origin..."
        git push
        rm -f "$MSG_CACHE"
    else
        echo "⚠️ Git reports nothing to commit."
    fi
}