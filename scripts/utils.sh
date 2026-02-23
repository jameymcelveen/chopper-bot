#!/bin/bash

copy_to_clipboard() {
    local CONTENT="$1"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo -n "$CONTENT" | pbcopy
    else
        # Fallback for Linux environments
        echo -n "$CONTENT" | xclip -selection clipboard 2>/dev/null || echo -n "$CONTENT" > /tmp/perry_msg
    fi
}

paste_from_clipboard() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        pbpaste
    else
        cat /tmp/perry_msg 2>/dev/null || echo "manual commit"
    fi
}

commit_with_clipboard() {
    local MSG=$(paste_from_clipboard)
    
    if [ -z "$MSG" ]; then
        echo "❌ Error: Clipboard is empty. Cannot commit."
        exit 1
    fi
    
    echo "💾 Attempting to commit with message: '$MSG'"
    
    git add .
    # The -m flag needs the variable quoted to handle spaces
    if git commit -m "$MSG"; then
        echo "✅ Commit successful. Pushing..."
        git push
    else
        echo "⚠️ Git commit failed (maybe nothing to commit?)."
    fi
}