#!/bin/bash

# Function to copy to clipboard
copy_to_clipboard() {
    local CONTENT="$1"
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo -n "$CONTENT" | pbcopy
    elif command -v clip.exe &> /dev/null; then
        echo -n "$CONTENT" | clip.exe
    elif command -v xclip &> /dev/null; then
        echo -n "$CONTENT" | xclip -selection clipboard
    else
        echo "⚠️ No clipboard utility found."
    fi
}

# Function to get content from clipboard
paste_from_clipboard() {
    if [[ "$OSTYPE" == "darwin"* ]]; then
        pbpaste
    elif command -v powershell.exe &> /dev/null; then
        powershell.exe -command "Get-Clipboard"
    elif command -v xclip &> /dev/null; then
        xclip -selection clipboard -o
    else
        echo "unknown_commit_msg"
    fi
}

# New function to handle the git flow
commit_with_clipboard() {
    local MSG=$(paste_from_clipboard)
    if [ "$MSG" == "unknown_commit_msg" ] || [ -z "$MSG" ]; then
        echo "❌ Error: Clipboard empty or utility missing."
        return 1
    fi
    
    echo "💾 Committing with message: $MSG"
    git add .
    git commit -m "$MSG"
    git push
}