#!/bin/bash

# --- Configuration & Includes ---
source scripts/utils.sh 2>/dev/null || echo "First run: utils not sourced yet."

echo "🤖 Updating Chopper's Memory & Tools..."

# 1. Handle Public vs Private AI files
cat <<EOF > ai.md
# Chopper-Bot (Public)
- Architecture: ROS 2 Humble / Jetson Orin Nano
- Control: Spektrum DSMX (UART)
- Naming: dash-case paths, underscore_packages
EOF

if [ ! -f "ai-private.md" ]; then
    echo "# Chopper-Bot (Private Settings)" > ai-private.md
    echo "ai-private.md" >> .gitignore
    echo "🔒 Created private ai-private.md and added to .gitignore"
fi

# 2. Add Clipboard Logic to the patch
COMMIT_MSG="feat: refactor gemini.sh to use utility includes and dual-file ai memory"
copy_to_clipboard "$COMMIT_MSG"

echo "✅ Patched. Now run 'make gemini' to commit and push."