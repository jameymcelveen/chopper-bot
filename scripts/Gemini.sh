#!/bin/bash

# --- Configuration ---
PROJECT_ROOT=$(pwd)
AI_SEED_FILE="ai.md"

echo "💾 Seeding Project Context & Adding Clipboard Automation..."

# 1. Create/Update ai.md (The Project "Memory Bank")
# This file is for seeding the AI in future sessions
cat <<EOF > "$AI_SEED_FILE"
# Chopper-Bot Project Context (Seed File)
- **Target Hardware:** NVIDIA Jetson Orin Nano (8GB)
- **Architecture:** ROS 2 Humble / Docker (ARM64) / Mac Dev Host
- **Naming Convention:** Dash-case for paths/files, underscore for Python packages.
- **Spektrum Logic:** DSMX Satellite via UART (/dev/ttyTHS1), 115200 baud, 16-byte frames.
- **Safety:** Killswitch on Ch5 (Aux1).
- **Current Status:** Spektrum parsing implemented, Motor Node scaffolded.
EOF

# 2. Add the Git Commit Clipboard Helper to the end of THIS script
# We'll use a temporary file to hold the suggested message
COMMIT_MSG="feat: implement ai.md seed and pbcopy automation for mac workflow"

# Check if we are on a Mac to avoid errors on the Jetson
if [[ "$OSTYPE" == "darwin"* ]]; then
    echo -n "$COMMIT_MSG" | pbcopy
    echo "📋 Suggested commit message copied to Mac clipboard: \"$COMMIT_MSG\""
else
    echo "📝 Suggested commit message (No pbcopy on Linux): \"$COMMIT_MSG\""
fi

# Ensure gitignore handles the seed if needed (optional)
# if ! grep -q "ai.md" .gitignore; then echo "ai.md" >> .gitignore; fi