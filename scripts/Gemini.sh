#!/bin/bash

if ! grep -q "nvidia" docker-compose.yml; then
    echo "🏎️ Enabling NVIDIA GPU runtime in docker-compose.yml..."
    # This inserts the runtime: nvidia line into the chopper service
    sed -i '' '/container_name: chopper-dev/a \
    runtime: nvidia' docker-compose.yml
fi