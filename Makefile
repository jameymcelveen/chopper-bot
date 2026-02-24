# --- Variables ---
DOCKER_COMPOSE = docker compose
CONTAINER_NAME = chopper-dev
WORKSPACE_DIR = /ros_ws

# --- Docker Commands ---
.PHONY: build
build: ## Build the Docker image
	$(DOCKER_COMPOSE) build

.PHONY: up
up: ## Start the container in the background
	$(DOCKER_COMPOSE) up -d

.PHONY: down
down: ## Stop the container
	$(DOCKER_COMPOSE) down

.PHONY: shell
shell: ## Enter the running container shell
	docker exec -it $(CONTAINER_NAME) bash

# --- ROS 2 Commands (Running inside Docker) ---
.PHONY: ros-build
ros-build: ## Build the ROS 2 workspace inside the container
	docker exec -it $(CONTAINER_NAME) bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

.PHONY: clean
clean: ## Remove build, install, and log folders
	rm -rf build/ install/ log/

.PHONY: test
test: ## Run ROS 2 tests
	docker exec -it $(CONTAINER_NAME) bash -c "source install/setup.bash && colcon test"

# --- Gemini Commands ---
.PHONY: gemini
gemini: ## Run the patcher and auto-commit
	python3 scripts/sync.py

# --- Launch Commands ---
.PHONY: run-spektrum
run-spektrum: ## Launch the Spektrum Bridge node
	docker exec -it $(CONTAINER_NAME) bash -c "source install/setup.bash && ros2 run chopper_brain spektrum_node"

.PHONY: help
help: ## Display this help message
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-20s\033[0m %s\n", $$1, $$2}'