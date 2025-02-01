help:
	@echo ""
	@echo "About:"
	@echo "This Makefile is a utility script for managing this Dockerized application efficiently."
	@echo "Unrelated to GCC or code compilation, it acts similarly to a Bash script by automating"
	@echo "repetitive tasks using shell commands, but leverages the structure and simplicity of make."
	@echo "It provides clear commands for common Docker tasks like building the image, starting and "
	@echo "stopping a container, logging, and accessing a shell inside the container."
	@echo ""

	@echo "Usage:"
	@echo "make build            - build the container image"
	@echo "make start            - start the container with verbose output"
	@echo "make start-quiet      - start the container without verbose output"
	@echo "make shell            - open a shell in the container"
	@echo "make stop             - stop the container"
	@echo ""

build:
	@echo "Building the ROS bridge container..."
	@docker compose -f ./docker/compose.yaml build

start:
	@echo "Starting the ROS bridge container..."
	@docker compose -f ./docker/compose.yaml up

start-quiet:
	@echo "Starting the ROS bridge container quietly..."
	@docker compose -f ./docker/compose.yaml up -d

shell:
	@docker exec -ti ros_bridge bash -l

stop:
	@echo "Stopping the ROS bridge container..."
	@docker compose -f ./docker/compose.yaml down