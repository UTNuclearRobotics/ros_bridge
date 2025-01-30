
build:
	@echo "Building the ROS bridge container..."
	@docker compose -f docker-compose.yaml build

start:
	@echo "Starting the ROS bridge container..."
	@docker compose -f docker-compose.yaml up

stop:
	@echo "Stopping the ROS bridge container..."
	@docker compose -f docker-compose.yaml down

shell:
	@docker exec -ti ros_bridge_c bash -l