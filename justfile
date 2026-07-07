# These are a collection of commonly used commands as part of the dev cycle.
#
# Commands run in host:      build_docker / run_docker / attach / stop_docker
# Commands run in container: everything else

set shell := ["bash", "-c"]

repo := justfile_directory()

default:
    @just --list

# run these in host

build_docker:
    docker build -t pat {{repo}}

run_docker:
    docker start pat-dev 2>/dev/null || docker run -d --name pat-dev \
        -v {{repo}}/src:/ws/src \
        -v {{repo}}/justfile:/ws/justfile \
        pat sleep infinity

stop_docker:
    docker rm -f pat-dev

attach:
    docker exec -it pat-dev bash

# run these in docker container

build:
    colcon build --symlink-install && \
    echo "To complete installation, run: source install/setup.sh"

run_node node:
    ros2 run pat_terminal {{node}}

run_sim:
    ros2 run plant_sim plant_sim_node

test:
    colcon test && colcon test-result --verbose

auto_test:
    while true; do \
        colcon build --symlink-install && just test; \
        inotifywait -qq -r -e modify,create,delete,move src; \
    done

# simulate host commands
set_mode mode:
    ros2 service call /set_mode pat_interfaces/srv/SetMode "{mode: {{mode}}}"
