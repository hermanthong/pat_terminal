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

# the full demo: sim + all four terminal nodes
launch:
    ros2 launch plant_sim simulation_launch.py

test:
    colcon test && colcon test-result --verbose

auto_test:
    while true; do \
        colcon build --symlink-install && just test; \
        inotifywait -qq -r -e modify,create,delete,move src; \
    done

# simulate host commands
# IDLE = 0,
# ACQUIRE = 1,
# HANDOFF = 2,
# LOCK = 3,
# COAST = 4,
# SAFE = 5,

set_mode mode:
    ros2 service call /set_mode pat_interfaces/srv/SetMode "{mode: {{mode}}}"

# script the demo blockage: just blockage true / just blockage false
blockage state:
    ros2 topic pub --once /blockage std_msgs/msg/Bool "{data: {{state}}}"

# move the counterpart's true bearing, eg. `just set_bearing 0.1 0.05`
set_bearing azimuth elevation:
    ros2 service call /set_bearing pat_interfaces/srv/SetBearing "{azimuth: {{azimuth}}, elevation: {{elevation}}}"
