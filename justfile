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
    source /opt/ros/humble/setup.bash && colcon build --symlink-install

run_node node:
    source install/setup.bash && ros2 run pat_terminal {{node}}

test:
    source install/setup.bash && colcon test && colcon test-result --verbose

echo topic:
    source install/setup.bash && ros2 topic echo {{topic}}
