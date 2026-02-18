#!/bin/bash

docker run -it --rm \
  --name roar \
  --cpus="1.0" \
  --memory="4g" \
  --pids-limit=256 \
  --network bridge \
  -v $(pwd)/roar_ws:/roar_ws \
  -v $HOME/.ssh:/root/.ssh:ro \
  roar-light:humble \
  bash -c "
    source /opt/ros/humble/setup.bash

    cd /roar_ws/src

    if [ ! -d ROAR-Supervisor ]; then
        echo 'Cloning ROAR-Supervisor...'
        git clone git@github.com:ASU-ROAR-Team/ROAR-Supervisor.git
    else
        echo 'ROAR-Supervisor already exists.'
    fi

    exec bash
  "
