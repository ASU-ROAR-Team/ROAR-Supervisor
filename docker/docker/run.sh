#!/bin/bash

docker run -it --rm \
  --name roar \
  --cpus="1.0" \
  --memory="4g" \
  --pids-limit=256 \
  --network bridge \
  -v $(pwd)/../roar_ws:/roar_ws \
  roar-light:humble \
  bash
