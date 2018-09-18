#!/bin/bash

echo -e "\e[35m\e[1mCreating docker image \e[0m"

docker build -t hero-test .
docker run -d --name hero-env -t hero-test:latest

