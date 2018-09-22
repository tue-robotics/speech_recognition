#!/bin/bash

echo -e "\e[33m\e[1mExecuting Unit Tests \e[0m"

docker exec hero-env ./_git/tests/digits.bash

