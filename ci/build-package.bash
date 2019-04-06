#!/bin/bash
#
# Package builder (CI script)
# This script can only be run after the install-package.sh

# Stop on errors
set -o errexit

# Execute script only in a Docker environment
if [ "$DOCKER" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a Docker script in a non-Docker environment. Exiting script."
    exit 1
fi

echo -e "\e[35m\e[1m PACKAGE     = speech_recognition \e[0m"

# Use docker environment variables in all exec commands instead of script variables
# Compile the package
echo -e "\e[35m\e[1m Compile the package \e[0m"
cd "$TUE_ENV_DIR"/system/src/speech_recognition && catkin build --this --no-status
source ~/.bashrc

# Run unit tests
echo -e "\e[35m\e[1m Run unit test on this package (catkin run_test --this --no-deps) \e[0m"
cd "$TUE_ENV_DIR"/system/src/speech_recognition && catkin run_tests --this --no-status --no-deps

# Check results of unit tests
echo -e "\e[35m\e[1m Check results of unit test on this package (catkin_test_results build/$PACKAGE) \e[0m"
cd "$TUE_ENV_DIR"/system/ && [ ! -d build/speech_recognition ] || \
    catkin_test_results build/speech_recognition

