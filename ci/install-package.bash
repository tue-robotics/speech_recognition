#!/bin/bash
#
# Package installer (CI script)
# This script uses the Docker image of tue-env and installs the current git
# repository as a tue-env package using tue-install in the CI

# Stop on errors
set -o errexit

# Standard argument parsing, example: install-package --branch=master --package=ros_robot

if [ "$CI" == "true" -a "$DOCKER" == "true" ]
then
    echo -e "\e[35m\e[1m BRANCH       = ${BRANCH} \e[0m"
    echo -e "\e[35m\e[1m COMMIT       = ${COMMIT} \e[0m"
    echo -e "\e[35m\e[1m PULL_REQUEST = ${PULL_REQUEST} \e[0m"

    echo -e "\e[35m\e[1m
    This build can be reproduced locally using the following commands:

    tue-get install docker
    ~/.tue/ci/install-package.sh --package=${PACKAGE} --branch=${BRANCH} --commit=${COMMIT} --pullrequest=${PULL_REQUEST}
    ~/.tue/ci/build-package.sh --package=${PACKAGE}

    Optionally fix your compilation errors and re-run only the last command
    \e[0m"

    # Set the package to the right commit
    echo -e "\e[35m\e[1m Reset package to this commit \e[0m"
    if [ $PULL_REQUEST == "false" ]
    then
        echo -e "\e[35m\e[1m cd $TUE_ENV_DIR/system/src/$PACKAGE && git reset --hard $COMMIT \e[0m"

        cd "$TUE_ENV_DIR"/system/src/speech_recognition && git reset --hard "$COMMIT"
    else
        echo -e "\e[35m\e[1m cd $TUE_ENV_DIR/system/src/$PACKAGE && git fetch origin pull/$PULL_REQUEST/head:PULLREQUEST && git checkout PULLREQUEST \e[0m"

        cd "$TUE_ENV_DIR"/system/src/speech_recognition && git fetch origin pull/"$PULL_REQUEST"/head:PULLREQUEST && git checkout PULLREQUEST
    fi
else
    echo "[install-package] At the latest commit of branch: $BRANCH"
fi

