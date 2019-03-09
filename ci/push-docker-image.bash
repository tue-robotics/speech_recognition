#! /usr/bin/env bash

# Stop on errors
set -o errexit

# Execute script only in a CI environment
if [ "$CI" != "true" ]
then
    echo -e "\e[35m\e[1m Error!\e[0m Trying to execute a CI script in a non-CI environment. Exiting script."
    exit 1
fi

# push the new Docker image to the Docker registry only after acceptance of pull request
if [ $TRAVIS_PULL_REQUEST == "false" ]
then
    # authenticate with the Docker Hub registry
    docker login -u="$DOCKER_HUB_USERNAME" -p="$DOCKER_HUB_PASSWORD"

    echo -e "\e[35m\e[1m docker push $IMAGE_NAME \e[0m"
    docker push $IMAGE_NAME

    echo -e "\e[35m\e[1m Succeeded, see: \e[0m"
    docker images
fi

