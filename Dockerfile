# ---------------------------------------------------------------------
#       Dockerfile to build complete speech recognition ROS package
# ---------------------------------------------------------------------

# Set the base image to Kaldi base
FROM tueroboticsamigo/kaldi:latest

# Build time arguments and their default values
ARG CI=false
ARG BRANCH=
ARG COMMIT=
ARG PULL_REQUEST=false

ENV CI=$CI \
    BRANCH=$BRANCH \
    PULL_REQUEST=$PULL_REQUEST \
    COMMIT=$COMMIT

# Update the image and install basic packages
RUN sudo apt-get update -qq && \
    # Make tue-env available to the intermediate image
    # This step needs to be executed at every RUN step
    source ~/.bashrc && \
    # Install speech recognition target
    tue-get install ros-speech_recognition --branch="$BRANCH" && \
    # Select appropriate commit when in CI with (out) PR
    "$TUE_ENV_DIR"/system/src/speech_recognition/ci/install-package.bash && \
    # Build the ROS package and perform unit tests
    "$TUE_ENV_DIR"/system/src/speech_recognition/ci/build-package.bash



