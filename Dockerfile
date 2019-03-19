# ---------------------------------------------------------------------
#       Dockerfile to build complete speech recognition ROS package
# ---------------------------------------------------------------------

# Set the base image to Kaldi base
FROM tuerobotics/kaldi:latest

# Build time arguments and their default values
ARG CI=false
ARG BRANCH=master
ARG PULL_REQUEST=false
ARG COMMIT=

# Update the image and install basic packages
RUN sudo apt-get update -qq && \
    # Set the CI args in the container as docker currently provides no method to
    # remove the environment variables
    # NOTE: The following exports will exist only in this container
    export CI=$CI && \
    export BRANCH=$BRANCH && \
    export PULL_REQUEST=$PULL_REQUEST && \
    export COMMIT=$COMMIT && \
    # Make tue-env available to the intermediate image
    # This step needs to be executed at every RUN step
    source ~/.bashrc && \
    # Install speech recognition target
    tue-get install ros-speech_recognition --branch="$BRANCH" && \
    # Select appropriate commit when in CI with (out) PR
    "$TUE_ENV_DIR"/system/src/speech_recognition/ci/install-package.bash && \
    # Build the ROS package and perform unit tests
    "$TUE_ENV_DIR"/system/src/speech_recognition/ci/build-package.bash && \
    # Remove Kaldi source files to reduce image size
    rm -rf "$KALDI_ROOT"/.git "$KALDI_ROOT"/windows "$KALDI_ROOT"/misc && \
    find /opt/kaldi/src/ -type f -not -name '*.so' -delete && \
    find /opt/kaldi/tools/ -type f \( -not -name '*.so' -and -not -name '*.so*' \) -delete && \
    sudo apt-get clean autoclean && \
    sudo apt-get autoremove -y && \
    sudo rm -rf /var/lib/apt/lists/*

