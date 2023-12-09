#!/usr/bin/env bash
# Sets up a workspace in the current directory

set -e

# Get all the apt dependencies that are necessary
sudo apt-get update
sudo apt-get install -y \
        git \
        curl \
        libgsl-dev \
        nano \
        python3-catkin-tools \
        sox \
        python-matplotlib \
        vim

sudo -H pip3 install -U \
        dash \
        joblib \
        matplotlib \
        networkx \
        numpy \
        pandas \
        plotly \
        pydub \
        requests \
        ruamel.yaml \
        'scikit-learn>=0.20,<0.21' \
        sphinx \
        sphinx-argparse \
        sphinx_rtd_theme \
        treeinterpreter \
        vcstool

# Create the workspace directories