# MIT License
#
# # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ubuntu:20.04
LABEL maintainer="Ignacio Vizzo <ivizzo@uni-bonn.de>"

# setup environment
ENV TERM xterm
ENV DEBIAN_FRONTEND=noninteractive

# Install essentials
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    ccache \
    clang-format \
    cmake \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python apt dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3 \
    python3-numpy \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python pip dependencies
RUN pip3 install --upgrade pip
RUN pip3 install --upgrade \
    black \
    numpy \
    pytest \
    setuptools \
    twine \
    wheel

# Install C++ Dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
    libblosc-dev \
    libboost-iostreams-dev \
    libboost-numpy-dev \
    libboost-python-dev \
    libboost-system-dev \
    libeigen3-dev \
    libtbb-dev \
    python-dev \
    python-numpy \
    && rm -rf /var/lib/apt/lists/*

# Install OpenVDB from source, with Python bindings
# TODO: Update these once changes has been merged to upstream
# RUN git clone --depth 1 https://github.com/AcademySoftwareFoundation/openvdb.git -b v9.0.0 \
RUN git clone --depth 1 https://github.com/nachovizzo/openvdb.git -b nacho/vdbfusion \
    && cd openvdb \
    && mkdir build && cd build \
    && cmake \
    -DOPENVDB_BUILD_PYTHON_MODULE=ON \
    -DUSE_NUMPY=ON \
    -DPYOPENVDB_INSTALL_DIRECTORY="/usr/local/lib/python3.8/dist-packages" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DUSE_ZLIB=OFF \
    ..\
    && make -j$(nproc) all install \
    && rm -rf /openvdb

