FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i -r 's|(archive\|security)\.ubuntu\.com/|ftp.jaist.ac.jp/pub/Linux/|' /etc/apt/sources.list && \
    apt-get update && apt-get upgrade -y && \    
    apt-get install -y build-essential apt-utils ca-certificates \
	pigz cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev \
	python-dev python-numpy python-pip \
    libhdf5-100 libhdf5-cpp-100 libhdf5-dev \
	libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
ENV OPENCV_VERSION="3.4.3"
RUN mkdir -p /app/opencv-$OPENCV_VERSION/build
RUN curl -L https://github.com/opencv/opencv/archive/$OPENCV_VERSION.tar.gz | tar xz && curl -L https://github.com/opencv/opencv_contrib/archive/$OPENCV_VERSION.tar.gz | tar xz

WORKDIR /app/opencv-$OPENCV_VERSION/build
RUN cmake -DWITH_TBB=ON \
    -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib-$OPENCV_VERSION/modules \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DCMAKE_BUILD_TYPE=RELEASE \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DPYTHON_EXECUTABLE=$(which python) \
    -DPYTHON_INCLUDE_DIR=$(python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())") \
    -DPYTHON_PACKAGES_PATH=$(python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") \
     .. 

RUN make -j4 install && make clean && ldconfig

RUN rm -rf /app/opencv-${OPENCV_VERSION}

WORKDIR /app
RUN echo "--- installing zense sdk ---"
COPY . /app
RUN mkdir -p /etc/udev/rules.d
RUN ./install_zense_sdk.sh

RUN ./setup.sh


CMD ["/bin/bash"]