FROM ubuntu:18.04

ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i -r 's|(archive\|security)\.ubuntu\.com/|ftp.jaist.ac.jp/pub/Linux/|' /etc/apt/sources.list && \
    apt-get update && apt-get upgrade -y && \
    apt-get install -y build-essential apt-utils ca-certificates \
    cmake git pkg-config software-properties-common \
    libswscale-dev wget autoconf automake unzip curl \
    python-dev python-pip libavcodec-dev libavformat-dev libgtk2.0-dev libv4l-dev &&\
    # SDK dependency
    add-apt-repository ppa:nilarimogard/webupd8 && \
    apt-get update && apt-get install -y libvdpau-va-gl1 i965-va-driver vdpauinfo && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN rm -rf /app/opencv-${OPENCV_VERSION}

WORKDIR /app
COPY . /app

#RUN echo "--- installing zense sdk ---"
RUN mkdir -p /etc/udev/rules.d
ENV PICOZENSE_PARENT_DIR="Vzense_SDK_linux"
ENV PICOZENSE_LIB="${PICOZENSE_PARENT_DIR}/Ubuntu18.04"
ENV PICOZENSE_INSTALL_DIR=/usr/local/PicoZenseSDK_V3

# Installing Zense SDK v3
RUN git clone https://github.com/Vzense/Vzense_SDK_linux.git && \
    cd Vzense_SDK_linux && \
    git checkout 0731133877a1674cc07cf85d05b2d0c4cf6f50cf
    

RUN ./install_zense_sdk.sh
RUN rm -rf "${PICOZENSE_PARENT_DIR}"

CMD [ /bin/bash ]