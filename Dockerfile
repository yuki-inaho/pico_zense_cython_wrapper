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
    apt-get update && apt-get install -y libvdpau-va-gl1 i965-va-driver vdpauinfo libvdpau-dev&& \
    rm -rf /var/lib/apt/lists/*

WORKDIR /app
RUN rm -rf /app/opencv-${OPENCV_VERSION}

WORKDIR /app
COPY . /app

RUN mkdir -p /etc/udev/rules.d

# Installing Zense SDK V3.4.4:1
ENV VZENSE_COMMIT_ID="0188093f085655965e899b47ad69b39d21e81f71"

ENV PICOZENSE_PARENT_DIR="Vzense_SDK_linux"
ENV PICOZENSE_LIB="${PICOZENSE_PARENT_DIR}/Ubuntu18.04"
ENV PICOZENSE_INSTALL_DIR=/usr/local/PicoZenseSDK_V3

RUN curl -L "https://github.com/Vzense/Vzense_SDK_Linux/archive/${VZENSE_COMMIT_ID}.tar.gz" | \
    tar -zx "Vzense_SDK_Linux-${VZENSE_COMMIT_ID}/Ubuntu18.04" && \
    mv "Vzense_SDK_Linux-${VZENSE_COMMIT_ID}" "${PICOZENSE_PARENT_DIR}"

RUN pip install -U pip
RUN pip install pyrsistent==0.16.1
RUN ./install_zense_sdk.sh
RUN rm -rf "${PICOZENSE_PARENT_DIR}"

CMD [ /bin/bash ]