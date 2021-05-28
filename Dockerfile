FROM ubuntu:latest

ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/New_York

RUN apt-get update \
  && apt-get install -y ssh \
    build-essential \
    gcc \
    g++ \
    gdb \
    clang \
    cmake \
    rsync \
    tar \
    python \
    git libeigen3-dev libpcl-dev libyaml-cpp-dev libceres-dev \
  && apt-get clean

RUN mkdir -p /usr/src
RUN cd /usr/src && git clone https://github.com/KjellKod/g3log && cd g3log && mkdir build
RUN cd /usr/src/g3log/build && cmake ..
RUN cd /usr/src/g3log/build && make install

ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib:/usr/lib64:/usr/local/lib
COPY . /usr/src/oh_my_loam
RUN cd /usr/src/oh_my_loam && mkdir build && cd build && cmake .. && make

CMD /usr/src/oh_my_loam/build/examples/main_noros_csv ${CONFIG} ${INPUT_FOLDER} ${OUTPUT_FOLDER}