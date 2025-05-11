ARG UBUNTU="22.04"
ARG CUDA="12.1.1"
ARG CUDNN="8"

FROM nvidia/cuda:${CUDA}-cudnn${CUDNN}-devel-ubuntu${UBUNTU}

ENV DEBIAN_FRONTEND noninteractive
ENV TZ="Asia/Tokyo"

RUN rm -f /etc/apt/sources.list.d/cuda.list \
    && apt-get update && apt-get install -y --no-install-recommends wget \
    && wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb \
    && dpkg -i cuda-keyring_1.1-1_all.deb \
    && rm -f cuda-keyring_1.1-1_all.deb \
    && apt-get purge --autoremove -y wget \
    && rm -rf /var/lib/apt/lists/ 

RUN apt-get update && \
    apt-get install -y ffmpeg libsm6 libxext6 git ninja-build libglib2.0-0 libsm6 libxrender-dev libxext6 wget \
    software-properties-common \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*
    
RUN add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y python3.10 python3.10-distutils && \
    apt-get install -y python3-pip && \
    rm -f /usr/bin/python && \
    ln -s /usr/bin/python3.10 /usr/bin/python
    
RUN pip install torch==2.5.1 torchvision==0.20.1 torchaudio==2.5.1 --index-url https://download.pytorch.org/whl/cu121
RUN pip install ipdb
RUN pip install numpy>=1.24.4 tqdm>=4.66.1 hydra-core>=1.3.2 iopath>=0.1.10 pillow>=9.4.0 matplotlib>=3.9.1 jupyter>=1.0.0 opencv-python>=4.7.0
