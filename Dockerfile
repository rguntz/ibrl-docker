# Use NVIDIA’s CUDA base image (CUDA 11.8, Ubuntu 22.04)
FROM nvidia/cuda:12.1.0-devel-ubuntu22.04

# Set working directory
WORKDIR /app
COPY . /app
    
# Prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && apt-get install -y \
    python3.9 python3.9-dev python3.9-distutils python3-pip \
    libgl1-mesa-dev libosmesa6-dev \
    libglib2.0-0 libglib2.0-dev libsm6 libxext6 libxrender-dev \
    patchelf build-essential wget unzip git && \
    update-alternatives --install /usr/bin/python python /usr/bin/python3.9 1 && \
    update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    rm -rf /var/lib/apt/lists/*

# Reset frontend back to default
ENV DEBIAN_FRONTEND=



# Install MuJoCo 2.1
RUN mkdir -p /root/.mujoco && \
    wget https://mujoco.org/download/mujoco210-linux-x86_64.tar.gz && \
    tar -xvzf mujoco210-linux-x86_64.tar.gz -C /root/.mujoco && \
    rm mujoco210-linux-x86_64.tar.gz

# Set environment variables
ENV PYTHONPATH=/app
ENV MUJOCO_PY_MUJOCO_PATH=/root/.mujoco/mujoco210
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia:/root/.mujoco/mujoco210/bin
ENV OMP_NUM_THREADS=1

# Install Python dependencies
RUN pip install --upgrade pip && \
    pip install torch==2.7.0 torchvision==0.22.0 torchaudio==2.7.0 --index-url https://download.pytorch.org/whl/cu128 && \
    pip install -r requirements.txt


# Default command
CMD ["bash"]