#!/bin/bash
xhost +

docker ps -a | grep kuavo_gendexgrasp_dev
if [ $? -eq 0 ]; then
    docker stop kuavo_gendexgrasp_dev
    docker rm kuavo_gendexgrasp_dev
fi

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PARENT_DIR="$(dirname "$SCRIPT_DIR")"
mkdir -p "$PARENT_DIR/.ccache"

CONTAINER_NAME="kuavo_gendexgrasp_dev"
# IMAGE_NAME="kuavo/gendexgrasp-dev:v5.0" 
IMAGE_NAME="kkxiaokang1234/kuavo-gendexgrasp-dev:v1.0"

# 映射 NVIDIA 驱动库和 CUDA 相关工具
# DOCKER_ARGS+=("-v /usr/local/cuda:/usr/local/cuda")  # 映射 CUDA 目录
# DOCKER_ARGS+=("-v /usr/local/cuda-12:/usr/local/cuda-12")  # 映射 CUDA 目录
# DOCKER_ARGS+=("-v /usr/local/cuda-12.2:/usr/local/cuda-12.2")  # 映射 CUDA 目录

docker run -it --gpus all --runtime=nvidia --net host \
    ${DOCKER_ARGS[@]} \
    --name $CONTAINER_NAME \
    --privileged \
    -v /dev:/dev \
    -v "${HOME}/.ros:/root/.ros" \
    -v "$PARENT_DIR/.ccache:/root/.ccache" \
    -v "$PARENT_DIR:/root/kuavo_ws" \
    -v "${HOME}/.config/lejuconfig:/root/.config/lejuconfig" \
    -v "${HOME}/GenDexGrasp/Gendexgrasp_ros_ok/:/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/" \
    -v "${HOME}/GenDexGrasp/curobot_ros_ws/bridge_ws/:/home/lab/GenDexGrasp/curobot_ros_ws/bridge_ws/" \
    -e ROS_MASTER_URI=http://192.168.0.147:11311 \
    -e ROS_IP=192.168.0.147 \
    --rm --workdir ${HOME}/GenDexGrasp/Gendexgrasp_ros_ok/ \
    --group-add=dialout \
    --ulimit rtprio=99 \
    --cap-add=sys_nice \
    -e DISPLAY=$DISPLAY \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ${IMAGE_NAME} \
    bash

# 设置 ROS 环境变量
export ROS_IP=192.168.0.147
export ROS_MASTER_URI=http://192.168.0.147:11311