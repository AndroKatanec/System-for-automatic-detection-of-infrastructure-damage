#!/bin/bash

XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth
touch $XAUTH
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -

echo "Running Docker Container"
CONTAINER_NAME=dipl_sim_container
IMAGE_NAME="androkatanec/dipl_sim_final_v7"

for (( i=1; i<=$#; i++));
do
  param="${!i}"

  if [ "$param" == "--image" ]; then
    j=$((i+1))
    IMAGE_NAME="${!j}"
  fi

done

if [ -z "$IMAGE_NAME" ]; then
  echo "ERROR: Docker image name not provided!"
  exit 1
fi

# Get distro of the built image
distro=$(docker images $IMAGE_NAME | tail -n1 | awk '{print $2}')
run_args=""

distro="focal"

echo "Running in $distro"

# Check if there is an already running container with the same distro
full_container_name="${CONTAINER_NAME}_${distro}"
running_container="$(docker container ls -al | grep $full_container_name)"
if [ -z "$running_container" ]; then
  echo "Running $full_container_name for the first time!"
else
  echo "Found an open $full_container_name container. Starting and attaching!"
  eval "docker start $full_container_name"
  eval "docker attach $full_container_name"
  exit 0
fi

# Check if using GPU
gpu_enabled="--gpus all"
if [ "$distro" == "focal-nogpu" ]; then
  gpu_enabled=""
fi

docker run \
  $run_args \
  -it \
  --network host \
  --privileged \
  $gpu_enabled \
  --volume=$XSOCK:$XSOCK:rw \
  --volume=$XAUTH:$XAUTH:rw \
  --env="XAUTHORITY=${XAUTH}" \
  --env DISPLAY=$DISPLAY \
  --env TERM=xterm-256color \
  --name $full_container_name \
  $IMAGE_NAME \
  /bin/bash

