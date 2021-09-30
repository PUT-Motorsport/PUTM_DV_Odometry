sudo docker run -it --gpus all --privileged \
                --env="DISPLAY=:0" \
                --env="QT_X11_NO_MITSHM=1" \
                --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                --env="XAUTHORITY=$XAUTH" \
                --volume="/tmp/.docker.xauth:/tmp/.docker.xauth" \
                --volume="$SHARED:/home/ue4/share:rw" \
                --env="NVIDIA_VISIBLE_DEVICES=all" \
                --env="NVIDIA_DRIVER_CAPABILITIES=all" \
                --network=host \
                odom:latest
