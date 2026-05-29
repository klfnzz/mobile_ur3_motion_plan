docker rm -f openrave
HOME=/home
xhost +  && docker run -it --name openrave --gpus all -e DISPLAY=${DISPLAY} --network host --privileged -v /dev:/dev -v /tmp/.X11-unix:/tmp/.X11-unix -v /home/qwe/Documents/Container/openrave/data:/root openrave
