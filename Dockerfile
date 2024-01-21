FROM ros:iron-perception


# Copy the source to calculate and install dependencies
COPY ./src /root/car_ws/src
RUN apt-get update
# rosdep install
RUN rosdep update && \
    rosdep install --from-paths /root/car_ws/src --ignore-src -r -y || true

RUN rm -rf /root/car_ws/src
RUN apt-get install -y ros-iron-nmea-navsat-driver
RUN apt-get install -y python3-pip
RUN pip3 install transforms3d
