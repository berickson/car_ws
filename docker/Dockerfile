from ros:humble-perception
run apt-get update

# my favorite productivity tools
run apt-get install -y screen nano htop

# platformio
run apt-get install -y git cmake python3-pip
run pip install -U platformio

# add ros package dependencies, this list comes from running this from container:
# rosdep install --from-paths src --ignore-src -y --simulate
run apt-get install -y flex bison libncurses5-dev usbutils curl clang-tidy libasio-dev
