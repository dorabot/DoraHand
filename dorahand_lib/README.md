# how to update dep

- sudo apt update
- sudo apt-get install make dpkg-dev libc6-dev gcc g++ build-essential cmake wget udev

# how to install dorahand-1.1.1-linux.dev

- cd package
- sudo ./protoc

# how to map the /dev/ttyACM0 to /dev/DoraHand_ttyUSB

- cd tool
- sudo ./install-udev.sh

# build the project

- mkdir build
- cd build
- cmake ..
- make
- ./demo_call_lib

# remove package

- sudo dpkg --remove dorahand
