sudo rm -rf /etc/udev/rules.d/end_effector.rules

echo 'SUBSYSTEMS=="usb", KERNEL=="ttyUSB[0-9]*", ACTION=="add", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="666", SYMLINK+="DoraHand_ttyUSB", GROUP="dialout"' | sudo tee -a /etc/udev/rules.d/end_effector.rules
echo 'SUBSYSTEMS=="usb", KERNEL=="ttyACM[0-9]*", ACTION=="add", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="666", SYMLINK+="DoraHand_ttyUSB", GROUP="dialout"' | sudo tee -a /etc/udev/rules.d/end_effector.rules

sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger