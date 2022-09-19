# Download the docker image from dorabot
https://dorahand.dorabot.org/binary_blob/dorahand_gui_1_3_0.tar
# Load the docker of dorahand_gui:1.3.0
docker load --input dorahand_gui_1_3_0.tar
# For USB version - Enter the docker of dorahand_gui:1.3.0
docker run --name dorahand_gui --device=/dev/DoraHand_ttyUSB:/dev/DoraHand_ttyUSB --net=host --env="DISPLAY" --rm -it dorahand_gui:1.3.0 bash
# For Ethernet version - Enter the docker of dorahand_gui:1.3.0
docker run --name dorahand_gui --net=host --env="DISPLAY" --rm -it dorahand_gui:1.3.0 bash
# For USB version - running gui
DDHPlatform -i usb
# For ETHERNET version - running gui
DDHPlatform -i eth
