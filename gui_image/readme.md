# Download the docker image from dorabot
https://dorahand.dorabot.org/binary_blob/dorahand_gui_1_0_1.tar
# Load the docker of dorahand_gui:1.0.1
docker load --input dorahand_gui_1_0_1.tar
# Enter the docker of dorahand_gui:1.0.1
docker run --name dorahand_gui --device=/dev/DoraHand_ttyUSB:/dev/DoraHand_ttyUSB --net=host --env="DISPLAY" --rm -it dorahand_gui:1.0.0  bash