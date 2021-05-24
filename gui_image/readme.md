# Download the docker image from dorabot
https://dorahand.dorabot.org/binary_blob/dorahand_gui_1_0_2.tar
# Load the docker of dorahand_gui:1.0.2
docker load --input dorahand_gui_1_0_2.tar
# Enter the docker of dorahand_gui:1.0.2
docker run --name dorahand_gui --device=/dev/DoraHand_ttyUSB:/dev/DoraHand_ttyUSB --net=host --env="DISPLAY" --rm -it dorahand_gui:1.0.2  bash
# Run dorahand gui  
DDHPlatform