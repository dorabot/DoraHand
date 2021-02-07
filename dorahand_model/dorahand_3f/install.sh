#!/bin/sh
#copy all necessary resource files to fixed resource path
sudo mkdir -p /opt/dorabot/resources/end_effectors/dorahand_3f
sudo cp end_effector.xacro /opt/dorabot/resources/end_effectors/dorahand_3f/
sudo cp -r meshes /opt/dorabot/resources/end_effectors/dorahand_3f/
