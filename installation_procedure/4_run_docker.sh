#!/bin/bash

sudo docker run -it --network host --privileged -v /dev/:/dev/ -v /run/udev:/run/udev username/img_name:tag

