**Important: This application needs to run on jetson nano**

## firmware update usage

1. copy firmware_update to jetson nano  
2. run command `sudo chmod +x firmware_update`  
3. run command `sudo ./firmware_update -h` to view help message.  

Note: for jetson nano A02 i2c-bus is 6, for jetson nano b01 CAM0 is 7, CAM1 is 8

For details, please refer to:
[firmware-update-instructions](https://www.arducam.com/docs/cameras-for-raspberry-pi/pivariety/firmware-update-instructions/)
## Test command

v4l2-ctl --set-fmt-video=width=1600,height=1200,pixelformat=Y10 --stream-mmap --stream-count=-1 -d /dev/video0