# RUN darknet_ros

###
* To maximize performance, make sure to build in Release mode. You can specify the build type by setting
```
catkin_make -DCMAKE_BUILD_TYPE=Release
```
*  https://github.com/leggedrobotics/darknet_ros

### Terminal 1:
```
roslaunch darknet_ros Braccio_darknet_ros.launch 
```

### Terminal 2: use usb_cam to publish webcam image, darknet_ros would subscribe to it.
```
roslaunch usb_cam usb_cam-test.launch 
```
