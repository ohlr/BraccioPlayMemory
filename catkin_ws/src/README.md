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
* if you have multiple camera on your computer, edit `src/usb_cam/launch/usb_cam-test.launch`:  
change `<param name="video_device" value="/dev/video0" />` to /dev/videoX

--- 
### Record webcam as rosbag:
* [Record](http://wiki.ros.org/rosbag/Commandline#record):  
eg.  
1. Run usb_cam roslaunch.
2. `$ rosbag record --duration=30 -a`  
* [Playback](http://wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data):  
`rosbag play <your bagfile>`  

---
### Notes:
* In case anyone who wants to know, here's what I have adjust:  

1. In `/src/darknet_ros/darknet_ros/yolo_network_config`:  
Put in our training cfg & weights. 
eg. `cfg/yolo-obj-webCam.cfg`  &  `weights/yolo-obj-1500.weights`  
(As the [instructions](https://github.com/leggedrobotics/darknet_ros#use-your-own-detection-objects) of darknet_ros said.)

2. In `src/darknet_ros/darknet_ros/config/Braccio_yolo_voc.yaml`, add `.cfg` & `weights` & `classes`(obj.names)  
eg.  
```
yolo_model:

  config_file:
    name: yolo-obj-webCam.cfg
  weight_file:
    name: yolo-obj-1500.weights
  threshold:
    value: 0.5
  detection_classes:
    names:
      - Boat
      - Girl
      - Boy
      - Horse
      - EuropeanBird
      - PacificBird
      - Flower
      - BlueCard
      - Tree
      - Pineapple
```

3. In `/src/darknet_ros/darknet_ros/launch/Braccio_darknet_ros.launch`:  
I've add: `<remap from="/camera/rgb/image_raw" to="/usb_cam/image_raw"/>`  
to make output of usb_cam node as the input of darknet_ros input.

---
### Reference.
* similar concept as [here](https://qiita.com/nnn112358/items/d696681d5b0577d633b6)
