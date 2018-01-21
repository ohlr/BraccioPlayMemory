# BraccioPlayMemory

## restricted access to the following lab computers:

 atbeetz11 atbeetz18 atbeetz19 atbeetz21 atbeetz25 in 02.09.038 

## Tasks:

### First prio:

Data Labeling:
* https://timebutt.github.io/static/how-to-train-yolov2-to-detect-custom-objects/
* https://github.com/puzzledqs/BBox-Label-Tool

BB Label Tool is already cloned into our repo. Images have correct format and are devided into folders. Start gui with 
```ruby
python main.py
```

Object Detection:
* [Yolo](https://pjreddie.com/darknet/yolo/)
* Needs OpenCV to run on webcam

Transfer Learning - Pytorch Implementation:
* https://github.com/longcw/yolo2-pytorch
we need to get the yolo | tiny yolo layers, into pytorch and train the last layer with our own data

### Second Prio (Later):
* Add metal plates to memory cards
* modify robot
* robot move srcipt (Ros Node)
* Transmitting information from yolo to ROS

