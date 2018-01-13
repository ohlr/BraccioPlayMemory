![Darknet Logo](http://pjreddie.com/media/files/darknet-black-small.png)

#Darknet#
Darknet is an open source neural network framework written in C and CUDA. It is fast, easy to install, and supports CPU and GPU computation.

For more information see the [Darknet project website](http://pjreddie.com/darknet).

For questions or issues please use the [Google Group](https://groups.google.com/forum/#!forum/darknet).


[How To Train](https://timebutt.github.io/static/how-to-train-yolov2-to-detect-custom-objects/)

```bash
./darknet detector train cfg/obj.data cfg/yolo-obj.cfg darknet19_448.conv.23
```

```bash
cp cfg/obj.data.example cfg/obj.data
sed -i "/home/oskar/OneDrive/YoloV2/darknet:${pwd}"
```

obj.data file
config the absolute paths of train and test
