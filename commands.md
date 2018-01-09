# Useful comands: 

## to get yolo running on webcam:
* ./darknet detector demo cfg/coco.data cfg/yolo.cfg yolo.weights
* ./darknet detector demo cfg/voc.data cfg/tiny-yolo-voc.cfg tiny-yolo-voc.weights

## Start labeling tool:
* python main.py

## reshape/rename images:
* mogrify -resize 320x320 *.jpg
* rename 's/.jpg$/.JPEG/' *.jpg
