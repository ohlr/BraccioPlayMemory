# Useful comands: 

## to get yolo running on webcam:
* ./darknet detector demo cfg/coco.data cfg/yolo.cfg yolo.weights
* ./darknet detector demo cfg/voc.data cfg/tiny-yolo-voc.cfg tiny-yolo-voc.weights

## Start labeling tool:
* python main.py

## reshape/rename images:
```bash
mogrify -resize 320x320 *.jpg
rename 's/.jpg$/.JPEG/' *.jpg
```

## Testing
run on webcam

```bash
./darknet detector demo cfg/obj.data cfg/yolo-obj-webCam.cfg TrainingTinyTiny/yolo-obj-1200.weights -c 2
```

-c 2 tag specifies which webcam to use

run on image
```bash
./darknet detector test cfg/obj.data cfg/yolo-obj.cfg  backup/yolo-obj_100.weights data/obj/IMAG3635.jpg -thresh 20  
```

-thresh 20 determines the threshold

## train

```bash
./darknet detector train cfg/obj.data cfg/yolo-obj.cfg tiny-yolo-voc.weights
```

## Working on lab PC

```bash
ssh -p 58022 w392@atcremers64.informatik.tu-muenchen.de
Enter PW

cd /usr/prakt/w392/BraccioPlayMemory/darknet

nvidia-smi

./darknet detector train cfg/obj.data cfg/yolo-obj.cfg tiny-yolo-voc.weights
```