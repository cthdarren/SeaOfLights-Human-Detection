from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import serial
from time import sleep
import threading
import cv2

ser = serial.Serial('/dev/serial/by-id/usb-STMicroelectronics_GENERIC_F103C8TX_CDC_in_FS_Mode_6D70229C4955-if00', 9600)
model = YOLO("yolov8n-pose.pt")

# positions of balls in order (in the snake-like pattern) converted to
# matrix format in the direction the camera is facing
ballPos = [[25-x, 16+x, 15-x, 6+x, 5-x] for x in range(5)]
# color = [[169,255,180]*25]
# global color
# nocolor = [item for sublist in generate for item in sublist]

def createColorWave():
    hue = 0
    wave = 0
    while True:
        servoPos = [[wave%255]*5 + [(wave+50)%255]*5 + [(wave+100)%255]*5 + [(wave+150)%255]*5 + [(wave+255)%200]*5]
        generate = [[hue%255,255,128]*5+[(hue+10)%255,255,128]*5+[(hue+20)%255,255,128]*5+[(hue+30)%255,255,128]*5+[(hue+40)%255,255,128]*5] 
        # generate = [[hue%255,0,255]*5+[(hue+10)%255,0,255]*5+[(hue+20)%255,0,255]*5+[(hue+30)%255,0,255]*5+[(hue+40)%255,0,255]*5] 
        color = [item for sublist in generate for item in sublist]
        servo = [item for sublist in servoPos for item in sublist]
        sendPacket(color, servo)
        sleep(0.05)
        hue+=2
        wave+=21

def sendPacket(colorPara, positionList):
    ser.write(b'\xff')
    ser.write(bytes(colorPara + positionList))
    
# x1 - x6 represents the vertical line coordinate between each column

createColorWave()
