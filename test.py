from ultralytics import YOLO
from ultralytics.yolo.v8.detect.predict import DetectionPredictor
import serial
import IPython
from time import sleep
import random
import threading

ser = serial.Serial('/dev/serial/by-id/usb-STMicroelectronics_GENERIC_F103C8TX_CDC_in_FS_Mode_6D70229C4955-if00', 9600)
model = YOLO("yolov8s-pose.pt")

# positions of balls in order (in the snake-like pattern) converted to
# matrix format in the direction the camera is facing
ballPos = [[25-x, 16+x, 15-x, 6+x, 5-x] for x in range(5)]
servoPos = [200]*25
# color = [[169,255,180]*25]
# global color
# nocolor = [item for sublist in generate for item in sublist]

def createColorWave():
    hue = 0
    while True:
        generate = [[hue%255,255,128]*5+[(hue+10)%255,255,128]*5+[(hue+20)%255,255,128]*5+[(hue+30)%255,255,128]*5+[(hue+40)%255,255,128]*5] 
        color = [item for sublist in generate for item in sublist]
        sendPacket(color, servoPos)
        sleep(0.05)
        hue+=2

def sendPacket(colorPara, positionList):
    ser.write(b'\xff')
    ser.write(bytes(colorPara + positionList))
    
# x1 - x6 represents the vertical line coordinate between each column
# in that row
def checkCol(xcoord,x1,x2,x3,x4,x5,x6):
    #if out of bounds of row
    if xcoord < x1 or xcoord > x6:
        return None 
    elif xcoord < x2:
        return 1
    elif xcoord < x3:
        return 2
    elif xcoord < x4:
        return 3
    elif xcoord < x5:
        return 4
    else:
        return 5

def translateCoordToBallPos(coord):
       return ballPos[coord[0]-1][coord[1]-1]

def servoPositions(coordList):
    print(coordList)
    positions = [200 for x in range(25)]
    if coordList == []:
        return [200 for x in range(25)]
    for coord in coordList:
        mainPos = translateCoordToBallPos(coord) 
        surroundingPos = []

        isFirstCol = coord[1] == 1
        isLastCol = coord[1] == 5
        isFirstRow = coord[0] == 1
        isLastRow = coord[0] == 5
        isEvenCol = coord[1] % 2 == 0

        if isEvenCol:
            if not isFirstRow:
                surroundingPos.append(mainPos - 1)
            if not isLastRow:
                surroundingPos.append(mainPos + 1)
            if not isFirstCol:
                surroundingPos.append(mainPos + (1 + 2*(len(ballPos)-coord[0])))
            if not isLastCol:
                surroundingPos.append(mainPos - (1 + 2*(coord[0]-1)))

        elif not isEvenCol:
            if not isFirstCol:
                surroundingPos.append(mainPos + (1 + 2*(coord[0] - 1)))
            if not isLastCol:
                surroundingPos.append(mainPos - (1 + 2*(len(ballPos) - coord[0])))
            if not isFirstRow:
                surroundingPos.append(mainPos + 1)
            if not isLastRow:
                surroundingPos.append(mainPos - 1)
                
        for pos in [x-1 for x in surroundingPos]:
            positions[pos] = 100 
            positions[mainPos - 1] = 0 
        return positions

thr1 = threading.Thread(target=createColorWave)
thr1.start()

results = model.predict(source="0", show=True, stream=True)
for result in results:
    kp = result.keypoints
    print(kp)
    coordList = []
    try:
        for person in kp:
            left_foot = person[-1][0], person[-1][1]
            right_foot = person[-2][0], person[-2][1]
            coords = (left_foot[0].item() + right_foot[0].item()) /2 ,( left_foot[1].item() + right_foot[1].item() )/ 2
            if coords[0] > 1000 or coords[0] < 1 or coords[1] > 1000 or coords[1] < 1:
                continue
            print(coords)
            if 428.6 < coords[1] or coords[1] < 245.6:
                continue
            elif coords[1] < 254.6:
                row = 1
                col = checkCol(coords[0], 156, 210.6, 281, 339, 416.6, 478)
            elif coords[1] < 277.6:
                row = 2
                col = checkCol(coords[0], 140, 206, 275, 346, 424, 500)
            elif coords [1] < 303.6:
                row = 3
                col = checkCol(coords[0], 109, 190, 275, 360, 450, 526)
            elif coords[1] < 350.6:
                row = 4
                col = checkCol(coords[0], 72, 173, 272, 369, 464, 561)
            else:
                row = 5
                col = checkCol(coords[0], 26, 134, 257, 388, 520, 616)
            if col == None:
                continue
            coordList.append([row,col])
        print(coordList)
            
        servoPos = servoPositions(coordList)
    except IndexError:
        print("No legs detected") 
        continue

# Use PySerial to talk to serial ports.
# On *nix, this will be /dev/<something>

# Data to send looks like this:
#   1 byte | sync byte (0xFF)
# 75 bytes | color data (R1, G1, B1, R2, G2, B2 ... R25, G25, B25)
# 25 bytes | servo positions (0-255 each)
#
# all data is in order, so controller will write colors[0] and positions[0] to the first LED/servo


