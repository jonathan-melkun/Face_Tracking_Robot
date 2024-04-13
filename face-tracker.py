# -*- coding: utf-8 -*-
"""
Created on Sat Apr  8 10:22:50 2023

@author: Jonathan Melkun

@description: Sends servo and stepper move commands to the arduino
"""
import os
os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import time
import numpy as np
import cv2
import face_recognition
from pySerialTransfer import pySerialTransfer as txfer

def mapA(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

class struct(object):
    servo1_move = 0
    servo2_move = 0
    stepper_move = 0
centering_loop = False
video_capture = cv2.VideoCapture(0, cv2.CAP_DSHOW)
print("Video Capture Success.")
face_locations = []
frameNum = 0
servo_step_x = 0
servo_step_y = 0
width = 640
height = 480
video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
DEADZONE = 0.06
DEADZONE_X = int(width*DEADZONE)
DEADZONE_Y = int(height*DEADZONE)
centerImgX=width//2
centerImgY=height//2

try:
    packet = struct
    link = txfer.SerialTransfer('COM7')

    link.open()
    time.sleep(2)
    print("Link is open.")
    while True:
        packet.servo1_move = 0
        packet.servo2_move = 0
        packet.stepper_move = 0
        ret, frame = video_capture.read()
        # Only process every tenth frame of video
        if frameNum % 10 == 0:
            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_frame = frame[:, :, ::-1]

            # Find all the faces in the current frame of video
            face = face_recognition.face_locations(rgb_frame)
            if len(face) != 0:
                centerX = (face[0][3]+face[0][1])//2
                centerY = (face[0][0]+face[0][2])//2
                if centerX < width//2 - DEADZONE_X or centerX > width//2 + DEADZONE_X or centerY < height//2 - DEADZONE_Y or centerY > height//2 + DEADZONE_Y:
                    packet.servo2_move = int(mapA(centerX-centerImgX, 0, width//2, 0, 45))


                    packet.servo1_move = int(mapA(centerY-centerImgY, 0, height//2, 0, 35))

                    packet.stepper_move = int(mapA(centerX-centerImgX, 0, width//2, 0, 500))
                    #print(packet.stepper_move)
                    #send to Arduino
                    sendSize = 0
                    sendSize = link.tx_obj(packet.servo1_move, start_pos=sendSize)
                    sendSize = link.tx_obj(packet.servo2_move, start_pos=sendSize)
                    sendSize = link.tx_obj(packet.stepper_move, start_pos=sendSize)
                    link.send(sendSize)
                    #print("{}, {}, {}, {}".format(packet.servo1_move, packet.servo2_move, packet.stepper_move, packet.centering))
                    #print("Data sent.\n")
                else:
                    print("deadzone")
        frameNum+=1

        # Display the results
        if len(face) != 0:
            # Draw a box around the face
            cv2.rectangle(frame, (face[0][3], face[0][0]),(face[0][1], face[0][2]), (0, 0, 255), 2)
        # Display the resulting image
        cv2.rectangle(frame, (np.size(frame, 1)//2+DEADZONE_X, np.size(frame, 0)//2+DEADZONE_X), (np.size(frame, 1)//2-DEADZONE_Y, np.size(frame, 0)//2-DEADZONE_Y), (0, 255, 0), 2)
        cv2.namedWindow('Video', cv2.WINDOW_GUI_NORMAL)
        cv2.imshow('Video', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            video_capture.release()
            cv2.destroyAllWindows()
        #receive information from Arduino for debugging
        # while(link.available()):
        #     print("Received data!")
        #     recSize = 0

        #     servo1_rec = link.rx_obj(obj_type='i', start_pos=recSize)
        #     recSize += txfer.STRUCT_FORMAT_LENGTHS['i']

        #     servo2_rec = link.rx_obj(obj_type='i', start_pos=recSize)
        #     recSize += txfer.STRUCT_FORMAT_LENGTHS['i']

        #     stepper_rec = link.rx_obj(obj_type='i', start_pos=recSize)
        #     recSize += txfer.STRUCT_FORMAT_LENGTHS['i']

        #     centering_rec = link.rx_obj(obj_type='b', start_pos=recSize)
        #     recSize += txfer.STRUCT_FORMAT_LENGTHS['b']

        #     print('{}, {}, {}, {}'.format(servo1_rec, servo2_rec, stepper_rec, centering_rec))
except KeyboardInterrupt:
    print('KeyboardInterrupt, closing')
    link.close()
    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()

except:
    import traceback
    traceback.print_exc()
    # Release handle to the webcam
    video_capture.release()
    cv2.destroyAllWindows()
    link.close()