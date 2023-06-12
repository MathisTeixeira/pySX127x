from time import time, sleep
import cv2
import numpy as np

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]

CODE_LENGTH = 1
ID_LENGTH = 1
PACKET_SIZE = 255 - CODE_LENGTH - ID_LENGTH
SLEEP_TIME = 100

times = []

for _ in range(20):
    cam = cv2.VideoCapture(0)

    ret, image = cam.read()

    ret, image = cv2.imencode(".jpg", image, encode_param)
    image_size = len(image)
    print("Image to packets", image_size)

    t0 = time()

    packets = []

    index = 0
    id = 0
    while index + PACKET_SIZE-1 < image_size:
        packets += [CODES["image"] + id + image[index : index + PACKET_SIZE-1]]
        index += PACKET_SIZE-1
        id += 1
    packets += [CODES["image"] + id + image[index : ]]

    times.append(time() - t0)
    print("TIME PACKETIFY", time() - t0)

print("MEAN", np.mean(times))