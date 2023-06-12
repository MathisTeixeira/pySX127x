from time import time, sleep
import cv2
import numpy as np

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]

CODE_LENGTH = 1
ID_LENGTH = 1
PACKET_SIZE = 255 - CODE_LENGTH - ID_LENGTH
SLEEP_TIME = 100

times = []
packets_len = []

CODES = {
    "ACK": 0x00,
    "msg": 0x01,
    "first packet": 0x02,
    "image packet": 0x03,
    "last packet": 0x04,
    "request image": 0x05,
    "alarm": 0x06,
    "stop": 0x07,
    "next": 0x08,
    "image": 0x09,
    "image size": 0x0a
}

cam = cv2.VideoCapture(0)

for _ in range(20):
    ret, image = cam.read()

    ret, image = cv2.imencode(".jpg", image, encode_param)
    image_size = len(image)
    # print("Image to packets", image_size)

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
    # print("TIME PACKETIFY", time() - t0)
    packets_len.append(len(packets))

print("TIME", np.mean(times))
print("PACKETS", np.mean(packets_len))