import cv2
import numpy as np

from SX127x.LoRa import *
from SX127x.board_config import BOARD

from time import sleep

import RPi.GPIO as GPIO

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

flame_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(flame_pin, GPIO.IN)

vibration_pin = 20
GPIO.setmode(GPIO.BCM)
GPIO.setup(vibration_pin, GPIO.IN)

BOARD.setup()

CODES = {
    "ACK": 0x00,
    "msg": 0x01,
    "first packet": 0x02,
    "image packet": 0x03,
    "last packet": 0x04,
    "request image": 0x05,
    "alarm": 0x06,
    "stop": 0x07,
    "next": 0x08
}

PACKET_SIZE = 254

class end_device(LoRa):
    def __init__(self, verbose=False):
        super(end_device, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        # self.set_dio_mapping([1,0,0,0,0,0])
        self.set_dio_mapping([0] * 6)

        self.nb_packets = 0
        self.image = None
        self.raw_image = []
        self.packets = None
        self.image_mode = False

        self.cam = cv2.VideoCapture(0)

    def on_rx_done(self):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)

        print("RX DONE", payload)

        msg_code = payload[0]
        msg = bytes(payload[1:]).decode("utf-8", "ignore")

        self.process(msg_code, msg)

        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)

        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)
        print("TX DONE")

    # def on_tx_done(self):
    #     print("TX DONE")
    #     self.set_mode(MODE.STDBY)
    #     self.clear_irq_flags(TxDone=1)
    #     self.write_payload([0x0f])
    #     self.set_mode(MODE.TX)

    def send(self, payload):
        print("SENDING", payload)
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)

        self.write_payload(payload)
        self.set_mode(MODE.TX)

    def image2packets(self, image):
        ret, image = cv2.imencode(".jpg", image, encode_param)
        image_size = len(image)
        print("Image to packets", image_size)

        packets = []

        index = 0
        while index + PACKET_SIZE < image_size:
            packets += [CODES["image packet"] + image[index : index + PACKET_SIZE]]
            index += PACKET_SIZE
        packets += [CODES["last packet"] + image[index : ]]

        self.nb_packets = len(packets)

        self.packets = iter(packets)
        print("end function")

    def process(self, msg_code, msg):
        if msg_code == CODES["request image"]:
            print("[RECV] Activate camera")
            self.image_mode = True
            # Activate cam
            _, self.image = self.cam.read()
            self.image2packets(self.image)

            nb_packets = list(f"{self.nb_packets:03}".encode())
            self.send([CODES["first packet"]] + nb_packets)


        elif msg_code == CODES["stop"]:
            print("[RECV] Stop camera")
            self.image_mode = False
            # Deactivate camera

        elif msg_code == CODES["alarm"]:
            print("[RECV] Toggle alarm")
            # Toggle alarm, message received from IHM

        elif msg_code == CODES["ACK"]:
            print("[RECV] ACK")

            if self.image_mode == True:
                if self.nb_packets > 0:
                    self.nb_packets -= 1
                    self.send(next(self.packets))
                
                else:
                    # Get new image
                    _, self.image = self.cam.read()
                    self.image2packets(self.image)

        else:
            print("[RECV] Wrong packet code. Received ", msg_code)
            self.send([0x0f])
            

def main():
    ed.clear_irq_flags(RxDone=1)
    ed.reset_ptr_rx()
    ed.set_mode(MODE.RXCONT)
    while True:
        print(ed.get_irq_flags())
        sleep(1)

ed = end_device(verbose=False)

ed.set_mode(MODE.STDBY)
ed.set_pa_config(pa_select=1)
ed.set_bw(BW.BW500)
ed.set_coding_rate(CODING_RATE.CR4_5)
ed.set_spreading_factor(7)
ed.set_rx_crc(False)
ed.set_low_data_rate_optim(False)

print(ed)

assert(ed.get_agc_auto_on() == 1)



def flame_callback(channel):
    payload = "Flame detected"

    print("[SEND]", payload)

    payload = list(payload.encode())
    payload = [CODES["msg"]] + payload

    ed.send(payload)

def vibration_callback(channel):
    payload = "Vibration detected"

    print("[SEND]", payload)

    payload = list(payload.encode())
    payload = [CODES["msg"]] + payload

    ed.send(payload)

GPIO.add_event_detect(flame_pin, GPIO.RISING, callback=flame_callback, bouncetime=300)
GPIO.add_event_detect(vibration_pin, GPIO.RISING, callback=vibration_callback, bouncetime=300)


try:
    print("START")
    main()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("Exit")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("Exit")
    ed.set_mode(MODE.SLEEP)
    BOARD.teardown()

"""
RX MODE

ON_EVENT
    payload = get_data()

    send ( payload )
    RXMODE

ON_RX
    if image_mode == True and nb_packets > 0
        nb_packets--
        if nb_packets < 0
            image_mode = False
        send (next_packet)
    if request_img
        img_mode = True
        activate cam
    if stop
        img_mode = False

while img_mode == True
    get frame
    packetify
    send packets

"""
