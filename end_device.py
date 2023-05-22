import cv2
import numpy as np

from SX127x.LoRa import *
from SX127x.board_config import BOARD

from time import sleep

BOARD.setup()

CODES = {
    "ACK": 0x00,
    "request image": 0x01,
    "alarm": 0x02,
    "stop": 0x03,
    "next": 0x04
}

PACKET_SIZE = 254

class end_device(LoRa):
    def __init__(self, verbose=False):
        super(gateway, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([1,0,0,0,0,0])

        self.nb_packets = 0
        self.image = None
        self.raw_image = []
        self.packets = None
        self.image_mode = False

        self.cam = cv2.VideoCapture(0)

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)

        msg = bytes(payload).decode("utf-8", "ignore")

        process(msg)

    def on_tx_done(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def send(self, payload):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)

        self.write_payload(payload)
        self.set_mode(MODE.TX)

    def image2packets(self, image):
        image_size = len(image)

        packets = []

        index = 0
        while index + PACKET_SIZE < image_size:
            packets += [0x03 + image[index : index + PACKET_SIZE]]
            index += PACKET_SIZE
        packets += [0x04 + image[index : ]]

        self.nb_packets = len(packets)

        self.packets = iter(packets)

    def process(self, msg):
        msg_code = msg[0].encode()

        if msg_code == CODES["request image"]:
            print("[RECV] Activate camera")
            self.image_mode = True
            # Activate cam
            self.image = self.cam.read()
            self.image2packets(self.image)


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
                    self.image = self.cam.read()
                    self.image2packets(self.image)

        else:
            print("[RECV] Wrong packet code. Received ", msg_code)
            

def main():
    while True:
        sleep(0.5)

gw = gateway(verbose=False)

gw.set_pa_config(pa_select=1, max_power=21, output_power=15)
gw.set_bw(BW.BW125)
gw.set_coding_rate(CODING_RATE.CR4_8)
gw.set_spreading_factor(12)
gw.set_rx_crc(True)
#gw.set_lna_gain(GAIN.G1)
#gw.set_implicit_header_mode(False)
gw.set_low_data_rate_optim(True)

assert(gw.get_agc_auto_on() == 1)

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
    gw.set_mode(MODE.SLEEP)
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
