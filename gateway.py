import cv2
import numpy as np

from SX127x.LoRa import *
from SX127x.board_config import BOARD

from time import sleep
import datetime

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

class gateway(LoRa):
    def __init__(self, verbose=False):
        super(gateway, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

        self.raw_image = []
        self.image = None
        self.nb_packets = 0

    def on_rx_done(self):
        print("RX DONE")
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)

        msg_code = payload[0]
        msg = bytes(payload[1:]).decode("utf-8", "ignore")

        self.process(msg_code, msg)

        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def send_ACK(self):
        self.write_payload([CODES["ACK"]])
        self.set_mode(MODE.TX)

    def send(self, payload):
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)

        self.write_payload(payload)
        self.set_mode(MODE.TX)

    def process(self, msg_code, msg):
        if msg_code == CODES["msg"]:
            print("[RECV] ", msg)

            self.send_ACK()
        
        elif msg_code == CODES["first packet"]:
            self.nb_packets = int(msg[:4])
            self.raw_image = []

            print("[RECV] Number of packets ", self.nb_packets)

            self.send_ACK()

        elif msg_code == CODES["image packet"]:
            self.nb_packets -= 1
            self.raw_image += msg

            print("[RECV] Number of packets left ", self.nb_packets)

            self.send_ACK()

        elif msg_code == CODES["last packet"]:
            # Check if we received all packets
            if self.nb_packets != 1:
                print("[ERROR] Incomplete image")
                return

            self.raw_image += msg
            npimg = np.fromstring(self.raw_image, dtype=np.uint8)
            self.image = cv2.imdecode(npimg, cv2.IMREAD_COLOR)

            date = datetime.datetime.now().strftime("%d-%m-%y-%H-%M-%S")
            cv2.imwrite(f"./images/{date}.png", self.image)

            print("[RECV] Received image")

            self.send_ACK()

        elif msg_code == CODES["ACK"]:
            print("[RECV] ACK")

        else:
            print("[RECV] Wrong packet code. Received ", msg_code, "|")

def main():
    gw.reset_ptr_rx()
    gw.set_mode(MODE.RXCONT)
    last_image = gw.image
    start = 0

    while True:
        if start == 0:
            print("SLEEP 2")
            sleep(2)
            start = 1
            print("SEND Request")
            gw.send([CODES["request image"]])
        else:
            sleep(0.5)
            

        # if gw.image is not None and last_image != gw.image:
        #     cv2.imshow("Camera", gw.image)
        #     last_image = gw.image

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
