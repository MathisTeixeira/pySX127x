import cv2
import numpy as np

from SX127x.LoRa import *
from SX127x.board_config import BOARD

from time import sleep

BOARD.setup()

CODES = {
    "ACK": 0x00,
    "msg": 0x01,
    "first packet": 0x02,
    "image packet": 0x03,
    "last packet": 0x04,
}

ACK_message = list("0".encode())

class gateway(LoRa):
    def __init__(self, verbose=False):
        super(gateway, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([0] * 6)

        self.raw_image = []
        self.image = None
        self.nb_packets = 0

    def on_rx_done(self):
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)

        msg = bytes(payload).decode("utf-8", "ignore")

        self.process(msg)

    def on_tx_done(self):
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def send_ACK(self):
        self.write_payload(ACK_message)
        self.set_mode(MODE.TX)

    def process(self, msg):
        msg_code = msg[0]

        if msg_code == CODES["msg"]:
            print("[RECV] ", msg[1:])

            self.send_ACK()
        
        elif msg_code == CODES["first packet"]:
            self.nb_packets = int(msg[1:4])
            self.raw_image = []

            print("[RECV] Number of packets ", self.nb_packets)

            self.send_ACK()

        elif msg_code == CODES["image packet"]:
            self.nb_packets -= 1
            self.image += msg[1:]

            print("[RECV] Number of packets ", self.nb_packets)

            self.send_ACK()

        elif msg_code == CODES["last packet"]:
            # Check if we received all packets
            if self.nb_packets != 1:
                print("[ERROR] Incomplete image")
                return

            self.raw_image += msg[1:]
            npimg = np.fromstring(self.raw_image, dtype=np.uint8)
            self.image = cv2.imdecode(npimg, cv2.IMREAD_COLOR)

            print("[RECV] Receive image")

            self.send_ACK()

        elif msg_code == CODES["ACK"]:
            print("[RECV] ACK")

        else:
            print("[RECV] Wrong packet code. Received ", msg_code)

            self.send_ACK()

def main():
    last_image = gw.image
    while True:
        gw.reset_ptr_rx()
        gw.set_mode(MODE.RXCONT)
        sleep(.5)
        rssi_value = gw.get_rssi_value()
        status = gw.get_modem_status()
        sys.stdout.flush()
        sys.stdout.write("\r%d %d %d" % (rssi_value, status['rx_ongoing'], status['modem_clear']))

        if gw.image is not None and last_image != gw.image:
            cv2.imshow("Camera", gw.image)
            last_image = gw.image

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
