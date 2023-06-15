import cv2
from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import RPi.GPIO as GPIO

DIO_RX = [0,0,0,0,0,0]
DIO_TX = [1,0,0,0,0,0]

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

BOARD.setup()

class gateway(LoRa):
    def __init__(self, verbose=False):
        super(gateway, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping(DIO_RX)

        self.raw_image = []
        self.image = None
        self.nb_packets = 0

    def on_rx_done(self):
        print("RX DONE")
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(RxDone=1)
        payload = self.read_payload(nocheck=True)

        msg_code = payload[0]
        msg = bytes(payload[1:])

        self.process(msg_code, msg)

        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print("TX DONE")

        self.set_mode(MODE.STDBY)
        self.set_dio_mapping(DIO_RX)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def send(self, payload):
        print("SENDING", payload)

        self.set_mode(MODE.STBY)
        self.set_dio_mapping(DIO_TX)

        self.clear_irq_flags(TxDone=1)

        self.write_payload(payload)
        self.set_mode(MODE.TX)

    def process(self, msg_code, msg):
        if msg_code == CODES["image size"]:
            self.packets = [None] * int(msg)   # /!\ int(msg) 
            self.nb_packets = int(msg)    # /!\ int(msg) 



        elif msg_code == CODES["image"]:
            id = int(msg[0])   # /!\ int(msg) 
            self.packets[id] = msg[1:]
            if id == self.nb_packets - 1:
                self.reconstruct_image()
                print("IMAGE", self.image)

    def reconstruct_image(self):
        if None in self.packets:
            print("Empty packet")

        self.joined_packets = []
        for packet in self.packets:
            self.joined_packets += packet

        ret, self.image = cv2.imdecode(self.joined_packets, cv2.IMREAD_GRAYSCALE)
        cv2.imwrite("./received_image.png", self.image)

    def start(self):
        print("START")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

        while True:
            pass

def main():
    gw = gateway(verbose=False)

    gw.set_mode(MODE.STDBY)
    gw.set_pa_config(pa_select=1)
    gw.set_bw(BW.BW500)
    gw.set_coding_rate(CODING_RATE.CR4_5)
    gw.set_spreading_factor(7)
    gw.set_rx_crc(False)
    gw.set_low_data_rate_optim(False)

    print(gw)

    assert(gw.get_agc_auto_on() == 1)
    # Listen
    # If code == "image size":
    #   packets = [None] * image_size
    # if code == "image":
    #   id = msg[0]
    #   packets[id] = msg
    #   if id == last_id:
    #       reconstruct_image()

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
