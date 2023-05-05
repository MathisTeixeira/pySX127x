#!/usr/bin/env python

import numpy as np
import sys
from time import sleep
from SX127x.LoRa import *
from SX127x.LoRaArgumentParser import LoRaArgumentParser
from SX127x.board_config import BOARD
from math import floor

BOARD.setup()

parser = LoRaArgumentParser("A simple LoRa beacon")
parser.add_argument('--single', '-S', dest='single', default=False, action="store_true", help="Single transmission")
parser.add_argument('--wait', '-w', dest='wait', default=1, action="store", type=float, help="Waiting time between transmissions (default is 0s)")

PACKET_SIZE = 255

class LoRaBeacon(LoRa):
    def __init__(self, verbose=False):
        super(LoRaBeacon, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping([1,0,0,0,0,0])

    def on_rx_done(self):
        print("\nRxDone")
        print(self.get_irq_flags())
        print(map(hex, self.read_payload(nocheck=True)))
        self.set_mode(MODE.SLEEP)
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

    def on_tx_done(self):
        print("tx done\n")

    def send_message(self, msg):
        size = len(msg)

        if size > 255:
            print("Didn't send. Message is too long.")
            return

        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)

        print("SEND ", msg)

        # Process msg
        msg = "0" + msg     # Add message id
        msg = msg.encode()
        msg = list(msg)

        self.write_payload(msg)
        self.set_mode(MODE.TX)

    def send_image(self, img):
        size = len(img)
        nb_packets = floor(size / PACKET_SIZE)

        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)
        
        # Process image
        msg = f"1{nb_packets:03}"
        msg = msg.encode()
        msg = list(msg)

        self.write_payload(msg)
        self.set_mode(MODE.TX)

        for i in range(nb_packets):
            self.set_mode(MODE.STDBY)
            self.clear_irq_flags(TxDone=1)

            msg = img[i * PACKET_SIZE : (i + 1) * PACKET_SIZE]
            msg = msg.encode()
            msg = list(msg)

            self.write_payload(msg)
            self.set_mode(MODE.TX)


lora = LoRaBeacon(verbose=False)
args = parser.parse_args(lora)

lora.set_pa_config(pa_select=1)


print(lora)
assert(lora.get_agc_auto_on() == 1)

print("Beacon config:")
print("  Wait %f s" % args.wait)
print("  Single tx = %s" % args.single)
print("")
try: input("Press enter to start...")
except: pass

def main():
    print("\nStart\n")
    lora.set_mode(MODE.TX)

#    encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
#    cam = cv2.VideoCapture(0)

    while True:
        cmd = input("Input \n")

        if cmd == "image":
            img = ""
            for i in range(300):
                img += f"{i}-"
            lora.send_image(img)
        else:
            lora.send_message(cmd)

        # Check sensor update
        # Send if new data

        # If cam enabled
        # Send image if first time
        # Send pixels otherwise

try:
    main()
except KeyboardInterrupt:
    sys.stdout.flush()
    print("")
    sys.stderr.write("KeyboardInterrupt\n")
finally:
    sys.stdout.flush()
    print("")
    lora.set_mode(MODE.SLEEP)
    print(lora)
    BOARD.teardown()
