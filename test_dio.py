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

DIO_RX = [0,0,0,0,0,0]
DIO_TX = [1,0,0,0,0,0]

class end_device(LoRa):
    def __init__(self, verbose=False):
        super(end_device, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping(DIO_TX)

    def on_rx_done(self):
        print("RX DONE")

    def on_tx_done(self):
        print("TX DONE")

    def send(self, payload):
        print("SENDING", payload)
        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(TxDone=1)

        self.write_payload(payload)
        self.set_mode(MODE.TX)
            

def main():
    ed.clear_irq_flags(RxDone=1)
    ed.reset_ptr_rx()
    ed.set_mode(MODE.RXCONT)
    i = 0
    while True:
        if i > 4:
            ed.set_dio_mapping(DIO_RX)
            ed.set_mode(MODE.STBY)
            ed.reset_ptr_rx()
            ed.set_mode(MODE.RXCONT)

        sleep(3)
        ed.send([0])
        i += 1

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

# GPIO.add_event_detect(flame_pin, GPIO.RISING, callback=flame_callback, bouncetime=300)
# GPIO.add_event_detect(vibration_pin, GPIO.RISING, callback=vibration_callback, bouncetime=300)


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
