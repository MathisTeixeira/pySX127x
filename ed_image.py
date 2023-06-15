import cv2
from time import sleep
from SX127x.LoRa import *
from SX127x.board_config import BOARD
import RPi.GPIO as GPIO

encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 20]

CODE_LENGTH = 1
ID_LENGTH = 1
PACKET_SIZE = 255
PAYLOAD_SIZE = PACKET_SIZE - CODE_LENGTH - ID_LENGTH

SLEEP_TIME = 100

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

class end_device(LoRa):
    def __init__(self, verbose=False):
        super(end_device, self).__init__(verbose)
        self.set_mode(MODE.SLEEP)
        self.set_dio_mapping(DIO_RX)

        self.packets = []
        self.nb_packets = 0
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 144)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 176)

        self.sending = False
        self.ack = False

    def on_rx_done(self):
        print("\nRX DONE", payload)

        self.set_mode(MODE.STDBY)
        self.clear_irq_flags(RxDone=1)

        payload = self.read_payload(nocheck=True)

        msg_code = payload[0]
        msg = payload[1:]

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

        self.set_mode(MODE.STDBY)
        self.set_dio_mapping(DIO_TX)

        self.clear_irq_flags(TxDone=1)

        self.write_payload(payload)
        self.set_mode(MODE.TX)
        print("end send")

    def image2packets(self, image):
        image_size = len(image)

        self.packets = []

        index = 0
        id = 0
        while index + PAYLOAD_SIZE < image_size:
            self.packets += [CODES["image"] + id + image[index : index + PAYLOAD_SIZE]]
            index += PAYLOAD_SIZE
            id += 1
        self.packets += [CODES["image"] + id + image[index : ]]

        self.nb_packets = len(self.packets)

    def send_image(self):
        for packet in self.packets:
            self.send(packet)
            sleep(SLEEP_TIME)

    def process(self, msg_code, msg):
        if msg_code == CODES["ACK"]:
            self.ack = True

    def start(self):
        print("START")
        self.reset_ptr_rx()
        self.set_mode(MODE.RXCONT)

        sleep(1)

        # Read frame
        ret, frame = self.cam.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Encode
        ret, encoded_frame = cv2.imencode(".jpg", frame, encode_param)

        # Split
        self.image2packets(encoded_frame)

        # Send
        self.send([CODES["image size"], self.nb_packets])

        while self.ack == False:
            print("Waiting for ACK...")
            sleep(0.1)

        self.send_image()

def main():
    ed.set_mode(MODE.STDBY)
    ed.set_pa_config(pa_select=1)
    ed.set_bw(BW.BW500)
    ed.set_coding_rate(CODING_RATE.CR4_5)
    ed.set_spreading_factor(7)
    ed.set_rx_crc(False)
    ed.set_low_data_rate_optim(False)

    print(ed)

    assert(ed.get_agc_auto_on() == 1)

    ed.start()
    # capture
    # encode
    # split
    # if send:
    #   send size + ack = 0
    # if send and ack != 0:
    #   send image

ed = end_device(verbose=False)

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
