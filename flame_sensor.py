import RPi.GPIO as GPIO
from time import sleep

flame_pin = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(flame_pin, GPIO.IN)

def callback(channel):
    print("Flame detected")

GPIO.add_event_detect(flame_pin, GPIO.BOTH, callback=callback, bouncetime=300)

while True:
    sleep(1)