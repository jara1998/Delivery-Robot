import RPi.GPIO as GPIO
import numpy as np
import time

GPIO.setmode(GPIO.BCM)
GPIO.setup(10, GPIO.OUT)
GPIO.setup(9, GPIO.OUT)
GPIO.setup(11, GPIO.OUT)
GPIO.setup(16, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)

while True:
	GPIO.output(10, GPIO.HIGH)
	GPIO.output(9, GPIO.HIGH)
	GPIO.output(11, GPIO.HIGH)
	GPIO.output(16, GPIO.HIGH)
	GPIO.output(20, GPIO.HIGH)
	GPIO.output(21, GPIO.HIGH)
	
	time.sleep(3)
	GPIO.output(10, GPIO.LOW)
	GPIO.output(9, GPIO.LOW)
	GPIO.output(11, GPIO.LOW)
	GPIO.output(16, GPIO.LOW)
	GPIO.output(20, GPIO.LOW)
	GPIO.output(21, GPIO.LOW)
	time.sleep(3)
	print("running")

	
