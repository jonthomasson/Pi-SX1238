#sx1238.py

# External module imports
import RPi.GPIO as GPIO
import spidev
import time

rxen_pin = 1 #rxen pin
mode_pin = 2 #mode pin
txen_pin = 3 #txen pin
reset_pin = 4 #reset pin
ce_pin = 5 #spi ce pin
irq_pin = 6 #irq pin
node_address = 7 #node address

#setup gpio
GPIO.setmode(GPIO.BOARD)
GPIO.setup(rxen_pin, GPIO.OUT)
GPIO.setup(mode_pin, GPIO.OUT)
GPIO.setup(txen_pin, GPIO.OUT)
GPIO.setup(reset_pin, GPIO.OUT)
GPIO.setup(ce_pin, GPIO.OUT)

# Enable SPI
spi = spidev.SpiDev()

def init():
    #setup spi
    init_spi()

    #disable mode pins of trx
    GPIO.output(rxen_pin, GPIO.LOW)
    GPIO.output(mode_pin, GPIO.LOW)
    GPIO.output(txen_pin, GPIO.LOW)

    #reset trx
    GPIO.output(rxen_pin, GPIO.LOW)
    time.sleep(.1)
    GPIO.output(rxen_pin, GPIO.HIGH)
    time.sleep(.1)
    GPIO.output(rxen_pin, GPIO.LOW)
    time.sleep(.1)

def init_spi():
    bus = 0 #default spi bus

    # Open a connection to a specific bus and device (chip select pin)
    spi.open(bus, ce_pin)

    # Set SPI speed and mode
    spi.max_speed_hz = 500000
    spi.mode = 0

def read_register(addr):
    
    #transmit register address
    tempbuf = addr & 0x7f
    result = spi.xfer2(tempbuf)

    return result

def write_register(addr, value):
    
    #transmit register address
    tempbuf = addr | 0x80
    spi.xfer2(tempbuf)
    #transmit value
    spi.xfer2(value) 
    


















#GPIO.cleanup()