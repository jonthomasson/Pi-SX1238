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
mode = 0

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

    #init registers
    write_register(0x09, 0) #reg paconfig = set transmit power to 0
    write_register(0x35, 0x8f) #reg fifothresh = fifo start condition not empty
    write_register(0x30, 0x80) #reg packetconfig1 = turn off crc
    write_register(0x31, 0x40) #reg packetconfig2 = packet mode
    write_register(0x25, 0x00) #reg preamblemsb = preamble length
    write_register(0x26, 0x03) #reg preamblelsb = preamble length
    write_register(0x06, 0xe4) #reg frfmsb = frequency 915MHz
    write_register(0x07, 0xc0) #reg frfmid = frequency 915MHz
    write_register(0x08, 0x00) #reg frflsb = frequency 915MHz
    write_register(0x27, 0x91) #reg syncconfig =  auto restart, sync on, fill auto, sync size 2 bytes
    write_register(0x28, 0x5a) #reg syncvalue1
    write_register(0x29, 0x5a) #reg syncvalue2
    write_register(0x02, 0x1a) #reg bitratemsb
    write_register(0x03, 0x0b) #reg bitratelsb
    write_register(0x04, 0x00) #reg fdevmsb = (deviation in Hz = fdev * 61)
    write_register(0x05, 0x52) #reg fdevlsb = see datasheet for max fdev limits (https://www.semtech.com/uploads/documents/sx1238.pdf page 22)
    write_register(0x12, 0x05) #reg rxbw
    write_register(0x33, node_address) #reg rxnodeadrs = set node address

    set_mode(5) #start out in rx mode

    write_register(0x40, 0) #DIO0 is "payload ready"


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
    

def set_mode(newMode):

    if newMode == mode:
        return

    rcv = 0
    regVal = newMode | 0x08
    write_register(0x01, regVal) #update the opmode register with the new mode

    while(rcv & 0x80) == 0x00: #wait for modeready
        rcv = read_register(0x3E) #check irqflags1

    mode = newMode

    if newMode == 3: #if transmit
        GPIO.output(rxen_pin, GPIO.LOW)
        GPIO.output(mode_pin, GPIO.LOW)
        GPIO.output(txen_pin, GPIO.HIGH)
    elif newMode == 5: #if receive
        GPIO.output(rxen_pin, GPIO.HIGH)
        GPIO.output(mode_pin, GPIO.LOW)
        GPIO.output(txen_pin, GPIO.LOW)
    elif newMode == 0: #if sleep
        GPIO.output(rxen_pin, GPIO.LOW)
        GPIO.output(mode_pin, GPIO.LOW)
        GPIO.output(txen_pin, GPIO.LOW)
    elif newMode == 1: #if standby
        GPIO.output(rxen_pin, GPIO.LOW)
        GPIO.output(mode_pin, GPIO.LOW)
        GPIO.output(txen_pin, GPIO.LOW)


    













#GPIO.cleanup()