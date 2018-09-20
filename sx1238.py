#sx1238.py

# External module imports
import RPi.GPIO as GPIO
import spidev
import time

rxen_pin = 29 #rxen pin
mode_pin = 31 #mode pin
txen_pin = 33 #txen pin
reset_pin = 35 #reset pin
ce_pin = 1 #spi ce pin
irq_pin = 37 #irq pin
node_address = 7 #node address
mode = 0

#setup gpio
#GPIO.setmode(GPIO.BOARD)
#GPIO.setup(rxen_pin, GPIO.OUT)
#GPIO.setup(mode_pin, GPIO.OUT)
#GPIO.setup(txen_pin, GPIO.OUT)
#GPIO.setup(reset_pin, GPIO.OUT)


# Enable SPI
spi = spidev.SpiDev()

def init():
    #setup spi
    init_spi()

    return

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

    #set_mode(5) #start out in rx mode

    #write_register(0x40, 0) #DIO0 is "payload ready"


def init_spi():
    bus = 0 #default spi bus
    
    # Open a connection to a specific bus and device (chip select pin)
    spi.open(bus, ce_pin)

    # Set SPI speed and mode
    spi.max_speed_hz = 4000000
    #spi.mode = 3

def read_register(addr):
    return spi.xfer([addr & 0x7F, 0])[1]
   

def write_register(addr, value):
    spi.xfer([addr | 0x80, value])

def handle_interrupt():
    #get the interrupt cause
    irqflags2 = read_register(0x3f)

    if mode == 5 and (irqflags2 & 0x04): #if mode = receive and payload is ready
        #a message has been received
        set_mode(1) #set mode to standby
        #save it in our buffer
        read_fifo()

def read_fifo():
    tempbuf = 0x00 & 0x7f #read from reg fifo
    payloadLength = spi.xfer2(tempbuf)
    

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


    
def destroy():
    spi.close()
    GPIO.cleanup()  











