#sx1238.py

# External module imports
from sx1238_registers import *
import RPi.GPIO as GPIO
import spidev
import time

class SX1238(object):
    def __init__(self, node_address, rxen_pin = 29, mode_pin = 31, txen_pin = 33, int_pin = 37, reset_pin = 35, spi_bus = 0, spi_device = 1):
        self.node_address = node_address
        self.rxen_pin = rxen_pin
        self.mode_pin = mode_pin
        self.txen_pin = txen_pin
        self.int_pin = int_pin
        self.reset_pin = reset_pin
        self.spi_bus = spi_bus
        self.int_lock = False
        self.spi_device = spi_device
        self.mode = ""
        self.promiscuous_mode = False
        self.data_sent = False
        self.data_len = 0
        self.sender_address = 0
        self.target_address = 0
        self.payload_length = 0
        self.ack_requested = 0
        self.ack_received = 0
        self.rssi = 0
        self.data = []
        self.sendSleepTime = 0.05

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.int_pin, GPIO.IN)
        GPIO.setup(self.reset_pin, GPIO.OUT)
        GPIO.setup(self.rxen_pin, GPIO.OUT)
        GPIO.setup(self.mode_pin, GPIO.OUT)
        GPIO.setup(self.txen_pin, GPIO.OUT)

        self.CONFIG = {
            REG_OPMODE: [REG_OPMODE, PACONFIG_OUTPUTPOWER_1], #output power to default
            REG_FIFOTHRESH: [REG_FIFOTHRESH, FIFOTHRESH_TXSTARTCONDITION_FIFOEMPTY | FIFOTHRESH_FIFOTHRESHOLD], #fifo start condition not empty
            REG_PACKETCONFIG1: [REG_PACKETCONFIG1, PACKETCONFIG1_PACKETFORMAT_VARIABLE], #turn off crc
            REG_PACKETCONFIG2: [REG_PACKETCONFIG2, PACKETCONFIG2_DATAMODE_PACKET], #packet mode
            REG_PREAMBLEMSB: [REG_PREAMBLEMSB, PREAMBLEMSB], #preamble length
            REG_PREAMBLELSB: [REG_PREAMBLELSB, PREAMBLELSB], 
            REG_FRFMSB: [REG_FRFMSB, FRFMSB], #frequency 915MHz
            REG_FRFMID: [REG_FRFMID, FRFMID], #frequency 915MHz
            REG_FRFLSB: [REG_FRFLSB, FRFLSB], #frequency 915MHz
            REG_SYNCCONFIG: [REG_SYNCCONFIG, SYNCCONFIG_AUTORESTARTRXMODE_ONWITHPLL | SYNCCONFIG_SYNC_ON | SYNCCONFIG_SYNCSIZE_1], #auto restart, sync on, fill auto, sync size 2 bytes
            REG_SYNCVALUE1: [REG_SYNCVALUE1, SYNCVALUE1], 
            REG_SYNCVALUE2: [REG_SYNCVALUE2, SYNCVALUE2], 
            REG_BITRATEMSB: [REG_BITRATEMSB, BITRATEMSB], #bit rates etc...
            REG_BITRATELSB: [REG_BITRATELSB, BITRATELSB], #bit rates etc...
            REG_FDEVMSB: [REG_FDEVMSB, FDEVMSB], #frequency deviation (deviation in Hz = fdev * 61)
            REG_FDEVLSB: [REG_FDEVLSB, FDEVLSB], #see datasheet for max fdev limits (https://www.semtech.com/uploads/documents/sx1238.pdf page 22)
            REG_RXBW: [REG_RXBW, RXBW] ,
            0x00: [255, 0]
        }

        #initialize SPI
        self.spi = spidev.SpiDev()
        self.spi.open(self.spi_bus, self.spi_device)
        self.spi.max_speed_hz = 4000000

        # Hard reset the SX1238 module
        GPIO.output(self.reset_pin, GPIO.LOW);
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.HIGH);
        time.sleep(0.1)
        GPIO.output(self.reset_pin, GPIO.LOW);
        time.sleep(0.1)

        #write config
        for value in self.CONFIG.values():
            self.write_register(value[0], value[1])

        #set node address
        self.set_address(self.node_address)

        #start out in standby mode
        self.set_mode(SX1238_MODE_STANDBY)
        
        # Wait for ModeReady
        while (self.read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00:
            pass

        #setup interrupt
        GPIO.remove_event_detect(self.int_pin)
        GPIO.add_event_detect(self.int_pin, GPIO.RISING, callback=self.interrupt_handler)
        
    def set_address(self, addr):
        self.node_address = addr
        self.write_register(REG_NODEADRS, self.node_address)

    def read_register(self, addr):
        return self.spi.xfer([addr & 0x7F, 0])[1]

    def write_register(self, addr, value):
        self.spi.xfer([addr | 0x80, value])

    def set_freqeuncy(self, FRF):
        self.write_register(REG_FRFMSB, FRF >> 16)
        self.write_register(REG_FRFMID, FRF >> 8)
        self.write_register(REG_FRFLSB, FRF)

    def set_mode(self, new_mode):
        if new_mode == self.mode:
            return

        reg_val = new_mode | 0x08 #added gausian
        self.write_register(REG_OPMODE, reg_val) #update the opmode register with the new mode

        while self.read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY == 0x00:
            pass

        self.mode = new_mode
        
        if new_mode == SX1238_MODE_TX: #if transmit
            GPIO.output(self.rxen_pin, GPIO.LOW)
            GPIO.output(self.mode_pin, GPIO.LOW)
            GPIO.output(self.txen_pin, GPIO.HIGH)
        elif new_mode == SX1238_MODE_RX: #if receive
            GPIO.output(self.rxen_pin, GPIO.HIGH)
            GPIO.output(self.mode_pin, GPIO.LOW)
            GPIO.output(self.txen_pin, GPIO.LOW)
        elif new_mode == SX1238_MODE_SLEEP: #if sleep
            GPIO.output(self.rxen_pin, GPIO.LOW)
            GPIO.output(self.mode_pin, GPIO.LOW)
            GPIO.output(self.txen_pin, GPIO.LOW)
        elif new_mode == SX1238_MODE_STANDBY: #if standby
            GPIO.output(self.rxen_pin, GPIO.LOW)
            GPIO.output(self.mode_pin, GPIO.LOW)
            GPIO.output(self.txen_pin, GPIO.LOW)
        else:
            return

    def can_send(self):
        if self.mode == SX1238_MODE_STANDBY:
            self.receive_begin()
            return True
        #if signal stronger than -100dBm is detected assume channel activity
        elif self.mode == SX1238_MODE_RX and self.payload_length == 0 and self.read_rssi() < CSMA_LIMIT:
            self.set_mode(SX1238_MODE_STANDBY)
            return True
        return False

    def send(self, to_address, buff = "", request_ack = False):
        #self.write_register(REG_PACKETCONFIG2, (self.read_register(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        now = time.time()
        while (not self.can_send()) and time.time() - now < SX1238_CSMA_LIMIT_S:
            self.receive_done()
        self.send_frame(to_address, buff, request_ack, False)

#    to increase the chance of getting a packet across, call this function instead of send
#    and it handles all the ACK requesting/retrying for you :)
#    The only twist is that you have to manually listen to ACK requests on the other side and send back the ACKs
#    The reason for the semi-automaton is that the lib is ingterrupt driven and
#    requires user action to read the received data and decide what to do with it
#    replies usually take only 5-8ms at 50kbps@915Mhz

    def send_with_retry(self, to_address, buff = "", retries = 3, retry_wait_time = 10):
        for i in range(0, retries):
            self.send(to_address, buff, True)
            sent_time = time.time()
            while (time.time() - sent_time) * 1000 < retry_wait_time:
                if self.ack_received(to_address):
                    return True
        return False

    def ack_received(self, from_node_id):
        if self.receive_done():
            return (self.sender_address == from_node_id or from_node_id == SX1238_BROADCAST_ADDR) and self.ack_received
        return False

    def ack_requested(self):
        return self.ack_requested and self.target_address != SX1238_BROADCAST_ADDR

    def send_ack(self, to_address = 0, buff = ""):
        to_address = to_address if to_address > 0 else self.sender_address
        while not self.can_send():
            self.receive_done()
        self.send_frame(to_address, buff, False, True)

    def send_frame(self, to_address, buff, request_ack, send_ack):
        #turn off receiver to prevent reception while filling fifo
        self.set_mode(SX1238_MODE_STANDBY)
        #wait for modeReady
        while (self.read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_MODEREADY) == 0x00:
            pass
        # DIO0 is "Packet Sent"
        self.write_register(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00)

        if (len(buff) > RF69_MAX_DATA_LEN):
            buff = buff[0:RF69_MAX_DATA_LEN]

        ack = 0
        if send_ack:
            ack = 0x80
        elif request_ack:
            ack = 0x40
        if isinstance(buff, basestring):
            self.spi.xfer2([REG_FIFO | 0x80, len(buff) + 3, to_address, self.address, ack] + [int(ord(i)) for i in list(buff)])
        else:
            self.spi.xfer2([REG_FIFO | 0x80, len(buff) + 3, to_address, self.address, ack] + buff)

        self.data_sent = False
        self.set_mode(SX1238_MODE_TX)
        slept = 0
        while not self.data_sent:
            time.sleep(self.sendSleepTime)
            slept += self.sendSleepTime
            if slept > 1.0:
                break
        self.set_mode(SX1238_MODE_RX)

    def interrupt_handler(self, pin):
        self.int_lock = True
        self.data_sent = True
        if self.mode == SX1238_MODE_RX and self.read_register(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY:
            self.set_mode(SX1238_MODE_STANDBY)
            self.payload_length, self.target_address, self.sender_address, CTLbyte = self.spi.xfer2([REG_FIFO & 0x7f,0,0,0,0])[1:]
            if self.payload_length > 66:
                self.payload_length = 66
            if not (self.promiscuous_mode or self.target_address == self.node_address or self.target_address == SX1238_BROADCAST_ADDR):
                self.payload_length = 0
                self.int_lock = False
                return
            self.data_len = self.payload_length - 3
            self.ack_received = CTLbyte & 0x80
            self.ack_requested = CTLbyte & 0x40

            self.data = self.spi.xfer2([REG_FIFO & 0x7f] + [0 for i in range(0, self.data_len)])[1:]

            self.rssi = self.read_rssi()
        self.int_lock = False

    def receive_begin(self):

        while self.int_lock:
            time.sleep(.1)
        self.data_len = 0
        self.sender_address = 0
        self.target_address = 0
        self.payload_length = 0
        self.ack_requested = 0
        self.ack_received = 0
        self.rssi = 0
        #if (self.read_register(REG_IRQFLAGS2) & RF_IRQFLAGS2_PAYLOADREADY):
            # avoid RX deadlocks
            #self.write_register(REG_PACKETCONFIG2, (self.read_register(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        #set DIO0 to "PAYLOADREADY" in receive mode
        self.write_register(REG_DIOMAPPING1, RF_DIOMAPPING1_DIO0_00)
        self.set_mode(SX1238_MODE_RX)

    def receive_done(self):
        if (self.mode == SX1238_MODE_RX or self.mode == SX1238_MODE_STANDBY) and self.payload_length > 0:
            self.set_mode(SX1238_MODE_STANDBY)
            return True
        #if self.read_register(REG_IRQFLAGS1) & RF_IRQFLAGS1_TIMEOUT:
            # https://github.com/russss/rfm69-python/blob/master/rfm69/rfm69.py#L112
            # Russss figured out that if you leave alone long enough it times out
            # tell it to stop being silly and listen for more packets
           # self.write_register(REG_PACKETCONFIG2, (self.read_register(REG_PACKETCONFIG2) & 0xFB) | RF_PACKET2_RXRESTART)
        elif self.mode == SX1238_MODE_RX:
            # already in RX no payload yet
            return False
        self.receive_begin()
        return False

    def read_rssi(self, force_trigger = False):
        rssi = 0
        #if force_trigger:
        #    self.write_register(REG_RSSICONFIG, RF_RSSI_START)
        #    while self.read_register(REG_RSSICONFIG) & RF_RSSI_DONE == 0x00:
        #        pass
        rssi = self.read_register(REG_RSSIVALUE) * -1
        rssi = rssi >> 1
        return rssi

    def promiscuous(self, onOff):
        self.promiscuous_mode = onOff

    def read_all_regs(self):
        results = []
        for address in range(1, 0x50):
            results.append([str(hex(address)), str(bin(self.read_register(address)))])
        return results

    def read_temperature(self, cal_factor):
        self.set_mode(SX1238_MODE_STANDBY)
        #self.write_register(REG_TEMP1, RF_TEMP1_MEAS_START)
        #while self.read_register(REG_TEMP1) & RF_TEMP1_MEAS_RUNNING:
        #    pass
        # COURSE_TEMP_COEF puts reading in the ballpark, user can add additional correction
        #'complement'corrects the slope, rising temp = rising val
        return (int(~self.read_register(REG_TEMP)) * -1) + COURSE_TEMP_COEF + cal_factor


    def shutdown(self):
        self.set_mode(SX1238_MODE_SLEEP)
        GPIO.cleanup()








