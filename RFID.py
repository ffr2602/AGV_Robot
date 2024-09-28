import serial

class RFID_setting():
    def __init__(self) -> None:
        self.ser = None
        self.ser = serial.Serial()
        self.ser.port = '/dev/serial/by-path/platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.3:1.0-port0'
        self.ser.baudrate = 9600
        self.ser.timeout = 1000
        self.ser.open()

        if self.ser.is_open:
           bufSend = [170, 85, 4, 0, 0, 1, 171]
           self.ser.write(bufSend)

    def set_ID(self, ID):
        if self.ser.is_open:
            SetID = int(ID)
            parset = [1, 2, 3, 4, 5, 6, 7, 8, 172]
            parset[4] = SetID >> 56 & 255
            parset[5] = SetID >> 48 & 255
            parset[6] = SetID >> 40 & 255
            parset[7] = SetID >> 32 & 255
            parset[0] = SetID >> 24 & 255
            parset[1] = SetID >> 16 & 255
            parset[2] = SetID >> 8 & 255
            parset[3] = SetID >> 0 & 255
            bufSend = [170, 81, 11, 0, 0]
            bufSend.extend(parset)
            self.ser.write(bufSend)
    

    