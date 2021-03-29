import serial
from serial import Serial
import serial.tools.list_ports as list_serial_ports

class UIFunctions:
    def __init__(self, uiref, MainWindow):
        self.uiref = uiref
        self.MainWindow = MainWindow
        self.setupConnections()
        self.refreshSerial()
        self.connected = False
        self.serialConnection = None


    def setupConnections(self):
        self.uiref.connectButton.clicked.connect(self.initSerial)
        self.uiref.refreshButton.clicked.connect(self.refreshSerial)

    def refreshSerial(self):
        print("Refreshing list of available serial ports")
        portlist = list_serial_ports.comports()
        for p in portlist:
            self.uiref.comPortDropdown.addItem(p.name, userData=p)

    def initSerial(self):
        print("Starting serial connection ")
        port = self.uiref.comPortDropdown.currentText()
        try:
            self.serialConnection =  Serial(
                port=port, \
                baudrate=9600, \
                parity=serial.PARITY_NONE, \
                stopbits=serial.STOPBITS_ONE, \
                bytesize=serial.EIGHTBITS, \
                timeout=0)
        except:
            self.connected = False
        self.connect = True
        self.uiref.connectButton.setText("Disconnect")
        self.uiref.connectButton.clicked.disconnect(self.initSerial)
        self.uiref.connectButton.clicked.connect(self.disconSerial)

    def disconSerial(self):
        self.serialConnection.close()
        self.connected = False
        self.uiref.connectButton.setText("Connect")
        self.uiref.connectButton.clicked.disconnect(self.disconSerial)
        self.uiref.connectButton.clicked.connect(self.initSerial)

    #TODO placeholder quit, clean stuff up here
    def quitApp(self):
        print("Cleaning up stuff...") #We're not really
        print("Quitting app...") #yeah... no.