import serial
from serial import Serial
import serial.tools.list_ports as list_serial_ports
from .serialMonitor import SerialMonitor

class UIFunctions:
    def __init__(self, uiref, MainWindow):
        self.uiref = uiref
        self.MainWindow = MainWindow
        self.setupConnections()
        self.refreshSerial()
        self.connected = False
        self.serialConnection = None
        self.serialMonitor = None

    def setupConnections(self):
        self.uiref.connectButton.clicked.connect(self.initSerial)
        self.uiref.refreshButton.clicked.connect(self.refreshSerial)

    def refreshSerial(self):
        print("Refreshing list of available serial ports")
        #Clear our old list
        for i in range(self.uiref.comPortDropdown.count()):
            self.uiref.comPortDropdown.removeItem(i)
        #And repopulate it
        portlist = list_serial_ports.comports()
        for p in portlist:
            self.uiref.comPortDropdown.addItem(p.name)#, userData=p) # Not actually making use of userData for now

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
        except Exception as e:
            self.connected = False
            print("Error trying to connect to the specified COM port:")
            print(e)
            return
        self.connect = True
        print("Serial connection established, waiting for data...")
        self.uiref.connectButton.setText("Disconnect")
        self.uiref.connectButton.clicked.disconnect(self.initSerial)
        self.uiref.connectButton.clicked.connect(self.disconSerial)
        #Now spin off our serial monitor class with this new connection
        self.serialMonitor = SerialMonitor(self.serialConnection)
        self.serialMonitor.newdata.connect(self.updateDataFromSerial)

    def disconSerial(self):
        #Quit our monitor thread first
        self.serialMonitor.quit()
        self.serialConnection.close()
        self.connected = False
        self.uiref.connectButton.setText("Connect")
        self.uiref.connectButton.clicked.disconnect(self.disconSerial)
        self.uiref.connectButton.clicked.connect(self.initSerial)

        """
        ('magx', ctypes.c_short),
        ('magy', ctypes.c_short),
        ('magz', ctypes.c_short),
        ('magh', ctypes.c_short),
        ('accelx', ctypes.c_short),
        ('accely', ctypes.c_short),
        ('accelz', ctypes.c_short),
        ('envtemp', ctypes.c_short),
        ('envpress', ctypes.c_long),
        ('envalt', ctypes.c_short),
        ('rgbxnorm', ctypes.c_short),
        ('rgbynorm', ctypes.c_short),
        ('rgbznorm', ctypes.c_short),
        """
    def updateDataFromSerial(self, serialdata):
        #Magnetic sensor data
        self.uiref.xdataMag.setText(str(serialdata.magx))
        self.uiref.ydataMag.setText(str(serialdata.magy))
        self.uiref.zdataMag.setText(str(serialdata.magz))
        self.uiref.hdataMag.setText(str(serialdata.magh))
        #Acceleration sensor data
        #
        #Environmental sensor data
        self.uiref.tempdataEnv.setText(str(serialdata.envtemp))
        self.uiref.pressdataEnv.setText(str(serialdata.envpress))
        self.uiref.altdataEnv.setText(str(serialdata.envalt))
        #RGB program data
        self.uiref.xnormData.setText(str(serialdata.rgbxnorm))
        self.uiref.ynormData.setText(str(serialdata.rgbynorm))
        self.uiref.znormData.setText(str(serialdata.rgbznorm))


    #TODO placeholder quit, clean stuff up here
    def quitApp(self):
        print("Cleaning up stuff...") #We're not really
        print("Quitting app...") #yeah... no.