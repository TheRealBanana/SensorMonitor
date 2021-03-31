import serial
from serial import Serial
import serial.tools.list_ports as list_serial_ports
from .serialMonitor import SerialMonitor

#Was going to just subclass the qlineedit but then I'd have to modify MainWindow.py and I dont want to do that
#This works and cleans up the code
def setElementValue(element, value):
    element.setText(str(value))
    element.setCursorPosition(0)

class UIFunctions:
    def __init__(self, uiref, MainWindow):
        self.uiref = uiref
        self.MainWindow = MainWindow
        self.setupConnections()
        self.refreshSerial()
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
            print("Error trying to connect to the specified COM port:")
            print(e)
            return
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
        self.uiref.connectButton.setText("Connect")
        self.uiref.connectButton.clicked.disconnect(self.disconSerial)
        self.uiref.connectButton.clicked.connect(self.initSerial)

        """
        ('magx', ctypes.c_int),
        ('magy', ctypes.c_int),
        ('magz', ctypes.c_int),
        ('magh', ctypes.c_float),
        ('accelx', ctypes.c_int),
        ('accely', ctypes.c_int),
        ('accelz', ctypes.c_int),
        ('gyrox', ctypes.c_int),
        ('gyroy', ctypes.c_int),
        ('gyroz', ctypes.c_int),
        ('yaw', ctypes.c_int),
        ('pitch', ctypes.c_int),
        ('roll', ctypes.c_int),
        ('envtemp', ctypes.c_float),
        ('envpress', ctypes.c_long),
        ('envalt', ctypes.c_int),
        ('rgbxnorm', ctypes.c_int),
        ('rgbynorm', ctypes.c_int),
        ('rgbznorm', ctypes.c_int),
        """
    def updateDataFromSerial(self, serialdata):
        #Magnetic sensor data
        setElementValue(self.uiref.xdataMag, serialdata.magx)
        setElementValue(self.uiref.ydataMag, serialdata.magy)
        setElementValue(self.uiref.zdataMag, serialdata.magz)
        setElementValue(self.uiref.hdataMag, serialdata.magh)
        #Acceleration sensor data
        setElementValue(self.uiref.xdataAcc, serialdata.accelx)
        setElementValue(self.uiref.ydataAcc, serialdata.accely)
        setElementValue(self.uiref.zdataAcc, serialdata.accelz)
        #Gyro sensor data
        setElementValue(self.uiref.xdataGyro, serialdata.gyrox)
        setElementValue(self.uiref.ydataGyro, serialdata.gyroy)
        setElementValue(self.uiref.zdataGyro, serialdata.gyroz)
        #Orientation sensor data
        setElementValue(self.uiref.yawdata, serialdata.yaw)
        setElementValue(self.uiref.pitchdata, serialdata.pitch)
        setElementValue(self.uiref.rolldata, serialdata.roll)
        #Environmental sensor data
        setElementValue(self.uiref.tempdataEnv, serialdata.envtemp)
        setElementValue(self.uiref.pressdataEnv, serialdata.envpress)
        setElementValue(self.uiref.altdataEnv, serialdata.envalt)
        #RGB program data
        setElementValue(self.uiref.xnormData, serialdata.rgbxnorm)
        setElementValue(self.uiref.ynormData, serialdata.rgbynorm)
        setElementValue(self.uiref.znormData, serialdata.rgbznorm)
        #I for some reason thought I had two sets of output data for RGB but I just have one
        #The other boxes we'll just use to color
        #Just plugging in the numbers gives a gradient from black to full color but we want
        #a gradient from white to full color. That requires subtracting the color component we want
        #from the other colors, and setting our color to 255.
        red = "background-color: rgb(%s, %s, %s);" % (255, 255-serialdata.rgbxnorm, 255-serialdata.rgbxnorm)
        green = "background-color: rgb(%s, %s, %s);" % (255-serialdata.rgbynorm, 255, 255-serialdata.rgbynorm)
        blue = "background-color: rgb(%s, %s, %s);" % (255-serialdata.rgbznorm, 255-serialdata.rgbznorm, 255)
        self.uiref.redData.setStyleSheet(red)
        self.uiref.greenData.setStyleSheet(green)
        self.uiref.blueData.setStyleSheet(blue)


    #TODO placeholder quit, clean stuff up here
    def quitApp(self):
        print("Cleaning up stuff...") #We're not really
        print("Quitting app...") #yeah... no.