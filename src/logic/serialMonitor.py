# This file simply reads off the wire until its told to quit

#Attempting to make use of QThreads and signals first. If that doesnt work standard threading will do.

from PyQt5.QtCore import QThread, QObject, pyqtSignal, QVariant
from time import sleep
import ctypes

coolit = 0.001 # throttle serial reading

#Has to be in the same order as the struct in our arduino code
class SD(ctypes.Structure):
    _fields_ = (
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
        ('envpress', ctypes.c_int),
        ('envalt', ctypes.c_float),
        ('rgbxnorm', ctypes.c_int),
        ('rgbynorm', ctypes.c_int),
        ('rgbznorm', ctypes.c_int),
    )

#Having some trouble with pyserial. For some reason its stopping at 32 bytes.
def _readline(ser):
    c = ser.read(1)
    line = c
    while c != 13 and line[-1] != 10 and line[-7:-2] != "ZENDZ":
        c = ser.read(1)
        line += c
    c = ser.read(1)
    line += c
    #Cut out the ZENDZ
    line = line[:-7] + line[-2:]
    return line

class MonThread(QObject):
    newdata = pyqtSignal(QVariant)
    def __init__(self, connection, parent=None):
        super(MonThread, self).__init__(parent)
        self.quitting = False
        self.ser = connection

    def stopthread(self):
        self.quitting = True

    def readdatafromserial(self):
        while self.quitting is False:
            sleep(coolit) # Dont want to go too crazy
            #Read a single line
            try:
                b = _readline(self.ser)
            except:
                continue
            #Now read the data into a cstruct
            try:
                data = SD.from_buffer_copy(b)
            except: #Random weird data. Probably a serial issue.
                continue
            self.newdata.emit(data)
        print("Quit called on MonThread")


class SerialMonitor(QObject):
    newdata = pyqtSignal(QVariant)

    def __init__(self, connection, parent=None):
        super(SerialMonitor, self).__init__(parent)
        self.connection = connection
        self.stopping = False
        self.thread = None
        self.mt = None
        self.initmon()

    def newdataemit(self, newdata):
        self.newdata.emit(newdata)

    def quit(self):
        #End the thread and then close out
        self.mt.stopthread()
        self.thread.terminate()
        self.thread.wait()
        #self.thread.deleteLater()
        self.thread = None

    def initmon(self):
        #Spin up qthread and start monitoring
        if self.thread is not None:
            self.thread.quit()
        self.thread = QThread()
        self.thread.setTerminationEnabled(True)
        self.mt = MonThread(self.connection)
        self.mt.moveToThread(self.thread)
        self.thread.started.connect(self.mt.readdatafromserial)
        self.mt.newdata.connect(self.newdataemit)
        self.thread.start()
