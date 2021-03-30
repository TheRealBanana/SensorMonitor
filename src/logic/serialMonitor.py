# This file simply reads off the wire until its told to quit

#Attempting to make use of QThreads and signals first. If that doesnt work standard threading will do.

from PyQt5.QtCore import QThread, QObject, pyqtSignal, QVariant
from time import sleep
import ctypes

coolit = 0.001 # throttle serial reading

#Not reading all the data we wanted but this is the data I can get right now
#Accel is all 0's because thats the data we aren't sure about for now.
class SD(ctypes.Structure):
    _fields_ = (
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
    )


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
                b = self.ser.readline()
            except:
                continue
            #Do we have a good line (i.e. ends with \r\n)?
            if len(b) < 2 or b[-1] != 10 or b[-2] != 13:
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
