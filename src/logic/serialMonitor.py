# This file simply reads off the wire until its told to quit

#Attempting to make use of QThreads and signals first. If that doesnt work standard threading will do.

from PyQt5.QtCore import QThread, QObject, pyqtSignal, QVariant
from time import sleep
import ctypes

coolit = 0.01 # throttle serial reading

#Test data class just to read 3 axis
class TD(ctypes.Structure):
    _fields_ = (
        ('x', ctypes.c_short),
        ('y', ctypes.c_short),
        ('z', ctypes.c_short)
    )


class MonThread(QObject):
    newdata = pyqtSignal(list)
    def __init__(self, connection, parent=None):
        super(MonThread, self).__init__(parent)
        self.quitting = False
        self.ser = connection

    def stopthread(self):
        self.stopping = True

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
                data = TD.from_buffer_copy(b)
            except: #Random weird data. Probably a serial issue.
                continue
            realdata = [data.x, data.y, data.z]
            self.newdata.emit(realdata)


class SerialMonitor(QObject):
    newdata = pyqtSignal(list)

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
        self.thread.stopthread()
        self.thread.quit()
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
