from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from logic.ui_functions import UIFunctions
from dialogs.MainWindow import Ui_MainWindow
import sys


class ClosableMainWindow(QtWidgets.QMainWindow):
    MainWindowClose = pyqtSignal()
    def __init__(self, parent=None):
        super(ClosableMainWindow, self).__init__(parent)

    def closeEvent(self, closeEvent):
        self.MainWindowClose.emit()
        closeEvent.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = ClosableMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    uifuncts = UIFunctions(ui, MainWindow)
    # Hook into the app's quiting sequence so it saves our settings before it quits
    MainWindow.MainWindowClose.connect(uifuncts.quitApp)

    #Now that we have the Ui set up, lets finish the initiation process
    #uifuncts.appInit()
    MainWindow.show()
    app.exec_()