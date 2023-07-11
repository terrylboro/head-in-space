from PyQt5.QtNetwork import *
from PyQt5.QtCore import *

class Server(QTcpServer):
    clientReadyToRead = pyqtSignal(str)
    clientDisconnected = pyqtSignal()

    def __init__(self, parent=None):
        QTcpServer.__init__(self, parent)
        self.socket = self.makeSocket()

    def makeSocket(self):
        socket = QTcpSocket(self)
        socket.readyRead.connect(self.onSocketReadyRead)
        socket.disconnected.connect(self.clientDIsconnected)
        return socket

    def onSocketReadyRead(self):
        data = self.socket.readAll().__str__()
        self.clientReadyToRead.emit(data)

    def incomingConnection(self, socketDescriptor):
        self.socket.setSocketDescriptor(socketDescriptor)