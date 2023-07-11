import sys
import time
from queue import Queue

from PyQt5.QtCore import QThreadPool
from PyQt5.QtWidgets import QWidget, QPushButton, QVBoxLayout
from PyQt5 import QtWidgets

from PlotGraphs import Plot_Graphs
from ViconComms import ViconComms
from WifiPort import Thread_Serial
from UDP_Unity_ForTerry import Thread_UDPUnity

from ControlWindow import  Ui_MainWindow



class MainWifiWindow(QtWidgets.QMainWindow, Ui_MainWindow):

    def __init__(self):
        super(MainWifiWindow, self).__init__()
        # IMU details
        self.queue_IMU = Queue(maxsize=0)
        # threading
        # setup thread pool
        self.threadpool = QThreadPool()
        print("Multithreading with maximum %d threads" % self.threadpool.maxThreadCount())
        # initialise wifi thread as None so graph only opens when there's connection
        self.wifi_thread = None
        self.isRunning = False  # Initialise to not running state
        # self.mSerial = serial.Serial()
        self.DataSaveEnabled = False
        self.UsingVicon = False  # Change this to true to incorporate Vicon
        self.UsingUnity = False  # set to true if wanting udp
        if self.UsingVicon:
            self.serial_worker = ViconComms()  # setup vicon with selected port

        # variables for saving data
        self.filename = "Filename"  # set default value
        self.filedir = str(sys.path[0])  # default path is current directory
        self.imudatafile = None  # this will be the variable storing the .txt file
        self.firstFrameFlag = True # this is set to false once we start saving

        # variables to process the incoming data frames
        self.prevFrameNum = None  # this will be populated  in DataSave

        # layout and windows
        self.graphwindow = None  # No external window yet.
        self.setupUi(self)
        # pre-fill boxes and labels
        if self.UsingVicon:
            self.serial_port_check()  # check COM ports used for Vicon trigger
        self.label_address.setText("Not Connected")
        self.prefill_entries()
        # decide which buttons you can press at start
        # self.startRunButton.setEnabled(False)
        self.stopRunButton.setEnabled(False)
        self.openGraphButton.setEnabled(False)
        self.saveDataButton.setEnabled(False)
        # connect with Unity
        self.unityButton.setEnabled(True)

        # connect signals
        self.wifiButton.clicked.connect(self.evt_start_wifi)
        self.openGraphButton.clicked.connect(self.show_new_window)
        # self.startRunButton.clicked.connect(self.data_collect_enable)
        self.stopRunButton.clicked.connect(self.stop_session)
        self.saveDataButton.clicked.connect(self.data_save_enable)
        self.ComNoSelect.currentTextChanged.connect(self.on_dropdown_changed)
        # self.wifi_thread.IMU_Data_Slot.connect(self.graphwindow.Plot_IMUData)
        # self.unityButton.clicked.connect(self.connectUnity)
        self.unityButton.clicked.connect(self.udpTest)

    # Open the graph only if the WiFi is connected
    def show_new_window(self, checked):
        """ open the graphing window. Only works if WiFi connected """
        if self.graphwindow is None:
            if self.wifi_thread:
                self.graphwindow = Plot_Graphs()
                self.graphwindow.Plot_Init()
                self.graphwindow.show()
                self.data_collect_enable() # start streaming on opening
                # self.startRunButton.setEnabled(True)

            else:
                print("Need to connect to WiFi!")

        else:
            self.graphwindow.close()  # Close window.
            self.graphwindow = None  # Discard reference (so we can reinstantiate another graph later).

    # When WiFi button is pressed, connect to IMU device (client) with TCP server
    def evt_start_wifi(self):
        """ connect to IMU device (client) with TCP server """
        if not self.wifi_thread:
            self.wifi_thread = Thread_Serial(self.queue_IMU)
            # self.wifi_thread.start()
            self.label_address.setText(self.wifi_thread.address[0])
            self.openGraphButton.setEnabled(True)
            # connect with Unity
            self.unityButton.setEnabled(True)
        else:
            print("already connected to WiFi!")

    # 串口检测 search for available COM ports
    def serial_port_check(self):
        # 检测所有存在的串口，将信息存储在字典中
        # Detect all existing serial ports and store the information in a dictionary
        """ detect serial ports and add them to dropdown box """
        self.Com_Dict = {}
        port_list = self.serial_worker.serial_port_check()
        self.ComNoSelect.clear()
        for port in port_list:
            self.Com_Dict["%s" % port[0]] = "%s" % port[1]
            self.ComNoSelect.addItem(port[0])
        # 无串口判断
        if len(self.Com_Dict) == 0:
            self.ComNoSelect.addItem("None Found")

    def serial_port_open(self):
        """ open serial comms with Vicon """
        self.mSerial.port = self.ComNoSelect.currentText()  # 串口号
        # self.thread.mSerial.baudrate = 1382400  # 波特率
        self.mSerial.baudrate = 1000000  # 波特率
        try:
            time.sleep(0.1)
            self.mSerial.open()
            self.mSerial.flushInput()
            time.sleep(0.5)
            # self.IMU_EEG_isEnable = True
        except:
            print("串口异常", "此串口不能被打开\nSerial port exception", "This serial port cannot be opened!")
            return None

    # pre-fill the boxes with the file path and default file name
    def prefill_entries(self):
        """ pre-fill the boxes with the file path and default file name """
        self.fileDirectionLineEdit.setText(self.filedir)  # 获取lineedit中的值
        self.fileNameLineEdit.setText(self.filename)  # 获取lineedit中的值
        print(self.fileDirectionLineEdit.text())
        print(self.fileNameLineEdit.text())

    def vicon_start(self):
        """ triggers Vicon system to start via serial signal """
        cmdstart = 'AB'
        while not (self.CMD_Return[0] == 0xAB):
            success_bytes = self.mSerial.write(bytes.fromhex(cmdstart))
            if self.mSerial.inWaiting():
                self.CMD_Return = self.mSerial.read(self.mSerial.inWaiting())
        print("VICON has started")

    def vicon_stop(self):
        """ sends signal to stop Vicon """
        cmdstop = 'CD'
        while (not (self.CMD_Return[0] == 0xCD)):
            success_bytes = self.mSerial.write(bytes.fromhex(cmdstop))
            if self.mSerial.inWaiting():
                self.CMD_Return = self.mSerial.read(self.mSerial.inWaiting())
        print("VICON has stoped")

    # enable data collection by the IMU system
    def data_collect_enable(self):
        """ set run state on. Open serial comms. Run data streaming via TCP. This does not perform saving,
         for this use self.data_save_enable() and self.data_save() """
        if not self.wifi_thread:
            print("Must connect to WiFi first!")
        else:
            self.isRunning = True
            self.wifi_thread.isStopped = False
            # self.Data_Save_Enable()
            if self.UsingVicon:
                # serial_worker = ViconComms(self.ComNoSelect.currentText()) # setup vicon with selected port
                self.serial_worker.signals.start_record.connect(self.data_save_enable)
                self.serial_worker.signals.stop_record.connect(self.stop_session)
                self.threadpool.start(self.serial_worker)
                # self.serial_port_open()
            self.wifi_thread.IMU_Data_Slot.connect(self.plot_data)
            # self.wifi_thread.UDP_Data_Slot.connect(self.udpTest)
            self.connectUnity()
            self.wifi_thread.start()
            self.stopRunButton.setEnabled(True)
            self.saveDataButton.setEnabled(True)
            # self.startRunButton.setEnabled(False)

    # saving the data
    def data_save(self, IMU_Data):
        # Vicon使用Triger方法同步，所以Python不再需要接收和保存Vicon的数据
        # Vicon uses the Trigger method to synchronize, so Python no longer needs to receive and save Vicon's data
        """ Writes the incoming data from the IMU device to a .txt file. With Vicon, the trigger method ensures it
        is already synchronised, so this data does not need to be saved here in Python """
        frametosave = IMU_Data
        # print("initiating data save")
        # if this is the first frame, we need to open a new file and set the previous frame number
        # if not self.prevFrameNum:
        if self.firstFrameFlag:
            # print("received first frame")
            self.filename = self.fileNameLineEdit.text()  # 获取lineedit中的值
            self.filedir = self.fileDirectionLineEdit.text()  # 获取lineedit中的值
            # print("1st frame, saving to: ", self.filename)
            self.imudatafile = open(str(self.filedir) + '/' + str(self.filename) + '.txt', "w")
            self.prevFrameNum = frametosave[0]
            frametosave[0] = 1
            self.firstFrameFlag = False
        # otherwise we can just find the difference between frame numbers
        else:
            # print("received frame, not first frame")
            # print("saving to: ", self.filename)
            incomingPrevFrameNum = frametosave[0]
            frametosave[0] = frametosave[0] - self.prevFrameNum
            self.prevFrameNum = incomingPrevFrameNum
        # in both cases, we write the incoming data to the file
        datatowrite = [round(x, 4) if i > 0 else x for i, x in enumerate(frametosave)]  # some kind of byte conversion?
        datatowrite = str(datatowrite)
        self.imudatafile.writelines([datatowrite[1:-1], '', '\n'])  # 序列转字符串时，会将[]也转成2个字符‘[’‘]’，存储时需要将字符‘[’‘]’去掉

    # setup the data
    def data_save_enable(self):
        """ initialise setup for saving data. Triggers Vicon if self.UsingVicon is true """
        self.DataSaveEnabled = True
        self.saveDataButton.setEnabled(False)
        # self.vicon_start()  # trigger the vicon system
        if self.UsingVicon:
            self.serial_worker.TrigerVicon_Start()
        # self.wifi_thread.IMU_Data_Slot.connect(self.data_save)  # connect the incoming data to the save function

    def stop_session(self):
        if not self.isRunning:
            print("Run not started yet!")
        else:
            if self.UsingVicon:
                print("using vicon")
                # self.vicon
                self.serial_worker.TrigerVicon_Stop()
            if self.DataSaveEnabled:
                # self.wifi_thread.IMU_Data_Slot.disconnect(self.data_save)
                self.imudatafile.close()
                self.DataSaveEnabled = False
            # self.thread.server.close()
            # self.vicon_stop()  # trigger the vicon system
            # self.wifi_thread.IMU_Data_Slot.disconnect() # disconnect WiFi slot from IMU
            self.wifi_thread.isStopped = True
            # self.graphwindow.close()  # Close graph window.
            # self.graphwindow = None  # Discard reference (so we can reinstantiate another graph later).
            self.firstFrameFlag = True # allows saving to a new file
            self.openGraphButton.setEnabled(True)
            # self.startRunButton.setEnabled(False)
            self.stopRunButton.setEnabled(True)
            self.saveDataButton.setEnabled(True)
            print("Returned to default")

    # def connect_imu_data(self):
    #     """ connect to the IMU data slot of the wifi server """
    #     self.graphwindow.connect()

    def plot_data(self, imudata):
        """ send the data received from the IMU data slot to be plotted  """
        # only perform plotting when the window is open
        if self.graphwindow:
            self.graphwindow.Plot_IMUdata(imudata)
            if self.DataSaveEnabled:
                self.data_save(imudata)

    def on_dropdown_changed(self, value):
        """ update the serial port of the Vicon according to the dropdown box val """
        if self.UsingVicon:
            self.serial_worker.set_port(value)

    def connectUnity(self):
        """ connect to unity to display head movements """
        # return
        # receive IMU data
        # take the gyroscope data from ear(s)
        # process it to conform to Unity
        # send it to unity
        # self.wifi_thread.IMU_Data_Slot.connect(self.plot_data)
        if self.UsingUnity:
            self.unityUDP = Thread_UDPUnity()
            # self.threadpool.start(self.unityUDP)
            self.wifi_thread.UDP_Data_Slot.connect(self.sendToUnity)

    def sendToUnity(self, imudata):
        # return
        # self.unityUDP.IMUtoPRY(imudata)
        # print(imudata)
        self.unityUDP.attitude_calc(imudata)
        self.unityUDP.run()
        # self.threadpool.start(self.unityUDP)

    def udpTest(self):
        import socket

        UDP_IP = "127.0.0.1"
        UDP_PORT = 11000
        MESSAGE = b"hello from PyCharm IMUs!"

        print("UDP target IP: %s" % UDP_IP)
        print("UDP target port: %s" % UDP_PORT)
        print("message: %s" % MESSAGE)

        sock = socket.socket(socket.AF_INET,  # Internet
                             socket.SOCK_DGRAM)  # UDP
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))





