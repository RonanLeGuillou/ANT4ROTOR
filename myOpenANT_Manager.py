# Copyright (c) 2020, Martin Schmoll <martin.schmoll@meduniwien.ac.at>
#
# The following software uses a modified version the openSource Project openant
# by Gustav Tiger available at https://github.com/Tigge/openant
#
# The software provides a client to communicate with the Powermeter 2INPOWER from Rotor
# https://rotorbike.com/catalog/default/rotor/road/powermeters/2inpower-dm-road.html.
# The client automatically activates and configures the powermeters <FastMode> allowing
# for an overall data-rate of 50 samples per second.
# Note: Higher Samplerates are possible with alternative settings.
#
# The software comes as it is, free of charge and is intended exclusively for non commercial use.
# The author of this software does not take responsibilty for potential malfunctions. Feel free
# to use, modify and extend the software to your personal needs.
#
# Attention: Please keep in mind that there is a 200ms delay in the data!

# Imports
from __future__ import absolute_import, print_function
from openANT.ant.easy.node import Node
from openANT.ant.easy.channel import Channel
import logging
import threading
import array
import time
from multiprocessing import Process, Value
from openANT.Powermeter_Data import Powermeter_Data

## Constants
NETWORK_KEY = [0xB9, 0xA5, 0x21, 0xFB, 0xBD, 0x72, 0xC3, 0x45]

# ROTOR - FastPage Types
PAGE_POWER_ONLY        = 0x10
PAGE_BATTERY_STATUS    = 0x52
PAGE_FAST_DATA_50Hz    = 0xF2
PAGE_FAST_DATA_01_5Hz  = 0xF3
PAGE_FAST_DATA_02_5Hz  = 0xF4

# ROTOR - Message IDs
ID_NONE                 = 0x00
ID_OCA                  = 0x1C      # Optimal Cycling Angle
ID_FORCE_LEFT           = 0x0A
ID_FORCE_RIGHT          = 0x0C
ID_FORCE_TOTAL          = 0x0E
ID_TORQUE_LEFT          = 0x0B
ID_TORQUE_RIGHT         = 0x0D
ID_TORQUE_TOTAL         = 0x0F
ID_POWER                = 0x14
ID_CRANK_ANGLE          = 0x03
ID_CADENCE              = 0x06
ID_BALANCE_LEFT         = 0x23
ID_BALANCE_RIGHT        = 0x26
ID_TORQUE_EFF_LEFT      = 0x21
ID_TORQUE_EFF_RIGHT     = 0x25
ID_PEDAL_SMOOTH_LEFT    = 0x22
ID_PEDAL_SMOOTH_RIGHT   = 0x25

# POWERMETER - CONTROL MODES
NORMAL              = 1
ACTIVATE_FASTMODE   = 2
CONFIGURE_FASTMODE  = 3
RESTORE_STANDARD    = 4


## Main Class
class myOpenAnt_Manager:

    def __init__(self):

        # Kill Variable
        self.__kill = Value('i', 0)

        ## State
        # -1 ... Not Connected
        #  0 ... Connecting
        #  1 ... Connected
        self.USB_STATUS = Value('i', -1)

        ## State
        # -2 ... Error
        # -1 ... Module not active
        #  0 ... Connecting
        #  1 ... Connected
        self.POWERMETER_State = Value('i', -1)
        self.HEARTRATE_State = Value('i', -1)

        ## Device ID's
        self.ID_Rotor_Powermeter = Value('i', 0)
        self.ID_HeartRateMonitor = Value('i', 0)

        ## Activate modules
        self.__activate_Powermeter = Value('i', 0)

        # FastMode Config
        self.PM_Periode_FastMode = 655          # 655 of 32768 -> ~50 Hz
        self.PM_Periode_NormalMode = 8182       # 8182 of 32768 -> ~4 Hz
        self.__PM_F1      = Value('i', 50)      # Main Frequency FastMode, Note: A higher main frequency might be choosen but requires testing and adjustments of PM_Periode_FastMode
        self.__PM_F2      = Value('i', 5)       # Interleaved Frequency FastMode
        self.__PM_TimeOut = Value('i', 60)      # TimeOut of FastMode in [min], Fastmode automatically ends after this time

        # self.MinCadence = 10                                  # [RPM]
        # self.__PM_timeout_Cadence = 60 / self.MinCadence      # [s]

        # Measurements requested for a 0xF2 data page
        self.__F2_Param01 = Value('i',ID_CRANK_ANGLE)
        self.__F2_Param02 = Value('i',ID_TORQUE_TOTAL)

        # Measurements requested for a 0xF3 data page
        self.__F3_Param01 = Value('i',ID_POWER)
        self.__F3_Param02 = Value('i',ID_CADENCE)

        # Measurements requested for a 0xF4 data page
        # Hint: Setting both Parameters to 'ID_NONE' will cause exclusive transmission of only 0xF3 data pages at a
        # maximum samplerate defined at __PM_F2.
        self.__F4_Param01 = Value('i', ID_NONE)
        self.__F4_Param02 = Value('i', ID_NONE)

        # Processing Variables
        self.__PM_SampleCount_50Hz          = Value('i', 0)
        self.__PM_SampleCount_5Hz_Params01  = Value('i', 0)
        self.__PM_SampleCount_5Hz_Params02  = Value('i', 0)
        self.__HR_SampleCount               = Value('i', 0)

        # Output Variables
        self.PM_SampleRate_50Hz         = Value('d', 0)
        self.PM_SampleRate_5Hz_Params01 = Value('d', 0)
        self.PM_SampleRate_5Hz_Params02 = Value('d', 0)
        self.HR_SampleRate              = Value('d', 0)

        # Data container to share data with other modules
        self.__data = Powermeter_Data()

        # PowerMeter Data
        self.__BatteryStatus              = Value('i', -1)
        self.__OCA                        = Value('d', 0.0)
        self.__Force_left                 = Value('d', 0.0)
        self.__Force_right                = Value('d', 0.0)
        self.__Force_total                = Value('d', 0.0)
        self.__Torque_left                = Value('d', 0.0)
        self.__Torque_right               = Value('d', 0.0)
        self.__Torque_total               = Value('d', 0.0)
        self.__Power                      = Value('d', 0.0)
        self.__CrankAngle                 = Value('d', 0.0)
        self.__Cadence                    = Value('d', 0.0)
        self.__Balance_Left               = Value('d', 0.0)
        self.__Balance_Right              = Value('d', 0.0)
        self.__Torque_Efficiency_Left     = Value('d', 0.0)
        self.__Torque_Efficiency_Right    = Value('d', 0.0)
        self.__Pedal_Smoothness_Left      = Value('d', 0.0)
        self.__Pedal_Smoothness_Right     = Value('d', 0.0)

        # Power Processing
        # self.__Power_TimeStamp_NewCycle = Value('d', 0.0)
        # self.__Torque_Left_integrated = Value('d', 0.0)
        # self.__Torque_Right_integrated = Value('d', 0.0)
        # self.__Torque_Total_integrated = Value('d', 0.0)
        # self.__Torque_Left_n = Value('d', 0.0)
        # self.__Torque_Right_n = Value('d', 0.0)
        # self.__Torque_Total_n = Value('d', 0.0)
        # self.__Power_Left_AVG = Value('d', 0.0)
        # self.__Power_Right_AVG = Value('d', 0.0)
        # self.__Power_Total_AVG = Value('d', 0.0)
        # self.__Cadence_AVG = Value('d', 0.0)



    # Start the Process
    def start(self, exit_Event, activate_Powermeter, ID_Rotor_Powermeter):

        # Exit Event
        self.__exit_Event = exit_Event

        # Enable Modules
        if activate_Powermeter:
            self.__activate_Powermeter.value = 1
            self.POWERMETER_State.value = 0

        # Set device IDs
        self.ID_Rotor_Powermeter.value = ID_Rotor_Powermeter

        # Set Logging Config
        logging.basicConfig()
        logger = logging.getLogger()
        logger.setLevel(logging.ERROR)

        # Start Connecting Thread
        self.__thread_Connecting_Worker = threading.Thread(name='ANT+_Manager_Connection_Thread', target=self.__connection_worker, args=())
        self.__thread_Connecting_Worker.daemon = True
        self.__thread_Connecting_Worker.start()


    # Tries to establish a connection to the ANT+ Stick
    def __connection_worker(self):

        sleepy_time = 2.0

        while not self.__exit_Event.isSet():

            # Start Connecion process
            if self.USB_STATUS.value == -1:

                # Trying to connect
                if self.__activate_Powermeter.value == 1:
                    self.POWERMETER_State.value = 0

                self.USB_STATUS.value = 0
                self.__read_Process = Process(target=self.__connect, args=())
                self.__read_Process.daemon = True
                self.__read_Process.start()

                # Wait for a response of the ANT+ Stick to the RESET command
                while self.USB_STATUS.value == 0:
                    time.sleep(0.1)


                if self.USB_STATUS.value == -1:
                    if self.__activate_Powermeter.value == 1:
                        self.POWERMETER_State.value = -2

                    print("ANT+ Stick not responding. Retry.")
                    self.__read_Process.terminate()
                    self.__read_Process.join()

                    # Retry to connect
                    self.USB_STATUS.value = 0
                    self.__read_Process = Process(target=self.__connect, args=())
                    self.__read_Process.daemon = True
                    self.__read_Process.start()

            elif self.USB_STATUS.value == 1:

                # Calculate Sample Rates
                self.PM_SampleRate_50Hz.value = self.__PM_SampleCount_50Hz.value / sleepy_time
                self.PM_SampleRate_5Hz_Params01.value = self.__PM_SampleCount_5Hz_Params01.value / sleepy_time
                self.PM_SampleRate_5Hz_Params02.value = self.__PM_SampleCount_5Hz_Params02.value / sleepy_time

                self.__PM_SampleCount_50Hz.value = 0
                self.__PM_SampleCount_5Hz_Params01.value = 0
                self.__PM_SampleCount_5Hz_Params02.value = 0

            # Sleep like a little french princess
            time.sleep(sleepy_time)


    # Establishes an ANT+ Connection
    def __connect(self):

        try:
            self.node = Node()
        except:
            self.USB_STATUS.value = -1
            return

        # Received some Data from ANT+ Stick - Scoooore!
        if self.node.ant.USB_OK == True:
            self.USB_STATUS.value = 1

        # Something went wrong while setting up USB connection -> Suicide XX
        else:
            self.USB_STATUS.value = -1
            return

        # Set Network Key
        self.node.set_network_key(0, NETWORK_KEY)

        # Powermeter
        self.__PM_Control_Mode          = NORMAL
        self.__PM_FastMode_active       = False
        self.__PM_FastMode_configured   = False
        self.__PM_RECEIVED_DATA         = False

        self.__RequestPage = None
        self.__RequestedPage_Received   = False

        # Start Controlling Thread
        self.__thread_Control_Worker = threading.Thread(name='ANT+_Manager_PowerMeter_Control', target=self.__worker_CONTROL_PM, args=())
        self.__thread_Control_Worker.daemon = True
        self.__thread_Control_Worker.start()

        # Setup Powermeter
        if self.__activate_Powermeter.value == 1:
            self.channel_PW = self.node.new_channel(Channel.Type.BIDIRECTIONAL_RECEIVE)
            self.channel_PW.on_broadcast_data = self.on_data_PW
            self.channel_PW.on_burst_data = self.on_data_PW
            str(self.channel_PW.set_period(self.PM_Periode_NormalMode))     # 8182 of 32768 -> ~4Hz; FastMode: 655 -> ~50Hz
            self.channel_PW.set_search_timeout(255)                         # '255' means infinite searching otherwise timeout in [s]
            self.channel_PW.set_rf_freq(57)                                 # 2457 MHz
            self.channel_PW.set_id(self.ID_Rotor_Powermeter.value, 11, 5)   # SerialNumber, DeviceType, Transmissiontype


        try:
            if self.__activate_Powermeter.value == 1:
                self.channel_PW.open()

            self.node.start()
            print('Node started')
        except Exception as error:
            print('error: ' + str(error))


    # This worker allows to adjust the Powermeter without interferring with receiving data
    def __worker_CONTROL_PM(self):

        while self.__kill.value == 0:

            if self.__PM_Control_Mode == NORMAL:
                pass

            elif self.__PM_Control_Mode == ACTIVATE_FASTMODE:
                if self.__activateFastMode():
                    self.__PM_Control_Mode = NORMAL

            elif self.__PM_Control_Mode == CONFIGURE_FASTMODE:
                if self.__configureFastMode():
                    self.__PM_Control_Mode = NORMAL

            elif self.__PM_Control_Mode == RESTORE_STANDARD:
                if self.__restoreStandardMode():
                    self.__PM_Control_Mode = NORMAL

            time.sleep(1)

        # Clean Exit
        print("Try clean exit")
        if self.__PM_FastMode_active:
            self.__restoreStandardMode()

        self.stop_node()


    # Activates the Fastmode on the ROTOR PowerMeter
    def __activateFastMode(self):

        F1 = self.__PM_F1.value
        F2 = self.__PM_F2.value
        TO = self.__PM_TimeOut.value

        CMD1 = [0xF0, 0x03, 0x00, 0x00, TO, F1, F2, 0xFF]

        error = False
        for i in range(10):
            try:
                print("ROTOR Powermeter: Try to activate FastMode")
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                error = False
                break

            except Exception as err:
                error = True
                print("Error Sending COMMAND: " + str(err))

        # Unable to send Message
        if error:
            return False

        # Request Page
        result = self.__wait4Response(PAGE_FAST_DATA_50Hz, 3)
        self.__PM_FastMode_active = result

        if self.__PM_FastMode_active:
            print(str(self.channel_PW.set_period(self.PM_Periode_FastMode)))

        return result


    # Requests a specific page and waits until it is received (or until time_out)
    def __wait4Response(self, PAGE_TYPE, time_out):

        result = False

        self.__RequestedPage_Received = False
        self.__RequestPage = PAGE_TYPE

        n = int(round(time_out / 0.1))

        for i in range(n):

            if self.__RequestedPage_Received:
                result = True
                break

            time.sleep(0.1)

        return result


    # Reverts the Powermeter into Standard Mode
    def __restoreStandardMode(self):

        CMD1 = [0xF0, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF]

        error = False

        for i in range(10):
            try:
                print("ROTOR Powermeter: Try to restore Standard Mode")
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))
                print("Send COMMAND" + self.toHexString(CMD1))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD1))

                error = False
                break

            except Exception as err:
                print("Error Sending COMMAND" + str(err))
                error = True

        # Unable to send Message
        if error:
            return False

        # Request Page
        result = self.__wait4Response(PAGE_POWER_ONLY, 3)

        if result:
            self.__PM_FastMode_active = False
            self.channel_PW.set_period(self.PM_Periode_NormalMode)

        return result


    # Configures the FastMode according to the desired Parameters
    def __configureFastMode(self):

        print("ROTOR Powermeter: Configure Fastmode")
        self.POWERMETER_State.value = 0

        # Select the data to which should be delivered
        F2_1 = self.__F2_Param01.value  # 40 Hz
        F2_2 = self.__F2_Param02.value
        F3_1 = self.__F3_Param01.value  # 5 Hz
        F3_2 = self.__F3_Param02.value
        F4_1 = self.__F4_Param01.value  # 5 Hz
        F4_2 = self.__F4_Param02.value

        CMD2 = [0xF0, 0x04, 0x00, 0x00, F2_1, F2_2, F3_1, F3_2]
        CMD3 = [0xF0, 0x05, 0x00, 0x00, F3_1, F3_2, F4_1, F4_2]

        error = False

        for i in range(10):
            try:
                print("Send COMMAND (try: "+str(i+1)+"): " + self.toHexString(CMD2))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD2))
                print("Repeat COMMAND: " + self.toHexString(CMD2))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD2))  # Send twice for safety

                error = False
                break

            except Exception as err:
                print("Error Sending COMMAND: " + str(err))
                error = True

        # Unable to send Message
        if error:
            print("ROTOR Powermeter: Configuration of FastMode failed!")
            self.POWERMETER_State.value = -2
            return False


        for i in range(10):
            try:
                print("Send COMMAND (try: "+str(i+1)+"): " + self.toHexString(CMD3))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD3))
                print("Repeat COMMAND: " + self.toHexString(CMD3))
                self.channel_PW.send_acknowledged_data(array.array('B', CMD3))   # Send twice for safety

                error = False
                break

            except Exception as err:
                print("Error Sending COMMAND: " + str(err))
                error = True

        # Unable to send Message
        if error:
            print("ROTOR Powermeter: Configuration of FastMode failed!")
            self.POWERMETER_State.value = -2
            return False

        print("ROTOR-Powermeter: FastMode successfully CONFIGURATED")
        self.__PM_FastMode_configured = True
        self.POWERMETER_State.value = 1

        return True


    # Converts byte data into hex string
    def toHexString(self, data):

        str = "["

        for x in data:
            str =  str + " " + format(x, '02X')

        str = str + " ]"

        return str.upper()



    # Receives and handles incoming Powermeter data from the OPENANT Library
    def on_data_PW(self, data):

        # Determining Page type
        page_type = data[0]

        # Interpret Battery Status
        if(page_type == PAGE_BATTERY_STATUS):
            value = data[7]
            mask = 0b00001110

            batstatus = value & mask
            batstatus = batstatus >> 1

            self.__BatteryStatus.value = batstatus
            print("Battery Status:" + str(batstatus))

        # Respond on requested Pages
        if  page_type == self.__RequestPage and \
            self.__RequestedPage_Received == False:
            self.__RequestedPage_Received = True


        # Successfully connected
        if not self.__PM_RECEIVED_DATA:
            self.__PM_RECEIVED_DATA = True
            print("ROTOR-Powermeter: Successfully Connected!")

        # Updating number of received Samples
        if page_type == PAGE_FAST_DATA_50Hz:
            self.__PM_SampleCount_50Hz.value = self.__PM_SampleCount_50Hz.value + 1
        elif page_type == PAGE_FAST_DATA_01_5Hz:
            self.__PM_SampleCount_5Hz_Params01.value = self.__PM_SampleCount_5Hz_Params01.value + 1

        elif page_type == PAGE_FAST_DATA_02_5Hz:
            self.__PM_SampleCount_5Hz_Params02.value = self.__PM_SampleCount_5Hz_Params02.value + 1



        # Fast Data
        if(page_type == PAGE_FAST_DATA_50Hz or
           page_type == PAGE_FAST_DATA_01_5Hz or
           page_type == PAGE_FAST_DATA_02_5Hz):

            # Fast Mode is already active - need to be reverted first to start clean
            if  not self.__PM_FastMode_active:
                print("ROTOR-Powermeter: FastMode active!")
                self.channel_PW.set_period(self.PM_Periode_FastMode)
                self.__PM_FastMode_active = True
                self.__PM_Control_Mode = RESTORE_STANDARD

            else:
                # Configure FastMode if necessary
                if  self.__PM_FastMode_configured == False and \
                    self.__PM_Control_Mode == NORMAL:
                    self.__PM_Control_Mode = CONFIGURE_FASTMODE


            # Parse Data
            (id1, value1, id2, value2) = self.parseFastPageData(data)

            # Update each value
            self.updateData(id1, value1)
            self.updateData(id2, value2)


        # Activate FastMode if not done yet
        elif  page_type == PAGE_POWER_ONLY and \
            not self.__PM_FastMode_active and \
            self.__PM_Control_Mode == NORMAL and\
            not self.__BatteryStatus.value == -1:

            self.__PM_Control_Mode = ACTIVATE_FASTMODE


    # Parses Fast Page Data
    # 0  # : 0xF2 : Fast Mode Page (0xF2, 0xF3, 0xF4)
    # 1  # : 0xXX : Time Tag. Frames counter
    # 2  # : 0xXX : Data 1 ID. (P.E:  Code 3 = Angle (Rad/1000), code 10 =Force(N/10))
    # 3  # : 0xXX : Data 1. Low Byte
    # 4  # : 0xXX : Data 1. High Byte. Data  = (HighByte * 256)+ LowByte
    # 5  # : 0xXX : Data 2 ID
    # 6  # : 0xXX : Data 2. Low Byte
    # 7  # : 0xXX : Data 2. High Byte. Data  = (HighByte * 256)+ LowByte
    def parseFastPageData(self, data):

        # First Data
        ID1 = data[2]
        LB1 = data[3]
        HB1 = data[4]

        if HB1 > 127:
            HB1 = HB1 - 256

        Value1 = float(256 * HB1 + LB1)

        # Second Data
        ID2 = data[5]
        LB2 = data[6]
        HB2 = data[7]

        if HB2 > 127:
            HB2 = HB2 - 256

        Value2 = float(256 * HB2 + LB2)

        return ID1, Value1, ID2, Value2


    # Updates a certain data field based on the given ID
    def updateData(self, id, value):

        if id == ID_OCA:
            # OCA [deg] = value / 100
            self.__OCA.value = value / 100.0

        elif id == ID_FORCE_LEFT:
            # Force [N] = value / 10
            self.__Force_left.value = value / 10.0

        elif id == ID_FORCE_RIGHT:
            # Force [N] = value / 10
            self.__Force_right.value = value / 10.0

        elif id == ID_FORCE_TOTAL:
            # Force [N] = value / 10
            self.__Force_total.value = value / 10.0

        elif id == ID_TORQUE_LEFT:
            # Torque [Nm] = value / 100
            self.__Torque_left.value = value / 100.0

        elif id == ID_TORQUE_RIGHT:
            # Torque [Nm] = value / 100
            self.__Torque_right.value = value / 100.0

        elif id == ID_TORQUE_TOTAL:
            # Torque [Nm] = value / 100
            self.__Torque_total.value = value / 100.0

        elif id == ID_POWER:
            # Power [W] = value / 10
            self.__Power.value = value / 10.0

        elif id == ID_CADENCE:
            # Cadence [RPM] = value / 100
            self.__Cadence.value = value / 100.0

        elif id == ID_CRANK_ANGLE:
            # CrankAngle [rad] = value / 1000
            # CrankAngle [deg] = CrankAngle [rad] * (180/PI)
            t_CrankAngle = value * 0.0572957795130823
            diff_CrankAngle = t_CrankAngle - self.__CrankAngle.value
            self.__CrankAngle.value = t_CrankAngle

        elif id == ID_BALANCE_LEFT:
            # Balance [%] = value / 100
            self.__Balance_Left.value = value / 100.0

        elif id == ID_BALANCE_RIGHT:
            # Balance [%] = value / 100
            self.__Balance_Right.value = value / 100.0

        elif id == ID_TORQUE_EFF_LEFT:
            # Torque Efficiency [%] = value / 2
            self.__Torque_Efficiency_Left.value = value / 2.0

        elif id == ID_TORQUE_EFF_RIGHT:
            # Torque Efficiency [%] = value / 2
            self.__Torque_Efficiency_Right.value = value / 2.0

        elif id == ID_PEDAL_SMOOTH_LEFT:
            # Pedal Smoothness [%] = value / 2
            self.__Pedal_Smoothness_Left.value = value / 2.0

        elif id == ID_PEDAL_SMOOTH_RIGHT:
            # Pedal Smoothness [%] = value / 2
            self.__Pedal_Smoothness_Right.value = value / 2.0


    # Returns the currently available Data as Powermeter_Data Object
    def getData(self):

        self.__data.BatteryStatus = self.__BatteryStatus.value

        self.__data.DataRate_F1 = self.PM_SampleRate_50Hz.value
        self.__data.DataRate_F2 = self.PM_SampleRate_5Hz_Params01.value
        self.__data.DataRate_F3 = self.PM_SampleRate_5Hz_Params02.value

        self.__data.OCA = self.__OCA

        self.__data.Force_Left = self.__Force_left.value
        self.__data.Force_Right = self.__Force_right.value
        self.__data.Force_Total = self.__Force_total.value

        self.__data.Torque_Left = self.__Torque_left.value
        self.__data.Torque_Right = self.__Torque_right.value
        self.__data.Torque_Total = self.__Torque_total.value

        self.__data.Power = self.__Power.value

        self.__data.CrankAngle = self.__CrankAngle.value
        self.__data.Cadence = self.__Cadence.value

        self.__data.Balance_Left = self.__Balance_Left
        self.__data.Balance_Right = self.__Balance_Right

        self.__data.Torque_Efficiency_Left = self.__Torque_Efficiency_Left
        self.__data.Torque_Efficiency_Right = self.__Torque_Efficiency_Right

        self.__data.Pedal_Smoothness_Left = self.__Pedal_Smoothness_Left
        self.__data.Pedal_Smoothness_Right = self.__Pedal_Smoothness_Right

        return self.__data



    def stop_node(self):
        self.node.stop()
        self.text.insert('1.0', 'NODE STOPPED')


    # Initiates a clean exit
    def clean_exit(self):
        self.__kill.value = 1


    # Returns the current state of the Power Meter
    def getState_PowerMeter(self):
        if self.__activate_Powermeter.value == 0:
            return -1

        if self.USB_STATUS.value == 1:
            return self.POWERMETER_State.value
        else:
            return -2

    # Returns the device ID of the Powermeter currently used
    def get_ID_Rotor_Powermeter(self):
        return self.ID_Rotor_Powermeter.value





# Simple Testroutine to display the current CrankAngle every second.
# It additionally shows the sample-rate at which the CrankAngle is received.
if __name__ == '__main__':

    # ID of your 2INPOWER
    activate_Powermeter = True
    rotor_ID = 54968

    # Define exit event
    exit = threading.Event()

    # Configure ANT-Manager
    myANT = myOpenAnt_Manager()
    myANT.start(exit, activate_Powermeter, rotor_ID)


    try:
        while True:

            if myANT.USB_STATUS.value == 1:

                if myANT.POWERMETER_State.value == 1:
                    data = myANT.getData()
                    print("CrankAngle = " + str(data.CrankAngle) + " DEG( Samplerate: " + str(myANT.PM_SampleRate_50Hz)+" Hz) ")

                elif myANT.POWERMETER_State.value == 0:
                    print("Waiting for device: connecting...")
                elif myANT.POWERMETER_State.value == -1:
                    print("Powermeter not activated...")
                elif myANT.POWERMETER_State.value == -2:
                    print("Error, retrying to connect...")

            else:
                print("-")

            time.sleep(1)
    except KeyboardInterrupt:

        print('\nUser requested Application STOP\n')
        myANT.clean_exit()   # Make sure that clean_exit is called, otherwise the Powermeter remains in FastMode until the defined TimeOut
        time.sleep(20)


    except Exception as error:
        pass
