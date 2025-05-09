import platform

if platform.system() == "Linux":
    try:
        import RPi.GPIO as GPIO
        IS_RPI = True
        DefaultPort = "/dev/ttyACM0"
    except ImportError:
        IS_RPI = False
        DefaultPort = "/dev/ttyACM0"
else:
    IS_RPI = False
    DefaultPort = "COM29"

import time
import threading
import os
import struct

import serial
from pyubx2 import UBXMessage, UBXReader, SET, POLL
from queue import Queue
from datetime import datetime
import csv

from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer
import subprocess

LOG_FOLDER_PATH = os.path.join(os.getcwd(), "logs")
MAX_LOG_FILE_SIZE = 30 * 1024 * 1024  # 30 MB
MAX_PPK_FILE_SIZE = 20 * 1024 * 1024  # 20 MB

MinFixGNSSSolution = 2
FixLedPin = 18
SerialTimeoutSec = 60  # seconds
SoftwareVersion = "v2.0.0"

class Logger:
    def __init__(self, port=DefaultPort, baudrate=38400):
        # NAV-PVT data's log file variables
        self.logFolderPath = None
        self.currentFileName = None
        self.logFileSize = 0
        self.logFileIndex = 0
        self.isCreatedFirstLogFile = False
        self.navPvtQueue = Queue()

        # PPK(RAWX & SFRX) data's log file variables
        self.ppkLogFilePtr = None
        self.ppkFileName = False
        self.ppkFileIndex = 0
        self.ppkFileSize = 0
        self.isCreatedFirstPPKFile = False
        self.ppkQueue = Queue()

        self.fixType = 0

        self.gnssBasedSystemTime = None  # this uses the file name, it takes the value from gnss time
        self.rpiSystemTime = None
        self.isValidSystemTime = False
        self.isValidSystemTime = False
        self.lastDataTime = 0

        self.diagnosticBuffer = []
        self.diagnosticFilePath = None
        self.startupTime = time.time()

        self.configChanged = False
        self.configFailed = False

        if IS_RPI:
            GPIO.setmode(GPIO.BCM)
            GPIO.setup(FixLedPin, GPIO.OUT)
            GPIO.output(FixLedPin, GPIO.LOW)

        try:
            self.serialPort = serial.Serial(port, baudrate, timeout=10)  # Create serial port object
        except Exception as e:
            self.diagnosticBuffer = [f"[ERROR] Serial port open failed: {e}"]
            self.gnssBasedSystemTime = datetime.now()
            self.StartFailedMode()
            return

        self.ubxReader = UBXReader(self.serialPort, protfilter=2)  # It just accepts UBX messages


    def GetCPUId(self):
        serial = "0000000000000000"
        try:
            with open('/proc/cpuinfo', 'r') as f:
                for line in f:
                    if line.startswith('Serial'):
                        serial = line.strip().split(":")[1].strip()
                        break
        except FileNotFoundError:
            serial = "xSerial"
        return f"wBox-{serial[-7:].upper()}"

    def LogDiagnostic(self, message, level="INFO"):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        line = f"[{level}] {timestamp} - {message}"
        print(line)  # ekrana da yaz

        self.diagnosticBuffer.append(line)

        if self.diagnosticFilePath:
            try:
                with open(self.diagnosticFilePath, "a") as f:
                    f.write(line + "\n")
            except Exception as e:
                print("Diagnostic log write error:", e)

    def StartFailedMode(self):
        def BlinkErrorLED():
            while IS_RPI:
                GPIO.output(FixLedPin, GPIO.HIGH)
                time.sleep(0.5)
                GPIO.output(FixLedPin, GPIO.LOW)
                time.sleep(0.5)

        if IS_RPI:
            blink_thread = threading.Thread(target=BlinkErrorLED)
            blink_thread.daemon = True
            blink_thread.start()

        self.LogDiagnostic("GNSS configuration failed. Entering FAILED MODE.", "ERROR")

        # GNSS time is not valid, use system time
        if not self.gnssBasedSystemTime:
            self.gnssBasedSystemTime = datetime.now()
        self.CreateLogFolder()

        # Flag file adding
        try:
            flag_path = os.path.join(self.logFolderPath, "FAILED_MODE.flag")
            with open(flag_path, "w") as f:
                f.write("GNSS configuration failed.\n")
                f.flush()
            self.LogDiagnostic("FAILED_MODE.flag file created.")
        except Exception as e:
            self.LogDiagnostic("FAILED_MODE.flag could not be created.", "ERROR")

        print("\n SYSTEM IN FAILED MODE — check diagnostic.log for details\n")

    def CreateLogFolder(self):
        try:
            currentTime = self.gnssBasedSystemTime.strftime("%Y-%m-%d_%H-%M-%S")
            folderName = f"Log_{currentTime}"
            self.logFolderPath = os.path.join(LOG_FOLDER_PATH, folderName)
            os.makedirs(self.logFolderPath, exist_ok=True)

            self.diagnosticFilePath = os.path.join(self.logFolderPath, "diagnostic.log")
            with open(self.diagnosticFilePath, "w") as f:
                f.write(f"[INFO] Log folder created at {currentTime}\n")
                f.write(f"[INFO] wBox Serial ID: {self.GetCPUId()}\n")
                f.write(f"[INFO] Software Version: {SoftwareVersion}\n")
                f.flush()

            # Write before added logs
            for line in self.diagnosticBuffer:
                with open(self.diagnosticFilePath, "a") as f:
                    f.write(line + "\n")

            self.LogDiagnostic("Diagnostic file initialized.")

        except Exception as e:
            self.LogDiagnostic(f"CreateLogFolder error: {e}", "ERROR")

    def CreateLogFile(self):
        if self.logFolderPath is None:
            self.CreateLogFolder()
        try:
            if self.logFolderPath is None:
                self.LogDiagnostic("Cannot create log file because folder is not available.", "ERROR")
                return

            while True:
                currentTime = self.gnssBasedSystemTime.strftime("%Y-%m-%d_%H-%M-%S")
                self.currentFileName = os.path.join(self.logFolderPath, f"GNSSLog_{currentTime}_{self.logFileIndex}.csv")
                if not os.path.exists(self.currentFileName):
                    self.logFileIndex += 1
                    break
            self.isCreatedFirstLogFile = True

            with open(self.currentFileName, "w", newline='') as file:
                writer = csv.writer(file)
                writer.writerow(
                    ['ITOW', 'Year', 'Month', 'Day', 'Hour', 'Minute', 'Second', 'Millisecond',
                     'FixType', 'NumSV',
                     'Latitude(deg)', 'Longitude(deg)', 'Altitude(m)', 'AltitudeMSL(m)', 'hAcc(m)', 'vAcc(m)',
                     'NEDNorth(m/s)', 'NEDEast(m/s)', 'NEDDown(m/s)',
                     'GroundSpeed(m/s)', 'SpeedAccuracy(m/s)', 'HeadingOfMotion(deg)', 'HeadingAccuracy(deg)',
                     'PositionDOP', 'HeadingOfVehicle(deg)',
                     'GNSSTime', 'SystemTime'])
                file.flush()
        except Exception as e:
            self.LogDiagnostic(f"CreateLogFile error: {e}", "ERROR")

    def CreatePPKFile(self):
        if self.logFolderPath is not None:
            while True:
                currentTime = self.gnssBasedSystemTime.strftime("%Y-%m-%d_%H-%M-%S")
                self.ppkFileName = os.path.join(self.logFolderPath,
                                                    f"GNSSRaw_{currentTime}_{self.ppkFileIndex}.ubx")
                if not os.path.exists(self.ppkFileName):
                    self.ppkFileIndex += 1
                    break
            self.ppkLogFilePtr = open(self.ppkFileName, "ab")
            self.isCreatedFirstPPKFile = True

    def CheckFileSize(self):
        if self.isCreatedFirstLogFile:
            self.logFileSize = os.path.getsize(self.currentFileName)
            if self.logFileSize > MAX_LOG_FILE_SIZE:
                self.CreateLogFile()
                self.logFileSize = 0

        if self.isCreatedFirstPPKFile:
            self.ppkFileSize = os.path.getsize(self.ppkFileName)
            if self.ppkFileSize > MAX_PPK_FILE_SIZE:
                self.CreatePPKFile()
                self.ppkFileSize = 0

    def WriteCSVLogFile(self, data):
        if data['year'] == -1:
            formatted_time = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            with open(self.currentFileName, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([formatted_time, "No Fix..."])
                file.flush()
        else:
            with open(self.currentFileName, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(
                    [data['ITOW'], data['year'], data['month'], data['day'], data['hour'], data['minute'],
                     data['second'], data['millisecond'],
                     data['fixType'], data['numSV'],
                     data['lat'], data['lon'], data['height'], data['hMSL'], data['hAcc'], data['vAcc'],
                     data['velN'], data['velE'], data['velD'],
                     data['gSpeed'], data['sAcc'], data['headOfMot'], data['headAcc'],
                     data['pDOP'], data['headOfVeh'],
                     self.gnssBasedSystemTime.strftime("%H:%M:%S.%f")[:-3], self.rpiSystemTime])
                file.flush()

    def SetSystemDateandTime(self, data):
        # Set GNSS date & time to system date & time
        self.gnssBasedSystemTime = datetime(data['year'], data['month'], data['day'], data['hour'], data['minute'],
                                            data['second'])
        self.gnssBasedSystemTime = self.gnssBasedSystemTime.replace(microsecond=data['millisecond'] * 1000)

        self.rpiSystemTime = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if self.isValidSystemTime == False:
            try:
                dateInput = self.gnssBasedSystemTime.strftime("%m%d%H%M%Y.%S")
                #subprocess.run(["sudo", "date", dateInput])
                print("Date & time updated...")
                self.isValidSystemTime = True
            except Exception as e:
                print("Error:", e)


        # Check UART1 channel, if it's open, close it
        # Check USB channel, output is just UBX ?
        # Check module's freq. if it's not 10Hz, set module working freq. 10Hz
        # Check NAV2-PVT message's enable status, it's not enabled, set enable with 10Hz
        # Check RAWX message's enable status, it's not enabled, set enable with 5Hz
        # Check SFRX message's enable status, it's not enabled, set enable with 5Hz
        # Set airborne 8g ?
        # If there is any change of config, write current configs to flash memory

    def ConfigureGNSS(self):
        max_retries = 3

        try:
            usbFirst = UBXMessage('CFG', 'CFG-PRT', SET, payload=(b'\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x00\x01\x00\x00\x00\x00\x00'))
            self.serialPort.write(usbFirst.serialize())
            self.LogDiagnostic("Initial USB UBX output configuration sent.")
            time.sleep(0.1)
            self.serialPort.write(usbFirst.serialize())
        except Exception as e:
            self.LogDiagnostic(f"Failed to send initial USB config: {e}", "ERROR")

        def _send_and_wait(msg, check_func, timeout=2.0):
            self.serialPort.write(msg.serialize())
            start = time.time()
            while time.time() - start < timeout:
                try:
                    raw, parsed = self.ubxReader.read()
                    if parsed and parsed.identity and check_func(parsed):
                        return True
                except Exception as e:
                    self.LogDiagnostic(f"Error reading UBX message: {e}", "ERROR")
                time.sleep(0.01)
            return False

        def _check_cfg_prt(portID, inUBX, inNMEA, inRTCM3, outUBX, outNMEA, outRTCM3):
            def check(parsed):
                return (parsed.identity == "CFG-PRT" and parsed.portID == portID and
                        parsed.inUBX == inUBX and parsed.inNMEA == inNMEA and parsed.inRTCM3 == inRTCM3 and
                        parsed.outUBX == outUBX and parsed.outNMEA == outNMEA and parsed.outRTCM3 == outRTCM3)
            return check

        def _check_cfg_rate(expected_ms):
            return lambda p: p.identity == "CFG-RATE" and p.measRate == expected_ms

        def _check_cfg_msg(msgID, expected_rate_usb):
            return lambda p: hasattr(p, 'msgID') and p.msgID == msgID and p.rateUSB == expected_rate_usb

        def _check_cfg_nav5(msgID, expected_dynModel):
            def check(parsed):
                return (parsed.identity == "CFG-NAV5" and parsed.dynModel == expected_dynModel)
            return check

        def apply_and_verify(poll_msg, verify_func, set_msg=None, timeout=2.0, name=""):
            if _send_and_wait(poll_msg, verify_func, timeout):
                self.LogDiagnostic(f"{name}: already configured correctly.")
                return True
            if not set_msg:
                self.LogDiagnostic(f"{name}: poll failed and no SET message provided.", "ERROR")
                self.configFailed = True
                return False
            for attempt in range(1, max_retries + 1):
                self.serialPort.write(set_msg.serialize())
                self.configChanged = True
                self.LogDiagnostic(f"{name}: SET sent (attempt {attempt}), verifying...")
                if _send_and_wait(poll_msg, verify_func, timeout):
                    self.LogDiagnostic(f"{name}: configuration verified.")
                    return True
            self.LogDiagnostic(f"{name}: configuration FAILED after {max_retries} retries.", "ERROR")
            self.configFailed = True
            return False
        try:
            # USB CFG-PRT
            usb_poll = UBXMessage('CFG', 'CFG-PRT', POLL, portID=3)
            usb_set = UBXMessage('CFG', 'CFG-PRT', SET, payload=(b'\x03\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x03\x00\x01\x00\x00\x00\x00\x00'))
            apply_and_verify(usb_poll, _check_cfg_prt(3, 1, 1, 0, 1, 0, 0), usb_set, name="USB CFG-PRT")

            # UART1 CFG-PRT
            uart_poll = UBXMessage('CFG', 'CFG-PRT', POLL, portID=1)
            uart_set = UBXMessage('CFG', 'CFG-PRT', SET, payload=(b'\x01\x00\x00\x00\xD0\x08\x00\x00\x00\x96\x00\x00\x03\x00\x00\x00\x00\x00\x00\x00'))
            apply_and_verify(uart_poll, _check_cfg_prt(1, 1, 1, 0, 0, 0, 0), uart_set, name="UART1 CFG-PRT")

            # CFG-RATE
            rate_poll = UBXMessage('CFG', 'CFG-RATE', POLL)
            rate_set = UBXMessage('CFG', 'CFG-RATE', SET, payload=b'\x64\x00\x01\x00\x00\x00')
            apply_and_verify(rate_poll, _check_cfg_rate(100), rate_set, name="CFG-RATE")

            # Airborne 4G Config
            airborne4g_poll = UBXMessage('CFG', 'CFG-NAV5', POLL)
            airborne4g_set = UBXMessage('CFG', 'CFG-NAV5', SET, payload=b'\xFF\xFF\x08\x03\x00\x00\x00\x00\x10\x27\x00\x00\x05\x00\xFA\x00\xFA\x00\x64\x00\x2c\x01\x00\x00\x00\x00\x10\x27\x00\x00\x00\x00\x00\x00\x00\x00')
            apply_and_verify(airborne4g_poll, _check_cfg_nav5(36, 8), airborne4g_set, name="Airborne")

            # RXM-SFRBX MSG
            sfrbx_poll = UBXMessage('CFG', 'CFG-MSG', POLL, payload=b'\x02\x13')
            sfrbx_set = UBXMessage('CFG', 'CFG-MSG', SET, payload=b'\x02\x13\x00\x00\x00\x02\x00\x00')
            apply_and_verify(sfrbx_poll, _check_cfg_msg(19, 2), sfrbx_set, name="SFRBX MSG")

            # RXM-RAWX MSG
            rawx_poll = UBXMessage('CFG', 'CFG-MSG', POLL, payload=b'\x02\x15')
            rawx_set = UBXMessage('CFG', 'CFG-MSG', SET, payload=b'\x02\x15\x00\x00\x00\x02\x00\x00')
            apply_and_verify(rawx_poll, _check_cfg_msg(21, 2), rawx_set, name="RAWX MSG")

            # NAV-PVT MSG
            navpvt_poll = UBXMessage('CFG', 'CFG-MSG', POLL, payload=b'\x01\x07')
            navpvt_set = UBXMessage('CFG', 'CFG-MSG', SET, payload=b'\x01\x07\x00\x00\x00\x01\x00\x00')
            apply_and_verify(navpvt_poll, _check_cfg_msg(7, 1), navpvt_set, name="NAV-PVT MSG")

            # If settings changed, save current configs in flash
            if self.configChanged:
                print("Saving config to flash...")
                save = UBXMessage('CFG', 'CFG-CFG', SET, payload=b'\x00\x00\x00\x00\xff\xff\x00\x00\x00\x00\x00\x00\x17')
                self.serialPort.write(save.serialize())
                print("Saved to flash")

        except Exception as e:
            self.LogDiagnostic(f"Exception during GNSS configuration: {e}", "ERROR")
            self.configFailed = True

        if self.configFailed:
            self.LogDiagnostic("One or more configuration steps FAILED.", "ERROR")
        else:
            self.LogDiagnostic("All GNSS configuration steps completed successfully.")

        return not self.configFailed

    def ReaderThread(self):
        while True:
            try:
                # if there is a data in buffer, it takes and parses
                (rawData, parsedData) = self.ubxReader.read()
                lastDataTime = time.time()
                # program enters the block when find the NAV-Pvt packet
                if parsedData and parsedData.identity == 'NAV-PVT':
                    totalSeconds = parsedData.iTOW // 1000
                    pvtData = {
                        "ITOW": parsedData.iTOW,
                        'year': parsedData.year, 'month': parsedData.month, 'day': parsedData.day,
                        'hour': (totalSeconds // 3600) % 24,
                        'minute': (totalSeconds // 60) % 60,
                        'second': (totalSeconds % 60),
                        'millisecond': (parsedData.iTOW % 1000),
                        'valid': parsedData.validTime, 'fixType': parsedData.fixType, 'numSV': parsedData.numSV,
                        'lon': parsedData.lon, 'lat': parsedData.lat, 'height': parsedData.height / 1000,
                        'hMSL': parsedData.hMSL / 1000,
                        'hAcc': parsedData.hAcc / 1000, 'vAcc': parsedData.vAcc / 1000,
                        'velN': parsedData.velN / 1000, 'velE': parsedData.velE / 1000,
                        'velD': parsedData.velD / 1000,
                        'gSpeed': parsedData.gSpeed / 1000, 'sAcc': parsedData.sAcc / 1000,
                        'headOfMot': parsedData.headMot, 'headAcc': parsedData.headAcc,
                        'pDOP': parsedData.pDOP, 'headOfVeh': parsedData.headVeh,
                    }
                    self.fixType = parsedData.fixType

                    # Set gnssBasedTime struct according to GNSS date and time
                    if parsedData.validTime:
                        self.SetSystemDateandTime(pvtData)

                    # System create log folder when GNSS fix reach minimum 2D fix or higher
                    if not self.isCreatedFirstLogFile and self.fixType >= MinFixGNSSSolution:
                        self.CreateLogFile()

                    # when created first log file so when system reach the 2D fix,
                    # anymore we put the data in the queue for writing to csv
                    if self.fixType >= MinFixGNSSSolution and self.isCreatedFirstLogFile:
                        self.navPvtQueue.put(pvtData)
                        # for debugging
                        # print(f"Lat: {pvtData['lat']},"
                        # f" Lon: {pvtData['lon']}, Height: {pvtData['height']}, VelN: {pvtData['velN']}")
                    elif self.fixType < MinFixGNSSSolution and self.isCreatedFirstLogFile:
                        pvtData = {'year': -1}
                        self.navPvtQueue.put(pvtData)

                    # green led lights while fixType equal 2D or than bigger
                    if IS_RPI:
                        GPIO.output(FixLedPin, GPIO.HIGH if parsedData.fixType >= MinFixGNSSSolution else GPIO.LOW)

                    # Checks current log file size, if it reachs maximum file size, system going to create new log file
                    self.CheckFileSize()

                elif parsedData.identity in['RXM-RAWX', 'RXM-SFRBX']:
                    if self.fixType > MinFixGNSSSolution and not self.isCreatedFirstPPKFile:
                        self.CreatePPKFile()
                    if self.isCreatedFirstPPKFile is True and hasattr(self, 'ppkLogFilePtr'):
                        self.ppkQueue.put(rawData)

            except KeyboardInterrupt:
                self.LogDiagnostic("KeyboardInterrupt received. Exiting program...", "WARN")
                self.navPvtQueue.put(None)
                self.ppkQueue.put(None)
                break

            except Exception as e:
                self.LogDiagnostic(f"Exception in ReaderThread: {e}", "ERROR")
                self.navPvtQueue.put(None)
                self.ppkQueue.put(None)
                self.StartFailedMode()
                break

            if time.time() - lastDataTime > SerialTimeoutSec:
                self.LogDiagnostic("No GNSS data received in over 60 seconds. Entering FAILED MODE.", "ERROR")
                self.StartFailedMode()


    def WriterThread(self):
            while True:
                if not self.navPvtQueue.empty():
                    pvtData = self.navPvtQueue.get()
                    if pvtData is None:
                        break
                    self.WriteCSVLogFile(pvtData)

                if not self.ppkQueue.empty():
                    rawData = self.ppkQueue.get()
                    if self.ppkLogFilePtr:  # file is open ?
                        self.ppkLogFilePtr.write(rawData)
                        self.ppkLogFilePtr.flush()
    def Run(self):
        success = self.ConfigureGNSS()

        if not success:
            self.StartFailedMode()
            return

        writer = threading.Thread(target=self.WriterThread)
        reader = threading.Thread(target=self.ReaderThread)

        writer.start()
        reader.start()

        reader.join()
        writer.join()
def StartFTPServer():
    authorizer = DummyAuthorizer()
    authorizer.add_user("aselsan", "1975", LOG_FOLDER_PATH, perm="elradfmw")
    handler = FTPHandler
    handler.passive_ports = range(30000, 30010)
    handler.masquerade_address = "192.168.1.28"  
    handler.authorizer = authorizer
    server = FTPServer(("0.0.0.0", 2121), handler)
    print("FTP server started on port 2121...")
    server.serve_forever()

if __name__ == "__main__":
    t1 = threading.Thread(target=lambda: Logger().Run())
    t2 = threading.Thread(target=StartFTPServer)

    t1.start()
    t2.start()

    t1.join()
    t2.join()

# TODO: FTP server name & password must be changed (aselsan 1975)
# FTP server ip must be changed with 192.168.1.28(static wbox's ip)
