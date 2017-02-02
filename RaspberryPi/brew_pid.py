import threading
import time
import serial
import datetime
import os
import errno
import sys
import socket

SERVER_PORT=12400
DEVICE = '/dev/cu.usbmodem1411' # the arduino serial interface (use dmesg when connecting)
BAUD = 9600
NR_SENSORS = 6
LOG_DIRECTORY = os.getenv("HOME") + '/TemperatureLogs/'

# In order to use debug mode, set the below variable to True, then in the shell:
# mkfifo /tmp/cmd_fifo
# mkfifo /tmp/rsp_fifo
# cat /tmp/rsp_fifo
# cat > /tmp/cmd_fifo
# nc localhost SERVER_PORT
# to write a sensor reading, paste e.g. this to cat > /tmp/cmd_fifo:
# s,1,OK,10,20
# Other minor changes made to the code are all marked with 'FOR TEST' label (and may have to be removed for production)
DEBUG_MODE=False
if DEBUG_MODE:
    serial_read_fd = open("/tmp/cmd_fifo", "r")
    serial_write_fd = open("/tmp/rsp_fifo", "w")
else:
    serial = serial.Serial(port=DEVICE, baudrate=BAUD, timeout=1.0)
    serial_read_fd = serial
    serial_write_fd = serial

print("Starting at: {0}".format(datetime.datetime.now()))

def writeCommand(command):
    global serialWriteLock
    serialWriteLock = threading.Lock()
    serialWriteLock.acquire()
    serial_write_fd.write(command)
    # flush is FOR TEST
    serial_write_fd.flush()
    serialWriteLock.release()

def mkdir_p(path):
    try:
        os.makedirs(path)
    except OSError as exc: # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(path):
            pass
        else: raise

def same_hour(datetime1, datetime2):
    return (datetime1.year == datetime2.year) and (datetime1.month == datetime2.month) and (datetime1.day == datetime2.day) and (datetime1.hour == datetime2.hour) and (datetime1.minute == datetime2.minute)

class Loggable:
    def __init__(self, name):
        self.name = name
        self.current_hour = []
        self.complete_hours = []
        self.last_reading = None
        self.lock = threading.Lock()

    def get_name(self):
        return self.name

    def locked_flush_hour(self, date):
        if self.last_reading != None and not same_hour(self.last_reading.date, date):
            self.complete_hours.append(self.current_hour)
            self.current_hour = []

    def consume_complete_hours(self, date):
        self.lock.acquire()
        self.locked_flush_hour(date)
        complete_hours = self.complete_hours
        self.complete_hours = []
        self.lock.release()
        return complete_hours


class TemperatureReading:
    def __init__(self, date, temp, target_temp):
        self.date = date
        self.temperature = temp
        self.target_temperature = target_temp

    def __repr__(self):
        return "{}, {}, {}".format(self.date, self.temperature, self.target_temperature)

class TemperatureSensor(Loggable):
    def __init__(self):
        Loggable.__init__(self, "temperature")

    def process_reading(self, temp, target_temp):
        sensor_reading = TemperatureReading(datetime.datetime.now(), temp, target_temp)
        self.lock.acquire()
        self.locked_flush_hour(sensor_reading.date)
        self.current_hour.append(sensor_reading)
        self.last_reading = sensor_reading
        self.lock.release()

    def request_reading(self):
        writeCommand("t$")

class HeaterCoolerSwitch:
    def __init__(self, date, on):
        self.date = date
        self.on = on

    def __repr__(self):
        return "{}, {}".format(self.date, self.on)

class PID(Loggable):
    def __init__(self, heater):
        Loggable.__init__(self, heater)

    def process_reading(self, on):
        reading = HeaterCoolerSwitch(datetime.datetime.now(), on)
        self.lock.acquire()
        self.locked_flush_hour(reading.date)
        self.current_hour.append(reading)
        self.last_reading = reading
        self.lock.release()


temperature_sensor = TemperatureSensor()
heater_pid = PID("heater")
cooler_pid = PID("cooler")

class StoppableThread(threading.Thread):
    def __init__(self):
        super(StoppableThread, self).__init__()
        self.keepRunning = True

    def loop(self):
        pass

    def run(self):
        while self.keepRunning:
            self.loop()

    def stop(self):
        self.keepRunning = False

class ReadingThread(StoppableThread):
    def __init__(self):
        super(ReadingThread, self).__init__()
        self.line = ""

    def loop(self):
        # FOR TEST: read(1) guarantees no buffering
        self.line += serial_read_fd.read(1)
        while '\n' in self.line:
            headTail = self.line.split('\n', 1)
            line = headTail[0]
            if line.startswith("t,"):
                tokens = line.split(',')
                temperature_sensor.process_reading(float(tokens[1]), float(tokens[2]))
            elif line.startswith("r,"):
                tokens = line.split(',')
                if tokens[1] == "h":
                    heater_pid.process_reading(tokens[2])
                elif tokens[1] == "c":
                    cooler_pid.process_reading(tokens[2])
                else:
                    print("Unknown relay type: {0}".format(tokens[1]))
            else:
                print("Unknown message: {0}".format(line))
            self.line = headTail[1]

class SamplingThread(StoppableThread):
    def __init__(self):
        super(SamplingThread, self).__init__()

    def loop(self):
        time.sleep(3)
        temperature_sensor.request_reading()


class WriteoutThread(StoppableThread):
    def __init__(self):
        super(WriteoutThread, self).__init__()

    def writeout_hour(self, name, hour):
        if len(hour) == 0:
            return
        date = hour[0].date
        hour_dir = "{0}/{1}/{2:0>4}/{3:0>2}/{4:0>2}/{5:0>2}".format(LOG_DIRECTORY, name, date.year, date.month, date.day, date.hour)
        mkdir_p(hour_dir)
        out_file = open(hour_dir+"/log", "w")
        for reading in hour:
            out_file.write("{}\n".format(str(reading)))
        out_file.close()

    def loop(self):
        time.sleep(1)

        date = datetime.datetime.now()
        loggables = [temperature_sensor, heater_pid, cooler_pid]
        for loggable in loggables:
            print(loggable)
            complete_hours = loggable.consume_complete_hours(date)
            for complete_hour in complete_hours:
                name = loggable.get_name()
                self.writeout_hour(name, complete_hour)


class ServerThread(StoppableThread):
    def __init__(self):
        super(ServerThread, self).__init__()
        self.socket = socket.socket()
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(('', SERVER_PORT))
        self.partial_message = ''
        self.client = None

    def process_message(self, message):
        message = message.lstrip('\n')
        if message.startswith('sensor'):
            place_name = message[7:]
            print("Requested sensor reading for: {}".format(place_name))
            if place_names_to_place.has_key(place_name):
                place = place_names_to_place[place_name]
                if hasattr(place, "sensor"):
                    sensor_last_reading = place.sensor.get_last_reading()
                    if sensor_last_reading == None:
                        self.client.send("error,no_sensor_readings,{}$".format(place_name))
                    else:
                        self.client.send("sensor_reading,{},{},{},{}$".format(place_name, sensor_last_reading.date, sensor_last_reading.humidity, sensor_last_reading.temperature))
            return True
        elif message.startswith('relay'):
            payload = message[6:]
            print("Requested relay switch for: {}".format(payload))
            tokens = payload.split(',')
            place_name = tokens[0]
            want_on = int(tokens[1])
            if place_names_to_place.has_key(place_name):
                place = place_names_to_place[place_name]
                if hasattr(place, "relay"):
                    place.relay.switch(want_on)
                else:
                    print("Requested relay switch for place without relay: {}".format(place_name))
            else:
                print("Couldn't find place: {}".format(place_name))
            return True
        print("Requested unknown command: {}".format(message))
        return False

    def process_input(self, message):
        self.partial_message += message
        while '$' in self.partial_message:
            headTail = self.partial_message.split('$', 1)
            one_message = headTail[0]
            handled = self.process_message(one_message)
            if not handled:
                self.client.send("error,unknown_command,{}$".format(one_message))
            self.partial_message = headTail[1]


    def loop(self):
        self.socket.listen(1)
        c, addr = self.socket.accept()
        print('Got connection from {}'.format(addr))
        self.client = c
        while True:
            msg = c.recv(1024)
            if msg == "":
                print('Connection from {} closed'.format(addr))
                break
            self.process_input(msg)

readingThread = ReadingThread()
readingThread.start()
samplingThread = SamplingThread()
samplingThread.start()
writeoutThread = WriteoutThread()
writeoutThread.start()
threads = [readingThread, samplingThread, writeoutThread]
try:
    while True:
        time.sleep(1)
        sys.stdout.flush()
except:
    print 'Interrupt'
    os._exit(0)
    # The below attempts to exit nicely, but doesn't work for blocking calls :(
    for thread in threads:
        thread.stop()
        while thread.isAlive():
            thread.join(0.5)
finally:
    print 'Exiting'
    os._exit(0)
