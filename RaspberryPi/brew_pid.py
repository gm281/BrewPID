import threading
import time
import serial
import datetime
import os
import errno
import sys
import socket

SERVER_PORT=12400
DEVICE = '/dev/ttyACM0' # the arduino serial interface (use dmesg when connecting)
BAUD = 9600
NR_SENSORS = 6
SENSOR_LOG_DIRECTORY = os.getenv("HOME") + '/SensorLogs/'
BATHROOM_MAX_HUMIDITY = 85
BATHROOM_HYSTERESIS = 1

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

class Place:
    def __init__(self, name, sensor_id, relay_id):
        self.name = name
        self.sensor_id = sensor_id
        self.relay_id = relay_id

bathroom = Place("Bathroom", 3, 2)
outside = Place("Outside", 1, -1)
hall = Place("Hall", 5, 0)
jasiu = Place("Jasiu", 2, 5)
study = Place("Study", 0, 4)
bedroom = Place("Bedroom", 4, 3)
places = [bathroom, outside, hall, jasiu, study, bedroom]
place_names_to_place = {}
for place in places:
    place_names_to_place[place.name] = place

def same_hour(datetime1, datetime2):
    return (datetime1.year == datetime2.year) and (datetime1.month == datetime2.month) and (datetime1.day == datetime2.day) and (datetime1.hour == datetime2.hour)

class SensorReading:
    def __init__(self, date, humidity, temp):
        self.date = date
        self.humidity = humidity
        self.temperature = temp

    def __repr__(self):
        return "{}, {}, {}".format(self.date, self.humidity, self.temperature)

class Sensor:
    def __init__(self, place):
        self.place = place
        self.sensor_id = place.sensor_id
        self.current_hour = []
        self.complete_hours = []
        self.last_reading = None
        self.lock = threading.Lock()

    def consume_complete_hours(self):
        self.lock.acquire()
        complete_hours = self.complete_hours
        self.complete_hours = []
        self.lock.release()
        return complete_hours

    def flush_hour(self):
        self.lock.acquire()
        self.complete_hours.append(self.current_hour)
        self.lock.release()
        self.current_hour = []

    def get_last_reading(self):
        self.lock.acquire()
        last_reading = self.last_reading
        self.lock.release()
        return last_reading

    def process_reading(self, humidity, temp):
        sensor_reading = SensorReading(datetime.datetime.now(), humidity, temp)
        if self.last_reading != None and not same_hour(self.last_reading.date, sensor_reading.date):
            self.flush_hour()
        self.lock.acquire()
        self.current_hour.append(sensor_reading)
        self.last_reading = sensor_reading
        self.lock.release()

    def request_reading(self):
        writeCommand("s{0}$".format(self.sensor_id))

class Relay:
    def __init__(self, place):
        self.place = place
        self.relay_id = place.relay_id

    def switch(self, on):
        writeCommand("r{0},{1}$".format(self.relay_id, int(on)))

id_to_sensor = {}
for place in places:
    if place.sensor_id >= 0:
        sensor = Sensor(place)
        id_to_sensor[place.sensor_id] = sensor
        place.sensor = sensor
    if place.relay_id >= 0:
        place.relay = Relay(place)


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

    def process_sensor_reading(self, sensor_id, humidity, temperature):
        global is_on
        sensor = id_to_sensor[sensor_id]
        if sensor is None:
            print('Warning: unknown sensor_id {0}, reading {}, {}'.format(sensor_id, humidity, temperature))
            return
        sensor.process_reading(humidity, temperature)

    def loop(self):
        # FOR TEST: read(1) guarantees no buffering
        self.line += serial_read_fd.read(1)
        while '\n' in self.line:
            headTail = self.line.split('\n', 1)
            line = headTail[0]
            if line.startswith("s,"):
                tokens = line.split(',')
                if tokens[2] == "OK":
                    self.process_sensor_reading(int(tokens[1]), float(tokens[3]), float(tokens[4]))
            else:
                print("Unknown message: {0}".format(line))
            self.line = headTail[1]

class SamplingThread(StoppableThread):
    def __init__(self):
        super(SamplingThread, self).__init__()

    def loop(self):
        time.sleep(3)
        for place in places:
            if hasattr(place, "sensor"):
                place.sensor.request_reading()
                time.sleep(0.1)

class WriteoutThread(StoppableThread):
    def __init__(self):
        super(WriteoutThread, self).__init__()

    def writeout_hour(self, place, hour):
        if len(hour) == 0:
            return
        date = hour[0].date
        hour_dir = "{0}/{1}/{2:0>4}/{3:0>2}/{4:0>2}/{5:0>2}".format(SENSOR_LOG_DIRECTORY, place.name, date.year, date.month, date.day, date.hour)
        mkdir_p(hour_dir)
        out_file = open(hour_dir+"/sensor.log", "w")
        for reading in hour:
            out_file.write("{}\n".format(str(reading)))
        out_file.close()

    def loop(self):
        time.sleep(1)
        for place in places:
            if hasattr(place, "sensor"):
                sensor = place.sensor
                complete_hours = sensor.consume_complete_hours()
                for complete_hour in complete_hours:
                    self.writeout_hour(sensor.place, complete_hour)

class VentilationThread(StoppableThread):
    def __init__(self):
        super(VentilationThread, self).__init__()
        self.is_on = False
        self.bathroom_sensor = bathroom.sensor
        self.bathroom_relay = bathroom.relay
        self.bathroom_relay.switch(self.is_on)
        self.bedroom_relay = bedroom.relay
        self.last_time = datetime.datetime.now()

    def want_bedroom_on(self, time):
        minutes = time.minute
        return (minutes >= 5) and (minutes < 10)

    def loop(self):
        time.sleep(1)
        last_reading = self.bathroom_sensor.last_reading
        if last_reading != None:
            want_on = last_reading.humidity > BATHROOM_MAX_HUMIDITY + BATHROOM_HYSTERESIS
            want_off = last_reading.humidity <= BATHROOM_MAX_HUMIDITY - BATHROOM_HYSTERESIS
            if (want_on and not self.is_on) or (want_off and self.is_on):
                switching_on = want_on
                print("Date: {0}, switching on: {1}, due to reading: {2}".format(datetime.datetime.now(), int(switching_on), str(last_reading)))
                self.bathroom_relay.switch(switching_on)
                self.is_on = switching_on
        current_time = datetime.datetime.now()
        want_bedroom_on = self.want_bedroom_on(current_time)
        wanted_bedroom_on = self.want_bedroom_on(self.last_time)
        if want_bedroom_on and not wanted_bedroom_on:
            print("Switching bedroom on");
            self.bedroom_relay.switch(True)
        if not want_bedroom_on and wanted_bedroom_on:
            print("Switching bedroom off");
            self.bedroom_relay.switch(False)
        self.last_time = current_time

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
ventilationThread = VentilationThread()
ventilationThread.start()
serverThread = ServerThread()
serverThread.start()
threads = [readingThread, samplingThread, writeoutThread, ventilationThread, serverThread]
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
