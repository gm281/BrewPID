import threading
import time
import serial
import datetime
import os
import errno
import sys
import socket
from bottle import run, post, request, response, get, route, static_file
import json

SERVER_PORT=12400
DEVICE = '/dev/cu.usbmodem1411' # the arduino serial interface (use dmesg when connecting)
BAUD = 9600
LOG_DIRECTORY = os.getenv("HOME") + '/TemperatureLogs/'
SCRIPT_DIRECTORY = os.path.dirname(os.path.realpath(__file__))


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
    return (datetime1.year == datetime2.year) and (datetime1.month == datetime2.month) and (datetime1.day == datetime2.day) and (datetime1.hour == datetime2.hour)

def smaller_hour(datetime1, datetime2):
    if datetime1.year < datetime2.year:
        return True
    if datetime1.year > datetime2.year:
        return False
    if datetime1.month < datetime2.month:
        return True
    if datetime1.month > datetime2.month:
        return False
    if datetime1.day < datetime2.day:
        return True
    if datetime1.day > datetime2.day:
        return False
    if datetime1.hour < datetime2.hour:
        return True
    return False

last_temperature_change_time = datetime.datetime.now()
temparature_change_accelerator_counter = 0
def temperature_delta():
    global last_temperature_change_time
    global temparature_change_accelerator_counter

    small_delta = 0.1
    accelerated_delta = 1.0

    time_now = datetime.datetime.now()
    time_delta = (time_now - last_temperature_change_time).total_seconds()
    last_temperature_change_time = time_now

    if time_delta < 5:
        temparature_change_accelerator_counter = temparature_change_accelerator_counter + 1
    else:
        temparature_change_accelerator_counter = 0

    if temparature_change_accelerator_counter > 2:
        return accelerated_delta
    else:
        return small_delta

class Loggable:
    def __init__(self, name):
        self.name = name
        self.current_hour = []
        self.complete_hours = []
        self.consumed_hours = []
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
        self.consumed_hours.extend(complete_hours)
        self.complete_hours = []
        self.lock.release()
        return complete_hours

    def toJSONRepresentable(self, cutoff_date):
        if cutoff_date == None:
            cutoff_date = datetime.datetime.min
        out = []
        self.lock.acquire()
        for hour in self.consumed_hours:
            for i in hour:
                if smaller_hour(i.date, cutoff_date):
                    break
                if cutoff_date < i.date:
                    out.append(i.toJSONRepresentable())
        for hour in self.complete_hours:
            for i in hour:
                if smaller_hour(i.date, cutoff_date):
                    break
                if cutoff_date < i.date:
                    out.append(i.toJSONRepresentable())
        for i in self.current_hour:
            if cutoff_date < i.date:
                out.append(i.toJSONRepresentable())
        self.lock.release()
        return out


class TemperatureReading:
    def __init__(self, date, temp, target_temp):
        self.date = date
        self.temperature = temp
        self.target_temperature = target_temp

    def __repr__(self):
        return "{}, {}, {}".format(self.date, self.temperature, self.target_temperature)

    def toJSONRepresentable(self):
        return { "date": self.date.isoformat(), "temp": self.temperature, "target_temp": self.target_temperature }

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

    def set_target_temperature(self, target_temperature):
        new_temp=int(target_temperature * 100)
        writeCommand("s{}$".format(new_temp))

    def get_target_temperature(self):
        self.lock.acquire()
        if self.last_reading != None:
            temp = self.last_reading.target_temperature
        else:
            temp = -1.0
        self.lock.release()
        return temp

class HeaterCoolerSwitch:
    def __init__(self, date, on):
        self.date = date
        self.on = on

    def __repr__(self):
        return "{}, {}".format(self.date, self.on)

    def toJSONRepresentable(self):
        return { "date": self.date.isoformat(), "on": self.on }

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

def str2bool(v):
    return v.strip().lower() in ("yes", "true", "t", "1")

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
                    heater_pid.process_reading(str2bool(tokens[2]))
                elif tokens[1] == "c":
                    cooler_pid.process_reading(str2bool(tokens[2]))
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
            complete_hours = loggable.consume_complete_hours(date)
            for complete_hour in complete_hours:
                name = loggable.get_name()
                self.writeout_hour(name, complete_hour)

readingThread = ReadingThread()
readingThread.start()
samplingThread = SamplingThread()
samplingThread.start()
writeoutThread = WriteoutThread()
writeoutThread.start()
threads = [readingThread, samplingThread, writeoutThread]

def parse_date(in_str):
    if in_str == "none":
        return None
    # Ignore sub-second portion 
    date=in_str.split('.')[0]
    return datetime.datetime.strptime(date, '%Y-%m-%dT%H:%M:%S')

@route('/data', method = 'GET')
def process():
    last_temp_date = parse_date(request.query['last_temp_date'])
    last_heater_date = parse_date(request.query['last_heater_date'])
    last_cooler_date = parse_date(request.query['last_cooler_date'])
    temps = temperature_sensor.toJSONRepresentable(last_temp_date)
    heats = heater_pid.toJSONRepresentable(last_heater_date)
    cools = cooler_pid.toJSONRepresentable(last_cooler_date)
    values = {"temperature": temps, "heater": heats, "cooler": cools }
    return json.dumps(values)

@route('/temp_adj/<direction>', method = 'GET')
def temp_adj(direction):
    current_target_temperature = temperature_sensor.get_target_temperature()
    if current_target_temperature < 0:
        return 'unknown_target_temperature'
    if direction == 'up':
        current_target_temperature = current_target_temperature + temperature_delta()
    elif direction == 'down':
        current_target_temperature = current_target_temperature - temperature_delta()
    else:
        return 'Unknown temp adj direction: {}'.format(direction)
    temperature_sensor.set_target_temperature(current_target_temperature)
    return ''

@route('/', method = 'GET')
def process():
    return static_file("index.html", root=SCRIPT_DIRECTORY)

@route('/js/<path:path>')
def callback(path):
    return static_file(path, root="{}/{}".format(SCRIPT_DIRECTORY, "node_modules/dygraphs/dist/"))

@route('/jquery.js')
def jq():
    return static_file('jquery.min.js', root=SCRIPT_DIRECTORY)


try:
    run(host='0.0.0.0', port=SERVER_PORT, debug=True)
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
