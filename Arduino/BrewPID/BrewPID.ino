#include <OneWire.h>
#include <DallasTemperature.h>

/***********************************************************************************************/
/* CONFIG */
/***********************************************************************************************/

#define BAUD_RATE   (9600UL)
#define ONE_WIRE_BUS A3
#define TEMPERATURE_SAMPLING_PERIOD_MS     1000UL
#define TEMPERATURE_HISTORY_SIZE           10
#define HEATING_COOLING_ADJUSTMENT_PERIOD_MS  (15 * 1000UL) // 10mins
#define DEFAULT_TARGET_TEMPERATURE         21.0
#define HEATER_RELAY_ID                    0
#define COOLER_RELAY_ID                    1
/***********************************************************************************************/
/* BASIC UTILS */
/***********************************************************************************************/
#define CONCAT_TOKENS( TokenA, TokenB )       TokenA ## TokenB
#define EXPAND_THEN_CONCAT( TokenA, TokenB )  CONCAT_TOKENS( TokenA, TokenB )
#define ASSERT( Expression )                  enum{ EXPAND_THEN_CONCAT( ASSERT_line_, __LINE__ ) = 1 / !!( Expression ) }
#define ASSERTM( Expression, Message )        enum{ EXPAND_THEN_CONCAT( Message ## _ASSERT_line_, __LINE__ ) = 1 / !!( Expression ) }
#define assert(x)                             if (!(x)) { Serial.print("Failure in line: "); Serial.print(__LINE__); Serial.print(", failing condition: "); Serial.println(#x);}
#define NOW     micros()
#define VALID_DELAY(_d) ((_d) > 0 && (_d) < ((-1UL) >> 2))
#define ABS(x)  (x<0 ? -x : x)
void delay_microseconds(unsigned long /* timestamp_t */ delay_us)
{
    if (delay_us < 16000UL)
        delayMicroseconds(delay_us);
    else
        delay(delay_us / 1000UL);
}
#define debug(_p) Serial.print(_p)
#define debugll(_p) Serial.print(((unsigned long)_p))
#define debugln(_p) Serial.println(_p)
#define debugllln(_p) Serial.println(((unsigned long)_p))

int freeRam () 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/***********************************************************************************************/
/* TYPES */
/***********************************************************************************************/
typedef unsigned long long timestamp_t;
typedef double temperature_t;

class TemperatureSensor
{
public:
    TemperatureSensor() = default;
    virtual ~TemperatureSensor() = default;

    virtual temperature_t readTemperature() const = 0;
};

class DallasTemperatureSensor final : TemperatureSensor
{
private:
    // TODO does this free() those on object deallocation?
    // What's best, storting by pointer?
    OneWire &oneWire;
    DallasTemperature &rawSensors;

public:
    DallasTemperatureSensor(int bus)
    {
        oneWire = OneWire(bus);
        rawSensors = DallasTemperature(&oneWire);
        rawSensors.begin();
    }

    temperature_t readTemperature() const override final
    {
        // Send the command to get temperatures
        rawSensors.requestTemperatures();
        // You can have more than one IC on the same bus. 0 refers to the first IC on the wire
        return rawSensors.getTempCByIndex(0);
    }
};

struct TimeSeriesValue {
    timestamp_t time;
    temperature_t value;
};

class Vector {
private:
    struct TimeSeriesValue *values;
    unsigned long firstIdx;
    unsigned long lastIdx;
    unsigned long maxSize;

// Example, Vector with 1 entry
// maxSize=2
// firstIdx=0
// lastIdx=1
// size=1
public:
    Vector(int maxSize)
    {
        this->maxSize = maxSize;
        firstIdx = 0;
        lastIdx = 0;
        values = malloc(sizeof(struct TimeSeriesValue) * maxSize);
    }

    int size() {
        return (int)(lastIdx-firstIdx);
    }


    void push_back(TimeSeriesValue value)
    {
        assert(lastIdx < firstIdx + maxSize);
        values[lastIdx % maxSize] = value;
        lastIdx++;
    }

    TimeSeriesValue drop_first()
    {
        assert(lastIdx > firstIdx);
        return values[(firstIdx++) % maxSize];
    }

    TimeSeriesValue getFromEnd(int index)
    {
        if (index >= size())
        {
            return values[(lastIdx-1) % maxSize];
        }
        return values[(lastIdx-1-index) % maxSize];
    }

    TimeSeriesValue getLast()
    {
        assert(lastIdx > firstIdx);
        return getFromEnd(0);
    }
};

class TimeSeries {
private:
    int maxSize;
    Vector history;
    timestamp_t integralDuration;
    timestamp_t lastIntegralTimestamp;
    long long accumulatedIntegral;

    long long integralDelta(timestamp_t timestamp, temperature_t value, timestamp_t lastTimestamp) {
        assert(timestamp >= lastTimestamp);
        double t = (double)(timestamp - lastIntegralTimestamp);

        return (long long)(value * t);

    }

    void accountIntegral(timestamp_t timestamp, temperature_t value) {
       if (lastIntegralTimestamp == 0) {
            lastIntegralTimestamp = timestamp;
        }
        accumulatedIntegral += integralDelta(timestamp, value, lastIntegralTimestamp);
        integralDuration += timestamp - lastIntegralTimestamp;
        debug("Integrating: ");
        debugll(timestamp);
        debug(", ");
        debug(value);
        debug(". Accumulated: t=");
        debugll(integralDuration);
        debug(", i=");
        debugll(accumulatedIntegral);
        debug(", lt=");
        debugllln(lastIntegralTimestamp);
        lastIntegralTimestamp = timestamp;
    }


public:
    void resetIntegral()
    {
        integralDuration = 0;
        lastIntegralTimestamp = 0;
        accumulatedIntegral = 0;
    }

    TimeSeries(int maxSize) : history(maxSize)
    {
        assert(maxSize > 2);
        assert(maxSize < 200);
        this->maxSize = maxSize;
        resetIntegral();
    }

    void storeValue(timestamp_t time, temperature_t value)
    {
        debug("Storing: ");
        debugll(time);
        debug(", ");
        debugln(value);

        if (history.size() > 0)
        {
            assert(history.getLast().time <= time);
        }
        accountIntegral(time, value);
        while(history.size() >= maxSize)
        {
            history.drop_first();
        }
        history.push_back({time, value});
    }

    temperature_t getLatestSmoothed() {
        // This ignores the timestaps, i.e. assumes periodic samples
        double alpha = 0.8;
        int i=19;
        double accumulator = history.getFromEnd(i).value;
        for (i--; i>=0; i--)
        {
            accumulator = accumulator * (1.0 - alpha) + history.getFromEnd(i).value * alpha;
        }
        return accumulator;
    }

    double integral(temperature_t expectedValue) {
        assert(history.size() > 0);
        long long relativeIntegral = (long long)(expectedValue * (double)integralDuration) - accumulatedIntegral;
        // Converting units: degreeC x microS -> decgreeC x S
        relativeIntegral = relativeIntegral / 1000;
        double integral = (double)relativeIntegral / 1000.0;
        debug("Returnining integral for t: ");
        debug(expectedValue);
        debug(", t: ");
        debugll(integralDuration);
        debug(", i: ");
        debugll(accumulatedIntegral);
        debug(" -> ");
        debugln(integral);
        return integral;
    }
};

class PID {
private:
    double kp;
    double ki;
    double kd;
    timestamp_t period;
    timestamp_t onTimestamp, offTimestamp;
    long relayId;
    timestamp_t lastScheduleTimestamp;
    temperature_t lastScheduleTemperature;

    double getDerivative(temperature_t currentTemperature, timestamp_t currentTime) {
        if (lastScheduleTimestamp == 0) {
            return 0;
        }
        double valueDelta = currentTemperature - lastScheduleTemperature;
        unsigned long long timeDelta =  (currentTime - lastScheduleTimestamp) / 1000UL;
        return valueDelta / timeDelta;
    }

public:
    PID(double kp, double ki, double kd, long relayId, timestamp_t period = HEATING_COOLING_ADJUSTMENT_PERIOD_MS)
    {
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        this->period = period * 1000ULL;
        this->onTimestamp = 0;
        this->offTimestamp = 0;
        this->relayId = relayId;
        this->lastScheduleTimestamp = 0;
        this->lastScheduleTemperature = 0;
    }

    double output(temperature_t targetTemperature, TimeSeries *timeseries)
    {
        timestamp_t timestamp = NOW;
        temperature_t currentTemperature = timeseries->getLatestSmoothed();
        double output = kp * (targetTemperature - currentTemperature);
        output += ki * timeseries->integral(targetTemperature);
        output += kd * getDerivative(currentTemperature, timestamp);
        lastScheduleTimestamp = timestamp;
        lastScheduleTemperature = timeseries->getLatestSmoothed();

        return output;
    }

    void schedule(double output, timestamp_t timestamp = NOW)
    {
        // Clip to 1.0
        output = max(output, 0.0);
        output = min(output, 1.0);

        timestamp_t duration = (timestamp_t)((double)period * output);
        onTimestamp = timestamp;
        offTimestamp = timestamp + duration;
        debug("Scheduled on: ");
        debugll(onTimestamp);
        debug(", off: ");
        debugllln(offTimestamp);
    }

    timestamp_t getOnTimestamp() {
        return onTimestamp;
    }

    timestamp_t getOffTimestamp() {
        return offTimestamp;
    }

    bool shouldBeOn(timestamp_t timestamp = NOW) {
        assert(onTimestamp <= offTimestamp);
        if (timestamp < onTimestamp) {
            return false;
        }
        if (timestamp < offTimestamp) {
            return true;
        }
        return false;
    }

    long getRelayId() {
        return relayId;
    }
};


/***********************************************************************************************/
/* STATS */
/***********************************************************************************************/

struct stat_accumulator {
    long min;
    long max;
    long nr_samples;
    long sum_samples;
    long sum_square_samples;
};

void stat_accumulator_init(struct stat_accumulator *accumulator)
{
    memset(accumulator, 0, sizeof(struct stat_accumulator));
    accumulator->min = 0x7FFFFFFFUL;
    accumulator->max = 0x80000000UL;
}

void stat_accumulator_sample(struct stat_accumulator *accumulator, long sample)
{
    accumulator->min = min(accumulator->min, sample);
    accumulator->max = max(accumulator->max, sample);
    accumulator->nr_samples++;
    accumulator->sum_samples += sample;
    accumulator->sum_square_samples += sample * sample;
}

void stat_accumulator_print(struct stat_accumulator *accumulator)
{
    long average, average_squares;
    double standard_deviation;

    if (accumulator->nr_samples == 0) {
        Serial.println("#0");
        return;
    }
    average = accumulator->sum_samples / accumulator->nr_samples;
    average_squares = accumulator->sum_square_samples / accumulator->nr_samples;
    standard_deviation = sqrt(average_squares - average * average);

    Serial.print("[");
    Serial.print(accumulator->min);
    Serial.print(",");
    Serial.print(average);
    Serial.print(",");
    Serial.print(accumulator->max);
    Serial.print("] #");
    Serial.print(accumulator->nr_samples);
    Serial.print(", sd: ");
    Serial.print(standard_deviation);
    Serial.println("");
}

/***********************************************************************************************/
/* COMMAND QUEUE */
/***********************************************************************************************/

typedef unsigned char command_type_t;
typedef unsigned long command_data_t;
struct command {
    timestamp_t timestamp;
    command_type_t type;
    command_data_t data;
};
typedef struct command command_t;
typedef unsigned int command_id_t;

/* Use 1KB for the command structures, half mem available. */
#define COMMANDS_QUEUE_SIZE (512 / sizeof(command_t))
command_t commands[COMMANDS_QUEUE_SIZE];
command_id_t commands_used = 0;

/* Make sure command type is wide enough to store all commands.
   On top of that we need space for calculations (which often
   involve multiplying by 2). */
ASSERT(1UL << (8 * sizeof(command_id_t)) > 2UL * COMMANDS_QUEUE_SIZE);

#define swap_commands(_i, _j) do {                                 \
    command_t _tmp;                                                  \
    memcpy(&_tmp, &commands[(_i)], sizeof(command_t));               \
    memcpy(&commands[(_i)], &commands[(_j)], sizeof(command_t));     \
    memcpy(&commands[(_j)], &_tmp, sizeof(command_t));               \
} while(0)

/* Call if the last element in the heap needs placing in the right position. */
void heapify_up() {
    command_id_t child_id, parent_id;
    //Serial.print("Heapifying with # commands: ");
    //Serial.println(commands_used);
    /* Corner case, when there are no commands. */
    if (commands_used == 0)
        return;
    child_id = commands_used - 1;
    //Serial.print("Child id: ");
    //Serial.println(child_id);
    while (child_id != 0) {
        parent_id = (child_id - 1)  / 2;
        //Serial.print("Parent id: ");
        //Serial.print(parent_id);
        //Serial.print(" with timestamps: parent: ");
        //Serial.print(commands[parent_id].timestamp);
        //Serial.print(", child: ");
        //Serial.println(commands[child_id].timestamp);
        /* If parent's timestamp is smaller/equal, the perculation must stop. */
        if (commands[parent_id].timestamp <= commands[child_id].timestamp)
            return;
        //Serial.println("Swapping.");
        /* Othewise, swap and continue. */
        swap_commands(parent_id, child_id);
        child_id = parent_id;
    }
}


/* Call if the first element in the heap needs placing in the right position. */
void heapify_down() {
    command_id_t parent_id, smallest_id, child_id;

    /* Corner case, when there are no commands. */
    if (commands_used == 0)
        return;

    /* Start off with the root. */
    smallest_id = 0;
    do {
        /* Parent id is whatever item was used to swap at the end of last iteration. */
        parent_id = smallest_id;

        child_id = 2 * parent_id + 1;
        if (child_id < commands_used &&
                commands[child_id].timestamp < commands[smallest_id].timestamp)
            smallest_id = child_id;

        child_id = 2 * parent_id + 2;
        if (child_id < commands_used &&
                commands[child_id].timestamp < commands[smallest_id].timestamp)
            smallest_id = child_id;

        swap_commands(parent_id, smallest_id);
    } while (smallest_id != parent_id);
}

struct command pop_command() {
    command_t out_command;
    assert(commands_used > 0);
    memcpy(&out_command, &commands[0], sizeof(command_t));
    swap_commands(0, commands_used-1);
    commands_used--;
    heapify_down();

#if 0
    if (out_command.type != 2 /*READ_SERIAL_COMMAND*/)
    {
        debug("Popping command: ");
        debug(out_command.type);
        debug(", ");
        debugll(out_command.timestamp);
        debug(", used: ");
        debug(commands_used);
        debug(", of: ");
        debugln(COMMANDS_QUEUE_SIZE);
    }
#endif
    return out_command;
}

void push_command(struct command command) {
#if 0
    if (command.type != 2 /*READ_SERIAL_COMMAND*/)
    {
        debug("Pushing command: ");
        debug(command.type);
        debug(", ");
        debugll(command.timestamp);
        debug(", used: ");
        debug(commands_used);
        debug(", of: ");
        debugln(COMMANDS_QUEUE_SIZE);
    }
#endif

    assert(commands_used + 1 < COMMANDS_QUEUE_SIZE);
    memcpy(&commands[commands_used], &command, sizeof(command_t));
    commands_used++;
    heapify_up();
}

void reset_queue() {
    commands_used = 0;
}

int queue_empty() {
    return (commands_used == 0);
}
/***********************************************************************************************/
/* COMMANDS */
/***********************************************************************************************/
typedef void (*command_handler_t)(command_t command);

enum {
    START_COMMAND,
    POWER_UP_COMMAND,
    READ_SERIAL_COMMAND,
    RELAY_SWITCH_COMMAND,
    SAMPLE_TEMPERATURE_COMMAND,
    READ_TEMPERATURE_COMMAND,
    HEATING_COOLING_COMMAND,
    TOGLE_HEATING_COOLING_COMMAND,
    NR_COMMAND_TYPES
};

/* -------- */
#define NR_RELAYS  6
struct relay_desired_state {
    int on;
};
struct relay_desired_state relays[NR_RELAYS];
long relay_pins[NR_RELAYS] = {2, 3, 4, 5, 6, 7};
bool relay_on_high[NR_RELAYS] = {0, 0, 0, 0, 0, 0};

void relay_switch_power_up() {
    debugln("relay_switch_power_up");
    int i;
    for (i=0; i<NR_RELAYS; i++) {
        pinMode(relay_pins[i], OUTPUT);
        relay_switch_command_init(i, 0);
    }
}

void relay_switch_command_init(long relay_nr, long switch_on)
{
    command_t command;

    debug("Relay #: ");
    debug(relay_nr);
    debugln((switch_on ? ", on" : ", off"));
    if (relay_nr < 0 || relay_nr >= NR_RELAYS) {
        return;
    }
    relays[relay_nr].on = switch_on ? 1 : 0;

    command.timestamp = NOW;
    command.type = RELAY_SWITCH_COMMAND;
    command.data = relay_nr;

    push_command(command);
}

void relay_switch_command_handler(struct command command)
{
    long relay_nr;
    int want_on, want_high;

    relay_nr = command.data;
    if (relay_nr < 0 || relay_nr >= NR_RELAYS) {
        return;
    }

    want_on = relays[relay_nr].on;
    // Could be XOR, but let's keep it simple stupid
    want_high = (relay_on_high[relay_nr] && want_on) || (!relay_on_high[relay_nr] && !want_on);
    digitalWrite(relay_pins[relay_nr], want_high ? HIGH : LOW);
    delay(100);
}

/* -------- */
DallasTemperatureSensor *temperature_sensor;
TimeSeries *temperature_time_series;

void sample_temperature_command_init()
{
    command_t command;

    command.timestamp = NOW + 1000UL * TEMPERATURE_SAMPLING_PERIOD_MS;
    command.type = SAMPLE_TEMPERATURE_COMMAND;
    command.data = 0;

    push_command(command);
}

void sample_temperature_command_handler(struct command command)
{
    timestamp_t time = NOW;
    debugln("sample_temperature_command_handler");
    temperature_t temperature = temperature_sensor->readTemperature();
    temperature_time_series->storeValue(time, temperature);

    // Schedule next command
    sample_temperature_command_init();
}

void sample_temperature_power_up()
{
    debugln("sample_temperature_power_up");
    temperature_sensor = new DallasTemperatureSensor(ONE_WIRE_BUS);
    temperature_time_series = new TimeSeries(TEMPERATURE_HISTORY_SIZE);
    // Kick off endless chain of temperature samplings
    sample_temperature_command_init();
}

/* -------- */
void read_temperature_command_init()
{
    command_t command;

    command.timestamp = NOW;
    command.type = READ_TEMPERATURE_COMMAND;
    command.data = 0;

    push_command(command);
}

void read_temperature_command_handler(struct command command)
{
    temperature_t temperature = temperature_time_series->getLatestSmoothed();
    Serial.print("t,");
    Serial.println(temperature);
}


/* -------- */
void togle_heating_cooling_command_init(void *pidp)
{
    command_t command;
    // Work-around for compiler not allowing PID* to be used as argument type
    // could be related to it disallowing typedefs
    PID *pid = (PID *)pidp;

    command.timestamp = pid->getOnTimestamp();
    command.type = TOGLE_HEATING_COOLING_COMMAND;
    command.data = (unsigned long)pid;
    push_command(command);

    command.timestamp = pid->getOffTimestamp();
    command.type = TOGLE_HEATING_COOLING_COMMAND;
    command.data = (unsigned long)pid;
    push_command(command);
}

void togle_heating_cooling_command_handler(struct command command)
{
    PID *pid = (PID *)command.data;
    relay_switch_command_init(pid->getRelayId(), pid->shouldBeOn() ? 1 : 0);
}

/* -------- */
temperature_t target_temperature;
PID *heaterPid;
PID *coolerPid;

// Allows target temperature change to reset the chain of commands
timestamp_t next_heating_cooling_reevaluation;
// Prevents too frequent resets
unsigned long reset_counter;

void heating_cooling_command_init(int throttled, unsigned long delay_period)
{
    command_t command;

    next_heating_cooling_reevaluation = NOW + delay_period;
    command.timestamp = next_heating_cooling_reevaluation;
    command.type = HEATING_COOLING_COMMAND;
    command.data = throttled;

    push_command(command);
}

void heating_cooling_command_handler(struct command command)
{
    if (NOW < next_heating_cooling_reevaluation)
    {
        // Some other command took over
        debugln("> Obsolete reevaluating of heating/cooling");
        return;
    }
    debugln("> Reevaluating heating/cooling");
    if (!command.data)
    {
        reset_counter = 0;
    }

    double heaterOutput = heaterPid->output(target_temperature, temperature_time_series);
    double coolerOutput = coolerPid->output(target_temperature, temperature_time_series);

    PID *outputPid;
    double output;
    // Figure out if to heat, cool or do nothing
    if (heaterOutput > 0.05)
    {
        outputPid = heaterPid;
        output = heaterOutput;
        debug("Scheduling heating, output: ");
        debugln(output);
    } else
    if (coolerOutput < -0.05)
    {
        outputPid = coolerPid;
        output = coolerOutput;
        debug("Scheduling cooling, output: ");
        debugln(output);
    } else
    {
        // Do nothing
    }

    if (outputPid) {
        // TODO: would be good to un-schedule the other PID
        //       right now cooling and heating can overlap after change
        outputPid->schedule(ABS(output));
        togle_heating_cooling_command_init(outputPid);
    }

    // Schedule next command
    heating_cooling_command_init(0, 1000UL * HEATING_COOLING_ADJUSTMENT_PERIOD_MS);
}

void heating_cooling_power_up()
{
    debugln("heating_cooling_power_up");
    reset_counter = 0;
    target_temperature = DEFAULT_TARGET_TEMPERATURE;
    heaterPid = new PID(0.1, 0, 0, HEATER_RELAY_ID);
    coolerPid = new PID(0.1, 0, 0, COOLER_RELAY_ID);
    // Kick off endless chain of heating/cooling adjustements, allowing
    // for a few temperature samples to be collected first
    heating_cooling_command_init(0, 10 * 1000UL * TEMPERATURE_SAMPLING_PERIOD_MS);
}

void heating_cooling_set_target_temperature(double/*temperature_t*/ target_t)
{
    target_temperature = target_t;
    debug("Set target temperature to:");
    debugln(target_t);
    temperature_time_series->resetIntegral();

    if (reset_counter++ > 5)
    {
        debugln("> Ignoring attempt to reset heating/cooling loop");
        return;
    }
    heating_cooling_command_init(1, 0);
}



/* -------- */
void power_up_command_handler(struct command command)
{
    relay_switch_power_up();
    sample_temperature_power_up();
    heating_cooling_power_up();
}

/* -------- */
#define BUFFER_LENGTH   32
struct serial_data {
    char buffer[BUFFER_LENGTH];
    int buffer_offset;
};
static struct serial_data read_serial_data = {{0}, 0};

long parse_long(char *buffer, char *end, char **new_buffer)
{
    int idx;
    long out;

    //Serial.print(buffer);
    idx = 0;
    for (idx=0; buffer + idx < end; idx++) {
        if (idx == 0 && buffer[idx] == '-') {
            continue;
        }
        if (buffer[idx] < '0' || buffer[idx] > '9') {
            buffer[idx] = '\0';
            break;
        }
    }
    *new_buffer = (buffer + idx + 1);
    if (idx == 0) {
        return 0;
    }
    out = atol(buffer);

    //Serial.print("Parsing \"");
    //Serial.print(buffer);
    //Serial.print("\", got: ");
    //Serial.print(out);
    //Serial.println("");

    return out;
}

void process_serial_command(void)
{
    char *b = read_serial_data.buffer;

    debug("> Processing serial: ");
    debugln(b);
    switch (*b) {
        case 's':
        {
            char *end;

            b++;
            end = read_serial_data.buffer + read_serial_data.buffer_offset;
            target_temperature = parse_long(b, end, &b);
            heating_cooling_set_target_temperature(target_temperature);

            break;
        }

        case 't':
        {
            b++;
            read_temperature_command_init();
            break;
        }

        default:
            Serial.print("Unknown command: \"");
            Serial.print(b);
            Serial.println("\"");
            break;
    }
    read_serial_data.buffer_offset = 0;
}

void read_serial_command_handler(struct command command)
{
    timestamp_t now = NOW;
    while (Serial.available() > 0) {
        char b;
        int idx;

        idx = read_serial_data.buffer_offset;
        b = Serial.read();
        assert(idx < BUFFER_LENGTH);
        read_serial_data.buffer[idx] = b;
        read_serial_data.buffer_offset++;
        if (b == '\n' || b == '$') {
            read_serial_data.buffer[idx] = '\0';
            process_serial_command();
        }
    }

    /* Finally, requeue to check serial line at the appropriate time. */
    command.type = READ_SERIAL_COMMAND;
    command.timestamp = NOW + 1000UL * 1000UL / BAUD_RATE;
    push_command(command);
}

void start_command_handler(struct command command)
{
    command_t power_up;
    command_t serial_input;

    Serial.println("Start command");
    power_up.type = POWER_UP_COMMAND;
    power_up.timestamp = NOW + 3 * 1000UL * 1000UL;
    push_command(power_up);

    serial_input.type = READ_SERIAL_COMMAND;
    serial_input.timestamp = NOW;
    push_command(serial_input);
}

void (*command_handlers[NR_COMMAND_TYPES])(struct command command) = {
    /* [START_COMMAND] =                 */ start_command_handler,
    /* [POWER_UP_COMMAND] =              */ power_up_command_handler,
    /* [READ_SERIAL_COMMAND] =           */ read_serial_command_handler,
    /* [RELAY_SWITCH_COMMAND] =          */ relay_switch_command_handler,
    /* [SAMPLE_TEMPERATURE_COMMAND] =    */ sample_temperature_command_handler,
    /* [READ_TEMPERATURE_COMMAND] =      */ read_temperature_command_handler,
    /* [HEATING_COOLING_COMMAND] =       */ heating_cooling_command_handler,
    /* [TOGLE_HEATING_COOLING_COMMAND] = */ togle_heating_cooling_command_handler,
};



/***********************************************************************************************/
/* MAIN LOOP */
/***********************************************************************************************/

void setup() {
    command_t start_command;

    Serial.begin(BAUD_RATE);
    Serial.print("setup");
    /* Reset the queue and initiate operation by queueing the start command. */
    reset_queue();
    start_command.type = START_COMMAND;
    start_command.timestamp = NOW;
    push_command(start_command);
}

// the loop routine runs over and over again forever:
void loop() {
    command_t current_command;
    command_handler_t handler;
    timestamp_t wait_time;

    timestamp_t start_t;
    timestamp_t current_t;

    assert(freeRam() > 512);
    /* If there are no commands left, idle. */
    if (queue_empty()) {
        Serial.println("WARNING: Empty queue");
        delay(1000);
        return;
    }

    /* Normal case: there is at least one command, pop it off the queue and exec. */
    current_command = pop_command();
    handler = command_handlers[current_command.type];
    wait_time = current_command.timestamp - NOW;

    if (VALID_DELAY(wait_time))
        delay_microseconds(wait_time);
    handler(current_command);
}

