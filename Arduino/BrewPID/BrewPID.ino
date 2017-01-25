#include <dht.h>

/***********************************************************************************************/
/* CONFIG */
/***********************************************************************************************/

#define BAUD_RATE   (9600UL)

/***********************************************************************************************/
/* UTILITIES */
/***********************************************************************************************/
#define CONCAT_TOKENS( TokenA, TokenB )       TokenA ## TokenB
#define EXPAND_THEN_CONCAT( TokenA, TokenB )  CONCAT_TOKENS( TokenA, TokenB )
#define ASSERT( Expression )                  enum{ EXPAND_THEN_CONCAT( ASSERT_line_, __LINE__ ) = 1 / !!( Expression ) }
#define ASSERTM( Expression, Message )        enum{ EXPAND_THEN_CONCAT( Message ## _ASSERT_line_, __LINE__ ) = 1 / !!( Expression ) }
#define assert(x)                             if (!(x)) { Serial.print("Failure in: "); Serial.println(__LINE__); }

typedef unsigned long timestamp_t;
#define NOW     micros()
#define VALID_DELAY(_d) ((_d) > 0 && (_d) < ((-1UL) >> 2))

#define ANALOG_TO_MILLIVOLTS(_a)     ((_a) * 5000UL / 1024UL)
#define ABS(_v)  ((_v) < 0 ? -(_v) : (_v))

void delay_microseconds(unsigned long /* timestamp_t */ delay_us)
{
    if (delay_us < 16000UL)
        delayMicroseconds(delay_us);
    else
        delay(delay_us / 1000UL);
}

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

    return out_command;
}

void push_command(struct command command) {
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
    SENSORS_SWITCH_COMMAND,
    READ_SENSOR_COMMAND,
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

void relay_switch_command_init(long relay_nr, long switch_on)
{
    command_t command;

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
long sensors_switch_pin = 9;

void sensors_switch_command_init(long switch_on)
{
    command_t command;

    command.timestamp = NOW;
    command.type = SENSORS_SWITCH_COMMAND;
    command.data = switch_on;

    push_command(command);
}

void sensors_switch_command_handler(struct command command)
{
    long want_on;

    want_on = command.data;
    digitalWrite(sensors_switch_pin, want_on ? HIGH : LOW);
    /* Delay is required before sensors can be used after power-up */
    if (want_on) {
        int i;
        for (i=0; i<10; i++) {
            delay(100);
        }
    }
}


/* -------- */
void read_sensor_command_init(long sensor_nr)
{
    command_t command;

    command.timestamp = NOW;
    command.type =  READ_SENSOR_COMMAND;
    command.data = sensor_nr;

    push_command(command);
}

#define NR_SENSORS  6
long sensor_pins[NR_RELAYS] = {16, 14, 15, 18, 19, 20};

dht DHT;
void read_sensor_command_handler(struct command command)
{
    int chk;
    long sensor_nr, pin_nr;

    sensor_nr = command.data;
    if (sensor_nr < 0 || sensor_nr >= NR_SENSORS) {
        return;
    }
    pin_nr = sensor_pins[sensor_nr];
    chk = DHT.read22(pin_nr);

    Serial.print("s,");
    Serial.print(sensor_nr);
    Serial.print(",");
    switch(chk) {
        case DHTLIB_OK:
            Serial.print("OK,");
            Serial.print(DHT.humidity, 1);
            Serial.print(",");
            Serial.println(DHT.temperature, 1);
            break;
        case DHTLIB_ERROR_CHECKSUM:
            Serial.println("CHKSUM_ERROR");
            break;
        case DHTLIB_ERROR_TIMEOUT:
            Serial.println("TIMEOUT_ERROR");
            break;
        default:
            Serial.print("UNKNOWN_ERROR,");
            Serial.println(chk);
            break;
    }
}

/* -------- */
void power_up_command_handler(struct command command)
{
    int i;
    delay(1000);
    for (i=0; i<NR_RELAYS; i++) {
        pinMode(relay_pins[i], OUTPUT);
        relay_switch_command_init(i, 0);
    }
    pinMode(sensors_switch_pin, OUTPUT);
    sensors_switch_command_init(1);
    for (i=0; i<NR_SENSORS; i++) {
        pinMode(sensor_pins[i], INPUT);
    }
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

    switch (*b) {
        case 'r':
        {
            long relay_nr, switch_on;
            char *end;

            b++;
            end = read_serial_data.buffer + read_serial_data.buffer_offset;
            relay_nr = parse_long(b, end, &b);
            switch_on = parse_long(b, end, &b);
            relay_switch_command_init(relay_nr, switch_on);

            break;
        }

        case 'a':
        {
            long want_on;
            char *end;

            b++;
            end = read_serial_data.buffer + read_serial_data.buffer_offset;
            want_on = parse_long(b, end, &b);
            sensors_switch_command_init(want_on);

            break;
        }

        case 's':
        {
            long sensor_nr;
            char *end;

            b++;
            end = read_serial_data.buffer + read_serial_data.buffer_offset;
            sensor_nr = parse_long(b, end, &b);
            read_sensor_command_init(sensor_nr);

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
    power_up.timestamp = NOW;
    push_command(power_up);

    serial_input.type = READ_SERIAL_COMMAND;
    serial_input.timestamp = NOW;
    push_command(serial_input);
}

void (*command_handlers[NR_COMMAND_TYPES])(struct command command) = {
    /* [START_COMMAND] =          */ start_command_handler,
    /* [POWER_UP_COMMAND] =       */ power_up_command_handler,
    /* [READ_SERIAL_COMMAND] =    */ read_serial_command_handler,
    /* [RELAY_SWITCH_COMMAND] =   */ relay_switch_command_handler,
    /* [SENSORS_SWITCH_COMMAND] = */ sensors_switch_command_handler,
    /* [READ_SENSOR_COMMAND] =    */ read_sensor_command_handler,
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



/***********************************************************************************************/
/* OLD CODE, NO LONGER IN ACTIVE USE */
/***********************************************************************************************/
#if 0

//// Testing heap
void loop() {
    command_t command;
    unsigned char i, j;

    Serial.println(commands_used);
    digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);               // wait for a second
    digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);               // wait for a second

    reset_queue();

    for (i=0; i<50; i++) {
        command.type = i;
        command.timestamp = random(1000);
        Serial.print("Adding type, idx: ");
        Serial.print(command.type);
        Serial.print(", timestamp: ");
        Serial.println(command.timestamp);
        push_command(command);
    }

    for (i=0; i<commands_used/2; i++) {
        command = pop_command();
        Serial.print("Popped type, idx: ");
        Serial.print(command.type);
        Serial.print(", timestamp: ");
        Serial.println(command.timestamp);
    }

    Serial.println("Done all");
    delay(10000);
}


#endif





