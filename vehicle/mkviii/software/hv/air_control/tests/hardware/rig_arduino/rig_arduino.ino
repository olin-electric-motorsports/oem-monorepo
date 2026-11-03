// rig_arduino: lets air_hw_test.py work the AIR board's pins over USB serial
// at 115200 baud. One command per line, one reply per line:
//   o 7    pin 7 output  -> ok        i 7    pin 7 input -> ok
//   w 7 1  pin 7 high    -> ok        r 7    read pin 7  -> 1 or 0
//   s 7 1  pin 7's safe level is high -> ok
// Anything else -> ?
// Watchdog: once commands start, if none arrives for 1 s (the laptop crashed,
// the test was stopped, or USB came out) every output goes to its safe level,
// or is let go if it has none, and the next command is answered "tripped"
// instead of being run.
const unsigned long TIMEOUT_MS = 1000;
int8_t safe_level[NUM_DIGITAL_PINS]; // -1: none set
bool is_output[NUM_DIGITAL_PINS];
unsigned long last_command;
bool armed = false, tripped = false;

void setup() {
    Serial.begin(115200);
    for (int pin = 0; pin < NUM_DIGITAL_PINS; ++pin) safe_level[pin] = -1;
}

// The laptop has gone quiet: put every output where it is safe
void trip() {
    for (int pin = 0; pin < NUM_DIGITAL_PINS; ++pin) {
        if (!is_output[pin]) continue;
        if (safe_level[pin] >= 0) {
            digitalWrite(pin, safe_level[pin] ? HIGH : LOW);
        } else {
            pinMode(pin, INPUT);
            is_output[pin] = false;
        }
    }
    armed = false;
    tripped = true;
}

void loop() {
    static char line[16];
    static uint8_t used = 0;
    if (armed && millis() - last_command > TIMEOUT_MS) trip();
    if (!Serial.available()) return;
    char c = Serial.read();
    if (c != '\n') {
        if (c != '\r' && used < sizeof(line) - 1) line[used++] = c;
        return;
    }
    line[used] = '\0';
    used = 0;
    last_command = millis();
    armed = true;
    if (tripped) {
        tripped = false;
        Serial.println("tripped");
        return;
    }
    char command;
    int pin, value = 0;
    int got = sscanf(line, "%c %d %d", &command, &pin, &value);
    if (got < 2 || pin < 0 || pin >= NUM_DIGITAL_PINS) Serial.println("?");
    else if (command == 'r') Serial.println(digitalRead(pin));
    else if (command == 'o') { pinMode(pin, OUTPUT); is_output[pin] = true; Serial.println("ok"); }
    else if (command == 'i') { pinMode(pin, INPUT); is_output[pin] = false; Serial.println("ok"); }
    else if (command == 'w' && got == 3) { digitalWrite(pin, value ? HIGH : LOW); Serial.println("ok"); }
    else if (command == 's' && got == 3) { safe_level[pin] = value ? 1 : 0; Serial.println("ok"); }
    else Serial.println("?");
}
