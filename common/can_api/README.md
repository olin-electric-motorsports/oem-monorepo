# Olin Electric Motorsports CAN API

The purpose of this project is to automatically generate C code that abstracts
CAN-bus interaction for STM32G4 microcontrollers.

Further documentation:

[Video: Using CAN API](https://youtu.be/b5lodbnx-aE)

[Video: CAN API Library Explanation - COMING SOON]()

[Whiteboard Software Diagram](https://miro.com/app/board/uXjVGDeQbGc=/?share_link_id=659340441364) 

## Context

Before 2021, OEM used a basic CAN API implementation that enabled sending and 
receiving CAN messages, but lacked a unified method to specify message ID and
content across files. 

The first version of the current CAN API library was created as a layer of 
abstraction that utilizes the CAN library as a network layer. Messages and their
signals are defined in YAML files.

In 2026, the library was re-factored to transition from ATMega16M1 
microcontrollers to the STM32G4 series of microcontrollers.

## Scope

The project is designed to be _robust_ and as _reusable_ as possible. The 
backend of the project uses an external library called
[cantools](https://github.com/cantools/cantools), which is thoroughly tested.

### Goals

- A DBC file can be generated from the individual YAML files
- All message collisions are detected at compile-time
- A `.c` and `.h` file are generated for each YAML file that include send and
  receive functions for each of the messages sent and received by the MCU as
  specified by the YAML file

### Non-goals

- _Getters_ and _setters_: Functions of these types are used to set the values
  of signals in a CAN message (like
  `set_bspd_brakelight_voltage(uint16_t voltage)`). These could be useful, but
  for now, we opt to use global variables because they have a simpler
  implementation.
- Multiple MCU compatibility. This library is exclusively for STM32G4 series
  microcontrollers. If the team ever moves towards an architecture that utilizes
  different types of MCUs within the same vehicle, the library could feasibly be
  updated to support multiple MCUs.
- Message ID assignment: This could be a future goal, see **Future Work** below

# Usage

Using the CAN API in your own project requires 3 steps:
1. YAML Setup
2. Bazel Integration
3. Firmware Implementation

# YAML Setup

To begin, we must define a YAML file for the MCU. As an example, we will use the
BMS MCU. The file we are writing will be in `vehicle/mkviii/software/bms/bms.yml`.
We begin with the name of the node/MCU, _bms_:

```yaml
# vehicle/mkviii/software/bms/bms.yml
name: bms # Name of the MCU
```
## Receiving CAN messages

Next we specify the messages received by the MCU. For example, the BMS should
receive the AIR Control Critical message, so we can specify the following:

```yaml
# All the received messages
subscribe:
  - name: air_control_critical
```

This says that the BMS should listen for the `air_control_critical` message
(this name is specified as the `name` of a message in a different YAML file).
To receive a message, only the message name is needed, the API will sort out 
the rest!

## Sending CAN messages

Now we specify the messages _sent_ by the MCU. When sending a message, we need
to provide some additional information. For each message, we must specify:

- name: *a unique message name. This name will be used to receive this message on other nodes in the network*
- id: *a unique message ID. Should align with CAN address space spreadsheet*
- freq_hz: *determines how often this message is sent*
- signals: *every CAN message is broken up into individual signals. For each signal, we must give a name, slice, and unit*
    - name: *name of the signal*
    - slice: *(start bit) + (length). Defines the position within the message of each signal. Total range is 0 to 64 bits in Classic CAN*
    - unit: *Each unit must have a type. Depending on this type, each unit might also need values, name, offset, and scale. See note on signal types below* 

**Signal Types**

The CAN API supports the following possible types: enum, int8_t, int16_t, uint8_t, uint16_t, and bool.

Integer types include a name, scale, and offset field:
```yaml
unit:
  type: uint8_t
  name: "%"
  offset: 0
  scale: 100 / 4096
```

enum types include values:
```yaml
unit:
  type: enum
  values:
    - NO_FAULT
    - FAULT_EXAMPLE_1
    - FAULT_EXAMPLE_2
```

bool types include a true and false value:
```yaml
unit:
  type: bool  # Bool is used for any value where it is either 0 or 1
  values:
    t: Relay open  # String to display when value is true
    f: Relay closed  # When value is false
```

As an example, we will take the following row from the MKV CAN Address Space:

| Name     | ID   | Length | Frequency (Hz) | Byte0      | Byte1        | Byte2       | Byte3        | Byte4        | Byte5  | Byte6                    | Byte7                 |
| -------- | ---- | ------ | -------------- | ---------- | ------------ | ----------- | ------------ | ------------ | ------ | ------------------------ | --------------------- |
| BMS Core | 0x10 | 8      | 16             | Fault Code | Relay Status | Temperature | Pack Voltage | SOC Estimate | BMS OK | Current-limiting enabled | Cell-balancing status |


The MKV CAN Address space specified that the minimum size of any signal is 1
byte. However, with our system, we can have signals that are smaller (like a
single bit). Here's how we might represent the message in a new way:

```yaml
publish: 
  - name: bms_core
    id: 0x10
    freq_hz: 16
    signals:
      - name: fault_code
        slice: 0 + 8  # This is (start bit) + (length)
        unit:
          type: enum
          values:
            - NO_FAULT
            - FAULT_EXAMPLE_1
            - FAULT_EXAMPLE_2
            - ...
      - name: temperature
        slice: 8 + 8
        unit:
          type: uint8_t  # C-type that this is stored as
          name: degC  # Used for displaying when decoding information
          offset: 0  # Offset used to calculate the true value
          scale: 5 / 4096  # Scale used to calculate the true value
      - name: pack_voltage
        slice: 16 + 8
        unit:
          type: uint8_t
          name: V
          offset: 0
          scale: 1000
      - name: soc  # State of Charge
        slice: 24 + 8
        unit:
          type: uint8_t
          name: "%"
          offset: 0
          scale: 100 / 4096
      - name: relay_state
        slice: 32 + 1
        unit:
          type: bool  # Bool is used for any value where it is either 0 or 1
          values:
            t: Relay open  # String to display when value is true
            f: Relay closed  # When value is false
      - name: bms_ok
        slice: 33 + 1
        unit:
          type: bool  # Bool is used for any value where it is either 0 or 1
          values:
            t: BMS OK
            f: BMS NOT OK
      - name: current_limiting_enabled
        slice: 34 + 1
        unit:
          type: bool  # Bool is used for any value where it is either 0 or 1
          values:
            t: Current limiting enabled
            f: Current limiting disabled
      - name: cell_balancing_status
        slice: 35 + 1
        unit:
          type: bool  # Bool is used for any value where it is either 0 or 1
          values:
            t: Cell balancing active
            f: Cell balancing inactive
```

Notice that we have reordered some of the fields. This is because it is better
to group signals together if they are less than a byte so that they can be
"packed" together (this is known as bit-packing). If we had a 1-bit signal 
followed by an 8-bit signal followed by a 1-bit, it would require at least 3
bytes of space since full bytes must be aligned.

# Bazel Integration

Now that we have our completed file, we can define our CAN API using Bazel. In
the `BUILD` file, we will load a rule and create a new target:

```python
# vehicle/mkviii/software/bms/BUILD

load("//common/can_api:can_defs.bzl", "can_api_files")

...

# Must include this
exports_files([
    "bms.yml",
])

# Defines the CAN API library
can_api_files(
    name = "bms_can_api",
    yaml = "bms.yml",
    dbc = "//vehicle/mkviii:mkviii.dbc", # Points to DBC target
    deps = [
        "//third_party:stm32cubeg4",
    ],
)
```

In addition, we should add the BMS YAML file to the list of MKV YAML files in
`vehicle/mkviii/BUILD`:

```python
# vehicle/mkviii/BUILD
load("//bazel/tools:defs.bzl", "dbc_gen")

# Generates DBC
dbc_gen(
    name = "mkviii.dbc",
    srcs = [
        "//vehicle/mkviii/software/air_control:air.yml",
        "//vehicle/mkviii/software/brakelight_bspd:bspd.yml",
        "//vehicle/mkviii/software/motor_controller:motor_controller.dbc",

        # New
        "//vehicle/mkviii/software/bms:bms.yml",
    ],
)
```

We do this so that we can use the DBC generator to link all of these YAML files
together into a single DBC.

Now that we have our CAN API created, we can test to make sure that it works:

```shell
$ bazel build --config=m4 //vehicle/mkviii/software/bms:bms_can_api
```

If this builds successfully, then the CAN API generation is working.

We can add `:can_api` to the `deps` field of our `cc_firmware` that defines the
BMS code. 

# Firmware Implementation

With the YAML file created and Bazel integration complete, we are ready to
implement the library into our project's code.

## Initialization

#### `can_init_{node_name}`

This replaces/wraps the `can_init` function. Currently, there is no additional
functionality, but in the future it could be used to initialize certain
MCU-specific things, or could be useful if more than one CAN bus existed on the
car.

Documentation (node_name = `bms_core`):

```c
/*
 * @brief Initialize the bms_core CAN peripheral
 *
 * Completes the neccesary configuration steps for initializing the FDCAN1
 * peripheral. Initializes the FDCAN1 peripheral, initializes RX filters,
 * initializes TX headers, and starts the FDCAN1 peripheral.
 *
 * @returns 0 for success, 1 for error
 */
int can_init_bms_core(void);
```

Usage (node_name = `bms_core`):

```c
//vehicle/mkviii/software/bms/bms.c

...

// In the setup section of your code
if (can_init_bms_core() != 0) {
    // Initialization failed ...
}

...

```

## Receiving messages

#### `can_receive_all`

This receives all CAN messages subscribed to by the node, specified by the YAML
file.

Documentation:

```c
/*
 * @brief Check for and unpack newly received messages
 * 
 * Checks the RX FIFO queue for any new CAN messages that passed the filters.
 * For each message in the queue (if any), the data and header information is
 * read into variables. Using the message ID, we determine the appropiate
 * cantools function to unpack the received data into the corresponding struct.
 *
 * @returns 0 for success, 1 for error
 */
int can_poll_receive_all(void);
```

Usage (assumes node subscribes to `bspd`):

```c
//vehicle/mkviii/software/bms/bms.c

...

// In the main loop section of your code
if (can_poll_receive_all() != 0) {
    // Receiving messages failed ...
}

// Now this struct is updated
my_var = bspd.status;

...

```


## Sending messages

In order to set the value of a signal, we use a struct. The CAN API generates a
struct whose fields are each of the signals of the message. 

**This step can be confusing since it might not be immediately clear where this struct was defined**.
For now, just trust that the struct exists and can be accessed by the name of
the message (i.e. `bms_core`).

For example, the struct for the `bms_core` message defined above might look
like:

```c
struct can_tools_bms_core_t {
    uint8_t fault_code;
    uint8_t temperature;
    uint8_t pack_voltage
    uint8_t soc;
    uint8_t relay_state;
    uint8_t bms_ok;
    uint8_t current_limiting_enabled;
    uint8_t cell_balancing_status;
};
```

**Note that even though the length of, for example, `relay_state` should be 1
according to the YAML, the struct will store it as a byte. This is because
before the message is sent using the `can_send` function, there is another,
private function that "packs" the message according to the correct lengths. So
the value of the `uint8_t relay_state` will be packed into a single bit when the
message is sent**.

The CAN API also defines a single variable of this type as
`struct can_tools_bms_core_t bms_core`. That way, you can access it like so:

```c
// Set the pack_voltage
bms_core.pack_voltage = get_pack_voltage();

// Set the relay_state
bms_core.relay_state = gpio_get_pin(BMS_RELAY);
```

This is the same variable that is used when sending the CAN message, so care
should be taken that there are no concurrency issues (say, if a value is set
during an interrupt which occurs while a CAN message is being sent).

Once the struct is updated, you can send it with the following:

#### `can_send_{message}`

This sends a specific CAN message. For example, to send the _BMS Core_ CAN
message, we could call `can_send_bms_core();`. This will take the aforementioned
`bms_core` variable (with all of the signals as struct fields), pack it into a
byte array, and send it as a CAN message.

Documentation (for a node publishing `bms_core` message):

```c
/*
 * @brief Send the bms_core message
 *
 * We use the can_tools_bms_core_pack function from cantools to
 * pack the outgoing message into an array of bytes. Then we add the message to
 * the TX FIFO queue, which sends the message to bus.
 * 
 * @returns 0 for success, 1 for error
 */
int can_send_bms_core(void);
```

Usage (to publish `bms_core` message):

```c
//vehicle/mkviii/software/bms/bms.c

...

// In the main loop section of your code

// Update struct with new values
bms_core.temperature = get_temp();

// Send the message
if (can_send_bms_core() != 0) {
    // Sending message failed ...
}

...

```

# Other Notes

## DBC Generation

The final feature of the project is DBC generation. As we saw, when we declare a
YAML file, we can also add it to the `srcs` of a `dbc_gen` target, like the one
in `vehicle/mkviii/BUILD`. In general, we want to have one DBC per vehicle. In
order to build the mkviii DBC, we can run:

```shell
$ bazel build //vehicle/mkviii:mkviii.dbc
```

Then we can access the DBC file in `bazel-bin/vehicle/mkviii/mkviii.dbc` and use it in
Wireshark, BusMaster, or any other software that uses a DBC.

## Concurrency

**⚠️ Important Usage Note. ⚠️**

The `can_send_{{ message }}` function must _not_ be called from an interrupt
context. If it is, there can be unpredictable behavior. In general, special care
should be taken around interrupts and concurrency. There is a global, `volatile`
variable for the CAN data containing each of the signals, and this struct is
used in the `can_send` function. Thus, if the `can_send` function happens in an
interrupt that occured when a variable was in the middle of being updated, the
data can become garbled.

The `can_send` function is wrapped in atomic block, meaning that during its
execution, no interrupts will occur.

## Future Work

### Automatic ID Generation

In the future, it would be great if the IDs were automatically generated based
on message priority and deadline. There is a
[paper](https://www-users.cs.york.ac.uk/~robdavis/papers/RTNS2016_can_extend.pdf)
that describes an algorithm that can be used to optimally generate CAN IDs, and
this could be interesting to implement later on. But since our CANbus is so
under-utilized, there isn't really any urgency.

### FD CAN and Extended IDs

The STM32G4 series supports flexible data-rate (FD) CAN. When compared to 
Classic CAN, FD CAN enables larger data payloads (8 -> 64 bytes) and enables an 
increased maximum data rate (1mbps -> 2-5mbps). The current implementation of
the CAN API only supports Classic CAN, which is adequate for our team's current
needs, but making the upgrade could be a cool project.

Similarly, the STM32G4 series also supports extended message IDs, which
increases the total number of unique messages that the bus can support. The
current address space still has plenty of available IDs, but adding this support
could be interesting.
