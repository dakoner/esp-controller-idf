# LED Controller Console (ESP-IDF)

This project is a console application for the ESP32 that provides precise control over GPIO pins for LED applications. It supports various modes including single pulses, PWM, hardware interrupts, and synchronized "throb" effects. The application runs a REPL (Read-Eval-Print Loop) over UART/USB.

## Features

- **Pulse Control**: Generate precise single pulses on GPIO pins.
- **PWM**: Hardware-timed PWM generation with adjustable frequency and duty cycle.
- **Interrupt Triggers**: Configure GPIO interrupts to trigger pulses on other pins with microsecond precision.
- **Repeating Pulses**: Generate repeating pulse trains using software timers.
- **Throb Effect**: Synchronized sinusoidal fading effect across 3 channels.
- **WiFi Info**: Display current WiFi connection details.
- **System Info**: Memory and GPIO status commands.
- **Boot Script**: Automatically executes commands from `boot.txt` on the storage partition at startup.

## Command Reference

The following commands are available in the console:

| Command | Arguments | Description |
|---------|-----------|-------------|
| `pulse` | `<pin> <value> <duration>` | Pulse a GPIO pin to a value (0 or 1) for a specific duration in microseconds. |
| `level` | `<pin> [value]` | Set a GPIO pin to a specific level (0 or 1). If `value` is omitted, prints the current level. |
| `pwm` | `<pin> <frequency> <duty>` | Start PWM on a pin. `frequency` (1-1,000,000 Hz), `duty` (0-99 %). |
| `stoppwm` | `<pin>` | Stop PWM output on a specific pin. |
| `interrupt` | `<pin> <edge> <target_pin> <width>` | Configure an interrupt on `<pin>` (edges: `RISING`, `FALLING`, `BOTH`) to trigger a pulse on `<target_pin>` for `<width>` microseconds. |
| `stopinterrupt` | `<pin>` | Remove the interrupt handler and configuration from a specific pin. |
| `repeat` | `<pin> <frequency> <duration>` | Start a repeating pulse train on a pin. `frequency` in Hz, `duration` in microseconds. |
| `stoprepeat` | `<pin>` | Stop the repeating pulse train on a specific pin. |
| `throb` | `<period> <pin1> <pin2> <pin3>` | Start a synchronized sinusoidal "throb" effect on 3 pins with a specified period (seconds). |
| `stopthrob` | | Stop the active throb effect. |
| `info` | | Display current GPIO configuration and status. |
| `mem` | `<address>` | Read and display 4 bytes from a physical memory address. |
| `wifi` | | Display current Wi-Fi status, MAC address, SSID, RSSI, and IP addresses. |
| `help` | | Print the list of all registered commands. |

## Storage and Configuration

The application mounts a FATFS partition at `/storage` (if configured in the partition table).

- **boot.txt**: If this file exists, the application reads it at startup and executes each line as a console command.
- **password.txt**: Stores the Wi-Fi password. If missing, the app may prompt for it.

## Hardware Setup

- **LEDs**: Connect LEDs (with appropriate current-limiting resistors) to the GPIO pins you intend to control.
- **Inputs**: For interrupt commands, connect switches or signal sources to the input pins.

## Build and Flash

This project uses the Espressif IoT Development Framework (ESP-IDF).

1.  **Build the project**:
    ```bash
    idf.py build
    ```

2.  **Flash and Monitor**:
    ```bash
    idf.py -p PORT flash monitor
    ```
    (Replace `PORT` with your serial port name, e.g., `COM3`, `/dev/ttyUSB0`)

    To exit the monitor, type `Ctrl-]`.

## Example Usage Session

```text
> help
... list of commands ...

> level 2 1
I (1234) led: Setting pin 2 to 1

> pulse 4 1 50000
I (5678) led: Pulsing pin 4 to 1 for 50000 us

> pwm 5 1000 50
I (9101) led: Pin: 5, Frequency: 1000, Duty: 50

> interrupt 0 RISING 2 1000
I (1121) led: Setting interrupt on pin 0, edge: RISING, to pulse pin 2 for 1000 us
```