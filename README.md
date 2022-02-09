## Only works on Linux.

1. Follow [these](https://arduino.github.io/arduino-cli/getting-started/) instructions to flash your Arduino with `client.ino` using arduino-cli. Following these steps will also get you the name of the USB port used for serial communication.

2. Compile `host.c` with `gcc -O2 host.c`.

3. With Arduino still plugged in to the same port, run the program with:

    ```./a.out {name_of_usb_port} {output_filename}```

File `{output_filename}` will contain as accurate as possible timestamp measurements based on NTP with nanosecond resolution. The format of `{output_filename}` is:

```{unix_timestamp_seconds} {nanoseconds}```

Each line represents the timestamp after what is supposed to be 1 minute on the Arduino from the previous.

Measurements on the Arduino are based on exact clock ticks using overflow interrupts.
