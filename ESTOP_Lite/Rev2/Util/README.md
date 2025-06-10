## Wireless ESTOP Setup

To avoid communication conflict between multiple Wireless ESTOP systems, users can now modify their ESTOP channel setup.

### Instructions
1. Identify the serial port name for you Wireless ESTOP device (transmitter/receiver). Plug in the device and enter the following command in terminal:
    ```bash
    ls /dev/serial/by-id
    ```
2. Unplug the device.
3. Run the program.
    ```bash
    ./setup
    ```
4. Enter the serial port name.
5. Now plug in the device again. Wait for the connection and then follow the instruction. 
6. You need to have the same channel setup for your transmitter and receiver.