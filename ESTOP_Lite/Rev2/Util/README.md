## Wireless ESTOP Channel Setup

To avoid communication conflict between multiple Wireless ESTOP systems, users can now modify their communication channel setup.

### Instructions
1. Identify the serial port name for you Wireless ESTOP device (transmitter/receiver). Plug in the device and enter the following command in terminal:
    ```bash
    ls /dev/serial/by-id
    ```
    Modify line 105 in ``channel_setup.py`` accordingly.
2. Unplug the device.
3. Run the python script.
    ```bash
    python3 channel_setup.py
    ```
4. Now plug in the device again. Wait for the connection and then follow the instruction. 
5. You need to have the same channel setup for your transmitter and receiver.