# Wireless ESTOP Lite
Wireless ESTOP that works with BEAR actuators

This repo currently contains：\
**Rev1:** The open-source code for the transmitter and receiver of the Rev1 version of our Wireless ESTOP Lite system, which uses RP2040 as its MCU and NRF24 for 2.4GHz wireless communication;\
**Rev2:** The binary code of the remote transmitter and the receiver, code for changing settings of the modules, and the schematic of the Wireless ESTOP Lite modules.

## Licensing
The Wireless ESTOP is dual-licensed under both commercial and open-source licenses. The work in this repo is under GNU General Public License (GPL) version 3, which is ideal for use cases such as open-source projects with open-source distribution, student/academic purposes, hobby projects, internal research projects without external distribution, or other projects where all GPL obligations can be met. Read the full text of [the GNU GPL version 3](https://www.gnu.org/licenses/gpl-3.0.html) for details.  
Contact us for commercial license should you need full rights to create and distribute the platform on your own terms without any open-source license obligations.

## Upload and update
All modules are loaded with most recent version(at the time of shipping) of the software. When a flash or update is needed, simply connect the module to your computer while holding down the BOOT button then a new drive should pop up. 

Download then Drag and drop the corresponding .uf2 file into the drive to finish the upload.

## Receiver Features
### Buttons, Indicators and Pinout on Receiver
The RGB indicator flashes in blue when a signal disturbance is detected. The three indicators on the right shows the status of ESTOP, transmitter connection and power status of the receiver module. \
<img src="https://github.com/Westwood-Robotics/Wireless_ESTOP/blob/Rev2/Pic/FRONT.jpg" width=50% height=50%>\
Connect the **ESTOP_SIGNAL_OUTPUT** pin and the **GND** pin on the PH 2mm 3Pin port respectively to the ESTOP and GND channel on your BEAR bus. \
You can also use the **3V3_IN** pin to power the receiver module, but you need to connect the ***EXT_3V3*** jumper on the top side of the board with solder. \
<img src="https://github.com/Westwood-Robotics/Wireless_ESTOP/blob/Rev2/Pic/EXT_3V3.jpg" width=400>

Connect the two terminals on your ESTOP swith(NC switch) to the **ESTOP_SWITCH** pin and the **GND** pin on the 1.25mm 2Pin port. You can power the board by connecting the **5V** pin and the **GND** pin to a 5V power supply. \
<img src="https://github.com/Westwood-Robotics/Wireless_ESTOP/blob/Rev2/Pic/ISO.jpg" width=60%>
> [!CAUTION]
> Only power the board with a single source. Avoid co-existance of power supply from 3V3, 5V and the USB-C port.

When working with a bus that has different series of BEAR actuators, it is recommended to use the BJT on the receiver module for stable ESTOP control, and the jumper on the bottom of the board should be modified accordingly. When the BJT is not used(by default) the ***Direct*** terminal should be connected to the center pad, and if the BJT is used, the ***CTRL*** terminal should be connected to the center pad:\
<img src="https://github.com/Westwood-Robotics/Wireless_ESTOP/blob/Rev2/Pic/BJT.jpg" width=400>\
**See below for how to write BJT settings to the RX module.**
> [!Note]
> Make sure you have the correct BJT setting in the software.

## Wireless ESTOP Channel and BJT Setup
To avoid communication conflict between multiple Wireless ESTOP systems, users can now modify their communication channel setup.
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
5. Now plug in the device again. Wait for the connection and then **follow the instruction**. 
6. You need to have the same channel setup for your transmitter and receiver.


## Further DIY
### Scheme
Feel free to develop upon the hardware of the Rev2 modules. The transmitter and receiver shares the same hardware and the schematic drawing is [here](https://github.com/Westwood-Robotics/Wireless_ESTOP/blob/Rev2/SCH_WirelessESTOP_Lite_2025-06-10.pdf)
### ARDUINO IDE
To work with the RP2040 on the Wireless ESTOP modules, please first config your ARDUINO IDE with Raspberry Pi Pico setup: https://github.com/earlephilhower/arduino-pico

## 3D Model
RX Module(Receiver): https://github.com/Westwood-Robotics/Wireless_ESTOP/blob/main/ESTOP_Lite/Rev0/Dummy/ESTOP_RX.STEP

## Update Log:Rev1->Rev2
1. Pin change: the following two pins has been moved: <br />
   NRF_CE -> 19<br />
   SPI0_CSN -> 5<br />
2. Add support for one additional SPI line, pins can also be used for general purposes. (scheme/drawing to be released)
3. ESTOP signal can be sent directly or via a BJT. Select it on the board then modify the code.
4. Change settings and IDs by writting to EEPROM.
