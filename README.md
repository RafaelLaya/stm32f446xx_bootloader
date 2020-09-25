# stm32f446xx_bootloader
The following is a custom bootloader for the STM32F446xx family of microcontrollers. 

# How to use
1. Flash the bootloader as usual (At sector #0, 0x0800_0000). Configure your linker script so that the bootloader only occupies sector #0 in flash. 
2. Press the hardware reset button while holding the user button to enter bootloader mode. Otherwise (if the user button is not pressed), the bootloader will automatically launch an user application starting at sector #1 (0x0800_4000). 
3. Launch the Python host utility (baud rate is 115200, even parity, no hardware flow control, and find the correct virtual port on your computer). 
python3 utility.py -d /dev/ttyS6 -b 115200 
4. The CLI will start running and you can select options intuitively from the menu. 
5. It is recommended to use the host utility to protect sector #0 in flash from Writes. 
6. To flash a user application, adjust the linker script of the user application to start in sector #1 (0x0800_4000) and the startup code should correct the vector table offset (In most cases the default is zero, thus you will need to add 0x4000 which corresponds to the offset of sector #1). Then you can either flash in your own ways or use the bootloader to write to memory. 

An example application can be found under example/ and the bootloader is under bootloader_stm32f446xx. In addition, the host utility is under host/

# Usage Example
See usage.txt where the following steps are shown:
0. Go into bootloader mode by pressing the reset button and the user button. Then executes utility.py
1. Prints the bootloader version
2. Prints supported command codes
3. Prints The Chip ID and revision number
4. Checks RDP protection level
5. Erases sector #1 in flash
6. Writes example.bin into sector #1 in flash
7. Enables write protection for sector 0
8. Checks the protection status of each sector
9. Downloads the OTP bytes into otp.bin
10. Downloads 8612 bytes from sector #1 into example_copy.bin
11. Jumps  to the user application (example.bin). At this point you should be able to press the user button t
12. Exits
13. Checks that example.bin and example_copy.bin are equal

# Dependencies
* The target uses the following tiny printf library for debug messages to keep the size of the bootloader small (based on the value of DEBUG_MESSAGES_ENABLED which is disabled by default): https://github.com/mpaland/printf
* The host utility uses st_link utility to disable PCROP protections, so you need to export STLINK_CLI as an environment variable if you require this feature. This is because to disable PCROP you need to elevate RDP to level 1 and then back to level 0 but the device detects an invalid access while communicating with the host. The target does have the code to handle the command, but the host will not send the command to avoid this situation and will instead use the st_link utility. Enabling protection doesn't have this dependency. 
* STM32's HAL: Can be stripped to bare-metal code to minimize the size of the bootloader, but the bootloader already fits within a single sector thus there is not a lot of incentive to bring it down further. 
* Other python libraries from the host side (serial, sys, argparse, time, os, traceback)
