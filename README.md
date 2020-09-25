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
See usage.txt

