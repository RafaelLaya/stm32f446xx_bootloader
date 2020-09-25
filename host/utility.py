#!/usr/bin/env python3

import serial 
import sys
import argparse
import time
import os
import traceback

BYTES_OF_LENGTH = 1
BYTES_OF_COMMAND_CODE = 1
BYTES_IN_PREAMBLE = BYTES_OF_LENGTH + BYTES_OF_COMMAND_CODE
BYTES_IN_CRC32 = 4
BYTES_IN_UINT32 = 4
MAX_UINT8 = 255
SMALL_DELAY = 0.1
BYTES_OF_PAYLOAD_LENGTH = 1

MAX_BYTES_PER_MEM_WRITE_PACKET = 0xFF - BYTES_OF_COMMAND_CODE - BYTES_IN_UINT32 - BYTES_IN_CRC32 - BYTES_OF_PAYLOAD_LENGTH
MAX_BYTES_PER_MEM_READ_PACKET = 0xFF - BYTES_IN_CRC32 - BYTES_OF_PAYLOAD_LENGTH

NUMBER_OF_SECTORS = 8

TIMEOUT = 10
NUM_OF_RETRIES = 10

SUPPORTED_COMMANDS = ['1', '2', '3', '4', '5', '6', '7', '8', '9', '10', '11', '12', '13', '14']

BL_COMMAND_GET_VERSION = 0x51
BL_COMMAND_GET_HELP = 0x52
BL_COMMAND_GET_CHIP_ID = 0x53
BL_COMMAND_GET_RDP_STATUS = 0x54
BL_COMMAND_GO_TO_ADDR = 0x55
BL_COMMAND_FLASH_ERASE = 0x56
BL_COMMAND_MEM_WRITE = 0x57
BL_COMMAND_EN_RW_PROTECT = 0x58
BL_COMMAND_DIS_RW_PROTECTION = 0x59
BL_COMMAND_READ_SECTOR_STATUS = 0x5A
BL_COMMAND_MEM_READ = 0x5B
BL_COMMAND_GO_TO_APP = 0x5C

BL_SUPPORTED_COMMAND_CODES = {
    BL_COMMAND_GET_VERSION: "BL_COMMAND_GET_VERSION",
    BL_COMMAND_GET_HELP: "BL_COMMAND_GET_HELP",
    BL_COMMAND_GET_CHIP_ID: "BL_COMMAND_GET_CID",
    BL_COMMAND_GET_RDP_STATUS: "BL_COMMAND_GET_RDP_STATUS",
    BL_COMMAND_GO_TO_ADDR: "BL_COMMAND_GO_TO_ADDR",
    BL_COMMAND_FLASH_ERASE: "BL_COMMAND_FLASH_ERASE",
    BL_COMMAND_MEM_WRITE: "BL_COMMAND_MEM_WRITE",
    BL_COMMAND_EN_RW_PROTECT: "BL_COMMAND_EN_RW_PROTECT",
    BL_COMMAND_DIS_RW_PROTECTION: "BL_COMMAND_DIS_RW_PROTECTION",
    BL_COMMAND_READ_SECTOR_STATUS: "BL_COMMAND_READ_SECTOR_STATUS",
    BL_COMMAND_MEM_READ: "BL_COMMAND_MEM_READ",
    BL_COMMAND_GO_TO_APP: "BL_COMMAND_GO_TO_APP"
}

ACK = 0xA5
STATUS_SUCCESS = 0xA6 
BL_PROTECTION_CODE_RW = 0xC1
BL_PROTECTION_CODE_W = 0xC2

def clear_serial_port():
    ser.reset_input_buffer()
    ser.reset_output_buffer()

def sync():
    zero = bytearray()
    zero.append(0)

    ser.timeout = 0
    for i in range(MAX_UINT8 * 2):
        ser.read(1)

    for i in range(MAX_UINT8 * 2):
        ser.write(0)
    ser.timeout = TIMEOUT

def send_data(data):
    time.sleep(SMALL_DELAY)
    if is_verbose:
        print("Will try to send:")
        print("[", end="")
        for i in range(len(data) - 1):
            print("{}, ".format(hex(data[i])), end="")

        print("{}]".format(hex(data[-1])))
    
    bytes_sent= ser.write(data)

    if is_verbose:
        print("Sent {} byte(s)\n".format(bytes_sent))


def receive_data(length):
    data = ser.read(length)

    if is_verbose:
        print("Received:")
        print("[", end="")
        for i in range(len(data) - 1):
            print("{}, ".format(hex(data[i])), end="")
        print("{}]".format(hex(data[-1])))
    
    return data

def calculate_crc(buff, length):
    Crc = 0xFFFFFFFF
    for data in buff[0:length]:
        Crc = Crc ^ data
        for i in range(BYTES_IN_CRC32 * 8):
            if(Crc & 0x80000000):
                Crc = (Crc << 1) ^ 0x04C11DB7
            else:
                Crc = (Crc << 1)
    
    Crc = Crc & 0xFFFFFFFF
    if is_verbose:
        print("Calculated CRC is {}".format(hex(Crc)))
        
    return Crc

def send_length_to_follow(full_length):
    if is_verbose:
        print("Length to follow is {}".format(full_length - BYTES_OF_LENGTH))

    data = bytearray()
    data.append(full_length - BYTES_OF_LENGTH)
    send_data(data)

def make_preamble(full_length, command_code):
    if is_verbose:
        print("Command code is {}".format(hex(command_code)))
    
    preamble = bytearray()
    preamble.append(full_length - BYTES_OF_LENGTH)
    preamble.append(command_code)
    return preamble

def append_crc32(data, crc32):
    data.append(crc32 & 0xFF)
    data.append((crc32 >> 8) & 0xFF)
    data.append((crc32 >> 16) & 0xFF)
    data.append((crc32 >> 24) & 0xFF)

def add_crc32(data, full_length):
    crc32 = calculate_crc(data, full_length - BYTES_IN_CRC32)
    append_crc32(data, crc32)

def add_crc32_strip_and_send(data, full_length):
    add_crc32(data, full_length)
    
    # Length to follow has already been sent
    data.pop(0)
    send_data(data)

def handle_ack_nack(reply_full_length):
    # Receive ACK or NACK
    data = receive_data(1)

    if (data[0] != ACK):
        if is_verbose:
            print("Didn't get ACK. Got {} instead\n".format(hex(data[0])))
        return 1

    if is_verbose:
        print("Got ACK\n")

    return 0

def get_reply():
    reply = bytearray(receive_data(BYTES_OF_LENGTH))

    reply_length = 0
    for i in range(BYTES_OF_LENGTH):
        reply_length += reply[i]

    if is_verbose:
        print("Reply will consist of {} bytes\n".format(hex(reply_length)))
    reply.extend(bytearray(receive_data(reply_length)))

    # Calculate the CRC starting from length to follow, but omit the last four since
    # that's the CRC calculated by the host
    crc_calculated = calculate_crc(reply, reply_length + BYTES_OF_LENGTH - BYTES_IN_CRC32)
    crc_received =  (reply[-1] << 24)  | (reply[-2] << 16)  | (reply[-3] << 8) | (reply[-4] << 0)

    if is_verbose:
        print("Received CRC is {}".format(hex(crc_received)))

    if crc_calculated != crc_received:
        print("There has been a CRC error. Please try again.")
        print("Calculated {} but got {}\n".format(crc_calculated, crc_received))
        return reply, 1
    elif is_verbose:
        print("CRC Check Successful\n")
    
    return reply, 0

def process_command(command_code, command_data):
    num_retries = NUM_OF_RETRIES + 1

    while num_retries > 0:
        full_length = BYTES_IN_PREAMBLE + len(command_data) + BYTES_IN_CRC32

        send_length_to_follow(full_length)
        data = make_preamble(full_length, command_code)

        command_data = bytearray(command_data)
        data.extend(command_data)

        add_crc32_strip_and_send(data, full_length)

        if handle_ack_nack(full_length) != 0:
            num_retries -= 1
            continue

        reply, error_code = get_reply()

        if error_code != 0:
            num_retries -= 1
            continue

        return reply

    raise Exception("Exceeded number of retries") 

def process_get_version_command():
    empty = bytearray()
    reply = process_command(BL_COMMAND_GET_VERSION, empty)
    print("The Bootloader version is {}.{}".format(str(reply[BYTES_OF_LENGTH]), str(reply[BYTES_OF_LENGTH + 1])))

def process_get_help_command():
    empty = bytearray()
    reply = process_command(BL_COMMAND_GET_HELP, empty)

    for i in range(BYTES_OF_LENGTH, len(reply) - BYTES_IN_CRC32):
        command = reply[i]
        if command in BL_SUPPORTED_COMMAND_CODES:
            print("Command {}:{} is supported".format(hex(command), BL_SUPPORTED_COMMAND_CODES[command]))
        else:
            print("Command Code {} is supported by the target, but is unknown to the host".format(hex(command)))

def process_get_chip_id_command():
    empty = bytearray()
    reply = process_command(BL_COMMAND_GET_CHIP_ID, empty)
    print("The chip id is {}".format(hex(reply[BYTES_OF_LENGTH] | (reply[BYTES_OF_LENGTH + 1] << 8))))
    print("The revision id is {}".format(hex(reply[BYTES_OF_LENGTH + 2] | (reply[BYTES_OF_LENGTH + 3] << 8))))

def process_get_rdp_status_command():
    empty = bytearray()
    reply = process_command(BL_COMMAND_GET_RDP_STATUS, empty)
    
    rdp_status = reply[1]

    if rdp_status == 0xAA:
        print("There is no Read Protection")
    elif rdp_status == 0xCC:
        print("Level 2, chip protection (debug and boot from RAM features disabled)")
    else:
        print("Level 1, read protection of memories (debug features limited)")

def process_go_to_addr_command(address):
    data = address.to_bytes(BYTES_IN_UINT32, byteorder='little')

    if is_verbose:
        print("Will try to jump to {}".format(hex(address)))
        print("In bytes, this is {}".format(data))

    reply = process_command(BL_COMMAND_GO_TO_ADDR, data)
    
    status = reply[BYTES_OF_LENGTH]

    if status == STATUS_SUCCESS:
        print("Jump was executed")
    else:
        print("The jump was not executed")

def process_flash_erase(initial, number):
    data = bytearray()
    initial = initial.to_bytes(1, byteorder='little')
    number = number.to_bytes(1, byteorder='little')
    data.extend(initial)
    data.extend(number)

    reply = process_command(BL_COMMAND_FLASH_ERASE, data)

    status = reply[BYTES_OF_LENGTH]

    if status == STATUS_SUCCESS:
        print("Erase was executed")
    else:
        print("Erase could not be executed. Please check if the region is protected (PCROP, Write, or RDP with respective security level)")

def process_mass_erase():
    # We will lose the connection to the bootloader if successful
    try:
        process_flash_erase(0xFF, 8)
        print("Erase might not have been executed")
    except Exception:
        print("\nErase has been executed\n")

def process_memory_write(base_addr, path):
    with open(path, "rb") as binary_file:
        remaining_length = os.fstat(binary_file.fileno()).st_size
        while remaining_length > 0:
            data = bytearray()
            bytes_in_next_package = min(MAX_BYTES_PER_MEM_WRITE_PACKET, remaining_length)

            # base addr
            data.extend(base_addr.to_bytes(4, byteorder="little"))

            # payload length
            data.extend(int(bytes_in_next_package).to_bytes(1, byteorder="little"))

            # payload 
            data.extend(binary_file.read(bytes_in_next_package))
            remaining_length -= bytes_in_next_package
            base_addr += bytes_in_next_package

            reply = process_command(BL_COMMAND_MEM_WRITE, data)

            num_retries = 0
            while reply[BYTES_OF_LENGTH] != STATUS_SUCCESS and num_retries < NUM_OF_RETRIES: 
                if is_verbose:
                    print("No success. Will try again. This is re-try number {}".format(num_retries + 1))
                reply = process_command(BL_COMMAND_MEM_WRITE, data)
                num_retries += 1

            if reply[BYTES_OF_LENGTH] != STATUS_SUCCESS:
                msg = "Target has reported that writing was not successful. Remember to always erase the respective sectors first."
                raise Exception(msg)
        
        print("Data has been uploaded")

def process_memory_read(address, length, path):
    with open(path, "wb") as binary_file:       
        while length > 0:
            data = bytearray()
            length_of_next_packet = length
            
            if is_verbose:
                print("Length of next package will be {}".format(length_of_next_packet))

            if length_of_next_packet > MAX_BYTES_PER_MEM_READ_PACKET:
                length_of_next_packet = MAX_BYTES_PER_MEM_READ_PACKET
            
            data.extend(address.to_bytes(4, byteorder="little"))
            data.extend(length_of_next_packet.to_bytes(1, byteorder="little"))

            reply = process_command(BL_COMMAND_MEM_READ, data)

            # strip length and CRC
            reply.pop(0)
            reply.pop(-1)
            reply.pop(-1)
            reply.pop(-1)
            reply.pop(-1)

            if len(reply) == 0:
                raise Exception("Reply had no payload. The pair (addr:{} len:{}) might not be valid.".format(address, length))
            
            if is_verbose:
                print("Will store {} which corresponds to {} bytes".format(reply, len(reply))) 

            length -= len(reply)
            address += len(reply)
            binary_file.write(reply)
            
        
    print("Data has been downloaded")


def get_enable_protection_args():
    print("Select the protection type:")
    print("\t1. Read-write protection")
    print("\t2. Write protection")
    protection_type = int(input("Protection type > "))
    if protection_type < 1 or protection_type > 2:
        raise Exception("Invalid protection type")

    number_of_sectors = int(input("Enter the number of sectors to protect > "))
    if number_of_sectors > NUMBER_OF_SECTORS:
        print("You have been corrected: The maximum number of sectors is {}".format(number_of_sectors))
        number_of_sectors = NUMBER_OF_SECTORS
    elif number_of_sectors < 0:
        print("You have been corrected: The minimum number of sectors is 0")
        number_of_sectors = 0

    sector_mask = 0
    for i in range(number_of_sectors):
        sector_number = int(input("Which is the sector {} to protect (Choose a value inside [0, {}])? > ".format(i, 
                                                                                                        NUMBER_OF_SECTORS - 1)))
        if sector_number > NUMBER_OF_SECTORS - 1:
            print("You have been corrected: The maximum sector number is {}".format(NUMBER_OF_SECTORS - 1))
            sector_number = NUMBER_OF_SECTORS - 1
        elif sector_number < 0:
            print("You have been corrected: The minimum sector number is 0")
            sector_number = 0
        sector_mask = sector_mask | (0x1 << sector_number)

    return sector_mask, protection_type

def process_enable_protection(sector_mask, protection_type):
    if is_verbose:
        print("The value of sector_mask is {}".format(hex(sector_mask)))
        print("The value of protection_type is {}".format(protection_type))
    data = bytearray()
    data.extend((sector_mask & 0xFF).to_bytes(1, "little"))
    
    protection_code = 0
    if protection_type == 1:
        protection_code = BL_PROTECTION_CODE_RW
    else:
        protection_code = BL_PROTECTION_CODE_W
    
    if is_verbose:
        print("The value of protection_code is {}".format(hex(protection_code)))

    data.extend((protection_code & 0xFF).to_bytes(1, "little"))

    if is_verbose:
        print("Thus data is {}".format(data))

    reply = process_command(BL_COMMAND_EN_RW_PROTECT, data)
    
    status = reply[BYTES_OF_LENGTH]

    if status == STATUS_SUCCESS:
        print("Protection was enabled")
    else:
        print("Protection was not enabled")

def process_disable_rw_protection():
    os.system(r'"$STLINK_CLI" -OB RDP=0 SPRMOD=0 WRP=0xFF')

def process_go_to_app(address):
    data = bytearray()
    data.extend(address.to_bytes(BYTES_IN_UINT32, byteorder="little"))
    reply = process_command(BL_COMMAND_GO_TO_APP, data) 

    status = reply[BYTES_OF_LENGTH]

    if status == STATUS_SUCCESS:
        print("Command was executed")
    else:
        print("Command was not executed satisfactorily. Check the base address provided: {}".format(hex(address)))

def process_read_protection_status():
    empty = bytearray()
    reply = process_command(BL_COMMAND_READ_SECTOR_STATUS, empty)

    protection_reg = (reply[BYTES_OF_LENGTH + 0] << 0)  | (reply[BYTES_OF_LENGTH + 1] << 8)  | \
                     (reply[BYTES_OF_LENGTH + 2] << 16) | (reply[BYTES_OF_LENGTH + 3] << 24)         

    if is_verbose:
        print("protection_reg is {}\n".format(hex(protection_reg)))

    protection_mode = (protection_reg >> 31) & 0x1
    sector_mask = (protection_reg >> 16) & 0xFF
    rdp = (protection_reg >> 8) & 0xFF

    if rdp == 0xAA:
        level = 0
    elif rdp == 0xCC:
        level = 2
    else:
        level = 1

    print("===== Protection Status =====")
    print("RDP Protection Level is {}".format(level))
    if protection_mode: print("Protection mode is PCROP")
    else: print("Protection Mode is Write Protection")

    for i in range(8):
        if (protection_mode):
            if ((sector_mask >> i) & 0x1):
                print("Sector {} is Read/Write protected".format(i))
            else:
                print("Sector {} is not Read/Write protected".format(i))
        else:
            if ((sector_mask >> i) & 0x1):
                print("Sector {} is not Write protected".format(i))
            else:
                print("Sector {} is Write protected".format(i))

def parse_args():
    global args
    parser = argparse.ArgumentParser(description=__doc__,
                                    formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("-d", "--serial_port", required=True,
                        help="serial port")
    parser.add_argument("-b", "--serial_baudrate", required=True,
                        help="serial baudrate")
    parser.add_argument("-v", "--verbose", required=False, help="Verbose Mode", action="store_true")
    args = parser.parse_args()

def main():
    parse_args()
    serial_port = args.serial_port
    serial_baudrate = args.serial_baudrate
    global is_verbose
    is_verbose = args.verbose

    global ser
    try:
        ser = serial.Serial(serial_port, baudrate=serial_baudrate, parity=serial.PARITY_EVEN, timeout=TIMEOUT, write_timeout=TIMEOUT)
        ser.isOpen()
    except serial.SerialException as e:
        sys.exit("{}".format(e))

    print("Success opening the serial port")

    while True:
        print("\n")
        print("+============================================================+")
        print("|                           Menu                             |")
        print("|          STM32F4 BootLoader Host Utility Version 1.0       |")
        print("+============================================================+")
        print("\n")

        print("Choose a command: ")
        print("\t0. Exit")
        print("\t1. Get Version")
        print("\t2. Get Help")
        print("\t3. Get Chip ID")
        print("\t4. Get RDP Status")
        print("\t5. Go to Address")
        print("\t6. Flash Erase")
        print("\t7. Flash Mass Erase")
        print("\t8. Memory Write")
        print("\t9. Enable RW Protection")
        print("\t10. Disable RW Protection")
        print("\t11. Read Sector Protection Status")
        print("\t12. Memory Read")
        print("\t13. Read OTP")
        print("\t14. Go to an application whose base address is known")

        print("Enter your command: ")

        sync()
        clear_serial_port()

        command = input("> ")

        if (not command in SUPPORTED_COMMANDS) and (command != "0"):
            print("Command \"{}\" is not valid. Please choose one of {}".format(command, SUPPORTED_COMMANDS))
            input("Press any key to continue...")
            continue
        
        try:
            if command == "0":
                sys.exit("Exiting...")
            elif command == "1":
                process_get_version_command()
            elif command == "2":
                process_get_help_command()
            elif command == "3":
                process_get_chip_id_command()
            elif command == "4":
                process_get_rdp_status_command()
            elif command == "5":
                address = int(input("Enter address to jump > "), base=16)
                process_go_to_addr_command(address)
            elif command == "6":
                initial = int(input("Enter first sector to erase (0-7)> "))
                number_of_sectors = int(input("Number of sectors to erase (1 <= initial+number <= 8) > "))
                process_flash_erase(initial, number_of_sectors)
            elif command == "7":
                process_mass_erase()
            elif command == "8":
                base_addr = int(input("Enter base address to write the binary file > "), base=16)
                path = os.path.abspath(input("Enter path to binary file > "))
                process_memory_write(base_addr, path)
            elif command == "9":
                sector_mask, protection_type = get_enable_protection_args()
                if is_verbose:
                    print("sector_mask = {}", sector_mask)
                    print("protection_type = {}", protection_type)
                process_enable_protection(sector_mask, protection_type)
            elif command == "10":
                process_disable_rw_protection()
            elif command == "11":
                process_read_protection_status()
            elif command == "12":
                address = int(input("Enter address > "), base=16) 
                length = int(input("Enter length > "))
                path = input("Enter path for storage > ")
                process_memory_read(address, length, path)
            elif command == "13":
                address = 0x1FFF7800
                length = 16 * 32 + 16
                path = input("Enter path for storage > ")
                process_memory_read(address, length, path)
            elif command == "14":
                address = int(input("Enter address > "), base=16) 
                process_go_to_app(address)
            else:
                raise Exception("Internal Error")
        except Exception as e:
            if is_verbose:
                print(e)
                traceback.print_exc()
            print("An error has occurred while executing your command. Please try again. Your command was \"{}\"".format(command))
        
        input("Press any key to continue...")


if __name__ == "__main__":
    main()
