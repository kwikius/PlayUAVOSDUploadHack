#pragma once

/*
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>

Authors:
   Andy Little
   Tom Ren
  
*/

#include <quan/serial_port.hpp>

// px4 bootloader codes
enum{
    // response codes
    NOP = 0x00,
    OK = 0x10,
    FAILED = 0x11,
    INSYNC = 0x12,
    INVALID = 0x13,//	# rev3+

    // protocol commands
    EOC = 0x20,
    GET_SYNC = 0x21,
    GET_DEVICE = 0x22,	// returns DEVICE_ID and FREQ bytes
    CHIP_ERASE = 0x23,
    CHIP_VERIFY = 0x24,//# rev2 only
    PROG_MULTI = 0x27,
    READ_MULTI = 0x28,//# rev2 only
    GET_CRC = 0x29,//	# rev3+
    GET_OTP = 0x2a, // read a byte from OTP at the given address
    GET_SN = 0x2b,    // read a word from UDID area ( Serial)  at the given address
    GET_CHIP = 0x2c, // read chip version (MCU IDCODE)
    REBOOT = 0x30,

    INFO_BL_REV = 1,//	# bootloader protocol revision
    BL_REV_MIN = 2,//	# minimum supported bootloader protocol
    BL_REV_MAX = 4,//	# maximum supported bootloader protocol
    INFO_BOARD_ID = 2,//	# board type
    INFO_BOARD_REV = 3,//	# board revision
    INFO_FLASH_SIZE = 4,//	# max firmware size in bytes

    PROG_MULTI_MAX = 60,//		# protocol max is 255, must be multiple of 4
    READ_MULTI_MAX = 60,//		# protocol max is 255, something overflows with >= 64

    START_TRANSFER = 0x24,      //tell the osd we will start send params
    SET_PARAMS = 0x25,           //actually send params
    GET_PARAMS = 0x26,          //recv params from osd
    END_TRANSFER = 0x28,
    SAVE_TO_EEPROM = 0x29,
};

class COSDConn{
public:
    COSDConn();
    ~COSDConn();


    void upload_firmware( std::string const & filename);
    void upload_params(std::string const & filename);
    void get_params(std::string const & filename);

private:
    void _send(uint8_t c);
    void _send( uint8_t const* arr, size_t len);
    void _recv(uint8_t * arr, size_t count = 1);
    int32_t _recv_int();
    uint32_t _recv_uint();
    void _getSync();
    void _sync();
    int32_t _get_board_max_flash_size();
    uint32_t _get_board_crc();
    void _reset_to_bootloader();
    void _reboot_to_app();
    void _erase();

    bool _try_connect();
    void _disconnect();

    bool _connected();
    void _throw_if_not_connected();

private:
    quan::serial_port* m_sp;
    bool m_good;
};

