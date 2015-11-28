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

#include "osdconn.h"

#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <cstring>
#include <vector>
#include <quan/min.hpp>
#include <quan/utility/timer.hpp>
#include <quan/conversion/itoa.hpp>
#include <cassert>

#include "osdconn.h"
#include "params.h"

namespace px4Uploader{

int32_t crc(std::vector<uint8_t> const & bytes,int32_t padlen);
}
struct Firmware{
    std::vector<uint8_t> imagebyte;
    int32_t image_size;
    uint32_t expected_crc;
    int32_t board_flash_size;
    Firmware():image_size{0}, expected_crc{0}, board_flash_size{0}{}
};
// code for the app to request a reboot to bootlaoder
// also works in PlayUAV version of the  bootloader itself as a nop
static constexpr uint8_t PROTO_BL_UPLOAD = 0x55;

COSDConn::COSDConn()
    :m_good(false)
{
    m_sp = nullptr;
}

COSDConn::~COSDConn()
{
    if(m_sp != nullptr)
    {
        m_sp->close();
        delete m_sp;
        m_sp = nullptr;
    }
}

void COSDConn::upload_firmware( std::string const & filename)
{
    if(!_try_connect()){
        return;
    }
    //tell the app to reboot to bootloader
    _sync();
    _reset_to_bootloader();
    try {
        _sync();
    }catch (std::exception){
        // we assume usb_to_osd failed due to reset to bootloader
        _disconnect();
        std::cout << "Going down for a reboot to the bootloader...\n";
    }

    std::cout << "We were re-enumerated\n";
    if(!_try_connect()){
        return;
    }
    std:: cout << "re-enumeration OK!\n";
    //------------------
    std::cout << "erasing... (Please wait)...\n";
    _erase();
    std::cout << "OK! ... board erased\n";
    std::cout << "uploading firmware... (Please wait)...\n";

    Firmware firmware;
    firmware.board_flash_size = _get_board_max_flash_size();

    std::ifstream in( filename, std::ios_base::in | std::ios_base::binary);

    if ( !in || !in.good() ){
        throw std::runtime_error("Failed to open bin file");
    }

    // get size of bin file
    in.seekg(0,in.end);
    firmware.image_size = in.tellg();
    in.seekg(0,in.beg);

    firmware.imagebyte.resize(firmware.image_size + (firmware.image_size % 4));

    for( int i = 0; i < static_cast<int32_t>(firmware.imagebyte.size());++i){
        if(i < firmware.image_size){
            firmware.imagebyte.at(i) = in.get();
        }else{
            firmware.imagebyte.at(i) = 0xff;
        }
    }

    firmware.expected_crc = px4Uploader::crc(firmware.imagebyte,firmware.board_flash_size);

    uint8_t arr [PROG_MULTI_MAX];
    int32_t image_bytes_left = firmware.imagebyte.size();
    int32_t image_idx = 0;

    while ( (image_bytes_left > 0) ){
        int32_t const sequence_length = quan::min(static_cast<int32_t>(PROG_MULTI_MAX),image_bytes_left);

        for ( int32_t i = 0;i < sequence_length; ++i){
            arr[i] = firmware.imagebyte.at(image_idx);
            ++ image_idx;
        }
        _send(PROG_MULTI);
        _send(static_cast<uint8_t>(sequence_length));
        _send(arr,sequence_length);
        _send(EOC);
        _getSync();
        image_bytes_left -= sequence_length;
    }
    if (image_bytes_left != 0 ){
        throw std::runtime_error("infile Failed while uploading bin file");
    }

    uint32_t board_crc = _get_board_crc();
    if ( firmware.expected_crc != board_crc){
        throw std::runtime_error("In firmware upload ...file crc doesnt match uploaded crc\n");
    }else{
        std::cout << "Uploaded firmware crc matches file .. Good\n";
    }
    std::cout << "OK! ... firmware uploaded\n";
    std::cout << "rebooting the board...\n";
    _reboot_to_app();
    std::cout << "OK! ... board rebooted\n";
    std::cout << "OK! ... firmware uploaded successfully\n";
    _disconnect();
}

void COSDConn::upload_params(const std::string &filename)
{
    if(!_try_connect()){
        return;
    }

    _sync();

    COSDParam osdparams;
    uint8_t paramsbuf[PARAMS_BUF_SIZE];

    if(filename.empty()){
        std::cout << "OK! ... loading default parameters\n";
        osdparams.get_default_params(paramsbuf);
    }
    else{
        std::cout << "OK! ... loading parameters from file:" << filename << std::endl;
        if(!osdparams.load_params_from_file(filename, paramsbuf)){
            return;
        }
    }

    std::cout << "OK! ... starting send parameters to board\n";
    _send(START_TRANSFER);
    _send(EOC);
    _getSync();

    uint8_t arr [PROG_MULTI_MAX];
    int32_t params_bytes_left = PARAMS_BUF_SIZE;
    int32_t params_idx = 0;

    while ( (params_bytes_left > 0) ){
        int32_t const sequence_length = quan::min(static_cast<int32_t>(PROG_MULTI_MAX),params_bytes_left);

        for ( int32_t i = 0;i < sequence_length; ++i){
            arr[i] = paramsbuf[params_idx];
            ++ params_idx;
        }
        _send(SET_PARAMS);
        _send(static_cast<uint8_t>(sequence_length));
        _send(arr,sequence_length);
        _send(EOC);
        _getSync();
        params_bytes_left -= sequence_length;
    }

    _send(END_TRANSFER);
    _send(EOC);
    _getSync();

    _send(SAVE_TO_EEPROM);
    _send(EOC);
    _getSync();
    std::cout << "OK! ... parameters has been stored on the board\n";
}

void COSDConn::get_params(const std::string &filename)
{
    if(!_try_connect()){
        return;
    }

    _sync();

    COSDParam osdparams;
    uint8_t paramsbuf[PARAMS_BUF_SIZE];
    osdparams.get_default_params(paramsbuf);

    std::cout << "OK! ... getting parameters from board\n";
    _send(GET_PARAMS);
    _send(EOC);
    //_recv(paramsbuf, PARAMS_BUF_SIZE);

    uint8_t arr [READ_MULTI_MAX];
    int32_t params_bytes_left = PARAMS_BUF_SIZE;
    int32_t params_idx = 0;

    while ( (params_bytes_left > 0) ){
        int32_t const sequence_length = quan::min(static_cast<int32_t>(PROG_MULTI_MAX),params_bytes_left);
        _recv(arr, sequence_length);
        for ( int32_t i = 0;i < sequence_length; ++i){
            paramsbuf[params_idx] = arr[i];
            ++ params_idx;
        }
        params_bytes_left -= sequence_length;
    }
    //_getSync();   //bug - Fixme!

    std::cout << "OK! ... saving parameters to file:" << filename << std::endl;
    osdparams.store_params_to_file(filename, paramsbuf);
}

bool COSDConn::_try_connect()
{
    std::cout << "trying to connect Playuav OSD board...\n";
    std::cout << "looking for likely ports...\n";
    try{
        // assume 5 ttyACM ports for now ACM0 to ACM4
        constexpr uint32_t num_acm_ports = 5;

        std::string port_name = "";
        for ( uint8_t i = 0; i < num_acm_ports; ++i){
            char int_name [4] = {'\0'};
            quan::itoasc(i,int_name,10);
            port_name = "/dev/ttyACM" + std::string{int_name};
            try {
                m_sp = new quan::serial_port(port_name.c_str());
                m_sp->init();
                m_good = m_sp->good() && (m_sp->set_baud(115200) == 0);
                if ( m_good){
                    std::cout << "Found PlayUAV OSD on " << port_name <<'\n';
                    break;
                }
            }catch(std::exception & e){
                // any exception means try another port
                _disconnect();
            }
        }
    }catch (std::exception & e){
        std::cout << "_try_connect() with exception:" <<  e.what() << "'\n";
    }

    return true;
}

void COSDConn::_disconnect()
{
    if(m_sp != nullptr){
        m_sp->close();
        delete m_sp;
        m_sp = nullptr;
        m_good = false;
    }
}
void COSDConn::_send(uint8_t c)
{
    _throw_if_not_connected();
    m_sp->write(&c,1);
}

void COSDConn::_send( uint8_t const* arr, size_t len)
{
    _throw_if_not_connected();
    m_sp->write(arr,len) ;
}

void COSDConn::_recv(uint8_t * arr, size_t count)
{
    _throw_if_not_connected();
    if ( m_sp->read(arr,count) == -1){
        throw std::runtime_error("usb_to_osd read failed");
    }
}

int32_t COSDConn::_recv_int()
{
    union{
        uint8_t arr[4];
        int32_t i;
    } u;
    _recv(u.arr,4);
    return u.i;
}

uint32_t COSDConn::_recv_uint()
{
    union{
        uint8_t arr[4];
        uint32_t ui;
    } u;
    _recv(u.arr,4);
    return u.ui;
}

void COSDConn::_getSync()
{
    quan:: timer<> timer;

    while ( timer() < quan::time::s{7}){
        if (m_sp->in_avail() > 1 ){
            break;
        }
    }
    if ( m_sp->in_avail() < 2){
        throw std::runtime_error("get_sync : expected INSYNC");
    }

    uint8_t ch;
    _recv(& ch);
    if ( ch != INSYNC){
        throw std::runtime_error("get_sync : expected INSYNC");
    }
    _recv(&ch);
    switch (ch){
    case OK:
        return;
    case INVALID:
        throw std::runtime_error("get_sync : INVALID OPERATION");
    case FAILED:
        throw std::runtime_error("get_sync : OPERATION FAILED");
    default:
        throw std::runtime_error("get_sync : unexpected ack char");
    }
}

void COSDConn::_sync()
{
    uint8_t const sync_cmd [] = {GET_SYNC, EOC};
    _send(sync_cmd,2);
    _getSync();
}

int32_t COSDConn::_get_board_max_flash_size()
{
    uint8_t const cmd [] = { GET_DEVICE, INFO_FLASH_SIZE, EOC};
    _send(cmd,3);
    int32_t result = _recv_int();
    _getSync();
    return result;
}

uint32_t COSDConn::_get_board_crc()
{
    uint8_t const cmd [] = {GET_CRC, EOC};
    _send(cmd,2);
    uint32_t result = _recv_uint();
    _getSync();
    return result;
}


// works  in bl or app
void COSDConn::_reset_to_bootloader()
{
    uint8_t const cmd [] = {PROTO_BL_UPLOAD, EOC};
    _send(cmd,2);
    _getSync();
    quan::timer<> t;
    while (m_sp->in_avail()){
        uint8_t ch;
        _recv(&ch);
    }
    m_sp->flush();
    while (t() < quan::time::ms{1000}) {;}
    if( !m_sp->good()){
        throw std::runtime_error("reset to bootloader : couldnt connect");
    }
}

// bootloader doesnt program reset vector
// until you send the reboot command
void COSDConn::_reboot_to_app()
{
    uint8_t const cmd [] = {REBOOT, EOC};
    _send(cmd,2);
    _getSync();
}

void COSDConn::_erase()
{
    m_sp->flush();
    _sync();
    uint8_t arr []= {CHIP_ERASE,EOC};
    _send(arr,2);
    quan::timer<> t;
    while ( t() < quan::time::s{20}){
        if (m_sp->in_avail() > 0 ){
            break;
        }
    }
    _getSync();
}

bool COSDConn::_connected()
{
    return m_good && m_sp->good();
}

void COSDConn::_throw_if_not_connected()
{
    if(!_connected()){
        throw std::runtime_error("not connected");
    }
}
