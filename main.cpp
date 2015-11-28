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

void usage(const char* app_name)
{
    std::cout << "usage :\n";
    std::cout << "1) write firmware to PlayUAV OSD board from <from_filename>\n";
    std::cout << "      " << app_name << " -fw_w <from_filename>\n\n";
    std::cout << "2) set parameters to PlayUAV OSD board from <from_filename>\n";
    std::cout << "      " << app_name << " -pm_w <from_filename>\n\n";
    std::cout << "3) set default parameters to PlayUAV OSD board\n";
    std::cout << "      " << app_name << " -pm_w\n\n";
    std::cout << "4) get parameters from PlayUAV OSD board and save to <to_filename>\n";
    std::cout << "      " << app_name << " -pm_r <to_filename>\n\n";
    std::cout << "(TODO: read, verify, erase_all, erase_sectors, upload to eeprom)\n\n";
}

COSDConn osdconn;

int main(int argc, const char* argv[])
{
    std::cout << "\n\n";
    std::cout << "   ***************************************\n";
    std::cout << "   *                                     *\n";
    std::cout << "   *     PlayUAV OSD                     *\n";
    std::cout << "   *     Firmware Utility v1.1           *\n";
    std::cout << "   *     For documentation:              *\n";
    std::cout << "   *     visit http://www.playuav.com    *\n";
    std::cout << "   *     Copyright (c) Nov 2015          *\n ;
    std::cout << "   *     Andy Little, Tom Ren            *\n";
    std::cout << "   *                                     *\n";
    std::cout << "   ***************************************\n\n";

    if ( ( argc < 2) ) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    try{
        if(( argc == 3) && (!strncmp(argv[1], "-fw_w", 5)) ){
            osdconn.upload_firmware(argv[2]);
        }else if(!strncmp(argv[1], "-pm_w", 5)){
            if(argc == 3){
                osdconn.upload_params(argv[2]);
            }else if(argc == 2){
                osdconn.upload_params("");
            }else{
                usage(argv[0]);
                return EXIT_FAILURE;
            }
        }else if(( argc == 3) && (!strncmp(argv[1], "-pm_r", 5))){
            osdconn.get_params(argv[2]);
        }else{
            usage(argv[0]);
            return EXIT_FAILURE;
        }
    }catch(std::exception & e){
        std::cout << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
