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

#include<string>
#include <map>

#define PARAMS_BUF_SIZE 1024

typedef std::map<std::string, int32_t> ParamsAddrMap;

class COSDParam{
public:
    COSDParam();
    ~COSDParam();

    bool load_params_from_file(const std::string & filename, uint8_t * buf_in);
    bool store_params_to_file(const std::string & filename, uint8_t * buf_in);

    void get_default_params(uint8_t * buf_in);
    void dump_params(uint8_t * buf);

private:
    void m_init_params();
    void m_set_params_default(const std::string & paramname, int32_t addr, uint16_t initval);
    void m_u16_to_buf(uint8_t * buf, int32_t addr, uint16_t val);
    void m_str_to_buf(uint8_t * buf, const std::string & paramname, const std::string & paramvalue);
    uint16_t m_get_u16_param(uint8_t * buf, int32_t addr);
    std::string m_param_serialize(uint8_t * buf, int32_t addr, const std::string & paramname);
    uint16_t m_panel_str_to_u16(const std::string paramvalue);


    ParamsAddrMap m_params_addr;
    uint8_t m_default_params[PARAMS_BUF_SIZE];
    const uint16_t m_firmware_version;
    const uint16_t m_protocol_type;
};
