
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

#include "params.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <math.h>
#include <list>

COSDParam::COSDParam():
    m_firmware_version{10},
    m_protocol_type{0}
{
    memset((char *)m_default_params, 0, PARAMS_BUF_SIZE);
    m_init_params();
}

COSDParam::~COSDParam()
{

}

bool COSDParam::load_params_from_file(const std::string &filename, uint8_t * buf_in)
{
    std::ifstream in( filename, std::ios_base::in);

    if ( !in || !in.good() ){
        std::cout << "Failed to open parameters file:" << filename << std::endl;
        return false;
    }

    in.seekg(0, std::ios::beg);
    while(in.peek() != EOF){
        char buf[256] = {0};
        in.getline(buf, 256);
        std::string strLine(buf);
        std::string strParamName = "";
        std::string strParamValue = "";

        int pos = strLine.find('=');
        if(pos != -1){
            try {
                strParamName = strLine.substr(0,pos);
                strParamValue = strLine.substr(pos+1,strLine.length()-1);
                m_str_to_buf(buf_in, strParamName, strParamValue);
            }catch(std::exception & e){
                std::cout << "Bad parameter item: " << strLine << std::endl;
                return false;
            }

        }
        else{
            std::cout << "Bad parameter item: " << strLine;
            return false;
        }
    }

    return true;
}

bool COSDParam::store_params_to_file(const std::string &filename, uint8_t * buf_in)
{
    std::ofstream fo;
    fo.open(filename);

    if ( !fo.is_open()){
        std::cout << "Failed to create parameters file:" << filename << std::endl;
        return false;
    }

    ParamsAddrMap::iterator iterParamsAddr;
    for(iterParamsAddr = m_params_addr.begin(); iterParamsAddr != m_params_addr.end(); iterParamsAddr++){
        if((iterParamsAddr->first).find("Misc_Start_Col_Sign") != std::string::npos) continue;
        if((iterParamsAddr->first).find("Altitude_Scale_Source") != std::string::npos) continue;    //not-used
        if((iterParamsAddr->first).find("Speed_Scale_Source") != std::string::npos) continue;    //not-used
        if((iterParamsAddr->first).find("Attitude_MP_Scale_Frac") != std::string::npos) continue;
        if((iterParamsAddr->first).find("Attitude_3D_Scale_Frac") != std::string::npos) continue;
        if((iterParamsAddr->first).find("Attitude_MP_Mode") != std::string::npos) continue;         //not-used
        if((iterParamsAddr->first).find("Misc_Firmware_ver") != std::string::npos) continue;   //not allowed modify

        fo << m_param_serialize(buf_in, iterParamsAddr->second, iterParamsAddr->first);
    }

    fo.flush();
    fo.close();

    return true;
}

void COSDParam::get_default_params(uint8_t *buf_in)
{
    memcpy(buf_in, m_default_params, PARAMS_BUF_SIZE);
}

void COSDParam::m_set_params_default(const std::string &paramname, int32_t addr, uint16_t initval)
{
    m_params_addr[paramname] = addr;
    m_u16_to_buf(m_default_params, addr, initval);
}

void COSDParam::m_u16_to_buf(uint8_t * buf, int32_t addr, uint16_t val)
{
    buf[addr] = (uint8_t)(val & 0xFF);
    buf[addr+1] = (uint8_t)((val >> 8) & 0xFF);
}

void COSDParam::m_str_to_buf(uint8_t *buf, const std::string &paramname, const std::string &paramvalue)
{
    try {
        int32_t paramAddr = 0;
        ParamsAddrMap::iterator iterParamsAddr;

        //panel : we store the panel using ',' as sperator in files for readable,
        //        but we store it on osd board as single value
        //    ex: 1,2,3 ====> 2^(1-1) + 2^(2-1) + 2^(3-1)=7
        //      : 2,4   ====> 2^(2-1) + 2^(4-1) = 10
        if((paramname.find("_Panel") != std::string::npos) &&
           (paramname.find("PWM") == std::string::npos)    &&
           (paramname.find("Max_Panels") == std::string::npos)){
            iterParamsAddr = m_params_addr.find(paramname);
            if(iterParamsAddr != m_params_addr.end()){
                paramAddr = iterParamsAddr->second;
                uint16_t nval = m_panel_str_to_u16(paramvalue);
                m_u16_to_buf(buf, paramAddr, nval);
            }
            return;
        }

        //Scale : we store the scale as float, and we use uint16_t in buffer for alignment.
        //        hence, we stroe the real and frac into two uint16_t
        if((paramname.find("Attitude_") != std::string::npos) &&
           (paramname.find("_Scale") != std::string::npos)){
            int ndp = paramvalue.find('.');
            uint16_t nval_real = 0;
            uint16_t nval_frac = 0;
            if(ndp != -1){
                nval_real = atoi(paramvalue.substr(0,ndp).c_str());
                nval_frac = atoi(paramvalue.substr(ndp+1,paramvalue.length()-1).c_str());
            }
            else{
                nval_real = atoi(paramvalue.c_str());
            }
            iterParamsAddr = m_params_addr.find(paramname + "_Real");
            if(iterParamsAddr != m_params_addr.end()){
                paramAddr = iterParamsAddr->second;
                m_u16_to_buf(buf, paramAddr, nval_real);
            }

            iterParamsAddr = m_params_addr.find(paramname + "_Frac");
            if(iterParamsAddr != m_params_addr.end()){
                paramAddr = iterParamsAddr->second;
                m_u16_to_buf(buf, paramAddr, nval_frac);
            }
            return;
        }

        //Misc_Start_Row : don't allow negative value
        if((paramname.find("Misc_Start_Row") != std::string::npos)){
            iterParamsAddr = m_params_addr.find(paramname);
            if(iterParamsAddr != m_params_addr.end()){
                paramAddr = iterParamsAddr->second;
                uint16_t nval = abs(atoi(paramvalue.c_str()));
                m_u16_to_buf(buf, paramAddr, nval);
            }
            return;
        }

        //Misc_Start_Col : allow negative value, store the sign and value seperately
        if((paramname.find("Misc_Start_Col") != std::string::npos)){
            int nval = atoi(paramvalue.c_str());
            uint16_t absval = abs(nval);
            uint16_t nsign = (nval < 0) ? 0 : 1;

            iterParamsAddr = m_params_addr.find(paramname);
            if(iterParamsAddr != m_params_addr.end()){
                paramAddr = iterParamsAddr->second;
                m_u16_to_buf(buf, paramAddr, absval);
            }

            iterParamsAddr = m_params_addr.find(paramname + "_Sign");
            if(iterParamsAddr != m_params_addr.end()){
                paramAddr = iterParamsAddr->second;
                m_u16_to_buf(buf, paramAddr, nsign);
            }
            return;
        }

        //It is a normal uint16_t value
        iterParamsAddr = m_params_addr.find(paramname);
        if(iterParamsAddr != m_params_addr.end())
        {
            paramAddr = iterParamsAddr->second;
            uint16_t nval = atoi(paramvalue.c_str());
            m_u16_to_buf(buf, paramAddr, nval);
        }

    }catch(std::exception & e){
        std::cout << paramname << "=" << paramvalue << "with exception:" << e.what() << std::endl;
    }
}

uint16_t COSDParam::m_get_u16_param(uint8_t *buf, int32_t addr)
{
    uint16_t ret;

    try{
        uint16_t u1 = static_cast<uint16_t>(buf[addr]);
        uint16_t u2 = static_cast<uint16_t>(buf[addr+1]);
        ret = static_cast<uint16_t>(u1 + (u2 << 8));
    }
    catch(std::exception){
        return 0;
    }
    return ret;
}

std::string COSDParam::m_param_serialize(uint8_t *buf, int32_t addr, const std::string &paramname)
{
    std::string strret = "";
    uint16_t realvalue = 0;

    try{
        //panel : we store the panel using ',' as sperator in files for readable,
        //        but we store it on osd board as single value
        //    ex: 1,2,3 ====> 2^(1-1) + 2^(2-1) + 2^(3-1)=7
        //      : 2,4   ====> 2^(2-1) + 2^(4-1) = 10
        if((paramname.find("_Panel") != std::string::npos) &&
           (paramname.find("PWM") == std::string::npos)    &&
           (paramname.find("Max_Panels") == std::string::npos)){
            realvalue = m_get_u16_param(buf, addr);
            if(realvalue == 0){
                strret = "0";
            }
            else if(realvalue == 1){
                strret = "1";
            }
            else{
                std::list<int> vals;
                std::list<int>::iterator it;
                for(int i=1; i<10; i++){
                    int b = realvalue & static_cast<int>(pow(2.0f, i-1));
                    if(b != 0)  vals.push_back(i);
                }
                vals.sort();
                for (it=vals.begin(); it!=vals.end(); ++it){
                    strret += std::to_string(*it) + ",";
                }
                strret = strret.substr(0, strret.length()-1);
            }

            return paramname + "=" + strret + "\n";
        }

        //Scale : we store the scale as float, and we use uint16_t in buffer for alignment.
        //        hence, we stroe the real and frac into two uint16_t
        if((paramname.find("Attitude_") != std::string::npos) &&
           (paramname.find("_Scale") != std::string::npos)){
            realvalue = m_get_u16_param(buf, addr);
            uint16_t fracvalue = m_get_u16_param(buf, addr+2);
            strret = std::to_string(realvalue) + "." + std::to_string(fracvalue);
            return paramname.substr(0,(paramname.rfind("_"))) + "=" + strret + "\n";;
        }

        //Misc_Start_Col : allow negative value, store the sign and value seperately
        if((paramname.find("Misc_Start_Col") != std::string::npos)){
            realvalue = m_get_u16_param(buf, addr);
            uint16_t signvalue = 0;
            ParamsAddrMap::iterator iterParamsAddr;

            iterParamsAddr = m_params_addr.find("Misc_Start_Col_Sign");
            if(iterParamsAddr != m_params_addr.end()){
                signvalue = m_get_u16_param(buf, iterParamsAddr->second);
            }

            if(signvalue == 0){
                //negative
                strret = "-" + std::to_string(realvalue);
            }
            else{
                //positive
                strret = std::to_string(realvalue);
            }

            return paramname + "=" + strret + "\n";
        }

        //normal uint16_t value
        realvalue = m_get_u16_param(buf, addr);
        strret = std::to_string(realvalue);

    }
    catch(std::exception & e){
        std::cout << "param_serialize() with :" << e.what() << std::endl;
        return strret;
    }

    return paramname + "=" + strret + "\n";
}

uint16_t COSDParam::m_panel_str_to_u16(const std::string paramvalue)
{
    uint16_t nret = 0;
    int ntmp = 0;
    try{
        std::istringstream ss(paramvalue);
        std::string s;
        while(getline(ss, s, ',')){
            ntmp = atoi(s.c_str());
            nret += static_cast<uint16_t>(pow(2.0f, ntmp-1));
        }

    }
    catch(std::exception){
        std::cout << "Bad parameter item: " << paramvalue << std::endl;
        return 0;
    }

    return nret;
}

void COSDParam::dump_params(uint8_t * buf)
{
    ParamsAddrMap::iterator iterParamsAddr;
    for(iterParamsAddr = m_params_addr.begin(); iterParamsAddr != m_params_addr.end(); iterParamsAddr++){
        std::cout << iterParamsAddr->first;
        std::cout << ":";
        std::cout << m_get_u16_param(buf, iterParamsAddr->second);
        std::cout << std::endl;
    }
}

void COSDParam::m_init_params()
{
    m_params_addr.clear();
    int32_t address = 0;

    m_set_params_default("ArmState_Enable",address, 1); address += 2;
    m_set_params_default("ArmState_Panel",address, 1); address += 2;
    m_set_params_default("ArmState_H_Position",address, 350); address += 2;
    m_set_params_default("ArmState_V_Position",address, 44); address += 2;
    m_set_params_default("ArmState_Font_Size",address, 0); address += 2;
    m_set_params_default("ArmState_H_Alignment",address, 2); address += 2;

    m_set_params_default("BatteryVoltage_Enable",address, 1); address += 2;
    m_set_params_default("BatteryVoltage_Panel",address, 1); address += 2;
    m_set_params_default("BatteryVoltage_H_Position",address, 350); address += 2;
    m_set_params_default("BatteryVoltage_V_Position",address, 4); address += 2;
    m_set_params_default("BatteryVoltage_Font_Size",address, 0); address += 2;
    m_set_params_default("BatteryVoltage_H_Alignment",address, 2); address += 2;

    m_set_params_default("BatteryCurrent_Enable",address, 1); address += 2;
    m_set_params_default("BatteryCurrent_Panel",address, 1); address += 2;
    m_set_params_default("BatteryCurrent_H_Position",address, 350); address += 2;
    m_set_params_default("BatteryCurrent_V_Position",address, 14); address += 2;
    m_set_params_default("BatteryCurrent_Font_Size",address, 0); address += 2;
    m_set_params_default("BatteryCurrent_H_Alignment",address, 2); address += 2;

    m_set_params_default("BatteryRemaining_Enable",address, 1); address += 2;
    m_set_params_default("BatteryRemaining_Panel",address, 1); address += 2;
    m_set_params_default("BatteryRemaining_H_Position",address, 350); address += 2;
    m_set_params_default("BatteryRemaining_V_Position",address, 24); address += 2;
    m_set_params_default("BatteryRemaining_Font_Size",address, 0); address += 2;
    m_set_params_default("BatteryRemaining_H_Alignment",address, 2); address += 2;

    m_set_params_default("FlightMode_Enable",address, 1); address += 2;
    m_set_params_default("FlightMode_Panel",address, 1); address += 2;
    m_set_params_default("FlightMode_H_Position",address, 350); address += 2;
    m_set_params_default("FlightMode_V_Position",address, 54); address += 2;
    m_set_params_default("FlightMode_Font_Size",address, 1); address += 2;
    m_set_params_default("FlightMode_H_Alignment",address, 2); address += 2;

    m_set_params_default("GPSStatus_Enable",address, 1); address += 2;
    m_set_params_default("GPSStatus_Panel",address, 1); address += 2;
    m_set_params_default("GPSStatus_H_Position",address, 0); address += 2;
    m_set_params_default("GPSStatus_V_Position",address, 230); address += 2;
    m_set_params_default("GPSStatus_Font_Size",address, 0); address += 2;
    m_set_params_default("GPSStatus_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPSHDOP_Enable",address, 1); address += 2;
    m_set_params_default("GPSHDOP_Panel",address, 1); address += 2;
    m_set_params_default("GPSHDOP_H_Position",address, 70); address += 2;
    m_set_params_default("GPSHDOP_V_Position",address, 230); address += 2;
    m_set_params_default("GPSHDOP_Font_Size",address, 0); address += 2;
    m_set_params_default("GPSHDOP_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPSLatitude_Enable",address, 1); address += 2;
    m_set_params_default("GPSLatitude_Panel",address, 1); address += 2;
    m_set_params_default("GPSLatitude_H_Position",address, 200); address += 2;
    m_set_params_default("GPSLatitude_V_Position",address, 230); address += 2;
    m_set_params_default("GPSLatitude_Font_Size",address, 0); address += 2;
    m_set_params_default("GPSLatitude_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPSLongitude_Enable",address, 1); address += 2;
    m_set_params_default("GPSLongitude_Panel",address, 1); address += 2;
    m_set_params_default("GPSLongitude_H_Position",address, 280); address += 2;
    m_set_params_default("GPSLongitude_V_Position",address, 230); address += 2;
    m_set_params_default("GPSLongitude_Font_Size",address, 0); address += 2;
    m_set_params_default("GPSLongitude_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPS2Status_Enable",address, 1); address += 2;
    m_set_params_default("GPS2Status_Panel",address, 2); address += 2;
    m_set_params_default("GPS2Status_H_Position",address, 0); address += 2;
    m_set_params_default("GPS2Status_V_Position",address, 230); address += 2;
    m_set_params_default("GPS2Status_Font_Size",address, 0); address += 2;
    m_set_params_default("GPS2Status_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPS2HDOP_Enable",address, 1); address += 2;
    m_set_params_default("GPS2HDOP_Panel",address, 2); address += 2;
    m_set_params_default("GPS2HDOP_H_Position",address, 70); address += 2;
    m_set_params_default("GPS2HDOP_V_Position",address, 230); address += 2;
    m_set_params_default("GPS2HDOP_Font_Size",address, 0); address += 2;
    m_set_params_default("GPS2HDOP_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPS2Latitude_Enable",address, 1); address += 2;
    m_set_params_default("GPS2Latitude_Panel",address, 2); address += 2;
    m_set_params_default("GPS2Latitude_H_Position",address, 200); address += 2;
    m_set_params_default("GPS2Latitude_V_Position",address, 230); address += 2;
    m_set_params_default("GPS2Latitude_Font_Size",address, 0); address += 2;
    m_set_params_default("GPS2Latitude_H_Alignment",address, 0); address += 2;

    m_set_params_default("GPS2Longitude_Enable",address, 1); address += 2;
    m_set_params_default("GPS2Longitude_Panel",address, 2); address += 2;
    m_set_params_default("GPS2Longitude_H_Position",address, 280); address += 2;
    m_set_params_default("GPS2Longitude_V_Position",address, 230); address += 2;
    m_set_params_default("GPS2Longitude_Font_Size",address, 0); address += 2;
    m_set_params_default("GPS2Longitude_H_Alignment",address, 0); address += 2;

    m_set_params_default("Time_Enable",address, 1); address += 2;
    m_set_params_default("Time_Panel",address, 1); address += 2;
    m_set_params_default("Time_H_Position",address, 350); address += 2;
    m_set_params_default("Time_V_Position",address, 220); address += 2;
    m_set_params_default("Time_Font_Size",address, 0); address += 2;
    m_set_params_default("Time_H_Alignment",address, 2); address += 2;

    m_set_params_default("Altitude_Absolute_Enable",address, 1); address += 2;
    m_set_params_default("Altitude_Absolute_Panel",address, 2); address += 2;
    m_set_params_default("Altitude_Absolute_H_Position",address, 5); address += 2;
    m_set_params_default("Altitude_Absolute_V_Position",address, 10); address += 2;
    m_set_params_default("Altitude_Absolute_Font_Size",address, 0); address += 2;
    m_set_params_default("Altitude_Absolute_H_Alignment",address, 0); address += 2;
    m_set_params_default("Altitude_Scale_Enable",address, 1); address += 2;
    m_set_params_default("Altitude_Scale_Panel",address, 1); address += 2;
    m_set_params_default("Altitude_Scale_H_Position",address, 350); address += 2;
    m_set_params_default("Altitude_Scale_Align",address, 1); address += 2;
    m_set_params_default("Altitude_Scale_Source",address, 0); address += 2;

    m_set_params_default("Speed_Ground_Enable",address, 1); address += 2;
    m_set_params_default("Speed_Ground_Panel",address, 2); address += 2;
    m_set_params_default("Speed_Ground_H_Position",address, 5); address += 2;
    m_set_params_default("Speed_Ground_V_Position",address, 40); address += 2;
    m_set_params_default("Speed_Ground_Font_Size",address, 0); address += 2;
    m_set_params_default("Speed_Ground_H_Alignment",address, 0); address += 2;
    m_set_params_default("Speed_Scale_Enable",address, 1); address += 2;
    m_set_params_default("Speed_Scale_Panel",address, 1); address += 2;
    m_set_params_default("Speed_Scale_H_Position",address, 10); address += 2;
    m_set_params_default("Speed_Scale_Align",address, 0); address += 2;
    m_set_params_default("Speed_Scale_Source",address, 0); address += 2;

    m_set_params_default("Throttle_Enable",address, 1); address += 2;
    m_set_params_default("Throttle_Panel",address, 1); address += 2;
    m_set_params_default("Throttle_Scale_Enable",address, 1); address += 2;
    m_set_params_default("Throttle_H_Position",address, 285); address += 2;
    m_set_params_default("Throttle_V_Position",address, 202); address += 2;

    m_set_params_default("HomeDistance_Enable",address, 1); address += 2;
    m_set_params_default("HomeDistance_Panel",address, 1); address += 2;
    m_set_params_default("HomeDistance_H_Position",address, 70); address += 2;
    m_set_params_default("HomeDistance_V_Position",address, 14); address += 2;
    m_set_params_default("HomeDistance_Font_Size",address, 0); address += 2;
    m_set_params_default("HomeDistance_H_Alignment",address, 0); address += 2;

    m_set_params_default("WPDistance_Enable",address, 1); address += 2;
    m_set_params_default("WPDistance_Panel",address, 1); address += 2;
    m_set_params_default("WPDistance_H_Position",address, 70); address += 2;
    m_set_params_default("WPDistance_V_Position",address, 24); address += 2;
    m_set_params_default("WPDistance_Font_Size",address, 0); address += 2;
    m_set_params_default("WPDistance_H_Alignment",address, 0); address += 2;

    m_set_params_default("CHWDIR_Tmode_Enable",address, 1); address += 2;
    m_set_params_default("CHWDIR_Tmode_Panel",address, 2); address += 2;
    m_set_params_default("CHWDIR_Tmode_V_Position",address, 15); address += 2;
    m_set_params_default("CHWDIR_Nmode_Enable",address, 1); address += 2;
    m_set_params_default("CHWDIR_Nmode_Panel",address, 1); address += 2;
    m_set_params_default("CHWDIR_Nmode_H_Position",address, 30); address += 2;
    m_set_params_default("CHWDIR_Nmode_V_Position",address, 35); address += 2;
    m_set_params_default("CHWDIR_Nmode_Radius",address, 20); address += 2;
    m_set_params_default("CHWDIR_Nmode_Home_Radius",address, 25); address += 2;
    m_set_params_default("CHWDIR_Nmode_WP_Radius",address, 25); address += 2;

    m_set_params_default("Attitude_MP_Enable",address, 1); address += 2;
    m_set_params_default("Attitude_MP_Panel",address, 1); address += 2;
    m_set_params_default("Attitude_MP_Mode",address, 0); address += 2;
    m_set_params_default("Attitude_3D_Enable",address, 1); address += 2;
    m_set_params_default("Attitude_3D_Panel",address, 2); address += 2;

    m_set_params_default("Misc_Units_Mode",address, 0); address += 2;
    m_set_params_default("Misc_Max_Panels",address, 3); address += 2;

    m_set_params_default("PWM_Video_Enable",address, 1); address += 2;
    m_set_params_default("PWM_Video_Chanel",address, 6); address += 2;
    m_set_params_default("PWM_Video_Value",address, 1200); address += 2;
    m_set_params_default("PWM_Panel_Enable",address, 1); address += 2;
    m_set_params_default("PWM_Panel_Chanel",address, 7); address += 2;
    m_set_params_default("PWM_Panel_Value",address, 1200); address += 2;

    m_set_params_default("Alarm_H_Position",address, 180); address += 2;
    m_set_params_default("Alarm_V_Position",address, 25); address += 2;
    m_set_params_default("Alarm_Font_Size",address, 1); address += 2;
    m_set_params_default("Alarm_H_Alignment",address, 1); address += 2;
    m_set_params_default("Alarm_GPS_Status_Enable",address, 1); address += 2;
    m_set_params_default("Alarm_Low_Batt_Enable",address, 1); address += 2;
    m_set_params_default("Alarm_Low_Batt",address, 20); address += 2;
    m_set_params_default("Alarm_Under_Speed_Enable",address, 0); address += 2;
    m_set_params_default("Alarm_Under_Speed",address, 2); address += 2;
    m_set_params_default("Alarm_Over_Speed_Enable",address, 0); address += 2;
    m_set_params_default("Alarm_Over_Speed",address, 100); address += 2;
    m_set_params_default("Alarm_Under_Alt_Enable",address, 0); address += 2;
    m_set_params_default("Alarm_Under_Alt",address, 10); address += 2;
    m_set_params_default("Alarm_Over_Alt_Enable",address, 0); address += 2;
    m_set_params_default("Alarm_Over_Alt",address, 1000); address += 2;

    m_set_params_default("ClimbRate_Enable",address, 1); address += 2;
    m_set_params_default("ClimbRate_Panel",address, 1); address += 2;
    m_set_params_default("ClimbRate_H_Position",address, 5); address += 2;
    m_set_params_default("ClimbRate_V_Position",address, 220); address += 2;
    m_set_params_default("ClimbRate_Font_Size",address, 0); address += 2;

    m_set_params_default("RSSI_Enable",address, 0); address += 2;
    m_set_params_default("RSSI_Panel",address, 1); address += 2;
    m_set_params_default("RSSI_H_Position",address, 70); address += 2;
    m_set_params_default("RSSI_V_Position",address, 220); address += 2;
    m_set_params_default("RSSI_Font_Size",address, 0); address += 2;
    m_set_params_default("RSSI_H_Alignment",address, 0); address += 2;
    m_set_params_default("RSSI_Min",address, 0); address += 2;
    m_set_params_default("RSSI_Max",address, 255); address += 2;
    m_set_params_default("RSSI_Raw_Enable",address, 0); address += 2;

    m_set_params_default("FC_Type",address, m_protocol_type); address += 2;

    m_set_params_default("Wind_Enable",address, 1); address += 2;
    m_set_params_default("Wind_Panel",address, 2); address += 2;
    m_set_params_default("Wind_H_Position",address, 10); address += 2;
    m_set_params_default("Wind_V_Position",address, 100); address += 2;

    m_set_params_default("Time_Type",address, 0); address += 2;

    m_set_params_default("Throttle_Scale_Type",address, 0); address += 2;

    m_set_params_default("Attitude_MP_H_Position",address, 180); address += 2;
    m_set_params_default("Attitude_MP_V_Position",address, 133); address += 2;
    m_set_params_default("Attitude_MP_Scale_Real",address, 1); address += 2;
    m_set_params_default("Attitude_MP_Scale_Frac",address, 0); address += 2;
    m_set_params_default("Attitude_3D_H_Position",address, 180); address += 2;
    m_set_params_default("Attitude_3D_V_Position",address, 133); address += 2;
    m_set_params_default("Attitude_3D_Scale_Real",address, 1); address += 2;
    m_set_params_default("Attitude_3D_Scale_Frac",address, 0); address += 2;
    m_set_params_default("Attitude_3D_Map_radius",address, 40); address += 2;

    m_set_params_default("Misc_Start_Row",address, 0); address += 2;
    m_set_params_default("Misc_Start_Col",address, 0); address += 2;

    m_set_params_default("Misc_Firmware_ver",address, m_firmware_version); address += 2;

    m_set_params_default("Misc_Video_Mode",address, 1); address += 2;

    m_set_params_default("Speed_Scale_V_Position",address, 133); address += 2;
    m_set_params_default("Altitude_Scale_V_Position",address, 133); address += 2;

    m_set_params_default("BatteryConsumed_Enable",address, 1); address += 2;
    m_set_params_default("BatteryConsumed_Panel",address, 1); address += 2;
    m_set_params_default("BatteryConsumed_H_Position",address, 350); address += 2;
    m_set_params_default("BatteryConsumed_V_Position",address, 34); address += 2;
    m_set_params_default("BatteryConsumed_Font_Size",address, 0); address += 2;
    m_set_params_default("BatteryConsumed_H_Alignment",address, 2); address += 2;

    m_set_params_default("TotalTrip_Enable",address, 1); address += 2;
    m_set_params_default("TotalTrip_Panel",address, 1); address += 2;
    m_set_params_default("TotalTrip_H_Position",address, 350); address += 2;
    m_set_params_default("TotalTrip_V_Position",address, 210); address += 2;
    m_set_params_default("TotalTrip_Font_Size",address, 0); address += 2;
    m_set_params_default("TotalTrip_H_Alignment",address, 2); address += 2;

    m_set_params_default("RSSI_Type",address, 0); address += 2;

    m_set_params_default("Map_Enable",address, 1); address += 2;
    m_set_params_default("Map_Panel",address, 4); address += 2;
    m_set_params_default("Map_Radius",address, 120); address += 2;
    m_set_params_default("Map_Font_Size",address, 1); address += 2;
    m_set_params_default("Map_H_Alignment",address, 0); address += 2;
    m_set_params_default("Map_V_Alignment",address, 0); address += 2;

    m_set_params_default("Altitude_Relative_Enable",address, 1); address += 2;
    m_set_params_default("Altitude_Relative_Panel",address, 2); address += 2;
    m_set_params_default("Altitude_Relative_H_Position",address, 5); address += 2;
    m_set_params_default("Altitude_Relative_V_Position",address, 25); address += 2;
    m_set_params_default("Altitude_Relative_Font_Size",address, 0); address += 2;
    m_set_params_default("Altitude_Relative_H_Alignment",address, 0); address += 2;

    //0:absolute altitude 1:relative altitude
    m_set_params_default("Altitude_Scale_Type",address, 1); address += 2;

    m_set_params_default("Speed_Air_Enable",address, 1); address += 2;
    m_set_params_default("Speed_Air_Panel",address, 2); address += 2;
    m_set_params_default("Speed_Air_H_Position",address, 5); address += 2;
    m_set_params_default("Speed_Air_V_Position",address, 55); address += 2;
    m_set_params_default("Speed_Air_Font_Size",address, 0); address += 2;
    m_set_params_default("Speed_Air_H_Alignment",address, 0); address += 2;

    //0:ground speed 1:air speed
    m_set_params_default("Speed_Scale_Type",address, 0); address += 2;

    // sign of start col. 1:positive 0:negative
    m_set_params_default("Misc_Start_Col_Sign",address, 1); address += 2;

    //1:4800、2:9600、3:19200、4:38400、5:43000、6:56000、7:57600、8:115200
    m_set_params_default("Misc_USART_BandRate",address, 7); address += 2;
}
