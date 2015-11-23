#include "params.h"

#include <iostream>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <math.h>

COSDParam::COSDParam()
{
    memset((char *)_default_params, 0, PARAMS_BUF_SIZE);
    memset((char *)_params, 0, PARAMS_BUF_SIZE);
    _init_params();
}

COSDParam::~COSDParam()
{

}

bool COSDParam::load_params_from_file(const std::string &filename)
{
    std::ifstream in( filename, std::ios_base::in);

    if ( !in || !in.good() ){
       throw std::runtime_error("Failed to open parameters file");
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
                _str_to_buf(_params, strParamName, strParamValue);
                //std::cout << strParamName << ":" << strParamValue << std::endl;
            }catch(std::exception & e){
                throw std::runtime_error("Bad parameter item: " + strLine);
            }

        }
        else{
            throw std::runtime_error("Bad parameter item: " + strLine);
        }
    }
    dump_params();
    return true;
}

void COSDParam::_set_params_default(const std::string &paramname, int32_t addr, uint16_t initval)
{
    _params_addr[paramname] = addr;
    _u16_to_buf(_default_params, addr, initval);
}

void COSDParam::_u16_to_buf(uint8_t * buf, int32_t addr, uint16_t val)
{
    buf[addr] = (uint8_t)(val & 0xFF);
    buf[addr+1] = (uint8_t)((val >> 8) & 0xFF);
}

void COSDParam::_str_to_buf(uint8_t *buf, const std::string &paramname, const std::string &paramvalue)
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
            iterParamsAddr = _params_addr.find(paramname);
            if(iterParamsAddr != _params_addr.end()){
                paramAddr = iterParamsAddr->second;
                uint16_t nval = _panel_str_to_u16(paramvalue);
                _u16_to_buf(_params, paramAddr, nval);
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
            iterParamsAddr = _params_addr.find(paramname + "_Real");
            if(iterParamsAddr != _params_addr.end()){
                paramAddr = iterParamsAddr->second;
                _u16_to_buf(_params, paramAddr, nval_real);
            }

            iterParamsAddr = _params_addr.find(paramname + "_Frac");
            if(iterParamsAddr != _params_addr.end()){
                paramAddr = iterParamsAddr->second;
                _u16_to_buf(_params, paramAddr, nval_frac);
            }
            return;
        }

        //Misc_Start_Row : don't allow negative value
        if((paramname.find("Misc_Start_Row") != std::string::npos)){
            iterParamsAddr = _params_addr.find(paramname);
            if(iterParamsAddr != _params_addr.end()){
                paramAddr = iterParamsAddr->second;
                uint16_t nval = abs(atoi(paramvalue.c_str()));
                _u16_to_buf(_params, paramAddr, nval);
            }
            return;
        }

        //Misc_Start_Col : allow negative value, store the sign and value seperately
        if((paramname.find("Misc_Start_Col") != std::string::npos)){
            int nval = atoi(paramvalue.c_str());
            uint16_t absval = abs(nval);
            uint16_t nsign = (nval < 0) ? 0 : 1;

            iterParamsAddr = _params_addr.find(paramname);
            if(iterParamsAddr != _params_addr.end()){
                paramAddr = iterParamsAddr->second;
                _u16_to_buf(_params, paramAddr, absval);
            }

            iterParamsAddr = _params_addr.find(paramname + "_Sign");
            if(iterParamsAddr != _params_addr.end()){
                paramAddr = iterParamsAddr->second;
                _u16_to_buf(_params, paramAddr, nsign);
            }
            return;
        }

        //It is a normal uint16_t value
        iterParamsAddr = _params_addr.find(paramname);
        if(iterParamsAddr != _params_addr.end())
        {
            paramAddr = iterParamsAddr->second;
            uint16_t nval = atoi(paramvalue.c_str());
            _u16_to_buf(_params, paramAddr, nval);
        }

    }catch(std::exception & e){
        throw std::runtime_error("Bad parameter item: " + paramname + "=" + paramvalue);
    }
}

uint16_t COSDParam::_get_u16_param(uint8_t *buf, int32_t addr)
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

uint16_t COSDParam::_panel_str_to_u16(const std::string paramvalue)
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
        throw std::runtime_error("Bad parameter item: " + paramvalue);
    }

    return nret;
}

void COSDParam::dump_params()
{
    ParamsAddrMap::iterator iterParamsAddr;
    for(iterParamsAddr = _params_addr.begin(); iterParamsAddr != _params_addr.end(); iterParamsAddr++){
        std::cout << iterParamsAddr->first;
        std::cout << ":";
        std::cout << _get_u16_param(_params, iterParamsAddr->second);
        std::cout << std::endl;
    }
}

void COSDParam::_init_params()
{
    _params_addr.clear();
    int32_t address = 0;

    _set_params_default("ArmState_Enable",address, 1); address += 2;
    _set_params_default("ArmState_Panel",address, 1); address += 2;
    _set_params_default("ArmState_H_Position",address, 350); address += 2;
    _set_params_default("ArmState_V_Position",address, 44); address += 2;
    _set_params_default("ArmState_Font_Size",address, 0); address += 2;
    _set_params_default("ArmState_H_Alignment",address, 2); address += 2;

    _set_params_default("BatteryVoltage_Enable",address, 1); address += 2;
    _set_params_default("BatteryVoltage_Panel",address, 1); address += 2;
    _set_params_default("BatteryVoltage_H_Position",address, 350); address += 2;
    _set_params_default("BatteryVoltage_V_Position",address, 4); address += 2;
    _set_params_default("BatteryVoltage_Font_Size",address, 0); address += 2;
    _set_params_default("BatteryVoltage_H_Alignment",address, 2); address += 2;

    _set_params_default("BatteryCurrent_Enable",address, 1); address += 2;
    _set_params_default("BatteryCurrent_Panel",address, 1); address += 2;
    _set_params_default("BatteryCurrent_H_Position",address, 350); address += 2;
    _set_params_default("BatteryCurrent_V_Position",address, 14); address += 2;
    _set_params_default("BatteryCurrent_Font_Size",address, 0); address += 2;
    _set_params_default("BatteryCurrent_H_Alignment",address, 2); address += 2;

    _set_params_default("BatteryRemaining_Enable",address, 1); address += 2;
    _set_params_default("BatteryRemaining_Panel",address, 1); address += 2;
    _set_params_default("BatteryRemaining_H_Position",address, 350); address += 2;
    _set_params_default("BatteryRemaining_V_Position",address, 24); address += 2;
    _set_params_default("BatteryRemaining_Font_Size",address, 0); address += 2;
    _set_params_default("BatteryRemaining_H_Alignment",address, 2); address += 2;

    _set_params_default("FlightMode_Enable",address, 1); address += 2;
    _set_params_default("FlightMode_Panel",address, 1); address += 2;
    _set_params_default("FlightMode_H_Position",address, 350); address += 2;
    _set_params_default("FlightMode_V_Position",address, 54); address += 2;
    _set_params_default("FlightMode_Font_Size",address, 1); address += 2;
    _set_params_default("FlightMode_H_Alignment",address, 2); address += 2;

    _set_params_default("GPSStatus_Enable",address, 1); address += 2;
    _set_params_default("GPSStatus_Panel",address, 1); address += 2;
    _set_params_default("GPSStatus_H_Position",address, 0); address += 2;
    _set_params_default("GPSStatus_V_Position",address, 230); address += 2;
    _set_params_default("GPSStatus_Font_Size",address, 0); address += 2;
    _set_params_default("GPSStatus_H_Alignment",address, 0); address += 2;

    _set_params_default("GPSHDOP_Enable",address, 1); address += 2;
    _set_params_default("GPSHDOP_Panel",address, 1); address += 2;
    _set_params_default("GPSHDOP_H_Position",address, 70); address += 2;
    _set_params_default("GPSHDOP_V_Position",address, 230); address += 2;
    _set_params_default("GPSHDOP_Font_Size",address, 0); address += 2;
    _set_params_default("GPSHDOP_H_Alignment",address, 0); address += 2;

    _set_params_default("GPSLatitude_Enable",address, 1); address += 2;
    _set_params_default("GPSLatitude_Panel",address, 1); address += 2;
    _set_params_default("GPSLatitude_H_Position",address, 200); address += 2;
    _set_params_default("GPSLatitude_V_Position",address, 230); address += 2;
    _set_params_default("GPSLatitude_Font_Size",address, 0); address += 2;
    _set_params_default("GPSLatitude_H_Alignment",address, 0); address += 2;

    _set_params_default("GPSLongitude_Enable",address, 1); address += 2;
    _set_params_default("GPSLongitude_Panel",address, 1); address += 2;
    _set_params_default("GPSLongitude_H_Position",address, 280); address += 2;
    _set_params_default("GPSLongitude_V_Position",address, 230); address += 2;
    _set_params_default("GPSLongitude_Font_Size",address, 0); address += 2;
    _set_params_default("GPSLongitude_H_Alignment",address, 0); address += 2;

    _set_params_default("GPS2Status_Enable",address, 1); address += 2;
    _set_params_default("GPS2Status_Panel",address, 2); address += 2;
    _set_params_default("GPS2Status_H_Position",address, 0); address += 2;
    _set_params_default("GPS2Status_V_Position",address, 230); address += 2;
    _set_params_default("GPS2Status_Font_Size",address, 0); address += 2;
    _set_params_default("GPS2Status_H_Alignment",address, 0); address += 2;

    _set_params_default("GPS2HDOP_Enable",address, 1); address += 2;
    _set_params_default("GPS2HDOP_Panel",address, 2); address += 2;
    _set_params_default("GPS2HDOP_H_Position",address, 70); address += 2;
    _set_params_default("GPS2HDOP_V_Position",address, 230); address += 2;
    _set_params_default("GPS2HDOP_Font_Size",address, 0); address += 2;
    _set_params_default("GPS2HDOP_H_Alignment",address, 0); address += 2;

    _set_params_default("GPS2Latitude_Enable",address, 1); address += 2;
    _set_params_default("GPS2Latitude_Panel",address, 2); address += 2;
    _set_params_default("GPS2Latitude_H_Position",address, 200); address += 2;
    _set_params_default("GPS2Latitude_V_Position",address, 230); address += 2;
    _set_params_default("GPS2Latitude_Font_Size",address, 0); address += 2;
    _set_params_default("GPS2Latitude_H_Alignment",address, 0); address += 2;

    _set_params_default("GPS2Longitude_Enable",address, 1); address += 2;
    _set_params_default("GPS2Longitude_Panel",address, 2); address += 2;
    _set_params_default("GPS2Longitude_H_Position",address, 280); address += 2;
    _set_params_default("GPS2Longitude_V_Position",address, 230); address += 2;
    _set_params_default("GPS2Longitude_Font_Size",address, 0); address += 2;
    _set_params_default("GPS2Longitude_H_Alignment",address, 0); address += 2;

    _set_params_default("Time_Enable",address, 1); address += 2;
    _set_params_default("Time_Panel",address, 1); address += 2;
    _set_params_default("Time_H_Position",address, 350); address += 2;
    _set_params_default("Time_V_Position",address, 220); address += 2;
    _set_params_default("Time_Font_Size",address, 0); address += 2;
    _set_params_default("Time_H_Alignment",address, 2); address += 2;

    _set_params_default("Altitude_Absolute_Enable",address, 1); address += 2;
    _set_params_default("Altitude_Absolute_Panel",address, 2); address += 2;
    _set_params_default("Altitude_Absolute_H_Position",address, 5); address += 2;
    _set_params_default("Altitude_Absolute_V_Position",address, 10); address += 2;
    _set_params_default("Altitude_Absolute_Font_Size",address, 0); address += 2;
    _set_params_default("Altitude_Absolute_H_Alignment",address, 0); address += 2;
    _set_params_default("Altitude_Scale_Enable",address, 1); address += 2;
    _set_params_default("Altitude_Scale_Panel",address, 1); address += 2;
    _set_params_default("Altitude_Scale_H_Position",address, 350); address += 2;
    _set_params_default("Altitude_Scale_Align",address, 1); address += 2;
    _set_params_default("Altitude_Scale_Source",address, 0); address += 2;

    _set_params_default("Speed_Ground_Enable",address, 1); address += 2;
    _set_params_default("Speed_Ground_Panel",address, 2); address += 2;
    _set_params_default("Speed_Ground_H_Position",address, 5); address += 2;
    _set_params_default("Speed_Ground_V_Position",address, 40); address += 2;
    _set_params_default("Speed_Ground_Font_Size",address, 0); address += 2;
    _set_params_default("Speed_Ground_H_Alignment",address, 0); address += 2;
    _set_params_default("Speed_Scale_Enable",address, 1); address += 2;
    _set_params_default("Speed_Scale_Panel",address, 1); address += 2;
    _set_params_default("Speed_Scale_H_Position",address, 10); address += 2;
    _set_params_default("Speed_Scale_Align",address, 0); address += 2;
    _set_params_default("Speed_Scale_Source",address, 0); address += 2;

    _set_params_default("Throttle_Enable",address, 1); address += 2;
    _set_params_default("Throttle_Panel",address, 1); address += 2;
    _set_params_default("Throttle_Scale_Enable",address, 1); address += 2;
    _set_params_default("Throttle_H_Position",address, 285); address += 2;
    _set_params_default("Throttle_V_Position",address, 202); address += 2;

    _set_params_default("HomeDistance_Enable",address, 1); address += 2;
    _set_params_default("HomeDistance_Panel",address, 1); address += 2;
    _set_params_default("HomeDistance_H_Position",address, 70); address += 2;
    _set_params_default("HomeDistance_V_Position",address, 14); address += 2;
    _set_params_default("HomeDistance_Font_Size",address, 0); address += 2;
    _set_params_default("HomeDistance_H_Alignment",address, 2); address += 2;
}
