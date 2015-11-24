#pragma once

#include<string>
#include <map>

#define PARAMS_BUF_SIZE 1024

typedef std::map<std::string, int32_t> ParamsAddrMap;

class COSDParam{
public:
    COSDParam();
    ~COSDParam();

    bool load_params_from_file(const std::string & filename);
    bool store_params_to_file(const std::string & filename);

    void dump_params();

private:
    void _init_params();
    void _set_params_default(const std::string & paramname, int32_t addr, uint16_t initval);
    void _u16_to_buf(uint8_t * buf, int32_t addr, uint16_t val);
    void _str_to_buf(uint8_t * buf, const std::string & paramname, const std::string & paramvalue);
    uint16_t _get_u16_param(uint8_t * buf, int32_t addr);
    std::string _param_serialize(uint8_t * buf, int32_t addr, const std::string & paramname);
    uint16_t _panel_str_to_u16(const std::string paramvalue);


    ParamsAddrMap _params_addr;
    uint8_t _default_params[PARAMS_BUF_SIZE];
    uint8_t _params[PARAMS_BUF_SIZE];
    const uint16_t _firmware_version;
    const uint16_t _protocol_type;
};
