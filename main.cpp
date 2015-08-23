

#include <quan/serial_port.hpp>
#include <iostream>
#include <stdexcept>
#include <string>
#include <fstream>
#include <cstring>
#include <vector>
#include <quan/min.hpp>
#include <quan/utility/timer.hpp>

namespace px4Uploader{

   int32_t crc(std::vector<uint8_t> const & bytes,int32_t padlen);
}

struct usb_to_osd_t{

    struct Firmware
    {
//        int board_id;
//        std::string magic;
//        std::string description;
//        std::string image;
        std::vector<uint8_t> imagebyte;
        int32_t image_size;
        uint32_t expected_crc;
        int32_t board_flash_size;
//        uint32_t build_time;
//        std::string summary;
//        std::string version;
//        int image_size;
//        std::string git_identity;
//        int board_revision;
        
        //static Firmware fw;
         Firmware():image_size{0}, expected_crc{0}, board_flash_size{0}{}
     } firmware;

   usb_to_osd_t( const char* port_name)
   :m_sp{port_name}, m_good{true}
   {
      m_sp.init();
      m_good = m_sp.good() && (m_sp.set_baud(115200) == 0);
      if( !m_good){
         throw std::runtime_error("usb_to_osd ctor : couldnt connect");
      }
   }

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
   };

   // code for the app to request a reboot to bootlaoder
   // also works in PlayUAV version of the  bootloader
   static constexpr uint8_t PROTO_BL_UPLOAD = 0x55;

   bool connected()
   {
      return m_good && m_sp.good();
   }

   void send(uint8_t c)
   {
      throw_if_not_connected();
      m_sp.write(&c,1);
   }
   
   void send( uint8_t const* arr, size_t len)
   {
      throw_if_not_connected();
      m_sp.write(arr,len) ;
      
   }

   void recv(uint8_t * arr, size_t count = 1)
   {
      throw_if_not_connected();
      if ( m_sp.read(arr,count) == -1){
         throw std::runtime_error("usb_to_osd read failed");
      }
   }
//
//   void flush()
//   {
//      m_sp.flush();
//   }

   // endiannness?
   int32_t recv_int()
   {
      union{
         uint8_t arr[4];
         int32_t i;
      } u;
      recv(u.arr,4);
      return u.i;
   }

    uint32_t recv_uint()
   {
      union{
         uint8_t arr[4];
         uint32_t ui;
      } u;
      recv(u.arr,4);
      return u.ui;
   }

   void getSync()
   {
      uint8_t ch;
      recv(& ch);
      if ( ch != INSYNC){
         std::cout << " ch = " << (int) ch <<'\n';
         throw std::runtime_error("get_sync : expected INSYNC");
      }
      recv(&ch);
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

   void sync()
   {
      m_sp.flush();
      uint8_t const sync_cmd [] = {GET_SYNC, EOC};
      send(sync_cmd,2);
      getSync();
   }

//   bool trySync()
//   {
//      uint8_t ch;
//      recv(&ch);
//      if (ch != INSYNC) {return false;}
//      recv(&ch);
//      return (ch == OK);
//   }

   // actually 3 params
   int32_t get_board_max_flash_size()
   {
      uint8_t const cmd [] = { GET_DEVICE, INFO_FLASH_SIZE, EOC};
      send(cmd,3);
      int32_t result = recv_int();
      getSync();
      return result;
   }

   uint32_t get_board_crc()
   {
      uint8_t const cmd [] = {GET_CRC, EOC};
      send(cmd,2);
      uint32_t result = recv_uint();
      getSync();
      return result;
   }

   // either in bl or not
   void reset_to_bootloader()
   {
      uint8_t const cmd [] = {PROTO_BL_UPLOAD, EOC};
      send(cmd,2);
      getSync();
      quan::timer<> t;
      while (m_sp.in_avail()){
         uint8_t ch;
         recv(&ch);
      }
      m_sp.flush();
      while (t() < quan::time::ms{1000}) {;}
      if( !m_sp.good()){
         throw std::runtime_error("reset to bootloader : couldnt connect");
      }
   }

   // bootloader doesnt program reset vector
   // until you send the reboot command
   void reboot_to_app()
   {
      uint8_t const cmd [] = {REBOOT, EOC};
      send(cmd,2);
      getSync();
   }

   void upload( std::string const & filename)
   {
      firmware.board_flash_size = this->get_board_max_flash_size();

      std::ifstream in( filename, std::ios_base::in | std::ios_base::binary);
      
      if ( !in || !in.good() ){
         throw std::runtime_error("Failed to open bin file");
      }
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
         //in.read((char*)arr,sequence_length);
         for ( int32_t i = 0;i < sequence_length; ++i){
            arr[i] = firmware.imagebyte.at(image_idx);
            ++ image_idx;
         }
         send(PROG_MULTI);
         send(static_cast<uint8_t>(sequence_length));
         send(arr,sequence_length);
         send(EOC);
         getSync();
         image_bytes_left -= sequence_length;
      }
      if (image_bytes_left != 0 ){
         throw std::runtime_error("infile Failed while uploading bin file");
      }

      uint32_t board_crc = get_board_crc();
      if ( firmware.expected_crc != board_crc){
         std::cout << "WARNING crc doesnt match\n";
      }else{
         std::cout << "firmware verified\n";
      }
   }

   void erase()
   {
      m_sp.flush();
      sync();
      uint8_t arr []= {CHIP_ERASE,EOC};
      send(arr,2);
      quan::timer<> t;
      while ( t() < quan::time::s{20}){
         if (m_sp.in_avail() > 0 )
         break;
      }
      
      getSync();
      
   }

   void close()
   {
      m_sp.close();
   }

private:
   quan::serial_port m_sp;
   bool m_good;

   void throw_if_not_connected()
   {
      if(!connected()){
         throw std::runtime_error(" not connected");
      }
   }
};

// TODO
// *algorithm*
// look for the "some signature" of the PlayUAV OSD on all the ACM ports
// If successfully connected then call reset to bootloader on it.
// try reading it after
// if successful then there was no firmware and it is already in bootloader

// if it fails then that is because it has reset itself
// so look for the signature o all ports again
// then do the programme part on that port

int main()
{
   try{

// ...getting there. 
     // if good firmware is loaded which can do a reset
     // then on reset enumeration 
     // gives a new file name to the port
      // This only works if valid firmware is currently loaded
// Tested and works one time so far !
       std::cout<< "PlayUav firmware loader Hack!\n";
// usually the first port
       usb_to_osd_t sp{"/dev/ttyACM0"};
       sp.sync();
       sp.reset_to_bootloader();
// let us not close the dead port to save confusion!
//################################
// usually the second port
      usb_to_osd_t sp1{"/dev/ttyACM1"};
      sp1.sync();
      std::cout << "erasing...\n";
      sp1.erase();
      std::cout << "erased\n";
      std::cout << "uploading...\n";
      sp1.upload("/home/andy/cpp/projects/osd_comm/px4fw.bin");
      std::cout << "uploaded\n";
      
      std::cout << "rebooting...\n";
      sp1.reboot_to_app();
      std::cout << "rebooted\n";
      return EXIT_SUCCESS;
   }catch (std::exception & e){
      std::cout << "Exception '" <<  e.what() << "'\n";
   }
}
