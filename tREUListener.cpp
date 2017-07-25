#include <iostream>
#include <chrono>
#include <thread>

#include "ComDriver.h"

struct TrPacket{
  unsigned char TPDO_buff[10];
  const unsigned short pdo_id = 0x200;
  unsigned char node_id;
  const unsigned char data_length = 8;
  float CtrlSetPoint;
  const float CanPDO1Input2 = 0;

  void UpdateBuffer() {
    TPDO_buff[0] = ((pdo_id + node_id) >> 4) & 0x00FF;
    TPDO_buff[1] = (((pdo_id + node_id) & 0x000F) << 4) | (data_length & 0x000F);
    TPDO_buff[2] = ((unsigned int)CtrlSetPoint & 0x000000FF);
    TPDO_buff[3] = ((unsigned int)CtrlSetPoint & 0x0000FF00) >> 8;
    TPDO_buff[4] = ((unsigned int)CtrlSetPoint & 0x00FF0000) >> 16;
    TPDO_buff[5] = ((unsigned int)CtrlSetPoint & 0xFF000000) >> 24;
    TPDO_buff[6] = (unsigned int)CanPDO1Input2 & 0x000000FF;
    TPDO_buff[7] = ((unsigned int)CanPDO1Input2 & 0x0000FF00) >> 8;
    TPDO_buff[8] = ((unsigned int)CanPDO1Input2 & 0x00FF0000) >> 16;
    TPDO_buff[9] = ((unsigned int)CanPDO1Input2 & 0xFF000000) >> 24;
  }
};

struct SyncPacket{
  unsigned char SYN_buff[2];
};

struct RecPacket{
  unsigned char RPDO_buff[10];
  unsigned short pdo_id;
  unsigned char node_id;
  unsigned char data_length;
  float RotEncPos;
  float CanPDO1Input2;

  void UpdateStruct() {
    pdo_id = ((RPDO_buff[0] & 0x00FF) << 4) | (RPDO_buff[1] >> 4);
    node_id = RPDO_buff[0] & 0x0F;
    RotEncPos = (float)((((unsigned int)RPDO_buff[5] & 0x000000FF) << 24) |
        (((unsigned int)RPDO_buff[4] & 0x000000FF) << 16) |
        (((unsigned int)RPDO_buff[3] & 0x000000FF) << 8) |
        ((unsigned int)RPDO_buff[2] & 0x000000FF));
    CanPDO1Input2 = (float)((((unsigned int)RPDO_buff[9] & 0x000000FF) << 24) |
        (((unsigned int)RPDO_buff[8] & 0x000000FF) << 16) |
        (((unsigned int)RPDO_buff[7] & 0x000000FF) << 8) |
        ((unsigned int)RPDO_buff[6] & 0x000000FF));
  }
};


int main(void) {
  std::string port_name = "/dev/cu.usbserial-AC0093YG";

  TrPacket tr_pack;
  tr_pack.node_id = 86;
  tr_pack.CtrlSetPoint = 1.9562f;
  tr_pack.UpdateBuffer();

  RecPacket rec_pack;
  ComportDriver comport;
//  if(comport.Connect(port_name,115200,sizeof(rec_pack.RPDO_buff))) {
  if(comport.Connect(port_name,115200,1)) {
    std::cout << "Connected " << std::endl;
  } else {
    std::cout << "Exiting Program!" << std::endl;
  }
  while (1) {
//    comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
    unsigned char data = 0;
    comport.ReadSensorPacket((unsigned char*)&data,1);
    std::cout << "data is " << data << std::endl;
//    comport.ReadSensorPacket(rec_pack.RPDO_buff,sizeof(rec_pack.RPDO_buff));
//    std::cout << "Data Received !" << std::endl;
//    rec_pack.UpdateStruct();
//    std::cout << "ID was : " << rec_pack.node_id << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  return(0);
}
