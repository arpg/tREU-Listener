#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>
#include <string>
#include <math.h>
#include "ComDriver.h"
#include <HAL/Gamepad.pb.h>
#include <HAL/Gamepad/GamepadDevice.h>
#define talk 0
#define listen 1
#define REU_HRT_PDO 0x7
#define REU_TRA_PDO 0x5
#define REU_REC_PDO 0x6
#define REU_ENC_PDO 0x1
#define REU_CNG_PAR (rec_pack.RPDO_buff[6]=='2')&&(rec_pack.RPDO_buff[7]=='2')
#define REU_CNG_PAR_SUCCESS (rec_pack.RPDO_buff[6]=='6')&&(rec_pack.RPDO_buff[7]=='0')
#define REU_READ_DATA (rec_pack.RPDO_buff[6]=='4')&&(rec_pack.RPDO_buff[7]=='2')
#define REU_DATA_IS_PRES (rec_pack.RPDO_buff[8]=='0')&&(rec_pack.RPDO_buff[9]=='8')&&(rec_pack.RPDO_buff[10]=='2')&&(rec_pack.RPDO_buff[11]=='2')

#define CTRL_DEMAND_MODE_PAR 0x8120
#define WAVE_GEN_TRIGGER_PAR 0x1221
#define OPERATION_PAR 0x012F
#define WAVE_GEN_TRIGGER_MODE_PAR 0x1121
#define WAVE_GEN_FADER_COUNT_PAR 0x0821
#define POS_RATE_LIMIT_PAR 0x0727
#define WAVE_GEN_OFFSET_PAR 0x0321
#define WAVE_GEN_MODE_PAR 0x0221
#define CTRL_CONTROL_MODE_PAR 0x8220
#define WAVE_GEN_FREQ_PAR 0x0121
#define WAVE_GEN_AMP_PAR 0x0421
#define CAN_PDO_BROADCAST_PAR 0x1321

#define VIEW_PRESSURE 0x0822


int slowCount;
enum string_code{
    eDEMAND,
    eTRIGGER,
    eOPERATION,
    eTRIGGERMODE,
    eFADERCNT,
    eRATELIMIT,
    eOFFSET,
    eWAVEMODE,
    eCTRLMODE,
    eFREQ,
    eAMP,
    eBROADCAST,
    ePRESSURE,
    eDEFAULT,
};

int find_start(struct RecPacket rec_pack);
char Ascii2Hex(char ascii_digit);
int Hex2Ascii(int value);
string_code hashit(const std::string &input);
void change_parameter(char value, float float_value, char parameter[]);
void GamepadCallback(hal::GamepadMsg& _msg);

struct TrPacket{
  unsigned char TPDO_buff[23];
  const unsigned char pdo_id = '6';
  unsigned char node_id;
  const unsigned char data_length = 'N';
  unsigned short parameter;
  unsigned short value;
  float float_value;
  bool value_is_float;
  void UpdateBuffer() {
    TPDO_buff[0] = ':';
    TPDO_buff[1] = 'S';
    TPDO_buff[2] = pdo_id;
    TPDO_buff[3] = (unsigned char)Hex2Ascii((node_id & 0xF0)>>4);
    TPDO_buff[4] = (unsigned char)Hex2Ascii((node_id & 0x0F));
    TPDO_buff[5] = data_length;
    TPDO_buff[6] = '2';
    TPDO_buff[7] = '2';
    TPDO_buff[8] = Hex2Ascii((parameter & 0xF000)>>12);
    TPDO_buff[9] = Hex2Ascii((parameter & 0x0F00)>>8);
    TPDO_buff[10] = Hex2Ascii((parameter & 0x00F0)>>4);
    TPDO_buff[11] = Hex2Ascii(parameter & 0x000F);
    if(!value_is_float/*value&&!float_value*/){
        TPDO_buff[12] = Hex2Ascii((value & 0xF000)>>12);
        TPDO_buff[13] = Hex2Ascii((value & 0x0F00)>>8);
        TPDO_buff[14] = Hex2Ascii((value & 0x00F0)>>4);
        TPDO_buff[15] = Hex2Ascii(value & 0x000F);
        TPDO_buff[16] = '0';
        TPDO_buff[17] = '0';
        TPDO_buff[18] = '0';
        TPDO_buff[19] = '0';
        TPDO_buff[20] = '0';
        TPDO_buff[21] = '0';
    }
    else if(value_is_float/*!value&&float_value*/){
        unsigned char *float_val = reinterpret_cast<unsigned char *>(&float_value);

        TPDO_buff[12] = '0';
        TPDO_buff[13] = '0';
        TPDO_buff[14] = Hex2Ascii(((int)float_val[0]&0xF0)>>4);
        TPDO_buff[15] = Hex2Ascii((int)float_val[0]&0x0F);
        TPDO_buff[16] = Hex2Ascii(((int)float_val[1]&0xF0)>>4);
        TPDO_buff[17] = Hex2Ascii((int)float_val[1]&0x0F);
        TPDO_buff[18] = Hex2Ascii(((int)float_val[2]&0xF0)>>4);
        TPDO_buff[19] = Hex2Ascii((int)float_val[2]&0x0F);
        TPDO_buff[20] = Hex2Ascii(((int)float_val[3]&0xF0)>>4);
        TPDO_buff[21] = Hex2Ascii((int)float_val[3]&0x0F);
    }
    TPDO_buff[22] = ';';
  }
}tr_pack;

struct SyncPacket{
  unsigned char SYN_buff[2];
};



struct RecPacket{
  unsigned char RPDO_buff[23];
  unsigned short pdo_id;
  unsigned short node_id;
  unsigned char data_length;
  double pressure;
  double mantissa;
  int exponent;
//  void UpdateStruct() {
//    pdo_id = ((RPDO_buff[0] & 0x00FF) << 4) | (RPDO_buff[1] >> 4);
//    node_id = RPDO_buff[0] & 0x0F;
//    RotEncPos = (float)((((unsigned int)RPDO_buff[5] & 0x000000FF) << 24) |
//        (((unsigned int)RPDO_buff[4] & 0x000000FF) << 16) |
//        (((unsigned int)RPDO_buff[3] & 0x000000FF) << 8) |
//        ((unsigned int)RPDO_buff[2] & 0x000000FF));
//    CanPDO1Input2 = (float)((((unsigned int)RPDO_buff[9] & 0x000000FF) << 24) |
//        (((unsigned int)RPDO_buff[8] & 0x000000FF) << 16) |
//        (((unsigned int)RPDO_buff[7] & 0x000000FF) << 8) |
//        ((unsigned int)RPDO_buff[6] & 0x000000FF));
//  }
  void UpdateStruct() {
      pdo_id = Ascii2Hex(RPDO_buff[2]);
      node_id = ((Ascii2Hex(RPDO_buff[3])&0xFF)<<4)|(Ascii2Hex(RPDO_buff[4])&0xF);
      int temp =((Ascii2Hex(RPDO_buff[21])/*&0x0000000F*/)<<24)|((Ascii2Hex(RPDO_buff[20])/*&0x000000F0*/)<<28)|
              ((Ascii2Hex(RPDO_buff[19])/*&0x00000F00*/)<<16)|((Ascii2Hex(RPDO_buff[18])/*&0x0000F000*/)<<20)|
              ((Ascii2Hex(RPDO_buff[17])/*&0x000F0000*/)<<8)|((Ascii2Hex(RPDO_buff[16])/*&0x00F00000*/)<<12)|
              ((Ascii2Hex(RPDO_buff[15])/*&0x0F000000*/))|((Ascii2Hex(RPDO_buff[14])/*&0xF0000000*/)<<4);
      /*int*/ exponent = pow(2,((temp&0x7FE00000)>>23)-127);
      temp = temp&0x7FFFFF;
      //std::cout << "Temp = " << temp << std::endl;
      /*float*/ mantissa = 0;
      for(int i=22;i>=0;i--)
      {
        mantissa += ((temp>>i)&0x1)*pow(2,(i-23));
      }
      //std::cout << std::hex << mantissa << std::dec << std::endl;
      mantissa += 1;
      pressure = exponent*mantissa;

  }
};
bool toggle = false;
bool trigger;
bool operational;
bool preoperational;
void GamepadCallback(hal::GamepadMsg& _msg) {

    if(_msg.buttons().data(0))
    {
        trigger = true;
    }
    if(_msg.buttons().data(1))
    {
        preoperational = true;
    }
    if(_msg.buttons().data(2))
    {
        operational = true;
    }
}

int main(void) {

  std::string port_name = "/dev/ttyUSB0";
  //TrPacket tr_pack;
  tr_pack.node_id = 83;
  tr_pack.UpdateBuffer();
  RecPacket rec_pack;
  ComportDriver comport;
  hal::Gamepad gamepad("gamepad:/");
  gamepad.RegisterGamepadDataCallback(&GamepadCallback);
  if(comport.Connect(port_name,115200,sizeof(rec_pack.RPDO_buff)))
  {
    std::cout << "Connected " << std::endl;
  }
  else
  {
    std::cout << "Exiting Program!" << std::endl;
  }

  while (1) {

#if talk
    bool all;
    char ID[3] = {0,0};
    char parameter[25];
    float float_value;
    char value = 0;
    std::cout << std::endl <<"REU ID: ";
    std::cin.clear();
    std::cin >> ID;
    if((ID[0]=='a')&&(ID[1]=='l')&&(ID[2]=='l'))
    {
        all = true;
    }
    else
    {
        tr_pack.node_id = (Ascii2Hex(ID[0])*10)+Ascii2Hex(ID[1]);
        all = false;
    }

    std::cout << std::endl <<"Parameters you can change:" << std::endl << std::endl << "\"d\" Ctrl Demand Mode (0 - Analog, 1 - Waveform Gen, 2 - CAN)" << std::endl << "\"t\" Wave Gen Trigger (0 - OFF, 1 - ON)" << std::endl << "\"o\" Operation Mode (1 - Pre-Operational, 2 - Operational)" << std::endl;
    std::cout << "\"tm\" Wave Gen Trigger Mode (0 - Continuous, 1 - One Cycle)" << std::endl << "\"f\" Wave Gen Fader Count (Set to 1 for jump)" << std::endl << "\"l\" Pos Loop RateLimiter Limit (Set to 35 for jump)" << std::endl;
    std::cout << "\"off\" Wave Gen Offset (Set to 1 inch for jump)" << std::endl << "\"m\" Wave Gen Mode (0 - DC, 1 - Square, 2 - Sine, 3 - Triangle, 4 - Custom)" << std::endl;
    std::cout << "\"c\" Ctrl Control Mode (1 - Current, 128 - Custom Config)" << std::endl << "\"fre\" Wave Gen Freq (Frequency in Hz)" << std::endl << "\"a\" Wave Gen Amp (Amp in inches)" << std::endl;
    std::cout << "\"b\" CAN PDO Broadcast Master (0 - OFF, 1 - ON)" << std::endl;
    std::cout << std::endl;
    std::cout << "Change which parameter: " << std::endl;
    std::cin.clear();
    std::cin >> parameter;

    std::cout << "To what value: ";
    std::cin.clear();
    std::cin >> float_value;
    value = (char)(static_cast<int>(float_value));
    if(!all)
    {
        change_parameter(value, float_value,parameter);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));

    }
    else if(all)
    {
        tr_pack.node_id = 18;
        change_parameter(value, float_value,parameter);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));

        tr_pack.node_id = 82;
        change_parameter(value, float_value,parameter);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));

        tr_pack.node_id = 83;
        change_parameter(value, float_value,parameter);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));

        tr_pack.node_id = 84;
        change_parameter(value, float_value,parameter);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
    }
    comport.ReadSensorPacket(rec_pack.RPDO_buff,sizeof(rec_pack.RPDO_buff));
    rec_pack.UpdateStruct();
    std::cout << rec_pack.RPDO_buff << std::endl;
#endif

    if(trigger)
    {
        int value;
        float f_value = 0;
        if(!toggle)
        {
            value = 1;
        }
        else
        {

            value = 0;
        }
        char tri[23]= "t";
        tr_pack.node_id = 18;
        change_parameter(value, f_value,tri);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 82;
        change_parameter(value, f_value,tri);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 83;
        change_parameter(value, f_value,tri);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 84;
        change_parameter(value, f_value,tri);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        toggle = !toggle;
        trigger = false;
        std::cout << "Triggering" << std::endl;
    }
    if(operational)
    {
        char op[10]= "o";
        tr_pack.node_id = 18;
        change_parameter(2, 2,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 82;
        change_parameter(2, 2,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 83;
        change_parameter(2, 2,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 84;
        change_parameter(2, 2,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        operational = false;
        std::cout << "Operational" << std::endl;
    }
    if(preoperational)
    {
        char op[10]= "o";
        tr_pack.node_id = 18;
        change_parameter(1, 1,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 82;
        change_parameter(1, 1,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 83;
        change_parameter(1, 1,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        tr_pack.node_id = 84;
        change_parameter(1, 1,op);
        comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));
        preoperational = false;
        std::cout << "Pre-operational" << std::endl;
    }

    tr_pack.node_id = 84;
    tr_pack.parameter = VIEW_PRESSURE;
    tr_pack.value = 0;

    tr_pack.UpdateBuffer();
    tr_pack.TPDO_buff[6] = '4';
    tr_pack.TPDO_buff[7] = '0';
    tr_pack.TPDO_buff[12] = '0';
    tr_pack.TPDO_buff[13] = '0';
    tr_pack.TPDO_buff[14] = '0';
    tr_pack.TPDO_buff[15] = '0';
    tr_pack.TPDO_buff[16] = '0';
    tr_pack.TPDO_buff[17] = '0';
    tr_pack.TPDO_buff[18] = '0';
    tr_pack.TPDO_buff[19] = '0';
    tr_pack.TPDO_buff[20] = '0';
    tr_pack.TPDO_buff[21] = '0';


   //comport.WriteComPort(tr_pack.TPDO_buff,sizeof(tr_pack.TPDO_buff));


    slowCount++;
#if listen
    comport.ReadSensorPacket(rec_pack.RPDO_buff,sizeof(rec_pack.RPDO_buff));
    //std::cout << rec_pack.RPDO_buff << std:: endl;
    if((rec_pack.RPDO_buff[0]==':') && (rec_pack.RPDO_buff[1]=='S'))
    {
        rec_pack.UpdateStruct();
        //std::cout << rec_pack.RPDO_buff << std::endl;
        switch(rec_pack.pdo_id)
        {
           case REU_HRT_PDO:
            //std::cout << "<3" << std::endl;
           break;
           case REU_REC_PDO:

           break;
           case REU_TRA_PDO:
            if(REU_READ_DATA)
            {
                if(REU_DATA_IS_PRES)
                {

                    //std::cout << "Calculated Pressure: " << rec_pack.pressure << "   Mantissa: " << rec_pack.mantissa << "   Exponent: " << rec_pack.exponent << std::endl;
                    std::cout << "Pressure: " << rec_pack.pressure <<std::endl;
                }
                //std::cout << rec_pack.RPDO_buff << std::endl;
            }
            break;
           case REU_ENC_PDO:
            break;

           default:
            //std::cout << rec_pack.RPDO_buff << std::endl;
            break;
        }
    }

    else
    {
        std::cout << "Misalligned" << std::endl;
        int allign = find_start(rec_pack);
        unsigned char garbage[25];
        comport.ReadSensorPacket(garbage,allign);
    }
#endif
  }
  return(0);
}









int find_start(struct RecPacket rec_pack){
    uint32_t allign;
    for(allign = 0;allign<sizeof(rec_pack.RPDO_buff)-1;allign++)
    {
        if((rec_pack.RPDO_buff[allign]==':')&&(rec_pack.RPDO_buff[allign+1]=='S'))
        {
            break;
        }
    }

    if(allign==sizeof(rec_pack.RPDO_buff)-1)
    {
        std::cout << "Could not find :S " << std::endl;
        return 0;
    }
    else
    {
        std::cout << "Allignment shift is: " << allign << std::endl;
        return allign;
    }
}

char Ascii2Hex(char ascii_digit)
{
    if((ascii_digit <='9')&&(ascii_digit >='0'))
    {
        return ascii_digit-'0';
    }
    else if ((ascii_digit <='F')&&(ascii_digit >='A'))
    {
        return ascii_digit-'A'+10;
    }
    else
    {
        std::cerr << "Unrecognized ascii character !" << std::endl;
        return 0;
    }

}

int Hex2Ascii(int value)
{
    if((value >= 0)&&(value < 10))
    {
        return value+'0';
    }
    else if((value >= 10)&&(value<16))
    {
        return value+'A'-10;
    }
    else
    {
        std::cerr << "Invalid number between 0 and 15" << std::endl;
        return 0;
    }


}



string_code hashit(std::string const & input)
{
    if(input == "d" /*demand_mode"*/) return eDEMAND;
    if(input == "t" /*wave_trigger"*/) return eTRIGGER;
    if(input == "o" /*operation_mode"*/) return eOPERATION;
    if(input == "tm"/*wave_trigger_mode"*/) return eTRIGGERMODE;
    if(input == "f"/*fader_count"*/) return eFADERCNT;
    if(input == "l"/*rate_limit"*/) return eRATELIMIT;
    if(input == "off"/*wave_offset"*/) return eOFFSET;
    if(input == "m"/*wave_mode"*/) return eWAVEMODE;
    if(input == "c"/*control_mode"*/) return eCTRLMODE;
    if(input == "fre"/*wave_freq"*/) return eFREQ;
    if(input == "a"/*wave_amp"*/) return eAMP;
    if(input == "b"/*broadcast"*/) return eBROADCAST;
    if(input == "p") return ePRESSURE;
    return eDEFAULT;

}

void change_parameter(char value, float float_value, char parameter[])
{

    switch(hashit(parameter))
    {
        case eDEMAND:
            tr_pack.parameter = CTRL_DEMAND_MODE_PAR;
            tr_pack.value = value;
            tr_pack.float_value = 0;
            tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case eTRIGGER:
            tr_pack.parameter = WAVE_GEN_TRIGGER_PAR;
            tr_pack.value = value;
            tr_pack.float_value = 0;
            tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case eOPERATION:
            tr_pack.parameter = OPERATION_PAR;
            tr_pack.value = value;
            tr_pack.float_value = 0;
            tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case eTRIGGERMODE:
            tr_pack.parameter = WAVE_GEN_TRIGGER_MODE_PAR;
            tr_pack.value = value;
            tr_pack.float_value = 0;
            tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case eFADERCNT:
            tr_pack.parameter = WAVE_GEN_FADER_COUNT_PAR;
            tr_pack.value = value;
            tr_pack.float_value = 0;
            tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case eRATELIMIT:
            tr_pack.parameter = POS_RATE_LIMIT_PAR;
            tr_pack.float_value = float_value;
            tr_pack.value = 0;
            tr_pack.value_is_float = true;
            tr_pack.UpdateBuffer();

            break;
        case eOFFSET:
            tr_pack.parameter = WAVE_GEN_OFFSET_PAR;
            tr_pack.float_value = float_value;
            tr_pack.value = 0;
            tr_pack.value_is_float = true;
            tr_pack.UpdateBuffer();

            break;
        case eWAVEMODE:
            tr_pack.parameter = WAVE_GEN_MODE_PAR;
           tr_pack.value = value;
           tr_pack.float_value = 0;
           tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case eCTRLMODE:
            tr_pack.parameter = CTRL_CONTROL_MODE_PAR;
            tr_pack.float_value = float_value;
            tr_pack.value = 0;
            tr_pack.value_is_float = true;
            tr_pack.UpdateBuffer();

            break;
        case eFREQ:
            tr_pack.parameter = WAVE_GEN_FREQ_PAR;
            tr_pack.float_value = float_value;
            tr_pack.value = 0;
            tr_pack.value_is_float = true;
            tr_pack.UpdateBuffer();

            break;
        case eAMP:
            tr_pack.parameter = WAVE_GEN_AMP_PAR;
            tr_pack.float_value = float_value;
            tr_pack.value = 0;
            tr_pack.value_is_float = true;
            tr_pack.UpdateBuffer();

            break;
        case eBROADCAST:
            tr_pack.parameter = CAN_PDO_BROADCAST_PAR;
            tr_pack.value = value;
            tr_pack.float_value = 0;
            tr_pack.value_is_float = false;
            tr_pack.UpdateBuffer();

            break;
        case ePRESSURE:
            tr_pack.parameter = VIEW_PRESSURE;
            tr_pack.value = 0;
            tr_pack.float_value = 0;
            tr_pack.UpdateBuffer();
            tr_pack.TPDO_buff[6] = '4';
            tr_pack.TPDO_buff[7] = '0';
            tr_pack.TPDO_buff[12] = '0';
            tr_pack.TPDO_buff[13] = '0';
            tr_pack.TPDO_buff[14] = '0';
            tr_pack.TPDO_buff[15] = '0';
            tr_pack.TPDO_buff[16] = '0';
            tr_pack.TPDO_buff[17] = '0';
            tr_pack.TPDO_buff[18] = '0';
            tr_pack.TPDO_buff[19] = '0';
            tr_pack.TPDO_buff[20] = '0';
            tr_pack.TPDO_buff[21] = '0';
            std::cout << "Transmit buffer: " << tr_pack.TPDO_buff << std::endl;
            break;
        default:
            std::cerr << "Invalid entry!" << std::endl;
            break;

    }
}
