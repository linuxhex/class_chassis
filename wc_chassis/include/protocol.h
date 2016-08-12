/*
 * protocol.h
 *
 *  Created on: Jan 16, 2015
 *      Author: wht
 */

#ifndef _PROTOCOL_WANGHONGTAO_2015_01_16_
#define _PROTOCOL_WANGHONGTAO_2015_01_16_

#include "buffer.h"

#define HOSTADDR 0x01
#define DEVCADDR 0x02

#define HEADER0  0x3E
#define HEADER1  0x4D
#define HEADER2  0x5C
#define HEADER3  0x6B
#define HEADER4  0x7A
#define HEADER5  0xA5


#define BUF_SIZE 256
typedef enum{
    SYNCHEAD0,
    SYNCHEAD1,
    SYNCHEAD2,
    SYNCHEAD3,
    SYNCHEAD4,
    SYNCHEAD5,
    SRCADDR,
    DSTADDR,
    CMDFIELD,
    DATLEN,
    DATFIELD,
    CHKSUM
}MSGStates;

typedef enum{
  NONE = 0x00,
  CURRENT = 0x01,
  SPEED = 0x02,
  POS = 0x03,
  ANGLE = 0x04,
  SPEED2 = 0x05,
  DO = 0x06,
  DI = 0x07,
  DA = 0x08,
  SPEED3 = 0x09,
  SPEED_TWO_WHEEL = 0x0a,
  RREMOTE_ID = 0x40,
  REMOTE_ID = 0x41,
  RREMOTE_VERIFY_KEY = 0x42,
  REMOTE_VERIFY_KEY = 0x43,
  CHARGE_CMD = 0x90,     //自动充电
  CHARGE_STATUS = 0x91,     //自动充电
  TIME = 0x7e,
  RTIME = 0xfe,
  RCURRENT = 0x81,
  RSPEED = 0x82,
  RPOS = 0x83,
  RANGLE = 0x84,
  RSPEED2 = 0x85,
  RULTRASONIC = 0x86,
  ULTRASONIC = 0x88,
  RDI = 0x87,
  RAD = 0x89,
  RYAW_ANGLE = 0x8a, 	//Request for Yaw
  YAW_ANGLE = 0x8b,
  RREMOTE_CMD = 0x8c, 	//Request for remote cmd
  REMOTE_CMD = 0x8d,
  REMOTE_RET = 0x8e,   //send remote ret
  SHUTDOWN_CMD = 0x94, //send shutdown cmd on charging
}CMDTypes;

typedef struct _MsgState
{
   unsigned short sz;           //decode size of data in the msg as per header
   MSGStates  State;        //decode state of msg decode
   unsigned short  chk;          //decode check sum data
   int  GoodPack;
}MsgState;                      //trn is =1 xms after transmit mode, and 0 at end of packet transmit

typedef struct _Current
{
    int system_time;
    int axis_id;
    int current;

}CurrentProtocol;

typedef struct _Speed
{
    int system_time;
    int axis_id;
    int velocity;

}SpeedProtocol;

typedef struct _Speed2
{
    int system_time;
    int speed_v;
    int speed_w;
}SpeedProtocol2;

typedef struct _SpeedTwoWheel
{
    int system_time; //system time
    short speed_left;     //left wheel speed
    short speed_right;  //right wheel speed
    int temp;
}SpeedTwoWheelProtocol;


typedef struct _Speed3
{
  int speed_v;
  int speed_w;
  int plan_type;
}SpeedProtocol3;

typedef struct _Pos
{
  int system_time;
  int axis_id;
  int position;

}PosProtocol;

typedef struct _Angle
{
    int system_time;
    int axis_id;
    int angle;

}AngleProtocol;

typedef struct _DO
{
  unsigned int usdo;
}DoProtocol;

typedef struct _DI
{
  unsigned int usdi;
}DiProtocol;

typedef struct _RAD
{
  int system_time;
  int axis_id;
  unsigned int ad_value;
}RAdProtocol;

typedef struct _DA
{
  int system_time;
  int axis_id;
  unsigned int da_value;
}DaProtocol;

typedef struct _Time
{
    int system_time;
    int remote_system_time;

}TimeProtocol;

typedef struct _RCurrent
{
    int system_time;
    int axis_id;

}RCurrentProtocol;

typedef struct _RSpeed
{
    int system_time;
    int axis_id;

}RSpeedProtocol;

typedef struct _RPos
{
    int system_time;
    int axis_id;

}RPosProtocol;

typedef struct _RAngle
{
    int system_time;
    int axis_id;

}RAngleProtocol;

typedef struct _RDI
{
    int system_time;
    int axis_id;
    unsigned int usdi;

}RDiProtocol;

typedef struct _RTime
{
    int system_time;

}RTimeProtocol;

typedef struct _RUltra {
}RUltraProtocol;

typedef struct _RYAWANGLE {
}RYawAngleProtocol;

typedef struct _YAWANGLE {
  short yaw;
  short roll;
  short pitch;
}YawAngleProtocol;

typedef struct _RREMOTE_ID
{
  unsigned char id;
}RRemoteIDProtocol;


typedef struct _CHARGE_CMD
{
  unsigned char cmd;
}RChargeCmdProtocol;

typedef struct _CHARGE_STATUS
{
  unsigned char  status;
}RChargeStatusProtocol;

typedef struct _SHUTDOWN_CM
{
  unsigned char cmd;
}ShutDownProtocol;


typedef struct _RREMOTE_VERIFY_KEY
{
  unsigned int key;
}RRemoteVerifyKeyProtocol;

typedef struct _REMOTE_VERIFY_KEY
{
  unsigned int check_key;
}RemoteVerifyKeyProtocol;

typedef struct _RREMOTECMD {
}RRemoteCmdProtocol;

typedef struct _REMOTECMD {
  unsigned char cmd;
  unsigned char index_H;
  unsigned char index_L;
}RemoteCmdProtocol;

typedef struct _REMOTERET {
  unsigned short ret;
}RemoteRetProtocol;

typedef struct _Ultra {
  uint8_t length[24];
}UltraProtocol;

typedef union _Data{
  SpeedProtocol speed_;
  SpeedProtocol2 speed2_;
  SpeedProtocol3 speed3_;
  SpeedTwoWheelProtocol speed4_;
  PosProtocol pos_;
  AngleProtocol angle_;
  DaProtocol da_;
  DoProtocol do_;
  DiProtocol di_;
  RDiProtocol r_di_;
  RPosProtocol r_pos_;
  RAdProtocol r_ad_;
  RUltraProtocol r_ultra_;
  UltraProtocol ultra_;
  RYawAngleProtocol r_yaw_anle_;
  YawAngleProtocol yaw_angle_;
  RRemoteCmdProtocol r_remote_cmd_;
  RemoteCmdProtocol remote_cmd_;
  RemoteRetProtocol remote_ret_;
  RRemoteIDProtocol r_remote_id_;
  RRemoteVerifyKeyProtocol r_remote_verify_key_;
  RemoteVerifyKeyProtocol remote_verify_key_;
  RChargeCmdProtocol chargeCmd;
  RChargeStatusProtocol chargeStatus;
  ShutDownProtocol shutdownCmd;
}Data;

typedef struct _AGVProtocol
{
    unsigned char header[6];
    unsigned char srcaddr;
    unsigned char dstaddr;
    //char *data;
    //TransData DataProtocol;
    CMDTypes type;
    Data data;
    int len;

    BufList buflist;

    unsigned char chksum;

}AGVProtocol;

#ifdef MCU
extern float volatile m_speed;
extern float volatile m_angle;

extern int volatile m_left_pos;
extern int volatile m_right_pos;
#else
extern float m_speed;
extern float m_angle;

extern int m_left_pos;
extern int m_right_pos;
#endif

float GetSpeed(void);
int GetPos(int id);
int GetDelta(int id);
void getYaw(short& yaw_angle, short& pitch_angle, short& roll_angle);
void getRemote(unsigned char& cmd, unsigned short& index);
unsigned int GetDI();
float GetAD(int id);
float GetAngle(void);
void CreateSpeed(unsigned char* ch,int* len,int id,float v);
void CreateRemoteRet(unsigned char* ch,int* len,int id, unsigned short ret);
void CreateTwoWheelSpeed(unsigned char* ch,int* len,short speedLeft, short speedRight);
void CreateSpeed2(unsigned char* ch,int* len,int v,int w);
void CreateSpeed3(unsigned char* ch,int* len,int v,int w, int plan_type);
void CreateRUltra(unsigned char* ch, int* len);
void createYawAngle(unsigned char* ch, int* len);
void createRemoteCmd(unsigned char* ch, int* len);
void CreatePos(unsigned char* ch,int* len,int id,int pos);
void CreateAngle(unsigned char* ch,int* len,int id,float angle);
void CreateDO(unsigned char* ch,int* len,int id,unsigned int usdo);
void CreateDA(unsigned char* ch,int* len,int id,float v);
void CreateRAD(unsigned char* ch,int* len,int id);
void CreateRPos(unsigned char* ch,int* len,int id);
void CreateRDI(unsigned char* ch,int* len,int id);
void CreateRemoteVerifyKey(unsigned char* ch,int* len,int id, unsigned int key);
void CreateRemoteID(unsigned char* ch,int* len,unsigned char remote_id);
unsigned int getRemoteVerifyKey(void);
unsigned char checksum(unsigned char* ch ,int len);
int IRQ_CH(unsigned char c);
void CreateChargeCmd(unsigned char* ch,int *len,unsigned char cmd);
unsigned char getChargeStatusValue(void);
void CreateShutdownCmd(unsigned char* ch,int *len,unsigned char cmd);

#endif//_PROTOCOL_WANGHONGTAO_2015_01_16_
