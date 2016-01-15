/*
 * protocol.c
 *
 *  Created on: Feb 19, 2011
 *      Author: Cao Rui
 */
#include "string.h"
#include "iostream"
#include "SPort.h"
#include <vector>
#include "protocol.h"

#ifdef MCU
#include "UART.h"
#endif

AGVProtocol sendProtocol;
AGVProtocol recProtocol;
int recLen = 0;

int bs_init = 0;
int rs_init = 0;

MSGStates rstatus;

extern std::vector<int> g_ultrasonic;

#ifdef MCU

float volatile m_speed = 0;
float volatile m_angle = 0;

int volatile m_left_pos = 0;
int volatile m_right_pos = 0;
int volatile m_left_delta = 0;
int volatile m_right_delta = 0;

unsigned char send[1024] = {0};
int len = 0;

#else
float m_speed = 0;
float m_angle = 0;

int m_left_pos = 0;
int m_right_pos = 0;
int volatile m_left_delta = 0;
int volatile m_right_delta = 0;
short volatile m_yaw_angle = 0;

unsigned int m_di = 0;

unsigned int m_ad[4] = {0};

#endif

void SInit_Proto(AGVProtocol* agvProtocol,CMDTypes type){

	if(!bs_init){
		Init(&(sendProtocol.buflist),50);

		agvProtocol->header[0] = HEADER0;
		agvProtocol->header[1] = HEADER1;
		agvProtocol->header[2] = HEADER2;
		agvProtocol->header[3] = HEADER3;
		agvProtocol->header[4] = HEADER4;
		agvProtocol->header[5] = HEADER5;
#ifdef MCU
		agvProtocol->srcaddr = DEVCADDR;
		agvProtocol->dstaddr = HOSTADDR;
#else
		agvProtocol->srcaddr = HOSTADDR;
		agvProtocol->dstaddr = DEVCADDR;
#endif
		bs_init = 1;
	}

	agvProtocol->chksum = 0;

	memset(&(agvProtocol->data),0,sizeof(Data));
	agvProtocol->len = 0;

	Clear(&(agvProtocol->buflist));
	agvProtocol->type = type;

}
void RInit_Proto(AGVProtocol* agvProtocol){

	if(!rs_init){

		Init(&(recProtocol.buflist),1024);

		agvProtocol->header[0] = HEADER0;
		agvProtocol->header[1] = HEADER1;
		agvProtocol->header[2] = HEADER2;
		agvProtocol->header[3] = HEADER3;
		agvProtocol->header[4] = HEADER4;
		agvProtocol->header[5] = HEADER5;

#ifdef MCU
		agvProtocol->srcaddr = HOSTADDR;
		agvProtocol->dstaddr = DEVCADDR;
#else
		agvProtocol->srcaddr = DEVCADDR;
		agvProtocol->dstaddr = HOSTADDR;
#endif


		agvProtocol->type = NONE;

		memset(&(agvProtocol->data),0,sizeof(Data));
		agvProtocol->len = 0;

		agvProtocol->chksum = 0;

		Clear(&(agvProtocol->buflist));

		rs_init = 1;

		rstatus = SYNCHEAD0;
	}

}
unsigned char checksum(unsigned char* ch ,int len){
	unsigned char sum = 0;
	int i = 0;
	for(; i < len; ++i)
	{
		sum += ch[i];
	}
	sum &= 0x00ff;
	return sum;
}
int Coder(unsigned char* ch,int* len,AGVProtocol* protol,Data* data){
	*len = 0;

	switch(protol->type){
		case CURRENT:
			break;
		case SPEED:
			protol->data.speed_.system_time = data->speed_.system_time;
			protol->data.speed_.axis_id = data->speed_.axis_id;
			protol->data.speed_.velocity = data->speed_.velocity;
			protol->len = sizeof(SpeedProtocol);
			break;
		case POS:
			protol->data.pos_.system_time = data->pos_.system_time;
			protol->data.pos_.axis_id = data->pos_.axis_id;
			protol->data.pos_.position = data->pos_.position;
			protol->len = sizeof(PosProtocol);
			break;
		case ANGLE:
			protol->data.angle_.system_time = data->angle_.system_time;
			protol->data.angle_.axis_id = data->angle_.axis_id;
			protol->data.angle_.angle = data->angle_.angle;
			protol->len = sizeof(AngleProtocol);
			break;
		case TIME:
			break;
		case SPEED2:
	        	protol->data.speed2_.system_time = data->angle_.system_time;
	        	protol->data.speed2_.speed_v = data->speed2_.speed_v;
	        	protol->data.speed2_.speed_w = data->speed2_.speed_w;
	        	protol->len = sizeof(AngleProtocol);
	        break;
		case SPEED3:
	        	protol->data.speed3_.speed_v = data->speed3_.speed_v;
	        	protol->data.speed3_.speed_w = data->speed3_.speed_w;
	        	protol->data.speed3_.plan_type = data->speed3_.plan_type;
	        	protol->len = sizeof(SpeedProtocol3);
	        break;
		case SPEED_TWO_WHEEL:
			protol->data.speed4_.system_time = data->speed4_.system_time;
			protol->data.speed4_.speed_left = data->speed4_.speed_left;
			protol->data.speed4_.speed_right= data->speed4_.speed_right;
			protol->data.speed4_.temp = 0;
			protol->len = sizeof(SpeedTwoWheelProtocol);
			break;
		case RULTRASONIC:
	        	protol->len = sizeof(RUltraProtocol);
	        break;
		case RYAW_ANGLE:
			protol->len = sizeof(RYawAngleProtocol);
	        break;
		case DO:
	        	protol->data.do_.usdo = data->do_.usdo ;
	        	protol->len = sizeof(DoProtocol);
	    	break;
		case DA:
			protol->data.da_.system_time = data->da_.system_time;
			protol->data.da_.axis_id =  data->da_.axis_id;
	        	protol->data.da_.da_value = data->da_.da_value;
	        	protol->len = sizeof(DaProtocol);
	    	break;
		case RCURRENT:
			break;
		case RSPEED:
			break;
		case RPOS:
			protol->data.r_pos_.system_time = data->r_pos_.system_time;
			protol->data.r_pos_.axis_id = data->r_pos_.axis_id;
			protol->len = sizeof(RPosProtocol);
			break;
		case RDI:
		   	protol->data.r_di_ .system_time = data->r_pos_.system_time;
	        	protol->data.r_di_.axis_id = data->r_pos_.axis_id;
	        	protol->len = sizeof(RDiProtocol);
	    	break;
		case RAD:
	    		protol->data.r_ad_.system_time = data->r_ad_.system_time;
	        	protol->data.r_ad_.axis_id = data->r_ad_.axis_id;
	        	protol->len = sizeof(RAdProtocol);
	    	break;
		case RTIME:
			break;
		case RSPEED2:
			std::cout<<"speed2 err!!!"<<std::cout;
			break;
		default:
			break;
	}
	Write(&(protol->buflist),protol->header,6);

	Write(&(protol->buflist),&(protol->srcaddr),1);
	Write(&(protol->buflist),&(protol->dstaddr),1);
	Write(&(protol->buflist),(unsigned char*)&(protol->type),1);
	Write(&(protol->buflist),(unsigned char*)&(protol->len),1);
	Write(&(protol->buflist),(unsigned char*)&(protol->data),protol->len);


	Read(&(protol->buflist),ch,len);
	protol->chksum = checksum(ch+6,*len-6);
	ch[*len] = protol->chksum;
	(*len)++;
	return *len;
}
int Decoder(AGVProtocol* protol,unsigned char* ch,int len){
		memcpy(&(protol->data),ch+4,sizeof(Data));

	switch(protol->type){
		case CURRENT:
			break;
		case SPEED:
			m_speed = protol->data.speed_.velocity;
			m_speed = m_speed / (1<<16);
			break;
		case POS:

		   // std::cout<<"decode:"<<protol->data.pos_.axis_id<<std::endl;
			if (protol->data.pos_.axis_id == 0){
			    //std::cout<<"left pos:"<<protol->data.pos_.position<<std::endl;
				m_left_delta = protol->data.pos_.position;
	    		m_left_pos = protol->data.pos_.system_time;
				//std::cout<<"left_pos:"<<m_left_pos<<std::endl;
			}else if (protol->data.pos_.axis_id == 1){
				m_right_delta = protol->data.pos_.position;
	    		m_right_pos = protol->data.pos_.system_time;
				//std::cout<<"right_pos:"<<m_right_pos<<std::endl;
			}

			break;
		case YAW_ANGLE:
			m_yaw_angle = protol->data.yaw_angle_.yaw; 
			break;
	    case ULTRASONIC:
			g_ultrasonic.clear();
			g_ultrasonic.resize(24);
			for (int i = 0; i < 24; ++i) {
				g_ultrasonic[i] = protol->data.ultra_.length[i];
			}
			break;
		case ANGLE:
			m_angle = protol->data.angle_.angle;
			m_angle = m_angle / (1<<16);
			break;
		case RDI:
		    m_di = protol->data.r_di_.usdi;
		    break;
		case RAD:
			if(protol->data.r_ad_.axis_id < 4){
				m_ad [protol->data.r_ad_.axis_id] = protol->data.r_ad_.ad_value;
			}
			break;
		case TIME:
			break;
		case RCURRENT:
			break;
		case RSPEED:
			break;
		case RPOS:
#ifdef MCU
				if(protol->data.angle_.axis_id == 0){
					CreatePos(send,&len,0,m_left_pos);
					uart0SendStr(send,len);
				}else if(protol->data.angle_.axis_id == 0){
					CreatePos(send,&len,1,m_right_pos);
					uart0SendStr(send,len);
				}
#endif
				break;
		case RTIME:
			break;
		default:
			break;
	}

		return 1;
	return 0;
}

void CreateTwoWheelSpeed(unsigned char* ch,int* len,short speedLeft, short speedRight){
  Data data;

  data.speed4_.system_time = 0xffffffff;
  data.speed4_.speed_left = speedLeft;
  data.speed4_.speed_right = speedRight;

  SInit_Proto(&sendProtocol,SPEED_TWO_WHEEL);

  Coder(ch,len,&sendProtocol,&data);
}


void CreateSpeed2(unsigned char* ch,int* len,int v,int w){
  Data data;

  data.speed2_.system_time = 0xffffffff;
  data.speed2_.speed_v = v;
  data.speed2_.speed_w = w;

  SInit_Proto(&sendProtocol,SPEED2);

  Coder(ch,len,&sendProtocol,&data);
}
void CreateSpeed3(unsigned char* ch, int* len, int v, int w, int plan_type) {
  Data data;

  data.speed3_.speed_v = v;
  data.speed3_.speed_w = w;
  data.speed3_.plan_type = plan_type;

  SInit_Proto(&sendProtocol,SPEED3);

  Coder(ch,len,&sendProtocol,&data);
}
void CreateRUltra(unsigned char* ch, int* len) {
  Data data;

  SInit_Proto(&sendProtocol,RULTRASONIC);

  Coder(ch,len,&sendProtocol,&data);
}

void createYawAngle(unsigned char* ch, int* len) {
  Data data;

  SInit_Proto(&sendProtocol,RYAW_ANGLE);

  Coder(ch,len,&sendProtocol,&data);
}

void CreateDO(unsigned char* ch,int* len,int id,unsigned int usdo){
  Data data;

  data.do_.usdo = usdo;

  SInit_Proto(&sendProtocol,DO);

  Coder(ch,len,&sendProtocol,&data);
}
void CreateRDI(unsigned char* ch,int* len,int id){
  Data data;

  data.r_di_.system_time = 0xffffffff;
  data.r_di_.axis_id = id;

  SInit_Proto(&sendProtocol,RDI);

  Coder(ch,len,&sendProtocol,&data);

  rs_init = 0; //清空状态机
}
void CreateSpeed(unsigned char* ch,int* len,int id,float v){
	Data data;

	data.speed_.system_time = 0xffffffff;
	data.speed_.axis_id = id;
	data.speed_.velocity = v ;

	SInit_Proto(&sendProtocol,SPEED);

	Coder(ch,len,&sendProtocol,&data);
}
void CreatePos(unsigned char* ch,int* len,int id,int pos){
	Data data;

	data.pos_.system_time = 0xffffffff;
	data.pos_.axis_id = id;
	data.pos_.position = pos;

	SInit_Proto(&sendProtocol,POS);

	Coder(ch,len,&sendProtocol,&data);
}
void CreateDA(unsigned char* ch,int* len,int id,float v){
  unsigned int i_value = (v / 5.0) * 0x000fff ;
  i_value &= 0x000fff;

  Data data;

  data.da_.system_time = 0xffffffff;
  data.da_.axis_id = id;
  data.da_.da_value = i_value;

  SInit_Proto(&sendProtocol,DA);

  Coder(ch,len,&sendProtocol,&data);
}
void CreateRAD(unsigned char* ch,int* len,int id){
  Data data;

  data.r_ad_.system_time = 0xffffffff;
  data.r_ad_.axis_id = id;

  SInit_Proto(&sendProtocol,RAD);

  Coder(ch,len,&sendProtocol,&data);

  rs_init = 0;
}
void CreateRPos(unsigned char* ch,int* len,int id){
	Data data;

	data.r_pos_.system_time = 0xffffffff;
	data.r_pos_.axis_id = id;

	SInit_Proto(&sendProtocol,RPOS);

	Coder(ch,len,&sendProtocol,&data);

	rs_init = 0;
}
void CreateAngle(unsigned char* ch,int* len,int id,float angle){
	Data data;

	data.angle_.system_time = 0xffffffff;
	data.angle_.axis_id = id;
	data.angle_.angle = angle * (1<<16);

	SInit_Proto(&sendProtocol,ANGLE);

	Coder(ch,len,&sendProtocol,&data);
}
float GetSpeed(void){
	return m_speed;
}
float GetAngle(void){
	return m_angle;
}
unsigned int GetDI(){
  return m_di;
}
float GetAD(int id){
  float ad = 0;
  if(id < 4){
    ad = m_ad[id];
    ad = 5.0 * (ad / 4096);
  }
  return ad;
}
int GetDelta(int id) {
  if (id == 0) {
    return m_left_delta;
  } else if (id == 1) {
    return m_right_delta;
  }
}
short getYaw(void)
{
	return m_yaw_angle;
}
int GetPos(int id){
	if(id == 0){
	    //std::cout<<"left_pos:"<<m_left_pos<<std::endl;
		return m_left_pos;
	}else if( id == 1){
	  //std::cout<<"right_pos:"<<m_right_pos<<std::endl;
		return m_right_pos;
	}
}

int IRQ_CH(unsigned char c){
	unsigned char tmp[50];
	int len = 0;

	RInit_Proto(&recProtocol);

#ifdef MCU
	zyIrqDisable();
#endif

	switch(rstatus){
	case SYNCHEAD0:
		if(c==HEADER0) rstatus = SYNCHEAD1;
		else rstatus = SYNCHEAD0;
		break;
	case SYNCHEAD1:
		if(c==HEADER1) rstatus = SYNCHEAD2;
		else rstatus = SYNCHEAD0;
		break;
	case SYNCHEAD2:
		if(c==HEADER2) rstatus = SYNCHEAD3;
		else rstatus = SYNCHEAD0;
		break;
	case SYNCHEAD3:
		if(c==HEADER3) rstatus = SYNCHEAD4;
		else rstatus = SYNCHEAD0;
		break;
	case SYNCHEAD4:
		if(c==HEADER4) rstatus = SYNCHEAD5;
		else rstatus = SYNCHEAD0;
		break;
	case SYNCHEAD5:
		if(c==HEADER5){
			Clear(&(recProtocol.buflist));
			rstatus = SRCADDR;
		}
		else rstatus = SYNCHEAD0;
		break;
	case SRCADDR:
		if(recProtocol.srcaddr == c)
		{
			Write(&(recProtocol.buflist),&c,1);
			rstatus = DSTADDR;
		}
		else rstatus = SYNCHEAD0;
		break;
	case DSTADDR:
		if(recProtocol.dstaddr == c)
		{
			Write(&(recProtocol.buflist),&c,1);
			rstatus = CMDFIELD;
		}
		else rstatus = SYNCHEAD0;
		break;
	case CMDFIELD:
		recProtocol.type = (CMDTypes)c;
		Write(&(recProtocol.buflist),&c,1);
		rstatus = DATLEN;
		//std::cout<<"cmd"<<std::endl;
		break;
	case DATLEN:
		recProtocol.len = c;
		Write(&(recProtocol.buflist),&c,1);
		rstatus = DATFIELD;
		break;
	case DATFIELD:
	    //Print(&(recProtocol.buflist));
		Write(&(recProtocol.buflist),&c,1);
		//std::cout<<"data len:"<<Size(&(recProtocol.buflist))<<" rlen:"<<recProtocol.len+4<<std::endl;
		if(Size(&(recProtocol.buflist)) < (recProtocol.len + 4)){

		}else{
			rstatus = CHKSUM;
			//std::cout<<"check sum"<<std::endl;
		}
		break;
	case CHKSUM:

		Read(&(recProtocol.buflist),&tmp[0],&len);
		//std::cout<<"rec len:"<<len<<std::endl;
		//std::cout<<"rec check:"<<int(c)<<" check:"<<int(checksum(&tmp[0],len))<<std::endl;
		if(c == checksum(&tmp[0],len)){
		  //std::cout<<" check sum"<<std::endl;
			if(!Decoder(&recProtocol,&tmp[0],len)){
				//����ʧ��
				rstatus = SYNCHEAD0;
#ifdef MCU
				zyIrqEnable();
#endif
				return 0;
			}
		}else{
      std::string str = cComm::ByteToHexString(tmp, len);
      std::string strc = cComm::ByteToHexString(&c, 1);
      std::cout << "check sum err: " << str << " c: " << strc << std::endl;
		}

		rstatus = SYNCHEAD0;
#ifdef MCU
		zyIrqEnable();
#endif
		return 1;
		break;
	default:
		rstatus = SYNCHEAD0;
		rs_init = 1;
	}
#ifdef MCU
	zyIrqEnable();
#endif
	return 0;
}


//
//
//void initms(MsgState *ms)
//{
//	ms->GoodPack=0;
//	ms->State = SYNCHEAD0;
//	ms->chk = 0;
//	ms->sz = 0;
//}
//#ifdef MCU
//int DecodeMsg(volatile CBuf *CBufptr, MACProtocol *msgptr, MsgState *ms)
//#else
//int DecodeMsg(CBuf *CBufptr, MACProtocol *msgptr, MsgState *ms)
//#endif
//{
//	char c,*temp;
//#ifdef MCU
//	DISIRQ();
//#endif
//	while(CanGetCBuf(CBufptr)){
//		ReadCBuf(CBufptr, (char *)&c, 1);
//#ifdef MCU
//		ENIRQ();
//#endif
////		printf("%02x [%04x] ",c,ms->chk);
//		switch(ms->State){
//			case SYNCHEAD0:
//				ms->chk=c;
//				ms->sz=0;
//				ms->GoodPack=0;
//				if(c==HEADER0) ms->State = SYNCHEAD1;
//				else ms->State = SYNCHEAD0;
//				break;
//			case SYNCHEAD1:
//				ms->GoodPack =0;
//				ms->chk+=c;
//				if(c==HEADER1) ms->State = SYNCHEAD2;
//				else ms->State = SYNCHEAD0;
//				break;
//			case SYNCHEAD2:
//				ms->chk+=c;
//				if(c==HEADER2) ms->State = SYNCHEAD3;
//				else ms->State = SYNCHEAD0;
//				break;
//			case SYNCHEAD3:
//				ms->chk+=c;
//				if(c==HEADER3) ms->State = SYNCHEAD4;
//				else ms->State = SYNCHEAD0;
//				break;
//			case SYNCHEAD4:
//				ms->chk+=c;
//				if(c==HEADER4) ms->State = SYNCHEAD5;
//				else ms->State = SYNCHEAD0;
//				break;
//			case SYNCHEAD5:
//				ms->chk+=c;
//				if(c==HEADER5) ms->State = SRCADDR;
//				else ms->State = SYNCHEAD0;
//				break;
//			case SRCADDR:
//				ms->chk+=c;
//				msgptr->srcaddr=c;
//				ms->State = DSTADDR;
//				break;
//			case DSTADDR:
//				ms->chk+=c;
//				msgptr->dstaddr=c;
//				ms->State = CMDFIELD;
//				break;
//			case CMDFIELD:
//				ms->chk+=c;
//				msgptr->cmd=c;
//				ms->State = DATLEN;
//				break;
//			case DATLEN:
//				ms->chk+=c;
//				msgptr->dlen=c;
//				ms->State = DATFIELD;
//				ms->sz=0;
//				//temp=(char*)&(msgptr->datapackage);// assign msg struct data point to *temp
//				break;
//			case DATFIELD:
//				if(ms->sz<msgptr->dlen){
//					ms->chk+=c;
//					ms->State = DATFIELD;
//					temp[ms->sz++]=c;// fill in the msg structure data field.
////					printf("%x ",temp[ms->sz-1]);
//				}
//				else {
//					ms->State = CHKSUM;
//					msgptr->chksum=(unsigned short)c;
//				}
//				break;
//			case CHKSUM:
//				ms->State = SYNCHEAD0;
//				msgptr->chksum+=(unsigned short)(c<<8);
//				if(msgptr->chksum==ms->chk) ms->GoodPack = 1;
//				else ms->GoodPack = -1;
//				break;
//			default:
//				ms->State = SYNCHEAD0;
//				ms->chk = 0;
//				ms->sz = 0;
//				ms->GoodPack = 0;
//				break;
//			}
//#ifdef MCU
//		DISIRQ();
//#endif
//	}
//#ifdef MCU
//	ENIRQ();
//#endif
//	return ms->GoodPack;
//}
