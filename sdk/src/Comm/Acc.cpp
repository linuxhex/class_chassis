#include "math.h"
#include "Acc.h"

//加减速限幅 current 当前值 acc加速度 dec减速度 set设定值
double Acc::CalAcc(double dt,double current,double acc,double dec,double set){

  double t_acc = acc*dt;
  double t_dec = dec*dt;
  double diff = 0.0;
  //std::cout<<"t_acc"<<t_acc<<" t_dec:"<<t_dec<<std::endl;
  //std::cout<<"set:"<<set<<std::endl;

  //当前速度正负区间内都是减速
  if(fabs(set) < fabs(current)){
    diff = t_dec;
  }
  else{
    diff = t_acc;
  }

  //速度限幅
  if(fabs(set - current) >= diff){
    if(set > current){
      return (current + diff);
    }
    else{
      return (current - diff);
    }
  }
  return set;

}
//加减速限幅 current 当前值 set设定值
double Acc::CalCAcc(double p,double current,double set){

  double res = 0;
  res = (1 - p)*current + p*(set);
  return res;

}
