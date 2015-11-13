#ifndef _WANGHONGTAO_ACC_2014_12_19_H_
#define _WANGHONGTAO_ACC_2014_12_19_H_

class Acc{
public:
  Acc(){};
  ~Acc(){};

  static double CalAcc(double dt,double current,double acc,double dec,double set);
  static double CalCAcc(double p,double current,double set);

};

#endif//_WANGHONGTAO_ACC_2014_12_19_H_
