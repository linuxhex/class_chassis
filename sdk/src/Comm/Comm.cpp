
#include "Comm.h"
#include <cctype>

#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"

using namespace boost;
using namespace boost::xpressive;
using namespace boost::filesystem;

#include <fstream>
#include <string>
#include <map>

using namespace std;

unsigned char* cComm::bit_value_ = new unsigned char[8];
bool load_value = cComm::load();

map<string , int> Config::m_configs;
string Config::m_RunPath="";

bool loadConfigResult = Config::loadConfig();

cComm::cComm(void) { }

cComm::~cComm(void) { }

bool cComm::load() {
  bit_value_[0] = 1;
  bit_value_[1] = 2;
  bit_value_[2] = 4;
  bit_value_[3] = 8;
  bit_value_[4] = 16;
  bit_value_[5] = 32;
  bit_value_[6] = 64;
  bit_value_[7] = 128;
  return true;
}

int cComm::SplitString(const string& input,
                       const string& delimiter, vector<string>& results) {
  int iPos = 0;
  int newPos = -1;
  int sizeS2 = (int)delimiter.size();
  int isize = (int)input.size();
  results.clear();

  if (isize == 0 || sizeS2 == 0) {
    return 0;
  }

  vector<int> positions;

  int numFound = 0;

  while ((newPos = input.find (delimiter, iPos))>0) {
    positions.push_back(newPos);
    iPos = newPos + sizeS2;
    numFound++;
  }

  if (numFound == 0) {
    if (input.size()>0) {
      results.push_back(input);
    }
    return 1;
  }

  if (positions.back() != isize) {
    positions.push_back(isize);
  }

  int offset = 0;
  string s("");

  for (int i=0; i < (int)positions.size(); ++i) {
    s = input.substr( offset, positions[i]- offset );

    offset = positions[i] + sizeS2;

    if (s.length()>0 && !all(s,is_space())) {
      results.push_back(s);
    }
  }
  return numFound;
}

void cComm::StringUpper(string& strDes) {
  std::transform(strDes.begin(),strDes.end(),strDes.begin(),::toupper);  //ת��д
}

string cComm::ByteToHexString(unsigned char* pData,int iLen) {
  string strTmp;

  char chTmp[8];
  for(int i = 0 ;i<iLen ;++i) {
    sprintf(chTmp, "%02X ", pData[i]);
    strTmp += chTmp;
  }
  return strTmp;
}

int cComm::HexStringToByte(unsigned char** pChar,int& iLen,string strHex) {
  vector<string> v_str;
  SplitString(strHex," ",v_str);
  vector<string>::iterator it = v_str.begin();

  if (v_str.size() <=0) {
    return 0;
  }
  *pChar = new unsigned char[v_str.size()];
  memset(*pChar,0,v_str.size());

  unsigned int index(0);
  for (;it != v_str.end();++it) {
    if (((*it).length() > 2) ||(index >= v_str.size()))
    {
      continue;
    }
    else if ( (*it).length() == 1)
    {
      char ch[1] = {0};
      memcpy(ch,(*it).c_str(),1);
      if ((ch[0] >='0') && (ch[0] <='9'))
        (*pChar)[index] = ch[0] - 48;//�ַ�0��ASCIIֵΪ48
      else if ((ch[0]>='a') && (ch[0]<='f'))
        (*pChar)[index] =ch[0] -'a' + 10;
      else if ((ch[0]>='A') && (ch[0]<='F'))
        (*pChar)[index] = ch[0]-'A'+10;
    }
    else if ((*it).length() == 2)
    {
      char ch[1] = {0};
      memcpy(ch,(*it).c_str(),1);
      if ((ch[0] >='0') && (ch[0] <='9'))
        (*pChar)[index] = ch[0] - 48;//�ַ�0��ASCIIֵΪ48
      else if ((ch[0]>='a') && (ch[0]<='f'))
        (*pChar)[index] = ch[0] -'a' + 10;
      else if ((ch[0]>='A') && (ch[0]<='F'))
        (*pChar)[index] = ch[0]-'A'+10;

      ch[0] = 0 ;
      memcpy(ch,(*it).c_str()+1,1);
      if ((ch[0] >='0') && (ch[0] <='9'))
        (*pChar)[index] = (*pChar)[index]*16 + (ch[0] - 48);//�ַ�0��ASCIIֵΪ48
      else if ((ch[0]>='a') && (ch[0]<='f'))
        (*pChar)[index] = (*pChar)[index]*16 + (ch[0] -'a' + 10);
      else if ((ch[0]>='A') && (ch[0]<='F'))
        (*pChar)[index] = (*pChar)[index]*16 + (ch[0]-'A'+10);
    }
    index++;
  }
  iLen = v_str.size();
  return iLen;
}

void cComm::Find_files(const string &strdir,const string& filename,vector<string>& v) {
  path dir(strdir);
  static xpressive::sregex_compiler rc;
  if(!rc[filename].regex_id()) {
    string str = replace_all_copy(replace_all_copy(filename,".","\\."),"*",".*");
    rc[filename] = rc.compile(str);
  }
  if (!exists(dir) || !is_directory(dir)) {
    return;
  }
}

string cComm::GetRunPath() {
  return boost::filesystem::initial_path<boost::filesystem::path>().string();
}

bool cComm::FileExist(string strPath) {
  return exists(strPath);
}

string cComm::Get_FileName(string strPath) {
  vector<string> v_path;
  SplitString(strPath,".",v_path);
  if ((v_path.size()<2)) {
    if(v_path.size()==1){
      return v_path[0];
    } else {
      return "";
    }
  }

  string strPath_name = v_path[0];
  vector<string> vp ;
  cComm::SplitString(strPath_name,"\\",vp);
  if (vp.size()>1) {
    return vp[vp.size()-1];
  } else {
    cComm::SplitString(strPath_name,"/",vp);
    if (vp.size()>1) {
      return vp[vp.size()-1];
    }
  }
  return strPath_name;
}

std::string cComm::Get_FileType(string strPath) {
  vector<string> v_path;
  SplitString(strPath,".",v_path);
  if ((v_path.size()>1)) {
    return v_path[v_path.size() -1];
  }
  return strPath;
}

unsigned int cComm::GetBit(const unsigned char &data,const unsigned char &pos) {
  return data & bit_value_[pos];
}

void cComm::sleep(float time) {
  boost::system_time const timeout=boost::get_system_time() + boost::posix_time::milliseconds((long)time);
  boost::thread::sleep(timeout);
}

bool Config::loadConfig() {
  m_RunPath = boost::filesystem::initial_path<boost::filesystem::path>().string();
  m_RunPath+="/config.txt";

  m_configs.clear();
  std::fstream stream(m_RunPath.c_str(),ios::in|ios::out);

  string key;
  int value;

  while(stream>>key) {
    stream>>value;
    m_configs[key] = value;
  }

  stream.close();
  return true;
}

int Config::getConfig(char* name) {
  map<string , int>::const_iterator cit  = m_configs.find(name) ;
  if (cit != m_configs.end()) {
    return cit->second;
  }
  return 0;
}

void Config::setConfig(char *name , int value) {
  m_configs[name] = value;
}


bool Config::SaveConfig() {
  std::fstream stream("config.txt",ios::in|ios::out);

  string key;
  int value;

  std::map<std::string, int>::iterator it;
  for (it = m_configs.begin(); it!= m_configs.end(); it++) {
    key = it->first;
    value = it->second;
    stream<<key<<" "<<value<<std::endl;
  }
  stream.close();
  return true;
}
