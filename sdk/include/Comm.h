#ifndef _COMM_KYOSHO_20110903_
#define _COMM_KYOSHO_20110903_

#include <string>
#include <algorithm> 
#include <vector>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>


#include <boost/algorithm/string.hpp>
#include <boost/xpressive/xpressive_dynamic.hpp>
#include <boost/filesystem.hpp>
#include <boost/math/common_factor_rt.hpp>


using namespace std;
//using namespace boost;


class cComm
{
public:
	cComm(void);
	~cComm(void);

	static void sleep(float time);
	static int SplitString(const string& input, const string& delimiter, vector<string>& results);
	static void StringUpper(string& strDes);

	template<typename T>
	static bool RangeIt(T& res,T min,T max){

		if (res < min)
		{
			res = min;
			return false;
		}
		if (res > max)
		{
			res = max;
			return false;
		}
		return true;
	};
	template<typename T>
	static bool SaveGetFromBuffer(T& res,char* buffer,T tmin,T tmax){
		T t;
		memcpy(&t,buffer,sizeof(T));
		if (!RangeIt(t,tmin,tmax))
		{
			memset(&tmax,0,sizeof(T));
			return false;
		}else
		{
			res = t;
			return true;
		}
	};
	template<typename T>
	static bool SetBufferOfType(char* buffer,const T& source)
	{
		if (!buffer)
		{
			return false;
		}
		memset(buffer,0,sizeof(T));
		memcpy(buffer,&source,sizeof(T));
		return true;
	};
	template<typename T>
	static bool num_valid(const char* str)
	try{
		boost::lexical_cast<T>(str);
		return true;
	}
	catch(boost::bad_lexical_cast)
	{
		return false;
	};
	template <class T> 
	static string ConvertToString(T value) {
		stringstream ss;
		ss << value;
		return ss.str();
	};
	template <class T>
	static T ConvertToNum(T &res,const string &str){
		try
		{
			res = boost::lexical_cast<T>(str.c_str());
			return res;
		}
		catch (boost::bad_lexical_cast& e)
		{
			return res;
		}	
	};
	template<class T>
	static T Max(T t1 ,T t2){
		if (t1 > t2)
		{
			return t1;
		}
		else
		{
			return t2;
		}
	};
	template<class T>
	static T Min(T t1 ,T t2){
		if (t1 < t2)
		{
			return t1;
		}
		else
		{
			return t2;
		}
	};
	template<class T>
	static T Lcm(T t1, T t2)
	{
		return boost::math::lcm(t1,t2);
	};
	static string ByteToHexString(unsigned char* pData,int iLen);
	static int HexStringToByte(unsigned char** pChar,int& iLen,string strHex);

	//static void Find_files(const path& dir,const string& filename,vector<path>& v);
	static void Find_files(const string &strdir,const string& filename,vector<string>& v);
	static string GetRunPath();
	static bool FileExist(string strPath);
	static string Get_FileName(string strPath);
	static string Get_FileType(string strPath);

	static bool load();
	static unsigned char* bit_value_;
	static unsigned int GetBit(const unsigned char &data,const unsigned char &pos );
};

class Config
{
private:
	static std::map<std::string , int> m_configs;
	static string m_RunPath;
public:
	static bool loadConfig();
	static bool SaveConfig();
	static int getConfig(char* name);
	static void setConfig(char *name , int value);

};

#define SLEEP(time) cComm::sleep(time)
#define SDelete( ptr )    do { if ( ptr ) { delete ( ptr ); ( ptr ) = NULL; } } while(0)
#endif //_COMM_KYOSHO_20110903_
