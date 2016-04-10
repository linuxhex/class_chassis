/* Copyright(C) Gaussian Automation. All rights reserved.
 */

#ifndef _BOOST_SERIAL_PORT_WANGHONGTAO_2014_08_30_
#define _BOOST_SERIAL_PORT_WANGHONGTAO_2014_08_30_

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "ByteList.hpp"
#include "Comm.h"

class Socket {
 public:
  Socket();
  ~Socket();

  void Init(const std::string& host_name, const std::string& port);

  void Send_data(U8* s_data, U16 len);
  void Read_data(U8* r_data, int &len, int need, int timeout);

 private:
  U8 m_szReadTemp[1024];
  ByteList m_lReadBuffer;

  U8 m_szWriteBuffer[1024];
  int m_nWriteBufferSize;

  std::string port_;
  std::string host_name_;

  bool open();
  void write();
  void read();

  void read_callback(const boost::system::error_code& error, size_t bytes_transferred);
  void wait_callback(const boost::system::error_code& error);

  boost::asio::ip::tcp::socket* socket_;
  boost::asio::io_service ios_;
//  boost::asio::io_service::work work(ios_);
  boost::thread* thread_;
  bool BeginThread();
  void EndThread();

  int ThreadRun();
};

#endif  //_BOOST_SERIAL_PORT_WANGHONGTAO_2014_08_30_

