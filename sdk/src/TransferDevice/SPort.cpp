/* Copyright(C) Gaussian Automation. All rights reserved.
 */

#include <ros/ros.h>
#include <string>
#include <boost/exception/all.hpp>
#include "SPort.h"
#include "Comm.h"
#include "parameter.h"

Socket::Socket() {
  socket_ = NULL;
  thread_ = NULL;
  m_lReadBuffer = new ByteList();
  m_lReadBuffer->Init(1024);
  memset(m_szWriteBuffer, 0, 1024);
  m_nWriteBufferSize = 0;

  host_name_ = "10.7.5.199";
  port_ = "5000";
}

Socket::~Socket() {
  if (socket_ != NULL) {
    delete socket_;
    socket_ = NULL;
  }
  if(thread_ != NULL){
      delete thread_;
      thread_ = NULL;
  }
  if(m_lReadBuffer !=NULL){
      delete m_lReadBuffer;
      m_lReadBuffer = NULL;
  }
}

bool Socket::open() {

   if (socket_ != NULL) {
      delete socket_;
      socket_ = NULL;
   }
   try {
      socket_ = new boost::asio::ip::tcp::socket(ios_);
      boost::asio::ip::tcp::resolver resolver(ios_);
      boost::asio::connect(*socket_, resolver.resolve({host_name_.c_str(), port_.c_str()}));
      ROS_INFO("[SOCKET] open socket successfully");
      return true;
  } catch (std::exception &e) {
    ROS_ERROR("[SOCKET] open socket failed");
    if (socket_ != NULL) {
       delete socket_;
       socket_ = NULL;
    }
    return false;
  }
}

void Socket::Send_data(unsigned char* s_data, unsigned short len) {
  if(m_lReadBuffer!=NULL){
      m_lReadBuffer->Clear();
      memset(m_szWriteBuffer, 0, 1024);
      m_nWriteBufferSize = len;
      memcpy(m_szWriteBuffer, s_data, len);
      write();
  }
}

void Socket::read_callback(const boost::system::error_code& error, std::size_t bytes_transferred) {
  if (error) {  // No data was read!
    ROS_ERROR("[SOCKET] read data error");
    return;
  }
  if(m_lReadBuffer!=NULL)
  m_lReadBuffer->Write(m_szReadTemp, bytes_transferred);
  SLEEP(1000);
  read();
}

void Socket::Read_data(unsigned char* r_data, int &len, int need, int timeout) {
    len = 0;
    int len_tmp = 0;
    while (1) {
      len_tmp = m_lReadBuffer->Size();
      if (len_tmp >= need){
         p_network->socket_connection_status = true;
         break;
      }
      if (timeout--) {
        SLEEP(1);
      } else {
        p_network->socket_connection_status = false;
        ROS_INFO("[SOCKET] time out");
        break;
      }
    }
    m_lReadBuffer->Read(r_data, len);
}

int Socket::ThreadRun() {
  try {
    while (1) {
      if (open()) {
        read();
        ios_.run();
        ios_.reset();
      }
      boost::this_thread::interruption_point();
      SLEEP(1000);
    }
    return 0;
  } catch(boost::thread_interrupted) {
    return -1;
  } 
}

bool Socket::BeginThread() {
  EndThread();
  thread_ = new boost::thread(boost::bind(&Socket::ThreadRun, this));
  return thread_ != NULL;
}

void Socket::EndThread() {
  if (thread_ != NULL) {
    ios_.stop();
    thread_->interrupt();
    thread_->join();
    delete thread_;
    thread_ = NULL;
  }
}

void Socket::read() {
  socket_->async_read_some(boost::asio::buffer(m_szReadTemp, 1024),
                            boost::bind(&Socket::read_callback, this,
                                        boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

}

void Socket::write() {
    if (socket_) {
        try{
            socket_->write_some(boost::asio::buffer(m_szWriteBuffer, m_nWriteBufferSize));
         }catch(boost::exception &e){
            cout << diagnostic_information(e)<<endl;
         }
    }
}

void Socket::Init() {
  port_      = p_network->port;
  host_name_ = p_network->host_name;
  BeginThread();

  while (!socket_) {
    ROS_INFO("[SOCKET] waitting for socket %s:%s",port_.c_str(), host_name_.c_str());
    SLEEP(1000);
  }
}
