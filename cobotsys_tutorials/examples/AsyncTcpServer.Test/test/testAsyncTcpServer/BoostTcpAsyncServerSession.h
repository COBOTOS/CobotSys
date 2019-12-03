/**============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved
 
==============================================================
File description:
 
 
==============================================================
 Date              Name             Description of Change
 19-7-30           lijun
============================================================**/

#ifndef COBOTOS_SESSION_H
#define COBOTOS_SESSION_H


#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include "boost/shared_ptr.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include "BoostTcpSocketProtocol.h"

using boost::asio::ip::tcp;

class BoostTcpAsyncServerSession
{
public:
    BoostTcpAsyncServerSession(boost::shared_ptr<boost::asio::io_service> io_service);

    tcp::socket& socket()
    {
        return *(_socket.get());
    }

    void start();

    boost::shared_ptr <boost::asio::ip::tcp::socket> getSocketStream(const std::string& id);

    int send(const std::string& sendData,size_t bytes_transferred);
private:
    std::string generateSocketID(boost::shared_ptr<boost::asio::ip::tcp::socket> socket);

    void insertHeartBeatMaps(const std::string& socketID, boost::posix_time::ptime curTime);

    void insertSocketMaps(const std::string& socketID, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);

    void handle_read(const boost::system::error_code& error, size_t bytes_transferred);

    void read_body(const boost::system::error_code& error, size_t bytes_transferred);

    void handle_write(const boost::system::error_code& error);

private:
    //已经连接的客户端，key为客户端id
    std::map<std::string, boost::shared_ptr<boost::asio::ip::tcp::socket>> _socketMaps;
    boost::recursive_mutex _socketMapsMutex;

    std::map<std::string, boost::posix_time::ptime> _clientSocketHeartBeatMap;
    boost::recursive_mutex _heartBeatMutex;

    boost::shared_ptr<boost::asio::ip::tcp::socket> _socket;
    enum { max_length = 1024 };
//    char _headData[max_length];
//    char _bodyData[max_length];

    std::vector<unsigned char> _headData;
    std::vector<unsigned char> _bodyData;
};


#endif //COBOTOS_SESSION_H
