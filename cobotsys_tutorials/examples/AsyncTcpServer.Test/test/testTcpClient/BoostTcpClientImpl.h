/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-4-17        qishimeng     Initial Draft   
============================================================== **/
#ifndef COBOTOS_BOOSTTCPCLIENTIMPL_H
#define COBOTOS_BOOSTTCPCLIENTIMPL_H

#include "boosttcp/BoostTcpClient.h"
#include "boosttimer/BoostTimerFactory.h"
#include "BoostTcpSocketProtocol.h"

using namespace TIGER_COMMON_NS_API;

class BoostTcpClientImpl : public BoostTcpClient, public boost::enable_shared_from_this<BoostTcpClientImpl>
{
    typedef boost::asio::ip::address Address;
    typedef boost::asio::ip::tcp::endpoint Endpoint;
    typedef boost::asio::ip::tcp::socket Socket;
    typedef boost::asio::ip::tcp::acceptor Acceptor;
public:
    BoostTcpClientImpl(const std::string& ip, int port, const std::string& id);

    ~BoostTcpClientImpl();

    virtual bool connect(int waitMs=5000);

    virtual bool isConnect();

    virtual std::string receive();

    virtual int receive(std::vector<unsigned char>& buffer);

    virtual int send(const std::string& sendData,bool enableEncode=false);

    //virtual std::size_t send(std::vector<char>& buffer,bool enableEncode=false);

    virtual void close();

    virtual void enableHeartBeat();

    virtual void disableHeartBeat();

private:


    void connectTimeoutTimer(boost::shared_ptr<TIGER_COMMON_NS_API::BoostTimer> connectTimer);

    void heatBeatTimer();

private:
    //心跳包频率
    unsigned int HEARTBEAT_TIMER = 1000;

    boost::shared_ptr<BoostTimer> _clientHeartTimer;

    boost::shared_ptr<TIGER_COMMON_NS_API::BoostTimer> _connectTimer;

    boost::shared_ptr<Socket> _socket;

    boost::asio::io_service _io;

    Endpoint _endPoint;

    std::string _id;

    boost::shared_ptr<BoostTcpSocketProtocol> _boostTcpSocketProtocol;
};


#endif //COBOTOS_BOOSTTCPCLIENTIMPL_H
