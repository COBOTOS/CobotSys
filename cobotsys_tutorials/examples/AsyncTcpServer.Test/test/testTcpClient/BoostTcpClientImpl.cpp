/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-4-16        qishimeng     Initial Draft
 19-4-23        zhoupeng/xuzhenhai add std::string BoostTcpClientImpl::receive()
============================================================== **/
#include "BoostTcpClientImpl.h"
#include "../testAsyncTcpServer/BoostTcpSocketProtocol.h"
#include "../testAsyncTcpServer/BoostTcpSocketMsgTypeEnum.h"
#include <parser/NumberUtils.h>
#include <logger/Logger.h>

BoostTcpClientImpl::BoostTcpClientImpl(const std::string& ip, int port, const std::string& id):_id(id)
{
   _socket = boost::make_shared<Socket>(_io);
   _endPoint.address(Address::from_string(ip));
   _endPoint.port(port);
    _connectTimer = BoostTimerFactory::createTimer();
    _clientHeartTimer= BoostTimerFactory::createTimer();
    _boostTcpSocketProtocol = boost::shared_ptr<BoostTcpSocketProtocol>(new BoostTcpSocketProtocol());
}

BoostTcpClient::~BoostTcpClient()
{
//    disableHeartBeat();
}

BoostTcpClientImpl::~BoostTcpClientImpl()
{
    _connectTimer->cancelTask();
    LOG_INFO<<"_connectTimer:"<<_connectTimer.use_count();
}
bool BoostTcpClientImpl::isConnect()
{
    return _socket->is_open();
}
bool BoostTcpClientImpl::connect(int waitMs)
{
    if(_socket->is_open())
    {
        return true;
    }
    _connectTimer->setTask(boost::bind(&BoostTcpClientImpl::connectTimeoutTimer, shared_from_this(), _connectTimer), 1000*waitMs);
    try
    {
        _socket->connect(_endPoint);
        //if id is not empty, send the id to server
        if (!_id.empty()) {
            _boostTcpSocketProtocol->send(_socket, _id, BoostTcpSocketMsgType::ClientID, true);
        }
        _connectTimer->cancelTask();
    }
    catch (std::exception &e)
    {
        //LOG_ERROR <<"exception: " <<e.what();
        _connectTimer->cancelTask();
        return false;
    }

    return true;
}


void BoostTcpClientImpl::connectTimeoutTimer(boost::shared_ptr<TIGER_COMMON_NS_API::BoostTimer> connectTimer)
{
    if (!connectTimer->taskCanceled())
    {
        try
        {
            LOG_ERROR << "connect timeout, connect failed";
            _socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
            _socket->close();
        }
        catch (std::exception &e)
        {
            LOG_ERROR <<"shutdown and close socket exception: " <<e.what()<< " ip=" << _endPoint.address().to_string()
                      << " port=" << _endPoint.port();
            return;
        }
    }
}


int BoostTcpClientImpl::receive(std::vector<unsigned char>& buffer)
{
    if(buffer.empty())
    {
        LOG_ERROR << "receive buffer is empty, cant receive data";
        return 0;
    }
    try
    {
        std::size_t len = _socket->receive(boost::asio::buffer(buffer));
        return len;
    }
    catch (std::exception &e)
    {
        LOG_ERROR <<"socket receive data exception: " <<e.what() << " ip=" << _endPoint.address().to_string()
                  << " port=" << _endPoint.port();
        return VOS_NOK;//-1
    }
}


std::string BoostTcpClientImpl::receive() {
    std::string output;
    try {
        size_t headLen = _boostTcpSocketProtocol->getHeadLen();
        std::vector<unsigned char> messageHeader;
        messageHeader.resize(headLen);

        size_t len= receive(messageHeader);
        if (len < headLen) {
            LOG_ERROR<<"read error message header size, expected head len=" << headLen << " read head len=" << len;
            return output;
        }
        size_t messageBodySize = _boostTcpSocketProtocol->getBodyLen(messageHeader);
        std::vector<unsigned char> messageBody;
        messageBody.resize(messageBodySize);

        len =  receive(messageBody);
        if (len == messageBodySize) {
            std::string message(messageBody.begin(),messageBody.end());
            return message;
        } else {
            LOG_ERROR<<"read error message body size, expected body len=" << messageBodySize << " read head len=" << len;
            return  output;
        }
    } catch (std::exception& ex) {
        LOG_ERROR<<" readDataFromServerLoop error ,"<<ex.what();
        return output;
    }
}

int BoostTcpClientImpl::send(const std::string& sendData,bool enableEncode)
{
    return _boostTcpSocketProtocol->send(_socket, sendData, BoostTcpSocketMsgType::Text, enableEncode);
}

void BoostTcpClientImpl::close()
{
    _socket->close();
}

void BoostTcpClientImpl::enableHeartBeat() {
    if(_clientHeartTimer->taskCanceled()){
        return;
    }
    _clientHeartTimer->setTask(boost::bind(&BoostTcpClientImpl::heatBeatTimer,shared_from_this()),5*HEARTBEAT_TIMER*1000);
}

void BoostTcpClientImpl::disableHeartBeat(){
    if(!_clientHeartTimer->taskCanceled()){
        _clientHeartTimer->cancelTask();
        LOG_INFO <<"cancel heart beat timer";
    }
}

void BoostTcpClientImpl::heatBeatTimer() {
     if(_socket->is_open()){
        std::string sendData = "heart beat";
        LOG_INFO<<"update heart beat";
        _boostTcpSocketProtocol->send(_socket, sendData, BoostTcpSocketMsgType::ClientHeartBeat, true);
        enableHeartBeat();
    }else{
         LOG_INFO<<"update heart fail for disconnected";
     }
}

boost::shared_ptr<BoostTcpClient> BoostTcpClient::create(const std::string& ip, int port, const std::string& id) {
    return boost::shared_ptr<BoostTcpClient>(new BoostTcpClientImpl(ip, port, id));
}

