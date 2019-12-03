/**============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved
 
==============================================================
File description:
 
 
==============================================================
 Date              Name             Description of Change
 19-7-30           lijun
============================================================**/

#include "BoostTcpAsyncServerSession.h"
#include <logger/Logger.h>


BoostTcpAsyncServerSession::BoostTcpAsyncServerSession(boost::shared_ptr<boost::asio::io_service> io_service){
    _socket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(*(io_service.get())));
}


void BoostTcpAsyncServerSession::start() {
    _socket->async_read_some(boost::asio::buffer(_headData, max_length),
                            boost::bind(&BoostTcpAsyncServerSession::handle_read, this,
                                        boost::asio::placeholders::error,
                                        boost::asio::placeholders::bytes_transferred));
}

void BoostTcpAsyncServerSession::handle_read(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!error) {
        std::string id = generateSocketID(_socket);
        insertHeartBeatMaps(id, boost::posix_time::second_clock::local_time());
        insertSocketMaps(id, _socket);

        BoostTcpSocketProtocol head;
        _headData.resize(head.getHeadLen());
        if (_headData.size() < head.getHeadLen()) {
            LOG_ERROR<<"please check the datas , the message was break the protocol~~"
                     << _socket->remote_endpoint().address().to_string()
                     << " : "<< _socket->remote_endpoint().port();
            LOG_ERROR << "_data length:"<<_headData.size()<<" _data sizeof:"<<sizeof(_headData)<<" head length:"<<head.getHeadLen();
        }

        unsigned char messageType = head.getMsgType(_headData);

        _socket->async_read_some(boost::asio::buffer(_bodyData, max_length),
                                 boost::bind(&BoostTcpAsyncServerSession::read_body, this,
                                             boost::asio::placeholders::error,
                                             boost::asio::placeholders::bytes_transferred));

        std::string tmpbuf(_headData.begin(), _headData.end());
        LOG_INFO<<"read_head:"<<tmpbuf;
    }
}

void BoostTcpAsyncServerSession::read_body(const boost::system::error_code& error, size_t bytes_transferred){

    BoostTcpSocketProtocol head;
    int messageBodySize =head.getBodyLen(_headData);
    if (messageBodySize > 0) {
        _bodyData.resize(messageBodySize);

        if (_bodyData.size() != messageBodySize) {
            LOG_ERROR << "read message body size error , the message was break the protocol~~"
                      << _socket->remote_endpoint().address().to_string()
                      << " : " << _socket->remote_endpoint().port();
        }
    }
    std::string tmpbuf(_bodyData.begin(), _bodyData.end());
    LOG_INFO<<"read_body:"<<tmpbuf;

    boost::asio::async_write(*(_socket.get()),
                             boost::asio::buffer("HeartBeat", bytes_transferred),
                             boost::bind(&BoostTcpAsyncServerSession::handle_write, this,
                                         boost::asio::placeholders::error));
}

void BoostTcpAsyncServerSession::handle_write(const boost::system::error_code& error) {
    if (!error) {
        _socket->async_read_some(boost::asio::buffer(_headData, max_length),
                                boost::bind(&BoostTcpAsyncServerSession::handle_read, this,
                                            boost::asio::placeholders::error,
                                            boost::asio::placeholders::bytes_transferred));

    }
}

int BoostTcpAsyncServerSession::send(const std::string& sendData,size_t bytes_transferred){

    boost::asio::async_write(*(_socket.get()),
                             boost::asio::buffer(sendData, bytes_transferred),
                             boost::bind(&BoostTcpAsyncServerSession::handle_write, this,
                                         boost::asio::placeholders::error));
}

std::string BoostTcpAsyncServerSession::generateSocketID(boost::shared_ptr<boost::asio::ip::tcp::socket> socket) {
    std::string id = socket->remote_endpoint().address().to_string();
    id.append(":").append(std::to_string((socket->remote_endpoint().port())));
    return id;
}

void BoostTcpAsyncServerSession::insertHeartBeatMaps(const std::string& socketID, boost::posix_time::ptime curTime) {
    boost::recursive_mutex::scoped_lock  lock(_heartBeatMutex);
    _clientSocketHeartBeatMap[socketID] = curTime;
}

void BoostTcpAsyncServerSession::insertSocketMaps(const std::string& socketID, boost::shared_ptr<boost::asio::ip::tcp::socket> socket) {
    boost::recursive_mutex::scoped_lock  lock(_socketMapsMutex);
    _socketMaps[socketID] = socket;
}

boost::shared_ptr <boost::asio::ip::tcp::socket> BoostTcpAsyncServerSession::getSocketStream(const std::string& id) {
    boost::recursive_mutex::scoped_lock  lock(_socketMapsMutex);
    boost::shared_ptr <boost::asio::ip::tcp::socket>  socket;
    auto iter = _socketMaps.find(id);
    if (iter != _socketMaps.end()) {
        socket = (*iter).second;
    }
    return socket;
}