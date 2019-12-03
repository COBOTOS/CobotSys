/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-4-23        zhoupeng/xuzhenhai created
============================================================== **/

#include <boost/shared_ptr.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include "logger/Logger.h"
#include "BoostTcpSocketProtocol.h"
#include <vector>



BoostTcpSocketProtocol::BoostTcpSocketProtocol(){

}

size_t BoostTcpSocketProtocol::getHeadLen() {
    return 5;
}

void BoostTcpSocketProtocol::appendBodyLen(std::vector<unsigned char>& buf, int len) {
    int index = 1;
    buf.at(index++)=((unsigned char) (0xff & len));
    buf.at(index++)=((unsigned char) ((0xff00 & len) >> 8));
    buf.at(index++)=((unsigned char) ((0xff0000 & len) >> 16));
    buf.at(index++)=((unsigned char) ((0xff000000 & len) >> 24));
}

void BoostTcpSocketProtocol::appendMsgType(std::vector<unsigned char>& buf, unsigned char type) {
    buf.at(0) = type;
}

size_t BoostTcpSocketProtocol::getBodyLen(std::vector<unsigned char>& buf) {
    size_t index = 1;
    size_t len = buf.at(index++);
    len  |= ((buf.at(index++) << 8) & 0xFF00);
    len |= ((buf.at(index++) << 16) & 0xFF0000);
    len |= ((buf.at(index++) << 24) & 0xFF000000);
    if (len < 0) {
        LOG_ERROR << "socket head len error, len="<<len;
        len = 0;
    }
    return len;
}

unsigned char BoostTcpSocketProtocol::getMsgType(std::vector<unsigned char>& buf) {
    return buf[0];
}


void BoostTcpSocketProtocol::closeSocket(boost::shared_ptr <boost::asio::ip::tcp::socket> socket) {
    try {
        LOG_ERROR << "connect timeout, connect failed";
        socket->shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        socket->close();
    }
    catch (std::exception &e) {
        LOG_ERROR << "shutdown and close socket exception: " << e.what() << " ip=" << socket->remote_endpoint().address().to_string()
                  << " port=" << socket->remote_endpoint().port();
        return;
    }
}

int BoostTcpSocketProtocol::send(boost::shared_ptr<boost::asio::ip::tcp::socket> socket,
                                         const std::string& sendData,
                                         BoostTcpSocketMsgType type,
                                         bool enableEncode) {
    if (sendData.empty()) {
        LOG_ERROR << "string buffer is empty, no need to send";
        return 0;
    }
    try {
        if(enableEncode) {
            BoostTcpSocketProtocol head;
            std::vector<unsigned char> headBuf;
            headBuf.resize(head.getHeadLen());
            head.appendMsgType(headBuf, static_cast<unsigned char>(type));
            head.appendBodyLen(headBuf, sendData.size());
            if(socket->is_open()){
                socket->send(boost::asio::buffer(headBuf));
            } else{
                LOG_ERROR <<"socket is close";
                return -1;
            }
        }
        return socket->send(boost::asio::buffer(sendData));
    }
    catch (std::exception &e)
    {
        LOG_ERROR <<"socket send data exception: " <<e.what() << " ip=" << socket->remote_endpoint().address().to_string()
                  << " port=" << socket->remote_endpoint().port();
        return -1;
    }
}
