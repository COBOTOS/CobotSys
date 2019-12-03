/**============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved
 
==============================================================
File description:
 
 
==============================================================
 Date              Name             Description of Change
 19-7-30           lijun
============================================================**/
#include "BoostTcpAsyncServerImpl.h"
#include <boost/thread.hpp>
#include <sys/prctl.h>
#include <logger/Logger.h>

BoostTcpAsyncServerImpl::BoostTcpAsyncServerImpl(boost::shared_ptr<boost::asio::io_service> io_service, const std::string& ip,short port) {
    this->io_service = io_service;

    _acceptor = boost::shared_ptr<boost::asio::ip::tcp::acceptor>(new boost::asio::ip::tcp::acceptor(*(io_service.get()), tcp::endpoint(tcp::v4(), port)));

    _session = boost::shared_ptr<BoostTcpAsyncServerSession>(new BoostTcpAsyncServerSession(io_service));

}

void BoostTcpAsyncServerImpl::accept() {
    try {
        _acceptor->async_accept(_session->socket(),
                                boost::bind(&BoostTcpAsyncServerImpl::handle_accept, this, _session,
                                            boost::asio::placeholders::error));
    }catch (boost::system::system_error& e){
        LOG_ERROR <<"Exception system_error:" << e.what();
    }
    catch (std::exception& e) {
        LOG_ERROR <<"Exception std:" << e.what();
    }
    catch (...) {
         LOG_ERROR<< "Unknown exception.";
    }
}



void BoostTcpAsyncServerImpl::handle_accept(boost::shared_ptr<BoostTcpAsyncServerSession> session, const boost::system::error_code& error) {
    if (!error) {
        session->start();
        LOG_INFO<<"a new client connected.";//<< session->remote_endpoint();
    } else{
        LOG_ERROR << "accept failed: " << error.message();
        return;
    }
    // 处理下一个连接，每次处理完了之后，需要再次accept。
    // 否则BOOST 将只处理一次，然后结束监听。
    // 所以这里可以处理一个情况，就是当你要结束监听的时候，只要在这里return
    // 那么io_service 的run() 函数就结束监听。但如果有其他的异步操作时，
    // run() 函数还是会继续运行的。
    accept();
}

bool BoostTcpAsyncServerImpl::isConnected(const std::string& id){
    _socket = _session->getSocketStream(id);
    if (_socket) {
        return true;
    } else{
        LOG_ERROR << "unknow socket id:"<<id<<" please check";
        return false;
    }
}

int BoostTcpAsyncServerImpl::send(const std::string& id, const std::string& data) {
    boost::shared_ptr <boost::asio::ip::tcp::socket>  socket = _session->getSocketStream(id);
    if (socket) {
//        return _session->send(data, BoostTcpSocketMsgType::Text, true);
    }
    else {
        LOG_ERROR << "unknow socket id:" << id << " when sendding " << data;
        return -1;
    }
}