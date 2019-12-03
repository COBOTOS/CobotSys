/**============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved
 
==============================================================
File description:
 
 
==============================================================
 Date              Name             Description of Change
 19-7-30           lijun
============================================================**/

#ifndef COBOTOS_SERVER_H
#define COBOTOS_SERVER_H

#include "BoostTcpAsyncServerSession.h"
#include "boost/shared_ptr.hpp"
#include <boost/enable_shared_from_this.hpp>
class BoostTcpAsyncServerImpl:public boost::enable_shared_from_this<BoostTcpAsyncServerImpl>
{
public:

    BoostTcpAsyncServerImpl(boost::shared_ptr<boost::asio::io_service> io_service, const std::string& ip ,short port);

    void accept();

    bool isConnected(const std::string& id);

    int send(const std::string& id, const std::string& data);

private:
    void handle_accept(boost::shared_ptr<BoostTcpAsyncServerSession> new_session,
                       const boost::system::error_code& error);

private:

    boost::shared_ptr<boost::asio::io_service> io_service;
    boost::shared_ptr<boost::asio::ip::tcp::acceptor> _acceptor;
    boost::shared_ptr <boost::asio::ip::tcp::socket>  _socket;
    boost::shared_ptr<BoostTcpAsyncServerSession> _session;

    std::string _address;
};


#endif //COBOTOS_SERVER_H
