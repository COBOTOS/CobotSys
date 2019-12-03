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
#include <logger/Logger.h>
#include "boost/shared_ptr.hpp"

using namespace std; // For atoi.

int main(int argc, char* argv[])
{
    try
    {
//        if (argc != 2)
//        {
//            LOG_ERROR << "Usage: async_tcp_echo_server <port>";
//            return 1;
//        }
        const std::string ip = "127.0.0.1";
        int port = 10001;

        boost::shared_ptr<boost::asio::io_service> ios = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service());
        boost::shared_ptr<BoostTcpAsyncServerImpl> _server = boost::shared_ptr<BoostTcpAsyncServerImpl>(new BoostTcpAsyncServerImpl(ios, ip, port));
        _server->accept();
//        boost::asio::io_service io_service;
//        server s(io_service, ip, port);

        ios->run();
    }
    catch (std::exception& e)
    {
        LOG_ERROR << "Exception: " << e.what();
    }

    return 0;
}