/**============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved
 
==============================================================
File description:
 
 
==============================================================
 Date              Name             Description of Change
 19-7-30           lijun
============================================================**/
#include "BoostTcpClientImpl.h"
#include <logger/Logger.h>
#include "boost/shared_ptr.hpp"
#include "BoostTcpClientImpl.h"

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
        boost::shared_ptr<BoostTcpClientImpl> _boostTcpClientImpl = boost::shared_ptr<BoostTcpClientImpl>(new BoostTcpClientImpl(ip,port,"1"));
        _boostTcpClientImpl->connect();
        _boostTcpClientImpl->enableHeartBeat();
    }
    catch (std::exception& e)
    {
        LOG_ERROR << "Exception: " << e.what();
    }

    return 0;
}