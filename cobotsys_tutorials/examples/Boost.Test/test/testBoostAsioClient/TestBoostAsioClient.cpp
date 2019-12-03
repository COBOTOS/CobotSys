/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-3-22        qishimeng     Initial Draft   
============================================================== **/

#include <iostream>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/asio/placeholders.hpp>
#include <boost/system/error_code.hpp>
#include <boost/bind/bind.hpp>

using namespace boost::asio;
using namespace std;

class TestBoostAsioClient
{
    typedef TestBoostAsioClient this_type;
    typedef ip::tcp::acceptor acceptor_type;
    typedef ip::tcp::endpoint endpoint_type;
    typedef ip::tcp::socket socket_type;
    typedef ip::address address_type;
    typedef boost::shared_ptr<socket_type> sock_ptr;
    typedef vector<char> buffer_type;

private:
    io_service m_io;
    buffer_type m_buf;
    endpoint_type m_ep;
    deadline_timer deadline_;
    sock_ptr _sock;
    bool _timeout;
    bool _isConnect;
public:
    TestBoostAsioClient(): m_buf(100, 0), m_ep(address_type::from_string("192.168.10.119"), 6688),deadline_(m_io)
    {
        _timeout = true;
        _isConnect = false;
        start();
    }

    void run()
    {    m_io.run();}

    void start()
    {
        //sock_ptr sock(new socket_type(m_io));
        _sock = boost::shared_ptr<socket_type>(new socket_type(m_io));
        deadline_.expires_from_now(boost::posix_time::seconds(10));
        _sock->async_connect(m_ep, boost::bind(&this_type::conn_handler, this, boost::asio::placeholders::error, _sock));
        deadline_.async_wait(boost::bind(&this_type::check_timeout, this));
        //sleep(1);
        //deadline_.cancel();
    }

    void conn_handler(const boost::system::error_code&ec, sock_ptr sock)
    {
        if (ec)
        {return;}

        cout<<"Receive from "<<sock->remote_endpoint().address()<<": "<<endl;
        sock->async_read_some(buffer(m_buf), boost::bind(&TestBoostAsioClient::read_handler, this, boost::asio::placeholders::error, sock));
    }

    void read_handler(const boost::system::error_code&ec, sock_ptr sock)
    {
        if (ec)
        {return;}
        _timeout = false;
        sock->async_read_some(buffer(m_buf), boost::bind(&TestBoostAsioClient::read_handler, this, boost::asio::placeholders::error, sock));
        cout<<"async_read_some:::"<<&m_buf[0]<<endl;
    }
    void check_timeout()
    {
        if(_timeout)
        {
            boost::system::error_code ignored_ec;
            std::cout << "timeout" << std::endl;
            _sock->close(ignored_ec);
        }
        else
        {
            std::cout << "has connected!" << std::endl;
        }
    }
};

int main()
{
    try
    {
        cout<<"TestBoostAsioClient start."<<endl;
        TestBoostAsioClient cl;
        cl.run();
    }
    catch (std::exception &e)
    {
        cout<<e.what()<<endl;
    }

    return 0;
}