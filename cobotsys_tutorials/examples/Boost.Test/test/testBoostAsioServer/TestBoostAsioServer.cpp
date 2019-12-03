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

class TestBoostAsioServer
{
    typedef TestBoostAsioServer this_type;
    typedef ip::tcp::acceptor acceptor_type;
    typedef ip::tcp::endpoint endpoint_type;
    typedef ip::tcp::socket socket_type;
    typedef ip::address address_type;
    typedef boost::shared_ptr<socket_type> sock_ptr;

private:
    io_service m_io;
    acceptor_type m_acceptor;


public:
    TestBoostAsioServer() : m_acceptor(m_io, endpoint_type(ip::tcp::v4(), 6688))
    {
        accept();
    }

    void run() {
        m_io.run();
    }

    void accept()
    {
        sock_ptr sock(new socket_type(m_io));
        m_acceptor.async_accept(*sock, boost::bind(&this_type::accept_handler, this, boost::asio::placeholders::error, sock));
    }

    void accept_handler(const boost::system::error_code& ec, sock_ptr sock)
    {
        if (ec)
        {
            return;
        }

        cout<<"Client:";
        cout<<sock->remote_endpoint().address()<<endl;
        sock->async_write_some(buffer("hello asio"), boost::bind(&this_type::write_handler, this, boost::asio::placeholders::error));
        // 发送完毕后继续监听，否则io_service将认为没有事件处理而结束运行
        accept();
    }

    void write_handler(const boost::system::error_code&ec)
    {
        cout<<"send msg complete"<<endl;
    }
};

int main()
{
    try
    {
        cout<<"TestBoostAsioServer start."<<endl;
        TestBoostAsioServer srv;
        srv.run();
    }
    catch (std::exception &e)
    {
        cout<<e.what()<<endl;
    }

    return 0;
}