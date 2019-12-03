//
// Created by cobot on 19-6-18.
//

#ifndef SPARROW_BOOST_TEST_BOOSTHTTPSERVERSESSION_H
#define SPARROW_BOOST_TEST_BOOSTHTTPSERVERSESSION_H

#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/beast/http/string_body.hpp>
#include <boost/enable_shared_from_this.hpp>
#include "BoostHttpServerImpl.h"

class BoostHttpServerSession : public boost::enable_shared_from_this<BoostHttpServerSession> {
public:
    BoostHttpServerSession(boost::shared_ptr<boost::asio::ip::tcp::socket> socket,RequestHandler _requstHandler);

    void startRead();

    void receiveData(boost::system::error_code ec, std::size_t bytes_transferred);

    void writeData(boost::system::error_code ec, std::size_t bytes_transferred, bool close);

    void close();

private:
    boost::shared_ptr<boost::asio::ip::tcp::socket> _socket;
    boost::asio::strand<
            boost::asio::io_context::executor_type> _strand;
    boost::shared_ptr<boost::beast::flat_buffer> _buffer;
    boost::shared_ptr<boost::beast::http::request<boost::beast::http::string_body>> _request;
    RequestHandler _requstHandler;

};


#endif //SPARROW_BOOST_TEST_BOOSTHTTPSERVERSESSION_H
