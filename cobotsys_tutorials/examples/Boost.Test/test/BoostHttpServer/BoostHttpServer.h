//
// Created by cobot on 19-6-18.
//

#ifndef SPARROW_BOOST_TEST_BOOSTHTTPSERVER_H
#define SPARROW_BOOST_TEST_BOOSTHTTPSERVER_H

#include <boost/function.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/shared_ptr.hpp>

typedef boost::beast::http::response<boost::beast::http::string_body> Response;
typedef boost::shared_ptr<boost::beast::http::request<boost::beast::http::string_body>> Request;
typedef boost::function<boost::shared_ptr<Response>(Request)> RequestHandler;

class BoostHttpServer {

public:
    ~BoostHttpServer();

    virtual void setRequestHandler(RequestHandler requestHandler) = 0;

    virtual void startAcceptor() = 0;

    static boost::shared_ptr<BoostHttpServer> createBoostHttpServer(std::string host, std::string port);
};


#endif //SPARROW_BOOST_TEST_BOOSTHTTPSERVER_H
