//
// Created by cobot on 19-6-18.
//

#include "BoostHttpServerImpl.h"
#include "BoostHttpServerSession.h"
#include <boost/make_shared.hpp>
#include <boost/thread.hpp>


#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/bind_executor.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <boost/lexical_cast.hpp>
#include <logger/Logger.h>


BoostHttpServerImpl::BoostHttpServerImpl(std::string host, std::string port) : _host(host), _port(port), _threads(4) {
    initServer();
}

BoostHttpServerImpl::BoostHttpServerImpl(std::string host, std::string port, int threads) : _host(host), _port(port),
                                                                                            _threads(threads) {
    initServer();
}

BoostHttpServerImpl::~BoostHttpServerImpl() {

}

void BoostHttpServerImpl::setRequestHandler(RequestHandler requstHandler) {
    this->_requstHandler = requstHandler;
}

BoostHttpServer::~BoostHttpServer() {

}

void BoostHttpServerImpl::initServer() {
    _requstHandler = [](Request request)->boost::shared_ptr<Response>{
        boost::beast::http::response<boost::beast::http::string_body>* res = new boost::beast::http::response<boost::beast::http::string_body>{boost::beast::http::status::bad_request, request->version()};


        boost::shared_ptr<Response> response = boost::shared_ptr<Response>(res);
        response->set(boost::beast::http::field::server, BOOST_BEAST_VERSION_STRING);
        response->set(boost::beast::http::field::content_type, "text/html");
//        response->set(boost::beast::http::field::body,"No request handler ");
//        response->keep_alive(response->keep_alive());
        response->body() = "No request handler ";
        response->prepare_payload();
        return response;
    };
    _context = boost::shared_ptr<boost::asio::io_context>(new boost::asio::io_context{_threads});

    boost::asio::ip::tcp::endpoint ep{boost::asio::ip::make_address(_host), boost::lexical_cast<unsigned short>(_port)};
    _acceptor = boost::shared_ptr<boost::asio::ip::tcp::acceptor>(new boost::asio::ip::tcp::acceptor{*_context});

    boost::system::error_code ec;
    // Open the acceptor
    _acceptor->open(ep.protocol(), ec);
    if (ec) {
        LOG_ERROR << "open acceptor fail";
        return;
    }

    // Allow address reuse
    _acceptor->set_option(boost::asio::socket_base::reuse_address(true), ec);
    if (ec) {
        LOG_ERROR << "set_option  fail";
        return;
    }

    // Bind to the server address
    _acceptor->bind(ep, ec);
    if (ec) {
        LOG_ERROR << "bind endpoint  fail";
        return;
    }

    // Start listening for connections
    _acceptor->listen(
            boost::asio::socket_base::max_listen_connections, ec);
    if (ec) {
        LOG_ERROR << "listen  fail";
        return;
    }

    if (!_acceptor->is_open()) {
        LOG_ERROR << "acceptor is not open";
        return;
    }

}

boost::shared_ptr<BoostHttpServer> BoostHttpServer::createBoostHttpServer(std::string host, std::string port) {
//    return boost::make_shared<BoostHttpServerImpl>(host, port, 4);
    return boost::shared_ptr<BoostHttpServer>(new BoostHttpServerImpl(host, port, 4));
}

void BoostHttpServerImpl::startAcceptor() {
    _socket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket{*_context});
    _acceptor->async_accept(*_socket, std::bind(
            &BoostHttpServerImpl::acceptor,
            shared_from_this(),
            std::placeholders::_1));
    if (!threadInited) {
        threadInited = true;
        std::vector<boost::thread> v;
        v.reserve(_threads - 1);
        for (auto i = _threads - 1; i > 0; --i)
            v.emplace_back([&]() { _context->run(); });
        _context->run();
    }
}

//boost::shared_ptr<BoostHttpServerSession> session;

void BoostHttpServerImpl::acceptor(boost::system::error_code ec) {
    if (ec) {
        LOG_ERROR << "acceptor error";
    } else {
        auto session = boost::make_shared<BoostHttpServerSession>(_socket,_requstHandler);
        session->startRead();
    }

    // Accept another connection
    startAcceptor();
}