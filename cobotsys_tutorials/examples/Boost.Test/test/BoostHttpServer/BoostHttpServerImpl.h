//
// Created by cobot on 19-6-18.
//

#ifndef SPARROW_BOOST_TEST_BOOSTHTTPSERVERIMPL_H
#define SPARROW_BOOST_TEST_BOOSTHTTPSERVERIMPL_H

#include "BoostHttpServer.h"
#include <map>
#include <boost/asio/ip/tcp.hpp>
#include <boost/enable_shared_from_this.hpp>

class BoostHttpServerImpl : public BoostHttpServer, public boost::enable_shared_from_this<BoostHttpServerImpl> {
public:
    BoostHttpServerImpl(std::string host, std::string port);

    BoostHttpServerImpl(std::string host, std::string port, int threads);

    ~BoostHttpServerImpl();

private:
//    std::map<std::string, RequstHandler> _requestHandlerMap;
    std::string _host;
    std::string _port;
    int _threads;
    bool threadInited = false;
    boost::shared_ptr<boost::asio::io_context> _context;
    boost::shared_ptr<boost::asio::ip::tcp::acceptor> _acceptor;
    boost::shared_ptr<boost::asio::ip::tcp::socket> _socket;
    RequestHandler _requstHandler;
private:
    void initServer();

    virtual void startAcceptor();

    virtual void setRequestHandler(RequestHandler requstHandler);

    void acceptor(boost::system::error_code ec);

    void startRead();
};


#endif //SPARROW_BOOST_TEST_BOOSTHTTPSERVERIMPL_H
