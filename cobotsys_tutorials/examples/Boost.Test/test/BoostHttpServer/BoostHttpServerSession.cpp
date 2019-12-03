//
// Created by cobot on 19-6-18.
//

#include "BoostHttpServerSession.h"
#include <boost/make_shared.hpp>
#include <logger/Logger.h>
#include <boost/beast/http/fields.hpp>
#include "BoostHttpServerSession.h"

BoostHttpServerSession::BoostHttpServerSession(boost::shared_ptr<boost::asio::ip::tcp::socket> socket,RequestHandler requstHandler) : _socket(
        socket), _strand(_socket->get_executor()),_requstHandler(requstHandler) {
    _buffer = boost::make_shared<boost::beast::flat_buffer>();
    _request = boost::make_shared<boost::beast::http::request<boost::beast::http::string_body>>();
}

void BoostHttpServerSession::close() {

}

void BoostHttpServerSession::receiveData(boost::system::error_code ec, std::size_t bytes_transferred) {
    // This means they closed the connection
    if (ec == boost::beast::http::error::end_of_stream)
        return close();

    if (ec)
         LOG_ERROR<<"receiveData data fail";

    // Send the response
//    handle_request(_request, _socket);
    LOG_INFO<<"receiveData request url="<<_request->target()<<",method=_request->method()"<<",body="<<_request->body();
    boost::shared_ptr<Response> response = _requstHandler(_request);
    boost::beast::http::message<false,boost::beast::http::string_body,boost::beast::http::fields> sr;

    LOG_INFO<<"response "<<response->body();
    boost::system::error_code errorCode;
    boost::beast::http::write(*_socket, *response, errorCode);
    if (ec)
        LOG_ERROR<< "writeData fail";
}

void BoostHttpServerSession::startRead() {
    boost::beast::http::async_read(*_socket, *_buffer, *_request,
                                   boost::asio::bind_executor(
                                           _strand,
                                           std::bind(
                                                   &BoostHttpServerSession::receiveData,
                                                   shared_from_this(),
                                                   std::placeholders::_1,
                                                   std::placeholders::_2)));
}

void BoostHttpServerSession::writeData(boost::system::error_code ec, std::size_t bytes_transferred, bool close) {
    if (ec)
        LOG_ERROR<< "writeData fail";

    if (close) {
        return this->close();
    }
    startRead();
}