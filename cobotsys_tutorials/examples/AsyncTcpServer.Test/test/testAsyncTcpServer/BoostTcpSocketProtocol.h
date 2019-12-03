/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-4-23        zhoupeng/xuzhenhai created
============================================================== **/

#ifndef TIGERCOMMON_BOOSTTCPSOCKETPROTOCL_H
#define TIGERCOMMON_BOOSTTCPSOCKETPROTOCL_H
#include <vector>
#include "defines/MacrosInterface.h"
#include "BoostTcpSocketMsgTypeEnum.h"


class BoostTcpSocketProtocol {
public:
    BoostTcpSocketProtocol();

     size_t getHeadLen();

     void appendBodyLen(std::vector<unsigned char>& buf, int len);

     void appendMsgType(std::vector<unsigned char>& buf, unsigned char type);

     size_t getBodyLen(std::vector<unsigned char>& buf);

     unsigned char getMsgType(std::vector<unsigned char>& buf);

    void closeSocket(boost::shared_ptr <boost::asio::ip::tcp::socket> socket);

    int send(boost::shared_ptr<boost::asio::ip::tcp::socket> socket,
                     const std::string& sendData,
                     BoostTcpSocketMsgType type,
                     bool enableEncode);

};


#endif //TIGERCOMMON_BOOSTTCPSOCKETPROTOCL_H
