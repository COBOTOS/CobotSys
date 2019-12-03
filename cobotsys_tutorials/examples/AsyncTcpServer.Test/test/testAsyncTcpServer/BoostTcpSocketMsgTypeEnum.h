/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date           Name          Description of Change
 19-4-23        zhoupeng/xuzhenhai created
============================================================== **/

#ifndef TIGERCOMMON_BOOSTTCPSOCKET_CONST_DATA_H
#define TIGERCOMMON_BOOSTTCPSOCKET_CONST_DATA_H
#include <string>
#include "defines/MacrosInterface.h"


enum  class BoostTcpSocketMsgType {
    ClientHeartBeat = 'h',
    ClientID ='i',
    Text = 't'
};
#endif //TIGERCOMMON_BOOSTTCPSOCKET_H
