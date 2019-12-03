/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-8-17           tangmingwu
============================================================== **/

#include "LsrLost.h"
#include <logger/Logger.h>
int main()
{
    LsrLost lsrLost;
    int i=88000;

    while(i--)
    {
        lsrLost.start();
    }
    LOG_INFO << "finish test";
    while(1) {
        sleep(10);
    }
    return 0;
}
