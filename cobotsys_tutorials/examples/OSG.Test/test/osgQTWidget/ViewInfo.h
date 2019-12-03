/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 8/22/19                 liuzhongxin
============================================================== **/


#ifndef COBOTOS_EDIT_VIEWINFO_H
#define COBOTOS_EDIT_VIEWINFO_H

struct ViewInfo
{
    int iX;
    int iY;
    int iWidth;
    int iHeight;
    std::string strName;
    bool draggable;
    bool visible;
};

#endif //COBOTOS_EDIT_VIEWINFO_H
