/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 8/22/19                 liuzhongxin
============================================================== **/


#ifndef COBOTOS_EDIT_OSGQTWIDGET_H
#define COBOTOS_EDIT_OSGQTWIDGET_H

#include <QTimer>
#include <QApplication>
#include <QGridLayout>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/MultiTouchTrackballManipulator>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgQt/GraphicsWindowQt>

#include <iostream>
#include <map>
#include <list>
#include "ViewInfo.h"
class OsgCompositeViewQTWidget : public QWidget
{
public:
    OsgCompositeViewQTWidget(QWidget* parent = 0, Qt::WindowFlags f = 0, osgViewer::ViewerBase::ThreadingModel threadingModel = osgViewer::ViewerBase::SingleThreaded);
    ~OsgCompositeViewQTWidget();


    void addView(ViewInfo viewInfo);
    bool removeView(std::string strName);
    bool setOsgRoot(osg::ref_ptr<osg::MatrixTransform> root, std::string strViewName);
    bool setViewClearColor(std::string strName, const osg::Vec4& color);
    bool setViewAngle(std::string strName, osg::Vec3d eyepoint, osg::Vec3d center, osg::Vec3d updirection);
    bool showView(std::string strName);
    bool hideView(std::string strName);


protected:
    virtual void paintEvent( QPaintEvent *event );

private:
    osg::ref_ptr<osgQt::GraphicsWindowQt> createGraphicsWindowContext(int x, int y, int w, int h, const std::string &name);
    bool inNameViewMap(std::string strName);
    bool inList(std::list<std::string> viewList, std::string strName);
    void adjustLayout();
private:
    osg::ref_ptr<osgViewer::CompositeViewer> _compositeViewer;
    std::map<std::string, osg::ref_ptr<osgViewer::View>> _nameViewMap;
    std::list<std::string> _strDraggableViewList;
    std::list<std::string> _strHiddenEmbeddedViewList;
    std::list<std::string> _strEmbeddedViewList;
    QGridLayout* _gridLayout;
    QTimer _timer;
    char* pszTestMemLeak;
};

#endif //COBOTOS_EDIT_OSGQTWIDGET_H
