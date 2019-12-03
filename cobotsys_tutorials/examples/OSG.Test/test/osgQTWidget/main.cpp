/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 8/22/19                 liuzhongxin
============================================================== **/
#include <QTimer>
#include <QApplication>
#include <QGridLayout>
#include <QMainWindow>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/MultiTouchTrackballManipulator>

#include <osgDB/ReadFile>

#include <osgQt/GraphicsWindowQt>
#include <osg/MatrixTransform>
#include <iostream>
#include <unistd.h>
#include "OsgCompositeViewQTWidget.h"

int main(int argc, char** argv)
{

    QApplication app(argc, argv);
//    QMainWindow* mainWindow = new QMainWindow();

    OsgCompositeViewQTWidget compositeViewWidget;
    compositeViewWidget.setGeometry(100, 100, 600, 600);
//    compositeViewWidget.show();
//    mainWindow->setCentralWidget(&compositeViewWidget);
    compositeViewWidget.show();
    ViewInfo viewInfo;
    viewInfo.iX = 700;
    viewInfo.iY = 100;
    viewInfo.iWidth = 400;
    viewInfo.iHeight = 400;
    viewInfo.strName = "car";
    viewInfo.draggable = false;
    viewInfo.visible = true;
    compositeViewWidget.addView(viewInfo);

    osg::ref_ptr<osg::MatrixTransform> root1 = new osg::MatrixTransform;
    osg::ref_ptr<osg::MatrixTransform> node = new osg::MatrixTransform;
    root1->addChild(node);
    node->addChild(osgDB::readRefNodeFile("dumptruck.osgt"));
    compositeViewWidget.setOsgRoot(root1,"car");

    viewInfo.iX = 1100;
    viewInfo.iY = 100;
    viewInfo.iWidth = 400;
    viewInfo.iHeight = 400;
    viewInfo.strName = "cow";
    viewInfo.draggable = false;
    viewInfo.visible = true;
    compositeViewWidget.addView(viewInfo);

    osg::ref_ptr<osg::MatrixTransform> root2 = new osg::MatrixTransform;
    root2->addChild(osgDB::readRefNodeFile("cow.osgt"));
    compositeViewWidget.setOsgRoot(root2,"cow");
    compositeViewWidget.setViewClearColor("cow", osg::Vec4(0.1,0.5,0.3,1));
    compositeViewWidget.setViewAngle("cow", osg::Vec3d(25,25,25), osg::Vec3d(0,0,0), osg::Vec3d(-1,-1,1));


    viewInfo.iX = 600;
    viewInfo.iY = 600;
    viewInfo.iWidth = 400;
    viewInfo.iHeight = 400;
    viewInfo.strName = "axes";
    viewInfo.draggable = false;
    viewInfo.visible = true;
    compositeViewWidget.addView(viewInfo);

    osg::ref_ptr<osg::MatrixTransform> root3 = new osg::MatrixTransform;
    osg::ref_ptr<osg::MatrixTransform> node3 = new osg::MatrixTransform;
    root3->addChild(node3);
    node3->addChild(osgDB::readRefNodeFile("axes.osgt"));
    compositeViewWidget.setOsgRoot(root3,"axes");


//    compositeViewWidget.hide();
//    compositeViewWidget.hideView("cow");

//    compositeViewWidget.showView("cow");

    compositeViewWidget.hideView("cow");

//    compositeViewWidget.show();

//    node->setMatrix(osg::Matrix::rotate(osg::inDegrees(-90.0f), 0.0f, 0.0f, 1.0f));


    return app.exec();
}
