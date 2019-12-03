/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 8/22/19                 liuzhongxin
============================================================== **/

#include <logger/Logger.h>
#include "OsgCompositeViewQTWidget.h"

OsgCompositeViewQTWidget::OsgCompositeViewQTWidget(QWidget *parent, Qt::WindowFlags f, osgViewer::ViewerBase::ThreadingModel threadingModel):QWidget(parent, f)
{
    _compositeViewer = new osgViewer::CompositeViewer;

    _compositeViewer->setThreadingModel(threadingModel);

    _compositeViewer->setKeyEventSetsDone(0);
//
    _gridLayout = new QGridLayout;
    _gridLayout->setParent(this);

    setLayout(_gridLayout);
//    grabKeyboard();
    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()) );
    _timer.start( 10 );
    pszTestMemLeak = (char*)malloc(160);
}

void OsgCompositeViewQTWidget::paintEvent(QPaintEvent *event)
{
    if(_strDraggableViewList.size() == 0)
    {
        _compositeViewer->frame();
        return;
    }
    for(auto itor = _strDraggableViewList.begin(); itor != _strDraggableViewList.end();)
    {
        auto camera = _nameViewMap[(*itor)]->getCamera();
        auto gc = dynamic_cast<osgQt::GraphicsWindowQt* >(camera->getGraphicsContext());
        auto widget = gc->getGLWidget();
        auto disparityX = abs(widget->geometry().x() - this->geometry().x());
        auto disparityY = abs(widget->geometry().y() - this->geometry().y());
        if(disparityX<50 && disparityY<50)
        {
            _strEmbeddedViewList.push_back((*itor));
            itor = _strDraggableViewList.erase(itor);
            adjustLayout();
        } else
        {
            itor++;
        }
    }
    _compositeViewer->frame();
}
OsgCompositeViewQTWidget::~OsgCompositeViewQTWidget()
{
    LOG_INFO << "~OsgCompositeViewQTWidget invoked";
    delete pszTestMemLeak;
}

void OsgCompositeViewQTWidget::addView(ViewInfo viewInfo)
{
    if(inNameViewMap(viewInfo.strName))
    {
        LOG_WARNING<<"this view has been exsited.";
        return;
    }
    osg::ref_ptr<osgQt::GraphicsWindowQt> gc;

    gc = createGraphicsWindowContext(viewInfo.iX, viewInfo.iY, viewInfo.iWidth, viewInfo.iHeight, viewInfo.strName);

    osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
    _compositeViewer->addView(view);
    _nameViewMap.insert(std::pair<std::string, osg::ref_ptr<osgViewer::View>>(viewInfo.strName,view));

    osg::ref_ptr<osg::Camera> camera = view->getCamera();
    camera->setGraphicsContext(gc);

    const osg::GraphicsContext::Traits* traits = gc->getTraits();
    camera->setViewport(new osg::Viewport(0, 0, traits->width, traits->height));
    camera->setProjectionMatrixAsPerspective( 30.f, static_cast<double>(traits->width)/static_cast<double>(traits->height), 1.f, 1000.f );

    view->addEventHandler( new osgViewer::StatsHandler );
    view->setCameraManipulator( new osgGA::MultiTouchTrackballManipulator ,false);

    if(!viewInfo.draggable)
    {
        if(!viewInfo.visible)
        {
            _strHiddenEmbeddedViewList.push_back(viewInfo.strName);
            gc->getGLWidget()->setParent(0);
            gc->getGLWidget()->hide();
        }
        else
        {
            _strEmbeddedViewList.push_back(viewInfo.strName);
            gc->getGLWidget()->setParent(this);
            adjustLayout();
        }
    }
    else
    {
        _strDraggableViewList.push_back(viewInfo.strName);
        if(viewInfo.visible)
        {
            gc->getGLWidget()->show();
        }
        else
        {
            gc->getGLWidget()->hide();
        }
    }

}

bool OsgCompositeViewQTWidget::removeView(std::string strName)
{
    if(!inNameViewMap(strName))
    {
        LOG_ERROR<<"view is not exist";
        return false;
    }
    if(inList(_strEmbeddedViewList, strName))
    {
        _strEmbeddedViewList.remove(strName);
    }
    if(inList(_strHiddenEmbeddedViewList, strName))
    {
        _strHiddenEmbeddedViewList.remove(strName);
    }
    if(inList(_strDraggableViewList, strName))
    {
        _strDraggableViewList.remove(strName);
    }

    auto gc = dynamic_cast<osgQt::GraphicsWindowQt*>(_nameViewMap[strName]->getCamera()->getGraphicsContext());
    delete gc->getGLWidget();

    _compositeViewer->removeView(_nameViewMap[strName]);
    _nameViewMap.erase(strName);
    adjustLayout();
    return true;
}

bool OsgCompositeViewQTWidget::setOsgRoot(osg::ref_ptr<osg::MatrixTransform> root, std::string strViewName)
{
    LOG_ASSERT(root.get() != NULL);
    if(!inNameViewMap(strViewName))
    {
        LOG_ERROR<<"view is not exist";
        return false;
    }
    _nameViewMap[strViewName]->setSceneData(root);
    return true;
}

bool OsgCompositeViewQTWidget::setViewClearColor(std::string strName, const osg::Vec4& color)
{
    if(! inNameViewMap(strName))
    {
        LOG_WARNING<<"this view has been exsited.";
        return false;
    }
    auto camera = _nameViewMap[strName]->getCamera();
    camera->setClearColor(color);
    return true;
}

bool OsgCompositeViewQTWidget::setViewAngle(std::string strName, osg::Vec3d eyepoint, osg::Vec3d center, osg::Vec3d updirection)
{
    if(! inNameViewMap(strName))
    {
        LOG_WARNING<<"this view has been exsited.";
        return false;
    }
    auto manipulator = dynamic_cast<osgGA::MultiTouchTrackballManipulator*>(_nameViewMap[strName]->getCameraManipulator());

    manipulator->setTransformation(eyepoint, center, updirection);
    return true;
}

bool OsgCompositeViewQTWidget::showView(std::string strName)
{
    if(! inNameViewMap(strName))
    {
        LOG_WARNING<<"this view has been exsited.";
        return false;
    }
    if(inList(_strDraggableViewList, strName))
    {
        auto gc = dynamic_cast<osgQt::GraphicsWindowQt*>(_nameViewMap[strName]->getCamera()->getGraphicsContext());
        gc->getGLWidget()->show();
    }
    if(inList(_strHiddenEmbeddedViewList, strName))
    {
        _strHiddenEmbeddedViewList.remove(strName);
        _strEmbeddedViewList.push_back(strName);
        auto camera = _nameViewMap[strName]->getCamera();
        auto gc = dynamic_cast<osgQt::GraphicsWindowQt* >(camera->getGraphicsContext());
        auto widget = gc->getGLWidget();
        widget->setParent(this);
        widget->show();
        adjustLayout();
    }
}

bool OsgCompositeViewQTWidget::hideView(std::string strName)
{
    if(! inNameViewMap(strName))
    {
        LOG_WARNING<<"this view has been exsited.";
        return false;
    }
    if(inList(_strDraggableViewList, strName))
    {
        auto gw = dynamic_cast<osgQt::GraphicsWindowQt*>(_nameViewMap[strName]->getCamera()->getGraphicsContext());
        gw->getGLWidget()->hide();
    }
    if(inList(_strEmbeddedViewList, strName))
    {
        _strEmbeddedViewList.remove(strName);
        auto camera = _nameViewMap[strName]->getCamera();
        auto gc = dynamic_cast<osgQt::GraphicsWindowQt* >(camera->getGraphicsContext());
//        _gridLayout->removeWidget(gc->getGLWidget());
        auto widget = gc->getGLWidget();
        widget->setParent(0);
        widget->hide();
        _strHiddenEmbeddedViewList.push_back(strName);
        adjustLayout();
    }

}
osg::ref_ptr<osgQt::GraphicsWindowQt> OsgCompositeViewQTWidget::createGraphicsWindowContext(int x, int y, int w, int h, const std::string &name)
{
    osg::ref_ptr<osg::DisplaySettings> ds = osg::DisplaySettings::instance();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = true;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();
    osg::ref_ptr<osgQt::GraphicsWindowQt> graphicsContext = new osgQt::GraphicsWindowQt(traits.get());
    return graphicsContext;
}

bool OsgCompositeViewQTWidget::inNameViewMap(std::string strName)
{
    if(_nameViewMap.find(strName) == _nameViewMap.end())
    {
        return false;
    }
    return true;
}

void OsgCompositeViewQTWidget::adjustLayout()
{
    int iNumOfEmbedded = _strEmbeddedViewList.size();
    LOG_ASSERT(iNumOfEmbedded>0);

    //以下两个变量分别代表行,列
    int horizontal = 1;
    int vertical = 1;
    while(true)
    {
        if(iNumOfEmbedded == horizontal*vertical)
        {
            break;
        }
        if(!(iNumOfEmbedded>horizontal*vertical && iNumOfEmbedded<(horizontal+1)*(vertical+1)))
        {
            horizontal++;
            vertical++;
            continue;
        }

        if(iNumOfEmbedded<=horizontal*(vertical+1))
        {
            vertical++;
            break;
        }
        horizontal++;
        vertical++;
        break;
    }

    auto perViewWidth = geometry().width()/vertical;
    auto perViewHeight = geometry().height()/horizontal;


    int x = 0, y =0;
    for(auto itor = _strEmbeddedViewList.begin(); itor != _strEmbeddedViewList.end(); itor++)
    {
        auto camera = _nameViewMap[(*itor)]->getCamera();

        auto gc = dynamic_cast<osgQt::GraphicsWindowQt* >(camera->getGraphicsContext());
        LOG_ASSERT(gc);
        _gridLayout->addWidget(gc->getGLWidget(),x,y);
        y++;
        if(y == vertical)
        {
            x++;
            y = 0;
        }
    }

}

bool OsgCompositeViewQTWidget::inList(std::list<std::string> viewList, std::string strName)
{
    auto itor = viewList.begin();
    for(; itor != viewList.end(); itor++)
    {
        if((*itor) == strName)
        {
            break;
        }
    }
    if(itor == viewList.end())
    {
        return false;
    }
    return true;
}