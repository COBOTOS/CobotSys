/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 8/19/19                 liuzhongxin
============================================================== **/
#include <logger/Logger.h>
#include <LocalStorageRepositoryFactoryInterface.h>
#include <LocalStorageRepositoryAPIKey.h>
#include <systemproperty/SystemProperty.h>

#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgGA/GUIEventHandler>
#include <osg/Geode>

using namespace LION_LSR_NS_API;
using namespace TIGER_COMMON_NS_API;


osgViewer::Viewer viewer;
osg::ref_ptr<osg::MatrixTransform> osgRootNode = new osg::MatrixTransform();
osg::ref_ptr<osg::MatrixTransform> anotherOsgRootNode = new osg::MatrixTransform();


class KeyboardEventHandler : public osgGA::GUIEventHandler
{
public:

    KeyboardEventHandler()
    {}

    virtual bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter&)
    {
        switch (ea.getEventType())
        {
            case(osgGA::GUIEventAdapter::KEYDOWN):
            {
                switch (ea.getKey())
                {
                    case '1':
                        viewer.setSceneData(osgRootNode.get());
                        return true;
                    case '2':
                        viewer.setSceneData(anotherOsgRootNode.get());
                        return true;
                }
            }
            default:
                break;

        }
        //return false to allow mouse manipulation
        return false;
    }
};


int main(int argc, char **argv)
{
    boost::shared_ptr<LocalStorageRepositoryFactoryInterface> lsrFactory = LocalStorageRepositoryFactoryInterface::create();
    auto lsr = lsrFactory->createLocalStorageRepository();
    std::string strSceneRelativePath  =  lsr->getItem(app_cassemble_scene_filepath);
    std::string strSceneFolderPath = SystemProperty::create()->getDevelDataRootPath() + strSceneRelativePath;


    osg::ref_ptr<osg::Node> fileNode = osgDB::readNodeFile(strSceneFolderPath+"geometry/base.stl");
    osg::ref_ptr<osg::Node> anotherFileNode = osgDB::readNodeFile(strSceneFolderPath+"geometry/joint1.stl");

    osgRootNode->addChild(fileNode);
    anotherOsgRootNode->addChild(anotherFileNode);

    viewer.setSceneData(osgRootNode);
    viewer.addEventHandler(new KeyboardEventHandler());

    return viewer.run();
}