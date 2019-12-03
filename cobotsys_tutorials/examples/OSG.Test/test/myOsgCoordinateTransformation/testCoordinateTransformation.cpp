/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-9-3           qishimeng        Inital
===============================================================**/

/* OpenSceneGraph example, osgpick.
*
*  Permission is hereby granted, free of charge, to any person obtaining a copy
*  of this software and associated documentation files (the "Software"), to deal
*  in the Software without restriction, including without limitation the rights
*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
*  copies of the Software, and to permit persons to whom the Software is
*  furnished to do so, subject to the following conditions:
*
*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
*  THE SOFTWARE.
*/

/* osgpick sample
* demonstrate use of osgUtil/PickVisitor for picking in a HUD or
* in a 3d scene,
*/

#include <osgUtil/Optimizer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgViewer/CompositeViewer>

#include <osgGA/TerrainManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osg/MatrixTransform>
#include <osg/PositionAttitudeTransform>

#include <osg/Material>
#include <osg/Geode>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osg/Camera>
#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osgSim/OverlayNode>

#include <osgText/Text>

#include <sstream>

// class to handle events with a pick
osg::ref_ptr<osg::Vec3Array> vertices;
osg::ref_ptr<osg::Geometry> geometry;

osg::ref_ptr<osg::Geode> createLine()
{
    osg::Vec3 start(0,0,0);
    vertices = new osg::Vec3Array;
    vertices->push_back(start);
    vertices->push_back(osg::Vec3(10,10,0));

    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));

    geometry = new osg::Geometry;
    geometry->setDataVariance(osg::Object::DYNAMIC);
    geometry->setVertexArray(vertices);

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.3f,0.3f,0.3f,1.0f));

    geometry->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

    //geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

    geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geometry);
    geode->setDataVariance(osg::Object::DYNAMIC);;
    return geode;
}
osg::AnimationPath* createAnimationPath(const osg::Vec3& center,float radius,double looptime)
{
    // set up the animation path
    osg::AnimationPath* animationPath = new osg::AnimationPath;
    animationPath->setLoopMode(osg::AnimationPath::LOOP);

    int numSamples = 40;
    float yaw = 0.0f;
    float yaw_delta = 2.0f*osg::PI/((float)numSamples-1.0f);
    float roll = osg::inDegrees(30.0f);

    double time=0.0f;
    double time_delta = looptime/(double)numSamples;
    for(int i=0;i<numSamples;++i)
    {
        osg::Vec3 position(center+osg::Vec3(sinf(yaw)*radius,cosf(yaw)*radius,0.0f));
        osg::Quat rotation(osg::Quat(roll,osg::Vec3(0.0,1.0,0.0))*osg::Quat(-(yaw+osg::inDegrees(90.0f)),osg::Vec3(0.0,0.0,1.0)));

        animationPath->insert(time,osg::AnimationPath::ControlPoint(position,rotation));

        yaw += yaw_delta;
        time += time_delta;

    }
    return animationPath;
}

osg::Node* createBase(const osg::Vec3& center,float radius)
{



    int numTilesX = 10;
    int numTilesY = 10;

    float width = 2*radius;
    float height = 2*radius;

    osg::Vec3 v000(center - osg::Vec3(width*0.5f,height*0.5f,0.0f));
    osg::Vec3 dx(osg::Vec3(width/((float)numTilesX),0.0,0.0f));
    osg::Vec3 dy(osg::Vec3(0.0f,height/((float)numTilesY),0.0f));

    // fill in vertices for grid, note numTilesX+1 * numTilesY+1...
    osg::Vec3Array* coords = new osg::Vec3Array;
    int iy;
    for(iy=0;iy<=numTilesY;++iy)
    {
        for(int ix=0;ix<=numTilesX;++ix)
        {
            coords->push_back(v000+dx*(float)ix+dy*(float)iy);
        }
    }

    //Just two colours - black and white.
    osg::Vec4Array* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f,1.0f,1.0f,1.0f)); // white
    colors->push_back(osg::Vec4(0.0f,0.0f,0.0f,1.0f)); // black

    osg::ref_ptr<osg::DrawElementsUShort> whitePrimitives = new osg::DrawElementsUShort(GL_QUADS);
    osg::ref_ptr<osg::DrawElementsUShort> blackPrimitives = new osg::DrawElementsUShort(GL_QUADS);

    int numIndicesPerRow=numTilesX+1;
    for(iy=0;iy<numTilesY;++iy)
    {
        for(int ix=0;ix<numTilesX;++ix)
        {
            osg::DrawElementsUShort* primitives = ((iy+ix)%2==0) ? whitePrimitives.get() : blackPrimitives.get();
            primitives->push_back(ix    +(iy+1)*numIndicesPerRow);
            primitives->push_back(ix    +iy*numIndicesPerRow);
            primitives->push_back((ix+1)+iy*numIndicesPerRow);
            primitives->push_back((ix+1)+(iy+1)*numIndicesPerRow);
        }
    }

    // set up a single normal
    osg::Vec3Array* normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f,0.0f,1.0f));

    osg::Geometry* geom = new osg::Geometry;
    geom->setVertexArray(coords);

    geom->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

    geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

    geom->addPrimitiveSet(whitePrimitives.get());
    geom->addPrimitiveSet(blackPrimitives.get());

    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(geom);

    return geode;
}

osg::Node* createMovingModel(const osg::Vec3& center, float radius)
{
    float animationLength = 10.0f;

    osg::AnimationPath* animationPath = createAnimationPath(center,radius,animationLength);

    osg::ref_ptr<osg::Group> model = new osg::Group;

    osg::ref_ptr<osg::Node> glider = osgDB::readRefNodeFile("glider.osgt");
    if (glider)
    {
        const osg::BoundingSphere& bs = glider->getBound();

        float size = radius/bs.radius()*0.3f;
        osg::MatrixTransform* positioned = new osg::MatrixTransform;
        positioned->setDataVariance(osg::Object::STATIC);
        positioned->setMatrix(osg::Matrix::translate(-bs.center())*
                              osg::Matrix::scale(size,size,size)*
                              osg::Matrix::rotate(osg::inDegrees(-90.0f),0.0f,0.0f,1.0f));

        positioned->addChild(glider);

        osg::PositionAttitudeTransform* xform = new osg::PositionAttitudeTransform;
        xform->setUpdateCallback(new osg::AnimationPathCallback(animationPath,0.0,1.0));
        xform->addChild(positioned);

        model->addChild(xform);
    }

    osg::ref_ptr<osg::Node> cessna = osgDB::readRefNodeFile("cessna.osgt");
    if (cessna)
    {
        const osg::BoundingSphere& bs = cessna->getBound();

        float size = radius/bs.radius()*0.3f;
        osg::MatrixTransform* positioned = new osg::MatrixTransform;
        positioned->setDataVariance(osg::Object::STATIC);
        positioned->setMatrix(osg::Matrix::translate(-bs.center())*
                              osg::Matrix::scale(size,size,size)*
                              osg::Matrix::rotate(osg::inDegrees(180.0f),0.0f,0.0f,1.0f));

        positioned->addChild(cessna);

        osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
        xform->setUpdateCallback(new osg::AnimationPathCallback(animationPath,0.0f,2.0));
        xform->addChild(positioned);

        model->addChild(xform);
    }

    return model.release();
}

osg::ref_ptr<osg::Group> createModel(bool overlay, osgSim::OverlayNode::OverlayTechnique technique)
{
    osg::Vec3 center(0.0f,0.0f,0.0f);
    float radius = 100.0f;

    osg::ref_ptr<osg::Group> root = new osg::Group;

    float baseHeight = center.z()-radius*0.5;
    osg::ref_ptr<osg::Node> baseModel = createBase(osg::Vec3(center.x(), center.y(), baseHeight),radius);
    osg::ref_ptr<osg::Node> movingModel = createMovingModel(center,radius*0.8f);

    if (overlay)
    {
        osgSim::OverlayNode* overlayNode = new osgSim::OverlayNode(technique);
        overlayNode->setContinuousUpdate(true);
        overlayNode->setOverlaySubgraph(movingModel);
        overlayNode->setOverlayBaseHeight(baseHeight-0.01);
        overlayNode->addChild(baseModel);
        root->addChild(overlayNode);
    }
    else
    {

        root->addChild(baseModel);
    }

    root->addChild(movingModel);

    return root;
}

class PickHandler : public osgGA::GUIEventHandler {
public:

    PickHandler(osgText::Text* updateText):
            _updateText(updateText) {}

    ~PickHandler() {}

    bool handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa);

    virtual void pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea);

    void setLabel(const std::string& name)
    {
        if (_updateText.get()) _updateText->setText(name);
    }

protected:

    osg::ref_ptr<osgText::Text>  _updateText;
};

bool PickHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::PUSH):
        {
            //geometry->setDataVariance(osg::Object::DYNAMIC);
            vertices->push_back(osg::Vec3(ea.getX()/100.0,ea.getY()/100.0,0));
            //vertices->dirty();

            geometry->setVertexArray(vertices);
            geometry->setPrimitiveSet(0,new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));
            //geometry->dirty();
            osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
            if (view) pick(view,ea);
            return false;
        }
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            if (ea.getKey()=='c')
            {
                osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
                osg::ref_ptr<osgGA::GUIEventAdapter> event = new osgGA::GUIEventAdapter(ea);
                event->setX((ea.getXmin()+ea.getXmax())*0.5);
                event->setY((ea.getYmin()+ea.getYmax())*0.5);
                if (view) pick(view,*event);
            }
            return false;
        }
        default:
            return false;
    }
}

void PickHandler::pick(osgViewer::View* view, const osgGA::GUIEventAdapter& ea)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;

    if (view->computeIntersections(ea,intersections))
    {
        for(osgUtil::LineSegmentIntersector::Intersections::iterator hitr = intersections.begin();
            hitr != intersections.end();
            ++hitr)
        {
            std::ostringstream os;
            if (!hitr->nodePath.empty() && !(hitr->nodePath.back()->getName().empty()))
            {
                // the geodes are identified by name.
                os<<"Object \""<<hitr->nodePath.back()->getName()<<"\""<<std::endl;
            }
            else if (hitr->drawable.valid())
            {
                os<<"Object \""<<hitr->drawable->className()<<"\""<<std::endl;
            }

            //std::cout<<"        local coords vertex("<< hitr->getLocalIntersectPoint()<<")"<<"  normal("<<hitr->getLocalIntersectNormal()<<")"<<std::endl;
            std::cout<<"        world coords vertex("<< hitr->getWorldIntersectPoint()<<")"<<"  normal("<<hitr->getWorldIntersectNormal()<<")"<<std::endl;
            const osgUtil::LineSegmentIntersector::Intersection::IndexList& vil = hitr->indexList;
            for(unsigned int i=0;i<vil.size();++i)
            {
                os<<"        vertex indices ["<<i<<"] = "<<vil[i]<<std::endl;
            }


        }
    }

}

osg::Node* createHUD(osgText::Text* updateText)
{

    // create the hud. derived from osgHud.cpp
    // adds a set of quads, each in a separate Geode - which can be picked individually
    // eg to be used as a menuing/help system!
    // Can pick texts too!

    osg::Camera* hudCamera = new osg::Camera;
    hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudCamera->setProjectionMatrixAsOrtho2D(0,1280,0,1024);
    hudCamera->setViewMatrix(osg::Matrix::identity());
    hudCamera->setRenderOrder(osg::Camera::POST_RENDER);
    hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);

    std::string timesFont("fonts/times.ttf");

    // turn lighting off for the text and disable depth test to ensure its always ontop.
    osg::Vec3 position(150.0f,800.0f,0.0f);
    osg::Vec3 delta(0.0f,-60.0f,0.0f);

    {
        osg::Geode* geode = new osg::Geode();
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        geode->setName("simple");
        hudCamera->addChild(geode);

        osgText::Text* text = new  osgText::Text;
        geode->addDrawable( text );

        text->setFont(timesFont);
        text->setText("Picking in Head Up Displays is simple!");
        text->setPosition(position);

        position += delta;
    }


    for (int i=0; i<5; i++) {
        osg::Vec3 dy(0.0f,-30.0f,0.0f);
        osg::Vec3 dx(120.0f,0.0f,0.0f);
        osg::Geode* geode = new osg::Geode();
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        const char *opts[]={"One", "Two", "Three", "January", "Feb", "2003"};
        osg::Geometry *quad=new osg::Geometry;
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        std::string name="subOption";
        name += " ";
        name += std::string(opts[i]);
        geode->setName(name);
        osg::Vec3Array* vertices = new osg::Vec3Array(4); // 1 quad
        osg::Vec4Array* colors = new osg::Vec4Array;
        colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(0.8-0.1*i,0.1*i,0.2*i, 1.0));
        quad->setColorArray(colors, osg::Array::BIND_OVERALL);
        (*vertices)[0]=position;
        (*vertices)[1]=position+dx;
        (*vertices)[2]=position+dx+dy;
        (*vertices)[3]=position+dy;
        quad->setVertexArray(vertices);
        quad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));
        geode->addDrawable(quad);
        hudCamera->addChild(geode);

        position += delta;
    }



    { // this displays what has been selected
        osg::Geode* geode = new osg::Geode();
        osg::StateSet* stateset = geode->getOrCreateStateSet();
        stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
        stateset->setMode(GL_DEPTH_TEST,osg::StateAttribute::OFF);
        geode->setName("The text label");
        geode->addDrawable( updateText );
        hudCamera->addChild(geode);

        updateText->setCharacterSize(20.0f);
        updateText->setFont(timesFont);
        updateText->setColor(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
        updateText->setText("");
        updateText->setPosition(position);
        updateText->setDataVariance(osg::Object::DYNAMIC);

        position += delta;
    }

    return hudCamera;

}

int main( int argc, char **argv )
{
    // use an ArgumentParser object to manage the program arguments.
    osg::ArgumentParser arguments(&argc,argv);

    // read the scene from the list of file specified commandline args.
    osg::ref_ptr<osg::Node> scene = osgDB::readRefNodeFiles(arguments);
    bool overlay;
    while (arguments.read("--overlay")) overlay = true;

    osgSim::OverlayNode::OverlayTechnique technique = osgSim::OverlayNode::OBJECT_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY;
    while (arguments.read("--object")) { technique = osgSim::OverlayNode::OBJECT_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY; overlay=true; }
    while (arguments.read("--ortho") || arguments.read("--orthographic")) { technique = osgSim::OverlayNode::VIEW_DEPENDENT_WITH_ORTHOGRAPHIC_OVERLAY; overlay=true; }
    while (arguments.read("--persp") || arguments.read("--perspective")) { technique = osgSim::OverlayNode::VIEW_DEPENDENT_WITH_PERSPECTIVE_OVERLAY; overlay=true; }
    osgViewer::Viewer viewer;
    //osg::ref_ptr<osg::Group> model = createModel(overlay, technique);
    osg::ref_ptr<osg::Group> model = createLine();
    if (!model)
    {
        return 1;
    }

    // tilt the scene so the default eye position is looking down on the model.
    osg::ref_ptr<osg::MatrixTransform> rootnode = new osg::MatrixTransform;
    rootnode->setMatrix(osg::Matrix::rotate(osg::inDegrees(30.0f),1.0f,0.0f,0.0f));
    rootnode->addChild(model);

    // run optimization over the scene graph
    osgUtil::Optimizer optimzer;
    optimzer.optimize(rootnode);

    // set the scene to render
    viewer.setSceneData(rootnode);

    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
//    if (!scene && arguments.read("--relative-camera-scene"))
//    {
//        // Create a test scene with a camera that has a relative reference frame.
//        osg::Group* group = new osg::Group();
//
//        osg::Geode* sphere = new osg::Geode();
//        sphere->setName("Sphere");
//        sphere->addDrawable(new osg::ShapeDrawable(new osg::Sphere()));
//
//        osg::Geode* cube = new osg::Geode();
//        cube->setName("Cube");
//        cube->addDrawable(new osg::ShapeDrawable(new osg::Box()));
//
//        osg::Camera* camera = new osg::Camera();
//        camera->setRenderOrder(osg::Camera::POST_RENDER);
//        camera->setClearMask(GL_DEPTH_BUFFER_BIT);
//        camera->setReferenceFrame(osg::Transform::RELATIVE_RF);
//        camera->setViewMatrix(osg::Matrix::translate(-2, 0, 0));
//
//        osg::MatrixTransform* xform = new osg::MatrixTransform(osg::Matrix::translate(1, 1, 1));
//        xform->addChild(camera);
//
//        group->addChild(sphere);
//        group->addChild(xform);
//        camera->addChild(cube);
//
//        scene = group;
//    }
    osg::ref_ptr<osgText::Text> updateText = new osgText::Text;
    // if not loaded assume no arguments passed in, try use default mode instead.
#if 0
    if (!scene) scene = osgDB::readRefNodeFile("fountain.osgt");

    osg::ref_ptr<osg::Group> group = dynamic_cast<osg::Group*>(scene.get());
    if (!group)
    {
        group = new osg::Group;
        group->addChild(scene.get());
    }

    osg::ref_ptr<osgText::Text> updateText = new osgText::Text;

    // add the HUD subgraph.
    //group->addChild(createHUD(updateText.get()));

    if (arguments.read("--CompositeViewer"))
    {
        osg::ref_ptr<osgViewer::View> view = new osgViewer::View;
        // add the handler for doing the picking
        view->addEventHandler(new PickHandler(updateText.get()));

        // set the scene to render
        view->setSceneData(group.get());

        view->setUpViewAcrossAllScreens();

        osgViewer::CompositeViewer viewer;
        viewer.addView(view.get());

        return viewer.run();

    }
    else
#endif
    {
        //osgViewer::Viewer viewer;


        // add all the camera manipulators
        {
            osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> keyswitchManipulator = new osgGA::KeySwitchMatrixManipulator;

            keyswitchManipulator->addMatrixManipulator( '1', "Trackball", new osgGA::TrackballManipulator() );
            keyswitchManipulator->addMatrixManipulator( '2', "Flight", new osgGA::FlightManipulator() );
            keyswitchManipulator->addMatrixManipulator( '3', "Drive", new osgGA::DriveManipulator() );

            unsigned int num = keyswitchManipulator->getNumMatrixManipulators();
            keyswitchManipulator->addMatrixManipulator( '4', "Terrain", new osgGA::TerrainManipulator() );

            std::string pathfile;
            char keyForAnimationPath = '5';
            while (arguments.read("-p",pathfile))
            {
                osgGA::AnimationPathManipulator* apm = new osgGA::AnimationPathManipulator(pathfile);
                if (apm || !apm->valid())
                {
                    num = keyswitchManipulator->getNumMatrixManipulators();
                    keyswitchManipulator->addMatrixManipulator( keyForAnimationPath, "Path", apm );
                    ++keyForAnimationPath;
                }
            }

            keyswitchManipulator->selectMatrixManipulator(num);

            viewer.setCameraManipulator( keyswitchManipulator.get() );
        }

        // add the handler for doing the picking
        viewer.addEventHandler(new PickHandler(updateText.get()));

        // set the scene to render
        viewer.setSceneData(model.get());

        return viewer.run();
    }

}
