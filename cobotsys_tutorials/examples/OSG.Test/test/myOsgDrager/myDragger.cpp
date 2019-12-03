/**===============================================================
Copyright (c) 2019 Wuhan Collaborative Robot Technology Co.,Ltd.
Unpublished - All rights reserved

=================================================================
File description:


=================================================================
 Date              Name             Description of Change
 19-9-4           qishimeng        Inital
===============================================================**/

#include "myDragger.h"
#include <osgManipulator/AntiSquish>

#include <osg/ShapeDrawable>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/PolygonMode>
#include <osg/CullFace>
#include <osg/Quat>
#include <osg/AutoTransform>
#include <iostream>
#include <osg/io_utils>
using namespace osgManipulator;

namespace
{

    osg::Geometry* createCircleGeometry(float radius, unsigned int numSegments)
    {
        const float angleDelta = 0.2f*osg::PI/(float)numSegments;
        const float r = radius;
        float angle = 0.15f*osg::PI;
        osg::Vec3Array* vertexArray = new osg::Vec3Array(numSegments);
        osg::Vec3Array* normalArray = new osg::Vec3Array(numSegments);
        for(unsigned int i = 0; i < numSegments; ++i,angle+=angleDelta)
        {
            float c = cosf(angle);
            float s = sinf(angle);
            (*vertexArray)[i].set(c*r,s*r,0.0f);
            (*normalArray)[i].set(c,s,0.0f);
        }
        osg::Geometry* geometry = new osg::Geometry();
        geometry->setVertexArray(vertexArray);
        geometry->setNormalArray(normalArray, osg::Array::BIND_PER_VERTEX);
        geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLE_STRIP,0,vertexArray->size()));
        return geometry;
    }

}

myDragger::myDragger(bool useAutoTransform)
{

    if (useAutoTransform)
    {
        float pixelSize = 50.0f;
        osg::MatrixTransform* scaler = new osg::MatrixTransform;
        scaler->setMatrix(osg::Matrix::scale(pixelSize, pixelSize, pixelSize));

        osg::AutoTransform *at = new osg::AutoTransform;
        at->setAutoScaleToScreen(true);
        at->addChild(scaler);

        AntiSquish* as = new AntiSquish;
        as->addChild(at);
        addChild(as);

        _xDragger = new RotateCylinderDragger();
        scaler->addChild(_xDragger.get());
        addDragger(_xDragger.get());

        _yDragger = new RotateCylinderDragger();
        scaler->addChild(_yDragger.get());
        addDragger(_yDragger.get());

        _zDragger = new RotateCylinderDragger();
        scaler->addChild(_zDragger.get());
        addDragger(_zDragger.get());

//        _xyzDragger = new RotateSphereDragger();
//        scaler->addChild(_xyzDragger.get());
//        addDragger(_xyzDragger.get());

    }
    else
    {
        _xDragger = new RotateCylinderDragger();
        addChild(_xDragger.get());
        addDragger(_xDragger.get());

        _yDragger = new RotateCylinderDragger();
        addChild(_yDragger.get());
        addDragger(_yDragger.get());

        _zDragger = new RotateCylinderDragger();
        addChild(_zDragger.get());
        addDragger(_zDragger.get());

//        _xAxisDragger = new Translate1DDragger()
////        _xyzDragger = new RotateSphereDragger();
//        addChild(_xAxisDragger.get());
//        addDragger(_xAxisDragger.get());


    }

    _xAxisDragger = new Translate1DDragger(osg::Vec3(0.0,0.0,0.0), osg::Vec3(0.0,0.0,1.0));
    addChild(_xAxisDragger.get());
    addDragger(_xAxisDragger.get());

    _yAxisDragger = new Translate1DDragger(osg::Vec3(0.0,0.0,0.0), osg::Vec3(0.0,0.0,1.0));
    addChild(_yAxisDragger.get());
    addDragger(_yAxisDragger.get());

    _zAxisDragger = new Translate1DDragger(osg::Vec3(0.0,0.0,0.0), osg::Vec3(0.0,0.0,1.0));
    addChild(_zAxisDragger.get());
    addDragger(_zAxisDragger.get());

    _axisLineWidth = 5.0f;
    _pickCylinderHeight = 0.15f;
    _coneHeight = 0.1f;
    _pickCylinderRadius = 0.005f;

    setParentDragger(getParentDragger());
}

myDragger::~myDragger()
{
}
bool myDragger::handle(const PointerInfo& pi, const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
//    std::cout << "handle"<< std::endl;
    OSG_NOTICE << "this matrix :" << getMatrix() << std::endl;
//    auto mat = dragger_listener->getMatrix();
    this->CompositeDragger::handle(pi,ea,aa);
    OSG_NOTICE << "update this matrix :" << getMatrix() << std::endl;
    return true;
}
void myDragger::setupDefaultGeometry()
{
    _geode = new osg::Geode;
    {
        osg::TessellationHints* hints = new osg::TessellationHints;
        hints->setCreateTop(true);
        hints->setCreateBottom(true);
        hints->setCreateBackFace(true);
        hints->setDetailRatio(2.0f);

        _cylinder = new osg::Cylinder;
        _cylinder->setHeight(_pickCylinderHeight);
//        _cylinder->setRadius(_pickCylinderRadius);
        osg::ShapeDrawable* cylinderDrawable = new osg::ShapeDrawable(_cylinder.get(), hints);
        _geode->addDrawable(cylinderDrawable);
        setDrawableToAlwaysCull(*cylinderDrawable);
        _geode->addDrawable(createCircleGeometry(1.0f, 100));
    }

    // Draw in line mode.
    {
        osg::PolygonMode* polymode = new osg::PolygonMode;
        polymode->setMode(osg::PolygonMode::FRONT_AND_BACK,osg::PolygonMode::LINE);
        _geode->getOrCreateStateSet()->setAttributeAndModes(polymode,osg::StateAttribute::OVERRIDE|osg::StateAttribute::ON);
        _lineWidth = new osg::LineWidth(_axisLineWidth);
        _geode->getOrCreateStateSet()->setAttributeAndModes(_lineWidth.get(), osg::StateAttribute::ON);

#if !defined(OSG_GLES2_AVAILABLE)
        _geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
#endif

    }

    // Add line to all the individual 1D draggers.
    _xDragger->addChild(_geode.get());
    _yDragger->addChild(_geode.get());
    _zDragger->addChild(_geode.get());



    // Rotate X-axis dragger appropriately.
    {
        float iRadians = osg::DegreesToRadians(0.0);
//        osg::Quat rotation; rotation.makeRotate(osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
        osg::Quat rotation; rotation.makeRotate(iRadians, osg::Vec3(1.0f, 0.0f, 0.0f));
        _xDragger->setMatrix(osg::Matrix(rotation));
    }

    // Rotate Y-axis dragger appropriately.
    {
        float iRadians = osg::DegreesToRadians(-90.0);
//        osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, -1.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
        osg::Quat rotation; rotation.makeRotate(iRadians, osg::Vec3(0.0f, 1.0f, 0.0f));
        _yDragger->setMatrix(osg::Matrix(rotation));
    }

    // Rotate Z-axis dragger appropriately.
    {
        float iRadians = osg::DegreesToRadians(90.0);
        osg::Quat rotation; rotation.makeRotate(iRadians, osg::Vec3(1.0f, 0.0f, 0.0f));
        _zDragger->setMatrix(osg::Matrix(rotation));
    }

    // Send different colors for each dragger.
    _xDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
    _yDragger->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
    _zDragger->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

    // Add invisible sphere for pick the spherical dragger.
//    {
//        osg::Drawable* sphereDrawable = new osg::ShapeDrawable(new osg::Sphere());
//        setDrawableToAlwaysCull(*sphereDrawable);
//        osg::Geode* sphereGeode = new osg::Geode;
//        sphereGeode->addDrawable(sphereDrawable);
//
//        _xyzDragger->addChild(sphereGeode);
//    }
#if 1
    {
        _lineGeode = new osg::Geode;
        {
            osg::Geometry* geometry = new osg::Geometry();

            osg::Vec3Array* vertices = new osg::Vec3Array(2);
            (*vertices)[0] = osg::Vec3(0.0f,0.0f,0.0f);
            (*vertices)[1] = osg::Vec3(0.0f,0.0f,1.0f);

            geometry->setVertexArray(vertices);
            geometry->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,2));

            _lineGeode->addDrawable(geometry);
        }

        // Turn of lighting for line and set line width.
        {
            _lineWidth = new osg::LineWidth();
            _lineWidth->setWidth(_axisLineWidth);
            _lineGeode->getOrCreateStateSet()->setAttributeAndModes(_lineWidth.get(), osg::StateAttribute::ON);
            _lineGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        }

        // Add line to all the individual 1D draggers.
        _xAxisDragger->addChild(_lineGeode.get());
        _yAxisDragger->addChild(_lineGeode.get());
        _zAxisDragger->addChild(_lineGeode.get());

        osg::Geode* geode = new osg::Geode;

        // Create a cone.
        {
            _cone = new osg::Cone (osg::Vec3(0.0f, 0.0f, 1.0f), _coneHeight * 0.25f, _coneHeight);
            osg::ShapeDrawable* coneDrawable = new osg::ShapeDrawable(_cone.get());
            // coneDrawable->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
            geode->addDrawable(coneDrawable);

            // This ensures correct lighting for scaled draggers.
#if !defined(OSG_GLES2_AVAILABLE)
            geode->getOrCreateStateSet()->setMode(GL_NORMALIZE, osg::StateAttribute::ON);
#endif
        }

        // Create an invisible cylinder for picking the line.
        {
            _cylinder = new osg::Cylinder (osg::Vec3(0.0f,0.0f,0.5f), 0.3f, 1.0f);
            osg::Drawable* geometry = new osg::ShapeDrawable(_cylinder.get());
            setDrawableToAlwaysCull(*geometry);
            geode->addDrawable(geometry);
        }

        // Add geode to all 1D draggers.
        _xAxisDragger->addChild(geode);
        _yAxisDragger->addChild(geode);
        _zAxisDragger->addChild(geode);

        // Rotate X-axis dragger appropriately.
        {
            osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(1.0f, 0.0f, 0.0f));
            _xAxisDragger->setMatrix(osg::Matrix(rotation));
        }

        // Rotate Y-axis dragger appropriately.
        {
            osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(0.0f, 1.0f, 0.0f));
            _yAxisDragger->setMatrix(osg::Matrix(rotation));
        }

        {
            osg::Quat rotation; rotation.makeRotate(osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(0.0f, 0.0f, 1.0f));
            _zAxisDragger->setMatrix(osg::Matrix(rotation));
        }

        // Send different colors for each dragger.
        _xAxisDragger->setColor(osg::Vec4(1.0f,0.0f,0.0f,1.0f));
        _yAxisDragger->setColor(osg::Vec4(0.0f,1.0f,0.0f,1.0f));
        _zAxisDragger->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));
    }
#endif
}

void myDragger::setAxisLineWidth(float linePixelWidth)
{
    _axisLineWidth = linePixelWidth;
    if (_lineWidth.valid())
        _lineWidth->setWidth(linePixelWidth);
}
void myDragger::setPickCylinderHeight(float pickCylinderHeight)
{
    _pickCylinderHeight = pickCylinderHeight;
    if (_cylinder.valid())
        _cylinder->setHeight(pickCylinderHeight);
}