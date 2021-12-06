/*
 * Copyright (c) 2011-2021, The DART development contributors:
 * https://github.com/dartsim/dart/blob/main/LICENSE
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "dart/gui/node/grid_node.hpp"

namespace dart::gui {

namespace {

//==============================================================================
::osg::Vec3 toVec3(const Eigen::Vector3d& point)
{
  return ::osg::Vec3(
      static_cast<float>(point.x()),
      static_cast<float>(point.y()),
      static_cast<float>(point.z()));
}

//==============================================================================
void setVertices(
    ::osg::Vec3Array* axisLineVertices,
    ::osg::Vec3Array* majorLineVertices,
    ::osg::Vec3Array* minorLineVertices,
    std::size_t numCells,
    std::size_t numMinorLinesPerMajorLine,
    float stepSize,
    GridNode::PlaneType planeType,
    const Eigen::Vector3d& offset)
{
  assert(axisLineVertices);
  assert(majorLineVertices);
  assert(minorLineVertices);

  int axis1 = 0;
  int axis2 = 1;

  switch (planeType) {
    case GridNode::PlaneType::XY: {
      axis1 = 0;
      axis2 = 1;
      break;
    }
    case GridNode::PlaneType::YZ: {
      axis1 = 1;
      axis2 = 2;
      break;
    }
    case GridNode::PlaneType::ZX: {
      axis1 = 2;
      axis2 = 0;
      break;
    }
  }

  const std::size_t numAxisLineVertices = 5;
  const std::size_t numMajorLineVertices
      = numMinorLinesPerMajorLine > 0
            ? 8 * (numCells / numMinorLinesPerMajorLine)
            : 0;
  const std::size_t numMinorLineVertices = 8 * numCells - numMajorLineVertices;

  axisLineVertices->clear();
  axisLineVertices->reserve(numAxisLineVertices);
  majorLineVertices->clear();
  majorLineVertices->reserve(numMajorLineVertices);
  minorLineVertices->clear();
  minorLineVertices->reserve(numMinorLineVertices);

  const auto posInAxis1 = numCells * stepSize;
  const auto axis1Negative = -posInAxis1;
  const auto axis1Positive = +posInAxis1;

  ::osg::Vec3 vec3;
  const ::osg::Vec3 osgOffset = toVec3(offset);

  //----------------
  // Axis Vertices
  //----------------

  // Origin
  axisLineVertices->push_back(vec3 + osgOffset);

  // Axis1 positive
  vec3[axis1] = posInAxis1;
  axisLineVertices->push_back(vec3 + osgOffset);

  // Axis1 negative
  vec3[axis1] = -posInAxis1;
  axisLineVertices->push_back(vec3 + osgOffset);
  vec3[axis1] = 0;

  // Axis2 positive
  vec3[axis2] = posInAxis1;
  axisLineVertices->push_back(vec3 + osgOffset);

  // Axis2 negative
  vec3[axis2] = -posInAxis1;
  axisLineVertices->push_back(vec3 + osgOffset);
  vec3[axis2] = 0;

  //-------------------------------
  // Major and minor line vertices
  //-------------------------------

  for (auto i = 1u; i < numCells + 1; ++i) {
    const float posInAxis2 = stepSize * i;

    const auto axis2Negative = -posInAxis2;
    const auto axis2Positive = +posInAxis2;

    if (numMinorLinesPerMajorLine > 0 && i % numMinorLinesPerMajorLine == 0) {
      vec3[axis1] = axis1Negative;
      vec3[axis2] = axis2Positive;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis1] = axis1Positive;
      vec3[axis2] = axis2Positive;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis1] = axis1Negative;
      vec3[axis2] = axis2Negative;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis1] = axis1Positive;
      vec3[axis2] = axis2Negative;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Negative;
      vec3[axis1] = axis2Positive;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Positive;
      vec3[axis1] = axis2Positive;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Negative;
      vec3[axis1] = axis2Negative;
      majorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Positive;
      vec3[axis1] = axis2Negative;
      majorLineVertices->push_back(vec3 + osgOffset);
    } else {
      vec3[axis1] = axis1Negative;
      vec3[axis2] = axis2Positive;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis1] = axis1Positive;
      vec3[axis2] = axis2Positive;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis1] = axis1Negative;
      vec3[axis2] = axis2Negative;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis1] = axis1Positive;
      vec3[axis2] = axis2Negative;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Negative;
      vec3[axis1] = axis2Positive;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Positive;
      vec3[axis1] = axis2Positive;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Negative;
      vec3[axis1] = axis2Negative;
      minorLineVertices->push_back(vec3 + osgOffset);

      vec3[axis2] = axis1Positive;
      vec3[axis1] = axis2Negative;
      minorLineVertices->push_back(vec3 + osgOffset);
    }
  }
}

} // namespace

//==============================================================================
GridNode::GridNode() : Node()
{
  initialize();
}

//==============================================================================
GridNode::~GridNode()
{
  // Do nothing
}

//==============================================================================
void GridNode::update()
{
  if (!mNeedUpdate) {
    return;
  }

  if (mDisplayGrid) {
    setVertices(
        mAxisLineVertices,
        mMajorLineVertices,
        mMinorLineVertices,
        mNumCells,
        mNumMinorLinesPerMajorLine,
        static_cast<float>(mMinorLineStepSize),
        mPlaneType,
        mOffset);

    mMajorLineFaces->clear();
    mMajorLineFaces->reserve(mMajorLineVertices->size());
    for (auto i = 0u; i < mMajorLineVertices->size(); ++i)
      mMajorLineFaces->push_back(i);

    mMinorLineFaces->clear();
    mMinorLineFaces->reserve(mMinorLineVertices->size());
    for (auto i = 0u; i < mMinorLineVertices->size(); ++i)
      mMinorLineFaces->push_back(i);

    mMinorLineGeom->setVertexArray(mMinorLineVertices);
    mMinorLineGeom->getOrCreateStateSet()->setAttributeAndModes(
        mMinorLineWidth);
    mMinorLineGeom->setPrimitiveSet(0, mMinorLineFaces);

    mMajorLineGeom->setVertexArray(mMajorLineVertices);
    mMajorLineGeom->getOrCreateStateSet()->setAttributeAndModes(
        mMajorLineWidth);
    mMajorLineGeom->setPrimitiveSet(0, mMajorLineFaces);

    static const ::osg::Vec4 xAxisLineColor(0.9f, 0.1f, 0.1f, 1.0f);
    static const ::osg::Vec4 yAxisLineColor(0.1f, 0.9f, 0.1f, 1.0f);
    static const ::osg::Vec4 zAxisLineColor(0.1f, 0.1f, 0.9f, 1.0f);

    switch (mPlaneType) {
      case GridNode::PlaneType::XY: {
        mAxisLineColor->at(0) = xAxisLineColor;
        mAxisLineColor->at(2) = yAxisLineColor;
        break;
      }
      case GridNode::PlaneType::YZ: {
        mAxisLineColor->at(0) = yAxisLineColor;
        mAxisLineColor->at(2) = zAxisLineColor;
        break;
      }
      case GridNode::PlaneType::ZX: {
        mAxisLineColor->at(0) = zAxisLineColor;
        mAxisLineColor->at(2) = xAxisLineColor;
        break;
      }
    }

    mAxisLineGeom->setColorArray(mAxisLineColor);
  }

  mNeedUpdate = false;
}

//==============================================================================
void GridNode::initialize()
{
  mNeedUpdate = true;

  mDisplayGrid = true;

  mPlaneType = PlaneType::XY;
  mNumCells = 20;
  mMinorLineStepSize = 0.1;
  mNumMinorLinesPerMajorLine = 5;

  mOffset = Eigen::Vector3d::Zero();

  mGeode = new ::osg::Geode;
  mGeode->getOrCreateStateSet()->setMode(
      GL_LIGHTING, ::osg::StateAttribute::OFF);
  m_osg_node->addChild(mGeode);

  mMinorLineGeom = new ::osg::Geometry;
  mMajorLineGeom = new ::osg::Geometry;
  mAxisLineGeom = new ::osg::Geometry;
  mGeode->addDrawable(mMinorLineGeom);
  mGeode->addDrawable(mMajorLineGeom);
  mGeode->addDrawable(mAxisLineGeom);

  mMinorLineVertices = new ::osg::Vec3Array;
  mMinorLineGeom->setVertexArray(mMinorLineVertices);
  mMinorLineGeom->setDataVariance(::osg::Object::STATIC);

  mMajorLineVertices = new ::osg::Vec3Array;
  mMajorLineGeom->setVertexArray(mMajorLineVertices);
  mMajorLineGeom->setDataVariance(::osg::Object::STATIC);

  mAxisLineVertices = new ::osg::Vec3Array;
  mAxisLineGeom->setVertexArray(mAxisLineVertices);
  mAxisLineGeom->setDataVariance(::osg::Object::STATIC);
  mAxisLineGeom->getOrCreateStateSet()->setMode(
      GL_BLEND, ::osg::StateAttribute::ON);
  mAxisLineGeom->getOrCreateStateSet()->setRenderingHint(
      ::osg::StateSet::TRANSPARENT_BIN);

  // Set grid color
  static const ::osg::Vec4 majorLineColor(0.4f, 0.4f, 0.4f, 1.0f);
  static const ::osg::Vec4 minorLineColor(0.5f, 0.5f, 0.5f, 1.0f);

  mAxisLineColor = new ::osg::Vec4Array;
  mAxisLineColor->resize(4);
  mAxisLineColor->at(0) = majorLineColor; // will be set on the fly
  mAxisLineColor->at(1) = majorLineColor;
  mAxisLineColor->at(2) = majorLineColor; // will be set on the fly
  mAxisLineColor->at(3) = majorLineColor;
  mAxisLineGeom->setColorArray(mAxisLineColor);
  mAxisLineGeom->setColorBinding(::osg::Geometry::BIND_PER_PRIMITIVE_SET);

  mMajorLineColor = new ::osg::Vec4Array;
  mMajorLineColor->resize(1);
  mMajorLineColor->at(0) = majorLineColor;
  mMajorLineGeom->setColorArray(mMajorLineColor);
  mMajorLineGeom->setColorBinding(::osg::Geometry::BIND_OVERALL);
  mMajorLineGeom->getOrCreateStateSet()->setMode(
      GL_BLEND, ::osg::StateAttribute::ON);
  mMajorLineGeom->getOrCreateStateSet()->setRenderingHint(
      ::osg::StateSet::TRANSPARENT_BIN);

  mMinorLineColor = new ::osg::Vec4Array;
  mMinorLineColor->resize(1);
  mMinorLineColor->at(0) = minorLineColor;
  mMinorLineGeom->setColorArray(mMinorLineColor);
  mMinorLineGeom->setColorBinding(::osg::Geometry::BIND_OVERALL);
  mMinorLineGeom->getOrCreateStateSet()->setMode(
      GL_BLEND, ::osg::StateAttribute::ON);
  mMinorLineGeom->getOrCreateStateSet()->setRenderingHint(
      ::osg::StateSet::TRANSPARENT_BIN);

  mMinorLineFaces = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::LINES, 0);
  mMinorLineGeom->addPrimitiveSet(mMinorLineFaces);

  mMajorLineFaces = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::LINES, 0);
  mMajorLineGeom->addPrimitiveSet(mMajorLineFaces);

  mAxis1PositiveFaces
      = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::LINES, 0);
  mAxis1NegativeFaces
      = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::LINES, 0);
  mAxis2PositiveFaces
      = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::LINES, 0);
  mAxis2NegativeFaces
      = new ::osg::DrawElementsUInt(::osg::PrimitiveSet::LINES, 0);
  mAxis1PositiveFaces->resize(2);
  mAxis1NegativeFaces->resize(2);
  mAxis2PositiveFaces->resize(2);
  mAxis2NegativeFaces->resize(2);
  mAxis1PositiveFaces->at(0) = 0;
  mAxis1PositiveFaces->at(1) = 1;
  mAxis1NegativeFaces->at(0) = 0;
  mAxis1NegativeFaces->at(1) = 2;
  mAxis2PositiveFaces->at(0) = 0;
  mAxis2PositiveFaces->at(1) = 3;
  mAxis2NegativeFaces->at(0) = 0;
  mAxis2NegativeFaces->at(1) = 4;
  mAxisLineGeom->addPrimitiveSet(mAxis1PositiveFaces);
  mAxisLineGeom->addPrimitiveSet(mAxis1NegativeFaces);
  mAxisLineGeom->addPrimitiveSet(mAxis2PositiveFaces);
  mAxisLineGeom->addPrimitiveSet(mAxis2NegativeFaces);

  mAxisLineWidth = new ::osg::LineWidth(2);
  mMajorLineWidth = new ::osg::LineWidth(2);
  mMinorLineWidth = new ::osg::LineWidth(1);

  mAxisLineGeom->getOrCreateStateSet()->setAttributeAndModes(mAxisLineWidth);
  mMinorLineGeom->getOrCreateStateSet()->setAttributeAndModes(mMinorLineWidth);
  mMajorLineGeom->getOrCreateStateSet()->setAttributeAndModes(mMajorLineWidth);
}

} // namespace dart::gui
