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

#pragma once

#include "dart/gui/node/node.hpp"

namespace dart::gui {

class DART_GUI_API GridNode : public Node
{
public:
  enum class PlaneType : unsigned char
  {
    XY = 0u,
    YZ = 1u,
    ZX = 2u,
  };

  GridNode();

  ~GridNode() override;

  void update() override;

protected:
  /// Initializes the memory used by this visual
  void initialize();

  /// Plane type among XY, YZ, ZX planes
  PlaneType mPlaneType;

  /// Number of cells along each axis
  std::size_t mNumCells;

  /// Step size of minor lines in meters
  double mMinorLineStepSize;

  /// Number of minor lines per major line
  std::size_t mNumMinorLinesPerMajorLine;

  /// Elevation that this visual should use
  Eigen::Vector3d mOffset;

  /// Whether to display the grid
  bool mDisplayGrid;

  /// Color for axis lines
  ::osg::ref_ptr<::osg::Vec4Array> mAxisLineColor;

  /// Color for major lines
  ::osg::ref_ptr<::osg::Vec4Array> mMajorLineColor;

  /// Color for minor lines
  ::osg::ref_ptr<::osg::Vec4Array> mMinorLineColor;

  /// Geode to hold the grid
  ::osg::ref_ptr<::osg::Geode> mGeode;

  /// Geometry to describe axis lines
  ::osg::ref_ptr<::osg::Geometry> mAxisLineGeom;

  /// Geometry to describe minor lines
  ::osg::ref_ptr<::osg::Geometry> mMajorLineGeom;

  /// Geometry to describe major lines
  ::osg::ref_ptr<::osg::Geometry> mMinorLineGeom;

  /// Vertices of axis lines
  ::osg::ref_ptr<::osg::Vec3Array> mMinorLineVertices;

  /// Vertices of major lines
  ::osg::ref_ptr<::osg::Vec3Array> mMajorLineVertices;

  /// Vertices of minor lines
  ::osg::ref_ptr<::osg::Vec3Array> mAxisLineVertices;

  /// Faces of the first axis positive line
  ::osg::ref_ptr<::osg::DrawElementsUInt> mAxis1PositiveFaces;

  /// Faces of the first axis negative line
  ::osg::ref_ptr<::osg::DrawElementsUInt> mAxis1NegativeFaces;

  /// Faces of the second axis positive line
  ::osg::ref_ptr<::osg::DrawElementsUInt> mAxis2PositiveFaces;

  /// Faces of the second axis negative line
  ::osg::ref_ptr<::osg::DrawElementsUInt> mAxis2NegativeFaces;

  /// Faces of major lines
  ::osg::ref_ptr<::osg::DrawElementsUInt> mMajorLineFaces;

  /// Faces of minor lines
  ::osg::ref_ptr<::osg::DrawElementsUInt> mMinorLineFaces;

  /// Line width for axis line
  ::osg::ref_ptr<::osg::LineWidth> mAxisLineWidth;

  /// Line width for major line
  ::osg::ref_ptr<::osg::LineWidth> mMajorLineWidth;

  /// Line width for minor line
  ::osg::ref_ptr<::osg::LineWidth> mMinorLineWidth;

  /// Dirty flag to notify this grid needs to be updated
  bool mNeedUpdate;
};

} // namespace dart::gui
