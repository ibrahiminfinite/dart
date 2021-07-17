"""
Copyright (c) 2011-2021, The DART development contributors
All rights reserved.

The list of contributors can be found at:
  https://github.com/dartsim/dart/blob/master/LICENSE
 
This file is provided under the following "BSD-style" License:
  Redistribution and use in source and binary forms, with or
  without modification, are permitted provided that the following
  conditions are met:
  * Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
  * Redistributions in binary form must reproduce the above
    copyright notice, this list of conditions and the following
    disclaimer in the documentation and/or other materials provided
    with the distribution.
  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
  USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
"""

import platform
import pytest
import dartpy as dart


def test_inertial_frame():
    frame = dart.multibody.Frame.GetInertialFrame()

    # Properties
    assert frame.is_inertial_frame()
    assert frame.get_name() == dart.multibody.InertialFrame.GetName()

    # Positions == 0
    assert frame.get_pose().is_identity()
    assert frame.get_orientation().is_identity()
    assert frame.get_position().is_identity()

    # Velocities == 0
    assert frame.get_spatial_velocity().is_zero()
    assert frame.get_angular_velocity().is_zero()
    assert frame.get_linear_velocity().is_zero()

    # Accelerations == 0
    assert frame.get_spatial_acceleration().is_zero()
    assert frame.get_angular_acceleration().is_zero()
    assert frame.get_linear_acceleration().is_zero()


def test_simple_relative_frame():
    frame = dart.multibody.FreeRelativeFrame()

    assert frame.get_name() == ""
    assert frame.get_pose().is_identity()
    assert frame.get_spatial_velocity().is_zero()
    assert frame.get_spatial_acceleration().is_zero()


if __name__ == "__main__":
    pytest.main()
