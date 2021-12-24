/*
 * Copyright (c) 2011-2022, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/master/LICENSE
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DART_CONSTRAINT_CONSTRAINEDGROUP_HPP_
#define DART_CONSTRAINT_CONSTRAINEDGROUP_HPP_

#include <memory>
#include <vector>

#include "dart/common/Deprecated.hpp"
#include "dart/common/MemoryAllocator.hpp"
#include "dart/common/StlContainers.hpp"
#include "dart/constraint/SmartPointer.hpp"

namespace dart {

namespace dynamics {
class Skeleton;
} // namespace dynamics

namespace constraint {

class ConstraintBase;
class ConstraintSolver;

/// ConstrainedGroup is a group of skeletons that interact each other with
/// constraints
/// \sa class ConstraintSolver
class ConstrainedGroup
{
public:
  //----------------------------------------------------------------------------
  // Constructor / Desctructor
  //----------------------------------------------------------------------------

  /// Default contructor
  DART_DEPRECATED(6.13)
  ConstrainedGroup();

  /// Contructor
  explicit ConstrainedGroup(common::MemoryAllocator& memoryAllocator);

  /// Destructor
  virtual ~ConstrainedGroup();

  //----------------------------------------------------------------------------
  // Setting
  //----------------------------------------------------------------------------

  /// Add constraint
  DART_DEPRECATED(6.13)
  void addConstraint(const ConstraintBasePtr& _constraint);

  /// Add constraint
  void addConstraint(ConstraintBase* constraint);
  // Note that it's the responsibility of the owner of ConstrainedGroup to
  // manage the lifetime of this class and ConstraintBases that are passed
  // to this class.

  /// Return number of constraints in this constrained group
  [[nodiscard]] std::size_t getNumConstraints() const;

  /// Return a constraint
  DART_DEPRECATED(6.13)
  [[nodiscard]] ConstraintBasePtr getConstraint(std::size_t _index);

  /// Return a constraint
  DART_DEPRECATED(6.13)
  [[nodiscard]] ConstConstraintBasePtr getConstraint(std::size_t _index) const;

  /// Return a constraint
  [[nodiscard]] ConstraintBase* getConstraint2(std::size_t index);
  // TODO(JS): In the next major release, rename this to getConstraint

  /// Return a constraint
  [[nodiscard]] const ConstraintBase* getConstraint2(std::size_t index) const;
  // TODO(JS): In the next major release, rename this to getConstraint

  template <typename Func>
  void eachConstraint(Func func);

  template <typename Func>
  void eachConstraint(Func func) const;

  /// Remove constraint
  DART_DEPRECATED(6.13)
  void removeConstraint(const ConstraintBasePtr& _constraint);

  /// Remove constraint
  void removeConstraint(const ConstraintBase* constraint);

  /// Remove all constraints
  void removeAllConstraints();

  /// Get total dimension of contraints in this group
  [[nodiscard]] std::size_t getTotalDimension() const;

  //----------------------------------------------------------------------------
  // Friendship
  //----------------------------------------------------------------------------

  friend class ConstraintSolver;

private:
#ifndef NDEBUG
  /// Return true if _constraint is contained
  [[nodiscard]] bool containConstraint(
      const ConstConstraintBasePtr& _constraint) const;

  /// Return true if _constraint is contained
  [[nodiscard]] bool hasConstraint(const ConstraintBase* constraint) const;
#endif

  /// Memory allocator
  common::MemoryAllocator& mMemoryAllocator;

  /// List of constraints
  std::vector<ConstraintBasePtr> mConstraints;
  // TODO(JS): In the next major release, replace this with mConstraints2

  /// List of constraints
  common::vector<ConstraintBase*> mConstraints2;
  // TODO(JS): In the next major release, rename this to mConstraints

  /// The key skeleton used for grouping constraints
  std::shared_ptr<dynamics::Skeleton> mRootSkeleton;
};

} // namespace constraint
} // namespace dart

#include "dart/constraint/detail/ConstrainedGroup-impl.hpp"

#endif // DART_CONSTRAINT_CONSTRAINEDGROUP_HPP_
