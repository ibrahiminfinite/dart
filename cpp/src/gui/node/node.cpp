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

#include "dart/gui/node/node.hpp"

#include "dart/common/macro.hpp"

namespace dart::gui {

//==============================================================================
class NodeUpdateCallback : public ::osg::NodeCallback
{
public:
  NodeUpdateCallback(Node* parent_node) : m_parent_node(parent_node)
  {
    DART_ASSERT(m_parent_node);
  }

  void operator()(osg::Node* node, osg::NodeVisitor* nv) override
  {
    m_parent_node->update();
    traverse(node, nv);
  }

private:
  Node* m_parent_node;
};

//==============================================================================
Node::Node()
{
  m_osg_node = new osg::Group();
  m_osg_node->setUpdateCallback(new NodeUpdateCallback(this));
}

//==============================================================================
Node::~Node()
{
  // Do nothing
}

//==============================================================================
void Node::add_child(Node* node)
{
  if (!node) {
    return;
  }

  auto osg_node = node->get_mutable_osg_node();
  if (!osg_node) {
    return;
  }

  if (m_osg_node->containsNode(osg_node)) {
    return;
  }

  m_osg_node->addChild(osg_node);
}

//==============================================================================
bool Node::remove_child(Node* node)
{
  if (!node) {
    return false;
  }

  auto osg_node = node->get_mutable_osg_node();
  if (!osg_node) {
    return false;
  }

  bool success = m_osg_node->removeChild(osg_node);

  std::cout << "Child node is removed" << std::endl;
  std::cout << "# of children: " << m_osg_node->getNumChildren() << std::endl;

  if (!success) {
    return false;
  }

  return true;
}

//==============================================================================
void Node::update()
{
  // Do nothing
}

//==============================================================================
osg::Group* Node::get_mutable_osg_node()
{
  return m_osg_node;
}

} // namespace dart::gui
