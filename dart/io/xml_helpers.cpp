/*
 * Copyright (c) 2011-2021, The DART development contributors
 * All rights reserved.
 *
 * The list of contributors can be found at:
 *   https://github.com/dartsim/dart/blob/main/LICENSE
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

#include "dart/io/xml_helpers.hpp"

#include <iostream>
#include <vector>

#include "dart/common/Console.hpp"
#include "dart/common/LocalResourceRetriever.hpp"
#include "dart/common/string.hpp"
#include "dart/math/Geometry.hpp"
#include "dart/math/type.hpp"

namespace dart {
namespace io {

//==============================================================================
math::Vector2d to_vector2d(const std::string& str)
{
  return to_vector<double, 2>(str);
}

//==============================================================================
math::Vector3d to_vector3d(const std::string& str)
{
  return to_vector<double, 3>(str);
}

//==============================================================================
math::Vector4d to_vector4d(const std::string& str)
{
  return to_vector<double, 4>(str);
}

//==============================================================================
math::Vector6d to_vector6d(const std::string& str)
{
  return to_vector<double, 6>(str);
}

//==============================================================================
math::Vector2i to_vector2i(const std::string& str)
{
  return to_vector<int, 2>(str);
}

//==============================================================================
math::Vector3i to_vector3i(const std::string& str)
{
  return to_vector<int, 3>(str);
}

//==============================================================================
std::string getValueString(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return str;
}

//==============================================================================
bool getValueBool(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  const std::string str
      = parentElement->FirstChildElement(name.c_str())->GetText();
  return common::to_bool(str);
}

//==============================================================================
int getValueInt(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return common::to_int(str);
}

//==============================================================================
unsigned int getValueUInt(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return common::to_uint(str);
}

//==============================================================================
float getValueFloat(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return common::to_float(str);
}

//==============================================================================
double getValueDouble(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return common::to_double(str);
}

//==============================================================================
Eigen::Vector2d getValueVector2d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_vector2d(str);
}

//==============================================================================
Eigen::Vector3d getValueVector3d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_vector3d(str);
}

//==============================================================================
Eigen::Vector3i getValueVector3i(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_vector3i(str);
}

//==============================================================================
Eigen::Vector6d getValueVector6d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_vector6d(str);
}

//==============================================================================
Eigen::VectorXd getValueVectorXd(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_vector_x<double>(str);
}

//==============================================================================
Eigen::Vector3d getValueVec3(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_vector3d(str);
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3d(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  std::string str = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_isometry3<double>(str);
}

//==============================================================================
Eigen::Isometry3d getValueIsometry3dWithExtrinsicRotation(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  const std::string str
      = parentElement->FirstChildElement(name.c_str())->GetText();

  return to_isometry3<double>(str, "extrinsic");
}

//==============================================================================
bool hasElement(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(parentElement != nullptr);
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str()) == nullptr ? false
                                                                   : true;
}

//==============================================================================
const tinyxml2::XMLElement* getElement(
    const tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str());
}

//==============================================================================
tinyxml2::XMLElement* getElement(
    tinyxml2::XMLElement* parentElement, const std::string& name)
{
  assert(!name.empty());

  return parentElement->FirstChildElement(name.c_str());
}

//==============================================================================
std::string toString(tinyxml2::XMLError errorCode)
{
  switch (errorCode) {
    case tinyxml2::XMLError::XML_SUCCESS:
      return "XML_SUCCESS";
    case tinyxml2::XMLError::XML_NO_ATTRIBUTE:
      return "XML_NO_ATTRIBUTE";
    case tinyxml2::XMLError::XML_WRONG_ATTRIBUTE_TYPE:
      return "XML_WRONG_ATTRIBUTE_TYPE";
    case tinyxml2::XMLError::XML_ERROR_FILE_NOT_FOUND:
      return "XML_ERROR_FILE_NOT_FOUND";
    case tinyxml2::XMLError::XML_ERROR_FILE_COULD_NOT_BE_OPENED:
      return "XML_ERROR_FILE_COULD_NOT_BE_OPENED";
    case tinyxml2::XMLError::XML_ERROR_FILE_READ_ERROR:
      return "XML_ERROR_FILE_READ_ERROR";
    case tinyxml2::XMLError::XML_ERROR_PARSING_ELEMENT:
      return "XML_ERROR_PARSING_ELEMENT";
    case tinyxml2::XMLError::XML_ERROR_PARSING_ATTRIBUTE:
      return "XML_ERROR_PARSING_ATTRIBUTE";
    case tinyxml2::XMLError::XML_ERROR_PARSING_TEXT:
      return "XML_ERROR_PARSING_TEXT";
    case tinyxml2::XMLError::XML_ERROR_PARSING_CDATA:
      return "XML_ERROR_PARSING_CDATA";
    case tinyxml2::XMLError::XML_ERROR_PARSING_COMMENT:
      return "XML_ERROR_PARSING_COMMENT";
    case tinyxml2::XMLError::XML_ERROR_PARSING_DECLARATION:
      return "XML_ERROR_PARSING_DECLARATION";
    case tinyxml2::XMLError::XML_ERROR_PARSING_UNKNOWN:
      return "XML_ERROR_PARSING_UNKNOWN";
    case tinyxml2::XMLError::XML_ERROR_EMPTY_DOCUMENT:
      return "XML_ERROR_EMPTY_DOCUMENT";
    case tinyxml2::XMLError::XML_ERROR_MISMATCHED_ELEMENT:
      return "XML_ERROR_MISMATCHED_ELEMENT";
    case tinyxml2::XMLError::XML_ERROR_PARSING:
      return "XML_ERROR_PARSING";
    case tinyxml2::XMLError::XML_CAN_NOT_CONVERT_TEXT:
      return "XML_CAN_NOT_CONVERT_TEXT";
    case tinyxml2::XMLError::XML_NO_TEXT_NODE:
      return "XML_NO_TEXT_NODE";
    case tinyxml2::XMLError::XML_ERROR_COUNT:
      return "XML_ERROR_COUNT";
    default:
      return "Unknow error";
  }
}

//==============================================================================
void openXMLFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr)
{
  common::ResourceRetrieverPtr retriever;
  if (retrieverOrNullPtr)
    retriever = retrieverOrNullPtr;
  else
    retriever = std::make_shared<common::LocalResourceRetriever>();

  const auto content = retriever->readAll(uri);
  const auto result = doc.Parse(&content.front());
  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[openXMLFile] Failed parsing XML: TinyXML2 returned error '"
           << toString(result) << "'.\n";
    throw std::runtime_error("Failed parsing XML.");
  }
}

//==============================================================================
bool readXmlFile(
    tinyxml2::XMLDocument& doc,
    const common::Uri& uri,
    const common::ResourceRetrieverPtr& retrieverOrNullPtr)
{
  common::ResourceRetrieverPtr retriever;
  if (retrieverOrNullPtr)
    retriever = retrieverOrNullPtr;
  else
    retriever = std::make_shared<common::LocalResourceRetriever>();

  const auto content = retriever->readAll(uri);
  const auto result = doc.Parse(&content.front());
  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[readXmlFile] Failed parsing XML: TinyXML2 returned error '"
           << toString(result) << "'.\n";
    return false;
  }

  return true;
}

//==============================================================================
bool hasAttribute(const tinyxml2::XMLElement* element, const char* const name)
{
  const char* const result = element->Attribute(name);
  return result != nullptr;
}

//==============================================================================
std::string getAttributeString(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const char* const result = element->Attribute(attributeName.c_str());

  if (nullptr == result) {
    dtwarn << "[getAttribute] Error in parsing string type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning empty string.\n";
    return std::string();
  }

  return std::string(result);
}

//==============================================================================
bool getAttributeBool(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  bool val = false;
  const int result = element->QueryBoolAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing bool type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning false instead.\n";
    return false;
  }

  return val;
}

//==============================================================================
int getAttributeInt(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  int val = 0;
  const int result = element->QueryIntAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing int type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0;
  }

  return val;
}

//==============================================================================
unsigned int getAttributeUInt(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  unsigned int val = 0u;
  const int result
      = element->QueryUnsignedAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing unsiged int type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0u;
  }

  return val;
}

//==============================================================================
float getAttributeFloat(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  float val = 0.0f;
  const int result = element->QueryFloatAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing float type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0.0f;
  }

  return val;
}

//==============================================================================
double getAttributeDouble(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  double val = 0.0;
  const int result = element->QueryDoubleAttribute(attributeName.c_str(), &val);

  if (result != tinyxml2::XML_SUCCESS) {
    dtwarn << "[getAttribute] Error in parsing double type attribute ["
           << attributeName << "] of an element [" << element->Name()
           << "]. Returning zero instead.\n";
    return 0.0;
  }

  return val;
}

//==============================================================================
char getAttributeChar(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return common::to_char(val);
}

//==============================================================================
Eigen::Vector2i getAttributeVector2i(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return to_vector2i(val);
}

//==============================================================================
Eigen::Vector2d getAttributeVector2d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return to_vector2d(val);
}

//==============================================================================
Eigen::Vector3d getAttributeVector3d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return to_vector3d(val);
}

//==============================================================================
Eigen::Vector4d getAttributeVector4d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return to_vector4d(val);
}

//==============================================================================
Eigen::Vector6d getAttributeVector6d(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return to_vector6d(val);
}

//==============================================================================
Eigen::VectorXd getAttributeVectorXd(
    const tinyxml2::XMLElement* element, const std::string& attributeName)
{
  const std::string val = getAttributeString(element, attributeName);

  return to_vector_x<double>(val);
}

//==============================================================================
bool copyNode(tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src)
{
  // Protect from evil
  if (destParent == nullptr) {
    return false;
  }

  // Get the document context where new memory will be allocated from
  tinyxml2::XMLDocument* doc = destParent->GetDocument();

  // Make the copy
  tinyxml2::XMLNode* copy = src.ShallowClone(doc);
  if (copy == nullptr) {
    return false;
  }

  // Add this child
  destParent->InsertEndChild(copy);

  // Add the grandkids
  for (const tinyxml2::XMLNode* node = src.FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    if (not copyNode(copy, *node)) {
      return false;
    }
  }

  return true;
}

//==============================================================================
bool copyChildNodes(tinyxml2::XMLNode* destParent, const tinyxml2::XMLNode& src)
{
  for (const tinyxml2::XMLNode* node = src.FirstChild(); node != nullptr;
       node = node->NextSibling()) {
    if (not copyNode(destParent, *node)) {
      return false;
    }
  }

  return true;
}

} // namespace io
} // namespace dart
