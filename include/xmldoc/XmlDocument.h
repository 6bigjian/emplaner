#ifndef __XmlDocument_h__
#define __XmlDocument_h__

#include <xercesc/parsers/XercesDOMParser.hpp>
#include <xercesc/dom/DOM.hpp>
#include <xercesc/sax/HandlerBase.hpp>
#include <xercesc/util/XMLString.hpp>
#include <xercesc/util/PlatformUtils.hpp>

#include <string>

using namespace std;
using namespace xercesc;

class XmlDomDocument
{
  DOMDocument* m_doc;

public:
  XmlDomDocument(const char* xmlfile);
  ~XmlDomDocument();

  string getChildValue(const char* parentTag, int parentIndex, const char* childTag, int childIndex);
  string getChildAttribute(const char* parentTag, int parentIndex, const char* childTag, int childIndex,
                           const char* attributeTag);
  int getRootElementCount(const char* rootElementTag);
  int getChildCount(const char* parentTag, int parentIndex, const char* childTag);

private:
  XmlDomDocument();
  XmlDomDocument(const XmlDomDocument&);
};

#endif