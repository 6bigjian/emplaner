#include <stdio.h>
#include "xmldoc/XmlDocument.h"

class XmlDomErrorHandler : public HandlerBase
{
public:
  void fatalError(const SAXParseException& exc)
  {
    printf("Fatal parsing error at line %d\n", (int)exc.getLineNumber());
    exit(-1);
  }
};

XercesDOMParser* parser = NULL;
ErrorHandler* errorHandler = NULL;

void createParser()
{
  if (!parser)
  {
    XMLPlatformUtils::Initialize();
    parser = new XercesDOMParser();
    errorHandler = (ErrorHandler*)new XmlDomErrorHandler();
    parser->setErrorHandler(errorHandler);
  }
}

XmlDomDocument::XmlDomDocument(const char* xmlfile) : m_doc(NULL)
{
  createParser();
  parser->parse(xmlfile);
  m_doc = parser->adoptDocument();
}

XmlDomDocument::~XmlDomDocument()
{
  if (m_doc)
    m_doc->release();
}

string XmlDomDocument::getChildValue(const char* parentTag, int parentIndex, const char* childTag, int childIndex)
{
  XMLCh* temp = XMLString::transcode(parentTag);
  DOMNodeList* list = m_doc->getElementsByTagName(temp);
  XMLString::release(&temp);

  DOMElement* parent = dynamic_cast<DOMElement*>(list->item(parentIndex));

  DOMElement* child =
      dynamic_cast<DOMElement*>(parent->getElementsByTagName(XMLString::transcode(childTag))->item(childIndex));
  string value;
  if (child)
  {
    char* temp2 = XMLString::transcode(child->getTextContent());
    value = temp2;
    XMLString::release(&temp2);
  }
  else
  {
    value = "";
  }
  return value;
}

string XmlDomDocument::getChildAttribute(const char* parentTag, int parentIndex, const char* childTag, int childIndex,
                                         const char* attributeTag)
{
  XMLCh* temp = XMLString::transcode(parentTag);
  DOMNodeList* list = m_doc->getElementsByTagName(temp);
  XMLString::release(&temp);

  DOMElement* parent = dynamic_cast<DOMElement*>(list->item(parentIndex));
  DOMElement* child =
      dynamic_cast<DOMElement*>(parent->getElementsByTagName(XMLString::transcode(childTag))->item(childIndex));
  string value;
  if (child)
  {
    temp = XMLString::transcode(attributeTag);
    char* temp2 = XMLString::transcode(child->getAttribute(temp));
    value = temp2;
    XMLString::release(&temp2);
  }
  else
  {
    value = "";
  }
  return value;
}

int XmlDomDocument::getRootElementCount(const char* rootElementTag)
{
  DOMNodeList* list = m_doc->getElementsByTagName(XMLString::transcode(rootElementTag));
  return (int)list->getLength();
}

int XmlDomDocument::getChildCount(const char* parentTag, int parentIndex, const char* childTag)
{
  XMLCh* temp = XMLString::transcode(parentTag);
  DOMNodeList* list = m_doc->getElementsByTagName(temp);
  XMLString::release(&temp);

  DOMElement* parent = dynamic_cast<DOMElement*>(list->item(parentIndex));
  DOMNodeList* childList = parent->getElementsByTagName(XMLString::transcode(childTag));
  return (int)childList->getLength();
}