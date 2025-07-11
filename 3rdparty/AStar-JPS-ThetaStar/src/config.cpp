#include "config.h"
#include "gl_const.h"
#include "tinyxml2.h"
#include <iostream>
#include <sstream>
#include <algorithm>
#include <math.h>

Config::Config()
{
    SearchParams = nullptr;
}


Config::~Config()
{
    if (SearchParams) delete[] SearchParams;
}

// Set configuration parameters directly, instead of reading from XML
void Config::setConfig(int st, double hw, int mt, int bt, int ad, int cc, int asq, int ps)
{
    if (SearchParams) delete[] SearchParams;
    N = 8;
    SearchParams = new double[N];
    SearchParams[CN_SP_ST] = st;
    SearchParams[CN_SP_HW] = hw;
    SearchParams[CN_SP_MT] = mt;
    SearchParams[CN_SP_BT] = bt;
    SearchParams[CN_SP_AD] = ad;
    SearchParams[CN_SP_CC] = cc;
    SearchParams[CN_SP_AS] = asq;
    SearchParams[CN_SP_PS] = ps;
}

// Set a parameter by tag name (as in gl_const.h) and string value (for bools and enums)
bool Config::setParamByTag(const std::string& tag, const std::string& value)
{
    if (!SearchParams) setDefaultConfigAstar(); // Initialize with A* defaults if not already initialized
    std::string v = value;
    std::transform(v.begin(), v.end(), v.begin(), ::tolower);
    if (tag == CNS_TAG_ST) {
        if (v == CNS_SP_ST_ASTAR) { SearchParams[CN_SP_ST] = CN_SP_ST_ASTAR; return true; }
        if (v == CNS_SP_ST_TH)    { SearchParams[CN_SP_ST] = CN_SP_ST_TH;    return true; }
        return false;
    }
    if (tag == CNS_TAG_MT) {
        if (v == CNS_SP_MT_DIAG)  { SearchParams[CN_SP_MT] = CN_SP_MT_DIAG;  return true; }
        if (v == CNS_SP_MT_MANH)  { SearchParams[CN_SP_MT] = CN_SP_MT_MANH;  return true; }
        if (v == CNS_SP_MT_EUCL)  { SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;  return true; }
        if (v == CNS_SP_MT_CHEB)  { SearchParams[CN_SP_MT] = CN_SP_MT_CHEB;  return true; }
        return false;
    }
    if (tag == CNS_TAG_BT) {
        if (v == CNS_SP_BT_GMIN)  { SearchParams[CN_SP_BT] = CN_SP_BT_GMIN;  return true; }
        if (v == CNS_SP_BT_GMAX)  { SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;  return true; }
        return false;
    }
    if (tag == CNS_TAG_AD || tag == CNS_TAG_CC || tag == CNS_TAG_AS || tag == CNS_TAG_PS) {
        if (v == "1" || v == "true") {
            if (tag == CNS_TAG_AD) SearchParams[CN_SP_AD] = 1;
            if (tag == CNS_TAG_CC) SearchParams[CN_SP_CC] = 1;
            if (tag == CNS_TAG_AS) SearchParams[CN_SP_AS] = 1;
            if (tag == CNS_TAG_PS) SearchParams[CN_SP_PS] = 1;
            return true;
        }
        if (v == "0" || v == "false") {
            if (tag == CNS_TAG_AD) SearchParams[CN_SP_AD] = 0;
            if (tag == CNS_TAG_CC) SearchParams[CN_SP_CC] = 0;
            if (tag == CNS_TAG_AS) SearchParams[CN_SP_AS] = 0;
            if (tag == CNS_TAG_PS) SearchParams[CN_SP_PS] = 0;
            return true;
        }
        return false;
    }
    return false;
}

// Set a parameter by tag name and double value (for numeric params)
bool Config::setParamByTag(const std::string& tag, double value)
{
    if (!SearchParams) setDefaultConfigAstar(); // Initialize with A* defaults if not already initialized
    if (tag == CNS_TAG_HW) {
        if (value < 1) value = 1;
        SearchParams[CN_SP_HW] = value;
        return true;
    }
    return false;
}

// Set all parameters to default values (A* with Euclidean, all options off)
void Config::setDefaultConfigAstar()
{
    if (SearchParams) delete[] SearchParams;
    N = 8;
    SearchParams = new double[N];
    SearchParams[CN_SP_ST] = CN_SP_ST_ASTAR;
    SearchParams[CN_SP_HW] = 1;
    SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
    SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
    SearchParams[CN_SP_AD] = 1;
    SearchParams[CN_SP_CC] = 0;
    SearchParams[CN_SP_AS] = 0;
    SearchParams[CN_SP_PS] = 0;
}

void Config::setDefaultConfigTheta()
{
    if (SearchParams) delete[] SearchParams;
    N = 8;
    SearchParams = new double[N];
    SearchParams[CN_SP_ST] = CN_SP_ST_TH;
    SearchParams[CN_SP_HW] = 1;
    SearchParams[CN_SP_MT] = CN_SP_MT_EUCL; // Theta* requires Euclidean metric
    SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
    SearchParams[CN_SP_AD] = 1;
    SearchParams[CN_SP_CC] = 0;
    SearchParams[CN_SP_AS] = 0;
    SearchParams[CN_SP_PS] = 0;
}


bool Config::getConfig(const char *FileName)
{
    std::string value;
    std::stringstream stream;
    tinyxml2::XMLElement *root = 0, *algorithm = 0, *element = 0, *options = 0;

    tinyxml2::XMLDocument doc;
    if (doc.LoadFile(FileName) != tinyxml2::XMLError::XML_SUCCESS) {
        std::cout << "Error opening XML file!" << std::endl;
        return false;
    }

    root = doc.FirstChildElement(CNS_TAG_ROOT);
    if (!root) {
        std::cout << "Error! No '" << CNS_TAG_ROOT << "' element found in XML file!" << std::endl;
        return false;
    }

    algorithm = root->FirstChildElement(CNS_TAG_ALG);
    if (!algorithm) {
        std::cout << "Error! No '" << CNS_TAG_ALG << "' tag found in XML file!" << std::endl;
        return false;
    }

    element = algorithm->FirstChildElement(CNS_TAG_ST);
    if (!element) {
        std::cout << "Error! No '" << CNS_TAG_ST << "' tag found in XML file!" << std::endl;
        return false;
    }
    if (element->GetText())
        value = element->GetText();
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);

    if (value == CNS_SP_ST_ASTAR || value == CNS_SP_ST_TH) {
        N = 8;
        SearchParams = new double[N];
        SearchParams[CN_SP_ST] = CN_SP_ST_ASTAR;
        if (value == CNS_SP_ST_TH)
            SearchParams[CN_SP_ST] = CN_SP_ST_TH;
        element = algorithm->FirstChildElement(CNS_TAG_HW);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_HW << "' tag found in algorithm section." << std::endl;
            std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
            SearchParams[CN_SP_HW] = 1;
        }
        else {
            stream << element->GetText();
            stream >> SearchParams[CN_SP_HW];
            stream.str("");
            stream.clear();

            if (SearchParams[CN_SP_HW] < 1) {
                std::cout << "Warning! Value of '" << CNS_TAG_HW << "' tag is not correctly specified. Should be >= 1."
                          << std::endl;
                std::cout << "Value of '" << CNS_TAG_HW << "' was defined to 1." << std::endl;
                SearchParams[CN_SP_HW] = 1;
            }
        }

        element = algorithm->FirstChildElement(CNS_TAG_MT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_MT << "' tag found in XML file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_MT << "' was defined to 'euclidean'." << std::endl;
            SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
        }
        else {
            if (element->GetText())
                value = element->GetText();
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CNS_SP_MT_MANH) SearchParams[CN_SP_MT] = CN_SP_MT_MANH;
            else if (value == CNS_SP_MT_EUCL) SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
            else if (value == CNS_SP_MT_DIAG) SearchParams[CN_SP_MT] = CN_SP_MT_DIAG;
            else if (value == CNS_SP_MT_CHEB) SearchParams[CN_SP_MT] = CN_SP_MT_CHEB;
            else {
                std::cout << "Warning! Value of'" << CNS_TAG_MT << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_MT << "' was defined to 'euclidean'" << std::endl;
                SearchParams[CN_SP_MT] = CN_SP_MT_EUCL;
            }
            if (SearchParams[CN_SP_ST] == CN_SP_ST_TH && SearchParams[CN_SP_MT] != CN_SP_MT_EUCL) {
                std::cout << "Warning! This type of metric is not admissible for Theta*!" << std::endl;
            }
        }


        element = algorithm->FirstChildElement(CNS_TAG_BT);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_BT << "' tag found in XML file" << std::endl;
            std::cout << "Value of '" << CNS_TAG_BT << "' was defined to 'g-max'" << std::endl;
            SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
        }
        else {
            value = element->GetText();
            std::transform(value.begin(), value.end(), value.begin(), ::tolower);
            if (value == CNS_SP_BT_GMIN) SearchParams[CN_SP_BT] = CN_SP_BT_GMIN;
            else if (value == CNS_SP_BT_GMAX) SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
            else {
                std::cout << "Warning! Value of '" << CNS_TAG_BT << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_BT << "' was defined to 'g-max'" << std::endl;
                SearchParams[CN_SP_BT] = CN_SP_BT_GMAX;
            }
        }
    }
    else {
        std::cout << "Error! Value of '" << CNS_TAG_ST << "' tag (algorithm name) is not correctly specified."
                  << std::endl;
        std::cout << "Supported algorithm's names are: '"  <<
                  CNS_SP_ST_ASTAR << "', '" << CNS_SP_ST_TH << "'." << std::endl;
        return false;
    }


    element = algorithm->FirstChildElement(CNS_TAG_AD);
    if (!element) {
        std::cout << "Warning! No '" << CNS_TAG_AD << "' element found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_AD << "' was defined to default - true" << std::endl;
        SearchParams[CN_SP_AD] = 1;
    }
    else {
        std::string check;
        stream << element->GetText();
        stream >> check;
        stream.clear();
        stream.str("");

        if (check != "1" && check != "true" && check != "0" && check != "false") {
            std::cout << "Warning! Value of '" << CNS_TAG_AD << "' is not correctly specified." << std::endl;
            std::cout << "Value of '" << CNS_TAG_AD << "' was defined to default - true " << std::endl;
            SearchParams[CN_SP_AD] = 1;
        }
        else if (check == "1" || check == "true")
            SearchParams[CN_SP_AD] = 1;
        else
            SearchParams[CN_SP_AD] = 0;
    }

    if (SearchParams[CN_SP_AD] == 0) {
        SearchParams[CN_SP_CC] = 0;
        SearchParams[CN_SP_AS] = 0;
    }
    else {
        element = algorithm->FirstChildElement(CNS_TAG_CC);
        if (!element) {
            std::cout << "Warning! No '" << CNS_TAG_CC << "' element found in XML file." << std::endl;
            std::cout << "Value of '" << CNS_TAG_CC << "' was defined to default - false" << std::endl;
            SearchParams[CN_SP_CC] = 0;
        }
        else {
            std::string check;
            stream << element->GetText();
            stream >> check;
            stream.clear();
            stream.str("");
            if (check != "1" && check != "true" && check != "0" && check != "false") {
                std::cout << "Warning! Value of '" << CNS_TAG_CC << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_CC << "' was defined to default - false" << std::endl;
                SearchParams[CN_SP_CC] = 0;
            }
            else {
                if (check == "1" || check == "true")
                    SearchParams[CN_SP_CC] = 1;
                else
                    SearchParams[CN_SP_CC] = 0;
            }
        }
        if (SearchParams[CN_SP_CC] == 0) {
            SearchParams[CN_SP_AS] = 0;
        }
        else {
            element = algorithm->FirstChildElement(CNS_TAG_AS);
            if (!element) {
                std::cout << "Warning! No '" << CNS_TAG_AS << "' element found in XML file." << std::endl;
                std::cout << "Value of '" << CNS_TAG_AS << "' was defined to default - false." << std::endl;
                SearchParams[CN_SP_AS] = 0;
            }
            else {
                std::string check;
                stream << element->GetText();
                stream >> check;
                stream.clear();
                stream.str("");
                if (check != "1" && check != "true" && check != "0" && check != "false") {
                    std::cout << "Warning! Value of '" << CNS_TAG_AS << "' is not correctly specified." << std::endl;
                    std::cout << "Value of '" << CNS_TAG_AS << "' was defined to default - false." << std::endl;
                    SearchParams[CN_SP_AS] = 0;
                }
                else {
                    if (check == "1" || check == "true")
                        SearchParams[CN_SP_AS] = 1;
                    else
                        SearchParams[CN_SP_AS] = 0;
                }
            }
        }
    }

    element = algorithm->FirstChildElement(CNS_TAG_PS);
    if (!element)
    {
        std::cout << "Warning! No '" << CNS_TAG_PS << "' element found in XML file." << std::endl;
        std::cout << "Value of '" << CNS_TAG_PS << "' was defined to default - false" << std::endl;
        SearchParams[CN_SP_PS] = 0;
    }
    else
    {
        SearchParams[CN_SP_PS] = 0;
        if(SearchParams[CN_SP_ST] != CN_SP_ST_TH)
        {
            std::string check;
            stream << element->GetText();
            stream >> check;
            stream.clear();
            stream.str("");

            if (check != "1" && check != "true" && check != "0" && check != "false") {
                std::cout << "Warning! Value of '" << CNS_TAG_PS << "' is not correctly specified." << std::endl;
                std::cout << "Value of '" << CNS_TAG_PS << "' was defined to default - false " << std::endl;
                SearchParams[CN_SP_PS] = 0;
            }
            else if (check == "1" || check == "true")
                SearchParams[CN_SP_PS] = 1;
            else
                SearchParams[CN_SP_PS] = 0;
        }
    }

    return true;
}
