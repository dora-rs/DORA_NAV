//
// Created by xiang on 24-4-24.
//

#ifndef MIAO_STRING_TOOLS_H
#define MIAO_STRING_TOOLS_H

#include <sstream>
#include <string>
#include <vector>

namespace lightning::miao {

/** @addtogroup utils **/
// @{

/** \file stringTools.h
 * \brief utility functions for handling strings
 */

/**
 * remove whitespaces from the start/end of a string
 */
std::string trim(const std::string& s);

/**
 * remove whitespaces from the left side of the string
 */
std::string trimLeft(const std::string& s);

/**
 * remove whitespaced from the right side of the string
 */
std::string trimRight(const std::string& s);

/**
 * convert the string to lower case
 */
std::string strToLower(const std::string& s);

/**
 * convert a string to upper case
 */
std::string strToUpper(const std::string& s);

/**
 * convert a string into an other type.
 */
template <typename T>
bool convertString(const std::string& s, T& x, bool failIfLeftoverChars = true) {
    std::istringstream i(s);
    char c;
    if (!(i >> x) || (failIfLeftoverChars && i.get(c))) return false;
    return true;
}

/**
 * convert a string into an other type.
 * Return the converted value. Throw error if parsing is wrong.
 */
template <typename T>
T stringToType(const std::string& s, bool failIfLeftoverChars = true) {
    T x;
    convertString(s, x, failIfLeftoverChars);
    return x;
}

/**
 * return true, if str starts with substr
 */
bool strStartsWith(const std::string& str, const std::string& substr);

/**
 * return true, if str ends with substr
 */
bool strEndsWith(const std::string& str, const std::string& substr);

/**
 * expand the given filename like a posix shell, e.g., ~ $CARMEN_HOME and other
 * will get expanded. Also command substitution, e.g. `pwd` will give the
 * current directory.
 */
std::string strExpandFilename(const std::string& filename);

/**
 * split a string into token based on the characters given in delim
 */
std::vector<std::string> strSplit(const std::string& s, const std::string& delim);

/**
 * read a line from is into currentLine.
 * @return the number of characters read into currentLine (excluding newline),
 * -1 on eof()
 */
int readLine(std::istream& is, std::stringstream& currentLine);

/**
 * read from string until the end of a line is reached.
 */
void skipLine(std::istream& is);

}  // namespace lightning::miao
#endif  // MIAO_STRING_TOOLS_H
