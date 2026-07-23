//
// Created by xiang on 24-4-24.
//

#include "string_tools.h"
#include <algorithm>
#include <vector>

#include <glog/logging.h>

namespace lightning::miao {

std::string trim(const std::string& s) {
    if (s.length() == 0) {
        return s;
    }
    std::string::size_type b = s.find_first_not_of(" \t\n");
    std::string::size_type e = s.find_last_not_of(" \t\n");
    if (b == std::string::npos) {
        return "";
    }
    return std::string(s, b, e - b + 1);
}

std::string trimLeft(const std::string& s) {
    if (s.length() == 0) {
        return s;
    }

    std::string::size_type b = s.find_first_not_of(" \t\n");
    std::string::size_type e = s.length() - 1;
    if (b == std::string::npos) {
        return "";
    }

    return std::string(s, b, e - b + 1);
}

std::string trimRight(const std::string& s) {
    if (s.length() == 0) {
        return s;
    }

    std::string::size_type b = 0;
    std::string::size_type e = s.find_last_not_of(" \t\n");
    if (e == std::string::npos) {
        return "";
    }

    return std::string(s, b, e - b + 1);
}

std::string strToLower(const std::string& s) {
    std::string ret;
    ret.reserve(s.size());
    std::transform(s.begin(), s.end(), back_inserter(ret), [](unsigned char c) { return std::tolower(c); });
    return ret;
}

std::string strToUpper(const std::string& s) {
    std::string ret;
    ret.reserve(s.size());
    std::transform(s.begin(), s.end(), back_inserter(ret), [](unsigned char c) { return std::toupper(c); });
    return ret;
}

std::string strExpandFilename(const std::string& filename) {
#if (defined(UNIX) || defined(CYGWIN)) && !defined(ANDROID)
    std::string result = filename;
    wordexp_t p;

    wordexp(filename.c_str(), &p, 0);
    if (p.we_wordc > 0) {
        result = p.we_wordv[0];
    }
    wordfree(&p);
    return result;
#else
    (void)filename;
    LOG(WARNING) << "not implemented";
    return std::string();
#endif
}

std::vector<std::string> strSplit(const std::string& str, const std::string& delimiters) {
    std::vector<std::string> tokens;
    if (str.empty()) {
        return tokens;
    }

    std::string::size_type lastPos = 0;
    std::string::size_type pos = 0;

    do {
        pos = str.find_first_of(delimiters, lastPos);
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        lastPos = pos + 1;
    } while (std::string::npos != pos);

    return tokens;
}

bool strStartsWith(const std::string& s, const std::string& start) {
    if (s.size() < start.size()) {
        return false;
    }
    return equal(start.begin(), start.end(), s.begin());
}

bool strEndsWith(const std::string& s, const std::string& end) {
    if (s.size() < end.size()) {
        return false;
    }

    return equal(end.rbegin(), end.rend(), s.rbegin());
}

int readLine(std::istream& is, std::stringstream& currentLine) {
    if (is.eof()) {
        return -1;
    }

    currentLine.str("");
    currentLine.clear();
    is.get(*currentLine.rdbuf());
    if (is.fail()) {
        // fail is set on empty lines
        is.clear();
    }
    skipLine(is);  // read \n not read by get()
    if (currentLine.str().empty() && is.eof()) {
        return -1;
    }
    return static_cast<int>(currentLine.str().size());
}

void skipLine(std::istream& is) {
    char c = ' ';
    while (c != '\n' && is.good() && !is.eof()) {
        is.get(c);
    }
}
}  // namespace lightning::miao