#ifndef STRINGUTIL_H
#define STRINGUTIL_H

#include<string>
#include<iostream>
#include<fstream>
#include <sstream>
#include <stdexcept>

class StringUtil {
public:
	static std::string loadFile( std::string filename );
	static std::string tabulate(const std::string& st, int n_tabs);
};


#endif
