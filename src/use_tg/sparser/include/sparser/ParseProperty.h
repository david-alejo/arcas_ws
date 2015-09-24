#ifndef PARSEPROPERTY_H
#define PARSEPROPERTY_H

#include <string>

#include <sstream>
#include <stdexcept>

#include <iostream>
#include <vector>
#include <sstream>
				 
#include <algorithm>



struct ParseProperty {
	
	ParseProperty( const std::string& name, const std::string& value );
	std::string name;
	std::string value;

	template<typename T> T as() const;
};

std::ostream& operator<<( std::ostream& os, const ParseProperty& p );

class BadConversion : public std::runtime_error {
public:
	BadConversion(const std::string& s): std::runtime_error(s) { }
};



template<typename T> 
inline T ParseProperty::as() const {
	T x;
	std::istringstream i( value );
	char c;
	if (!(i >> x) || (i.get(c)))
	throw BadConversion( value );
	return x;
}

// Specialization for chars
template<> 
inline const char * ParseProperty::as<const char *>() const {
	return value.c_str();
}

// Specialization for bools
template<> 
inline bool ParseProperty::as<bool>() const {
	bool return_value = false;
	std::string copy(value);
	
	std::transform(value.begin(), value.end(), copy.begin(), ::tolower);
	
	if (copy == "true") {
		return_value = true;
	} else if (copy == "false") {
		return_value = false;
	}
	
	return return_value;
}

// Specialization for vector of doubles
template<> 
inline std::vector<double> ParseProperty::as<std::vector<double> >() const {
	std::istringstream iss(value);
	std::vector<double> ret;
	
	try {
		while ( !iss.eof()) {
			double d;
			iss >> d;
			ret.push_back(d);
		}
		
	} catch (std::exception) {
		std::cerr << "Error in ParseProperty::as<vector<double> >\n";
	}
	
	return ret;
}

template<typename T>
inline void convert(const std::string& s, T& x, bool failIfLeftoverChars = true) {
   std::istringstream i(s);
   char c;
   if (!(i >> x) || (failIfLeftoverChars && i.get(c)))
     throw BadConversion(s);
 }



#endif
