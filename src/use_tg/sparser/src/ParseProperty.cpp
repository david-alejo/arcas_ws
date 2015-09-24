
#include "ParseProperty.h"

using namespace std;

ParseProperty::ParseProperty( const std::string& name, const std::string& value ) {
		this->name = name;
		this->value = value;
		// cout << "< " << name << " = " << value << " >\n";
	}


ostream& operator<<( ostream& os, const ParseProperty& p ) {
	os << p.name << " = " << p.value;
	return os;
}

