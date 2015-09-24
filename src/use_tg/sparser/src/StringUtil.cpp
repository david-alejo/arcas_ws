
#include "StringUtil.h"

#include <stdexcept>
#include <sstream>

using namespace std;

std::string StringUtil::loadFile( std::string filename ) {

	std::ifstream is( filename.c_str() );
	if( is.bad() || !is.is_open() ) {
		try {	is.close(); } catch(...) {}

		throw std::runtime_error("impossible to read the configuration file");
	}

	std::string s;
	s.reserve(is.rdbuf()->in_avail());

	char c;
	while( is.get(c) ) {
		if ( s.capacity() == s.size() ) {
			s.reserve(s.capacity() * 3);
		}
		s.append(1, c);
	}

	is.close();
	return s;
}

string StringUtil::tabulate(const std::string& st, int n_tabs)
{
  ostringstream os;
  
  for (int i = 0; i < n_tabs;i ++) {
    os << "\t";
  }
  
  os << st;
  
  return os.str();
}

