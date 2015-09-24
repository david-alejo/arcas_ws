
#ifndef MAPUTIL_H
#define MAPUTIL_H

#include <strings.h>

struct strCmp {
	bool operator()( const char* s1, const char* s2 ) const {
	return strcasecmp( s1, s2 ) < 0;
	}
};

#endif
