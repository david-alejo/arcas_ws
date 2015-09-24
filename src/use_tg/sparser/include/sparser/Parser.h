
#ifndef PARSER_H
#define PARSER_H

#include <string>
using std::string;

#include "ParseContext.h"
#include "ParseBlock.h"


//void parse( string s, ParseContext *cont );

class Parser {
public:
	static ParseBlock *load( const char *filename );
	static void apply( string s, ParseContext *cont );
};


#endif

