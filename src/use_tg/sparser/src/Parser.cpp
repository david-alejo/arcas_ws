
#include "Parser.h"

#include <boost/regex.hpp>

#include <stdexcept>
using std::runtime_error;

#include "StringUtil.h"

ParseBlock *Parser::load( const char *filename ) {

	std::string s;
	s = StringUtil::loadFile( filename );

	ParseBlock *root = new ParseBlock();
	apply( s, root );
	return root;
}


void Parser::apply( string s, ParseContext *cont ) {

	std::string sre;
	boost::regex re;
	boost::match_results<std::string::iterator > matches;


	std::string ParseBlockOpen = "(\\w+)\\s*\\{";
	std::string ParseBlockClose = "(\\})";
	std::string setProperty = "(\\w+)\\s*\\=\\s([^;]*);";
	std::string comment = "\\/\\/([^\\n]*)";
	std::string trim = "(\\s+)";

	std::string::iterator begin = s.begin();
	std::string::iterator end = s.end();
	
	while( begin != end ) {

	
		sre = "(" + ParseBlockOpen + "|" + setProperty + "|" + comment + "|" + ParseBlockClose + "|" + trim +")";

		re = sre;
	
		if (boost::regex_search(  begin, end, matches, re )) {
		
			if ( matches[3].first != matches[3].second ) {
				std::string name( matches[3].first, matches[3].second );
				std::string value( matches[4].first, matches[4].second );
				cont = cont->setProperty( name, value );
			}
			if ( matches[2].first != matches[2].second ) {
				std::string name( matches[2].first, matches[2].second );
				cont = cont->ParseBlockOpen( name );
			}
			if ( matches[5].first != matches[5].second ) {
				std::string value( matches[5].first, matches[5].second );
				cont = cont->comment( value );
			}
			if ( matches[6].first != matches[6].second ) {
				cont = cont->ParseBlockClose();
			}
				begin = matches[0].second;
		} else {
			std::string remaining( begin, end );
			throw new runtime_error("Syntax error: ^" + remaining);
		}
		
	}

}

