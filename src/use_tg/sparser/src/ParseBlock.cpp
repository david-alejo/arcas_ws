#include "ParseBlock.h"


using std::make_pair;

#include <stdexcept>
#include <sstream>
using std::runtime_error;
using namespace std;

#include "Checker.h"
#include "StringUtil.h"
#include "Parser.h"

ParseBlock::ParseBlock( ParseContext *back, std::string name) {
	this->back = back;
	this->name = name;
	if (back != NULL) {
	  depth = back->getDepth() + 1;
	} else {
	  depth = 0;
	}
	// cout << "< " << name << " >\n";
}

// Recursive copy constructor
ParseBlock::ParseBlock(ParseBlock &p) {
	blockMap.clear();
	propertyMap.clear();
	
	name = p.name;
	back = NULL;
	depth = depth;
	
	// Copy the block map
	BlockMap::iterator bm_it = p.blockMap.begin();
	for ( ; bm_it != p.blockMap.end() ; bm_it ++) {
		Blocks::iterator block_it = bm_it->second->begin();
		const char *cname = bm_it->first;
		blockMap.insert( make_pair( cname, new Blocks ) ); // Insert a new list
		
		for ( ; block_it != bm_it->second->end(); block_it++) {
			ParseBlock *b = new ParseBlock ( **block_it);
			b -> back = this; // Very important!! Change its back attribute to this 
			blockMap[cname]->push_back( b );
		}
		
	}
	
	// Copy the property map
	PropertyMap::iterator pm_it = p.propertyMap.begin();
	for ( ; pm_it != p.propertyMap.end() ; pm_it ++) {
		Properties::iterator property_it = pm_it->second->begin();
		const char *cname = pm_it->first;
		propertyMap.insert( make_pair( cname, new Properties ) ); // Insert a new list
		
		for ( ; property_it != pm_it->second->end(); property_it++) {
			ParseProperty *p = new ParseProperty ( **property_it);
			propertyMap[cname]->push_back( p );
		}
	}
	
	
}

// Destructor
ParseBlock::~ParseBlock() {
	
	// Free the block map
	BlockMap::iterator bm_it = blockMap.begin();
	
	for ( ; bm_it != blockMap.end() ; bm_it ++) {
		if (bm_it->second != NULL) {
			Blocks::iterator block_it = bm_it->second->begin();
		
			for ( ; block_it != bm_it->second->end(); block_it++) {
				delete *block_it;
			}
			bm_it->second ->clear();
			delete bm_it->second;
		}
		
	}
	blockMap.clear();
	
	// Free the Property map
	PropertyMap::iterator pm_it = propertyMap.begin();
	
	for ( ; pm_it != propertyMap.end() ; pm_it ++) {
		if (pm_it->second != NULL) {
			Properties::iterator property_it = pm_it->second->begin();
		
			for ( ; property_it != pm_it->second->end(); property_it++) {
				delete *property_it;
			}
			pm_it->second ->clear();
			delete pm_it->second;
		}
		
	}
	propertyMap.clear();
	
}
	

// Accessors *****************************

ParseProperty& ParseBlock::operator() ( const char * name) {
	Properties *p = propertyMap[ name ];

	if (!p) {
		char *s = new char[255];
		sprintf( s, "ParseBlock::Property %s NOT found in block %s.", name, this->name.c_str() );
		throw new std::runtime_error( s );
	}
	
	return *(*(p->begin()));
}

ParseBlock& ParseBlock::operator[]( const char *name ) {

	Blocks *p = blockMap[ name ];

	if (!p) {
		char *s = new char[255];
		sprintf( s, "ParseBlock::Block %s NOT found in block %s.", name, this->name.c_str() );
		throw new std::runtime_error( s );
	}
	
	return *(*(p->begin()));
}

bool ParseBlock::hasBlock( const char *name ) const {
	return (  blockMap.find(name) != blockMap.end() );
}

bool ParseBlock::hasProperty( const char *name ) const {
	return (  propertyMap.find(name) != propertyMap.end() );

}

ParseBlock::Properties *ParseBlock::getProperties( const char *name ) const {
	ParseBlock::Properties *ret = NULL;
	if ( hasProperty(name)) {
		ret = propertyMap.find(name)->second;
	}
	
	return ret;
}

ParseBlock::Blocks *ParseBlock::getBlocks( const char *name ) const {
	ParseBlock::Blocks *ret = NULL;
	
	if ( hasBlock(name) ) {
		ret = blockMap.find(name)->second;
	}
	
	return ret;
}



// Parse Context Inherited Members implementation *****************************


ParseContext *ParseBlock::ParseBlockOpen( const std::string& name ) {
	ParseBlock *b = new ParseBlock( this, name );
	const char * cname = name.c_str();

	if ( blockMap.count( cname ) == 0 ) {
		blockMap.insert( make_pair( cname, new Blocks ) );
	}

	blockMap[cname]->push_back( b );

	return b;
}

ParseContext *ParseBlock::ParseBlockClose() {
	// cout << "</ " << name << " >\n";
	if (!back) {
		throw new runtime_error("Root level reached parsing: impossible to close ParseBlock");
	}
	return back;
}

ParseContext *ParseBlock::setProperty( const std::string& name, const std::string& value ) {

	const char * cname = name.c_str();

	if ( propertyMap.count( cname ) == 0 ) {
		propertyMap.insert( make_pair( cname, new Properties ) );
	}

	propertyMap[cname]->push_back( new ParseProperty( name, value ) );
	
	return this;
}


ParseContext *ParseBlock::setBlock( const std::string& name, ParseBlock *block ) {

	const char * cname = name.c_str();
	
	block->name = name;

	if ( blockMap.count( cname ) == 0 ) {
		blockMap.insert( make_pair( cname, new Blocks ) );
	}
	
	block->back = this;

	blockMap[cname]->push_back( block );
	
	return this;
}

ParseContext *ParseBlock::comment( const std::string& value ) {
	return this;
}


void ParseBlock::load( const char *filename ) {
	std::string s;
	s = StringUtil::loadFile( filename );
	Parser::apply( s, this );
}

void ParseBlock::checkUsing( Checker *checker ) const {
	checker->apply( this );
}
 


ostream& operator<<( ostream& os, ParseBlock& b) {

	if (b.name != "") { 
	  os << StringUtil::tabulate(b.name, b.depth - 1) << " {\n";
	} else {
	  b.calculateDepthRecursively();
	}

	for ( ParseBlock::PropertyMap::const_iterator i = b.propertyMap.begin(); i != b.propertyMap.end(); i++ ) {
		ParseBlock::Properties& p = *(i->second);
		for ( ParseBlock::Properties::iterator j = p.begin(); j!= p.end(); j++ ) {
			ostringstream aux;
			aux << *(*j);
			os << StringUtil::tabulate(aux.str(), b.depth) << ";\n";
		}
	} 

	for ( ParseBlock::BlockMap::const_iterator i = b.blockMap.begin(); i != b.blockMap.end(); i++ ) {
		ParseBlock::Blocks& p = *(i->second);
		for ( ParseBlock::Blocks::iterator j = p.begin(); j!= p.end(); j++ ) {
			os << *(*j);
		}
	} 


	if (b.name != "") {
	  os << StringUtil::tabulate("}\n", b.depth - 1);
	}
	
	return os;
}

void ParseBlock::calculateDepthRecursively(int initial_depth)
{
  depth = initial_depth;
  
	for ( ParseBlock::BlockMap::const_iterator i = blockMap.begin(); i != blockMap.end(); i++ ) {
		ParseBlock::Blocks& p = *(i->second);
		for ( ParseBlock::Blocks::iterator j = p.begin(); j!= p.end(); j++ ) {
			(*j)->calculateDepthRecursively(initial_depth + 1);
		}
	} 
}


