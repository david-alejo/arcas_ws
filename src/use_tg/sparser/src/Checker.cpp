#include "Checker.h"
#include<stdio.h>
#include <string>
#include <iostream>
using std::make_pair;

// OneOrMore check implementation

// Clone function
Check *OneOrMore::Clone() {
	return new OneOrMore(*this);
}

void OneOrMore::operator()( ParseBlock::Properties *p) {
	if ( !p ) {	throw std::runtime_error( ": should appear one or more times" ); }
}
void OneOrMore::operator()( ParseBlock::Blocks *p ) {
	if ( !p ) {	throw std::runtime_error( ": should appear one or more times" ); }
}


// NTimes check implementation
NTimes::NTimes( int _times ): times(_times) {}

// Copy constructor
NTimes::NTimes( NTimes &n): times(n.times) {}

// Clone function
Check *NTimes::Clone() {
	return new NTimes(*this);
}

void NTimes::operator()( ParseBlock::Properties *p) {
	try {
		if ( !p )  { throw 0; }
		if ( (int)p->size() != times ) { throw 0; }
		

	} catch (...) {
		char *s = new char[255];
		sprintf( s, ": should appear %d times", times );
		throw std::runtime_error( s );
	}
}

void NTimes::operator()( ParseBlock::Blocks *p) {
	try {
		if ( !p )  { throw 0; }
		if ( (int)p->size() != times ) { throw 0; }
		
	} catch (...) {
		char *s = new char[255];
		sprintf( s, ": should appear %d times", times );
		throw std::runtime_error( s );
	}
}

// Checker Implementation ********************

// Copy constructor
Checker::Checker(Checker &c) {
	// Delete the Checker Map info
	CheckerMap::iterator checker_map_it = c.checkerMap.begin();
	
	for ( ; checker_map_it != c.checkerMap.end(); checker_map_it ++) {
		Checker *c = new Checker( *checker_map_it->second);
		
		checkerMap.insert( make_pair( checker_map_it->first, c ));
	}
	
	// Delete the properties Checks
	CheckMap::iterator cm_it = c.propertyChecks.begin();
	for ( ; cm_it != c.propertyChecks.end(); cm_it ++) {
		Check *c = cm_it->second->Clone();
		
		propertyChecks.insert( make_pair( cm_it->first, c ));
	}
	
	// Delete the block Checks
	cm_it = c.blockChecks.begin();
	for ( ; cm_it != c.blockChecks.end(); cm_it ++) {
		Check *c = cm_it->second->Clone();
		
		blockChecks.insert( make_pair( cm_it->first, c ));
	}
	
}

// Destructor
Checker::~Checker() {
	// Delete the Checker Map info
	CheckerMap::iterator checker_map_it = checkerMap.begin();
	
	for ( ; checker_map_it != checkerMap.end(); checker_map_it ++) {
		delete checker_map_it->second;
	}
	checkerMap.clear();
	
	// Delete the properties Checks
	CheckMap::iterator cm_it = propertyChecks.begin();
	for ( ; cm_it != propertyChecks.end(); cm_it ++) {
		delete cm_it->second;
	}
	propertyChecks.clear();
	
	// Delete the block Checks
	cm_it = blockChecks.begin();
	for ( ; cm_it != blockChecks.end(); cm_it ++) {
		delete cm_it->second;
	}
	blockChecks.clear();
	
}

// Beware: The pointer Check * will be freed in the destructor!! Do not free it in the main program
void Checker::addProperty( const char *name, Check *c) {
	propertyChecks.insert( make_pair( name, c ));
}

// Beware: The pointer Check * will be freed in the destructor!! Do not free it
void Checker::addBlock( const char *name, Check *c) {
	blockChecks.insert( make_pair( name, c ));
}

// Beware: The pointer Check * will be freed in the destructor!! Do not free it
void Checker::addChecker( const char *name, Checker * c) {
	checkerMap.insert( make_pair( name, c ));
}

std::string Checker::prependContext( std::runtime_error& e, const char *name ) {
	std::string s = "\\";
	s.append( name );
	s.append( e.what() );
	return s;
}

void Checker::apply( const ParseBlock *block ) {
	

	for (CheckMap::iterator i = propertyChecks.begin(); i != propertyChecks.end(); i++ ) {
		const char *name = i->first;
//		std::cout << "check property " << name << "\n";
		try {
			(*(i->second))( block->getProperties( name ) );

		} catch ( std::runtime_error& e ) {
			throw std::runtime_error( prependContext( e, name ));
		}
	}
	

	for (CheckMap::iterator i = blockChecks.begin(); i != blockChecks.end(); i++ ) {
		const char *name = i->first;
		ParseBlock::Blocks *innerBlocks = block->getBlocks( name );

//s		std::cout << "check block " << name << "\n";
		try {
			(*(i->second))( innerBlocks );

			if ( checkerMap.count( name ) != 0 ) {
				for (	ParseBlock::Blocks::iterator j = innerBlocks->begin(); 
					j != innerBlocks->end();
					j++) {

					checkerMap[name]->apply( *j );
				}
			}
			
		} catch ( std::runtime_error& e ) {
			throw std::runtime_error( prependContext( e, name ));
		}
	}
}



