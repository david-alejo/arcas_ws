#ifndef BLOCKCHECKER_H
#define BLOCKCHECKER_H


#include <map>
using std::map;
#include "mapUtil.h"

#include <stdexcept>

#include "ParseBlock.h"



struct Check {
	virtual void operator()( ParseBlock::Properties * ) = 0;
	virtual void operator()( ParseBlock::Blocks * ) = 0;
	virtual Check *Clone() = 0;
};


struct OneOrMore : public Check {
	virtual void operator()( ParseBlock::Properties *p);
	virtual void operator()( ParseBlock::Blocks *p );
	
	//! Implements the cloning method
	virtual Check* Clone();
	//! Default Constructor
	OneOrMore() {}
	//! Implements the copy contructor
	OneOrMore(OneOrMore &o) {}
};

struct NTimes : public Check {
	//! Contructor with the number of times
	NTimes( int times );
	//! Copy constructor
	NTimes( NTimes &n);

	virtual void operator()( ParseBlock::Properties *p);
	virtual void operator()( ParseBlock::Blocks *p );
	
	//! Implements the cloning function
	virtual Check* Clone();
	
	int times;
};


class Checker {

public:
	//! Default constructor
	Checker() {}
	
	//! Copy constructor
	Checker(Checker &c);
	
	//! Destructor
	~Checker();
	
	void addProperty( const char *name, Check *);
	void addBlock( const char *name, Check *);
	void addChecker( const char *name, Checker *);
		
	void apply( const ParseBlock *block );

protected:
	std::string prependContext( std::runtime_error& e, const char *name );

	typedef map< const char *, Check *, strCmp> CheckMap;
	typedef map< const char *, Checker *, strCmp> CheckerMap;

	CheckMap blockChecks;
	CheckMap propertyChecks;
	CheckerMap checkerMap;
};	


#endif
