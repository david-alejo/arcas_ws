#ifndef PARSEBLOCK_H
#define PARSEBLOCK_H

#include <iostream>
using std::ostream;

#include <list>
using std::list;

#include "mapUtil.h"
#include <map>
using std::map;


#include "ParseContext.h"
#include "ParseProperty.h"


class Checker;


class ParseBlock: public ParseContext {

	friend ostream& operator<<( ostream& os, ParseBlock& b);
public:
	typedef list<ParseBlock *> Blocks;
	typedef list<ParseProperty *> Properties;


	ParseBlock( ParseContext *back = NULL, const std::string name = std::string(""));
	
	//! \brief Copy constructor
	ParseBlock(ParseBlock &p);
	
	~ParseBlock();

	// Accessors

	/// \brief Returns the first occurence of the property, or throws std::runtime_error if none
	ParseProperty& operator() ( const char *);

	/// \brief Returns the first occurence of the block, or throws std::runtime_error if none
	ParseBlock& operator[]( const char * );

	/// \brief Returns the list of properties with a given name or NULL
	Properties *getProperties( const char * ) const;

	/// \brief Returns the list of blocks with a given name or NULL
	Blocks *getBlocks( const char * ) const;

	/// \brief Returns true if the blocks exists
	bool hasBlock( const char *name ) const;

	/// \brief Returns true if the propertie exists
	bool hasProperty( const char *name ) const;
	
	virtual ParseContext *setProperty( const std::string& name, const std::string& value );
	
	virtual ParseContext *setBlock( const std::string& name, ParseBlock* block );

	void load( const char *filename );
	void checkUsing( Checker *checker ) const;

protected:
	typedef map<const char *, Properties *, strCmp> PropertyMap;
	typedef map<const char *, Blocks *, strCmp> BlockMap;

	virtual ParseContext *ParseBlockOpen( const std::string& name );
	virtual ParseContext *ParseBlockClose();

	virtual ParseContext *comment( const std::string& value );

private:
	std::string name;
	ParseContext *back;

	PropertyMap propertyMap;
	BlockMap blockMap;
	
	void calculateDepthRecursively(int initial_depth = 0);
	
// 	ParseBlock &operator = (const ParseBlock &b);

};

ostream& operator<<( ostream& os, const ParseBlock& b);

#endif
