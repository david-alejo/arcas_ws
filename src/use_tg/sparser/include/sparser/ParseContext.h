
#ifndef PARSECONTEXT_H
#define PARSECONTEXT_H

class Parser;

class ParseContext {
	friend class Parser;
	
	public:
		int getDepth() const {return depth;};
	  
		virtual ~ParseContext() {};
 
protected:
	virtual ParseContext *ParseBlockOpen( const std::string& name ) = 0;
	virtual ParseContext *ParseBlockClose() = 0;
	virtual ParseContext *setProperty( const std::string& name, const std::string& value ) = 0;
	virtual ParseContext *comment( const std::string& value ) = 0;
	
	int depth;
	
};


#endif


