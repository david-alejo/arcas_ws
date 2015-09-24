

#include<string>
#include<iostream>
#include <vector>
using namespace std;


#include "all.h"


Checker *createMyChecker() {

	Checker *config = new Checker;
	config->addProperty("source", new OneOrMore() );

	Checker *source = new Checker;
	source->addProperty("identifier", new NTimes(1) );
	source->addBlock("config", new NTimes(1) );
	source->addChecker("config", config );

	Checker *capture = new Checker;
	capture->addBlock("source", new NTimes(1) );
	capture->addChecker("source", source );
 
	Checker *checker = new Checker;
	checker->addBlock("capture", new OneOrMore() );
	checker->addChecker("capture", capture );

	return checker;
}

int main() {

	ParseBlock config;
	Checker *checker2 = createMyChecker();
	Checker *checker = new Checker(*checker2);
	
	try {
		config.load( "polo" );
		config.checkUsing( checker );
		
		cout << "Config Content: " << endl;
		cout << config << endl;
		
		cout << "Adding One block: " << endl;
		ParseBlock *additional = new ParseBlock;
		additional->setProperty("test", "10");
		config.setBlock("additional", additional);
		
		cout << "Printing new content: " << endl;
		cout << config << endl;

		// Ejemplo de acceso a los campos:
		// El operador [] accede a los bloques
		// el operador () accede a las propiedades 

		if ( config["capture"]["source"]["format"].hasProperty( "width" ) ) {
			cout << config["capture"]["source"]["format"]("width").as<int>() << "\n";
		}
		
		cout << config["capture"]["source"]["format"]("width").as<const char *>() << "\n";
		
		std::vector<double> vec = config["capture"]("vector").as<vector<double> >();
		
		cout << "Vector size = " << vec.size() << "\tVector = ";
		
		for (unsigned int i = 0; i < vec.size(); i++) {
			cout << vec.at(i) << " ";
		}
		cout << endl;

	} catch (std::runtime_error *e) {
		cout << e->what() << endl;
	}
	
	delete checker;
	delete checker2;
	checker = NULL;
	
	return 0;
}
