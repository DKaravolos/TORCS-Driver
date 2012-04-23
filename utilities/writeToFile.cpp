#include "writeToFile.h"

using namespace std;

void writeToFile(string message)
{
	cout << "wrapper 1: "; 
	writeToFile(message, true);
}

void writeToFile(string message, bool append)
{
	ofstream outputfile;
	cout << "general writing " << message << " to general_output" << endl;
	if(append) 
		outputfile.open("general_output.txt", ios::app);
	else 
		outputfile.open("general_output.txt");

	if( outputfile.is_open()) 
	{
		outputfile << message << endl;
		outputfile.close();
	}
	else 
		cout << "Can't open file 'general_output.txt'" << endl;

}

/*
void writeToFile(string message, string file_name)
{
	cout << "wrapper 2: "; 
	writeToFile(message, file_name, false);
	
}
*/

void writeToFile(string message, string file_name, bool append)
{
	ofstream special_output;
	cout << " special writing " << message << "to " << file_name << endl;
	if(append) 
		special_output.open(file_name, ios::app);
	else 
		special_output.open(file_name, ios::trunc);

	if( special_output.is_open()) 
	{
		special_output << message << endl;
		special_output.close();
	}
	else 
		cout << "Can't open file '" << file_name << "'\n";
}
