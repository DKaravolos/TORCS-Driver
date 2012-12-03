#include "Writer.h"

using namespace std;

Writer::Writer(string file_name)
{
	m_file = file_name;
	ofstream out (m_file.c_str(), ios::trunc);

	if( out.is_open()) 
	{
		out << "" << endl;
		out.close();
	}
	else 
		cerr << "Can't open file '" << m_file << "'\n";
}


Writer::~Writer(void)
{
}

void Writer::write(string message)
{
	//cout << "writing without bool: " << message << endl ;

	ofstream out (m_file.c_str(), ios::app);

	if( out.is_open()) 
	{
		out << message << endl;
		out.close();
	}
	else 
		cout << "Can't open file '" << m_file << "'\n";
}

void Writer::write(string message, bool append)
{
	//cout << "writing with bool: " << message << endl ;
	ofstream out;

	if(append)
		out.open(m_file.c_str(), ios::app);
	else
		out.open(m_file.c_str(),ios::trunc);

	if( out.is_open()) 
	{
		out << message << endl;
		out.close();
	}
	else 
		cout << "Can't open file '" << m_file << "'\n";
}
