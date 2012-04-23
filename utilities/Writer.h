#ifndef WRITER_H
#define WRITER_H
#include <string>
#include <iostream>
#include <fstream>

class Writer
{
public:
	Writer(std::string file_name);
	~Writer(void);

	void write(std::string message);
	void write(std::string message, bool append);

protected:
	std::string m_file;
};

#endif
