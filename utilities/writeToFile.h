#ifndef WRITE_TO_FILE_H
#define WRITE_TO_FILE_H
#include <iostream>
#include <fstream>
#include <string>

void writeToFile(std::string message);
void writeToFile(std::string message, bool append);

//void writeToFile(std::string message, std::string file_name); //overloading fails, string is interpreted as bool
void writeToFile(std::string message, std::string file_name, bool append);


#endif //WRITE_TO_FILE_H