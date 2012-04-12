#ifndef READPARAMFILE_H
#define READPARAMFILE_H
//#pragma once

#include<fstream>
#include<string>
#include<vector>

void read_moveTo( std::ifstream * ifile, std::string label );
std::vector< double > read_doubleArray( std::string temp );
void readParameterFile( )

#endif