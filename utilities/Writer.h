#ifndef WRITER_H
#define WRITER_H

	#include <string>
	#include <iostream>
	#include <fstream>
	//for writeActionTable:
	#include<iomanip>
	#include <map>
	#include <vector>
	#include "..\rlcpp\State.h"
	//end

	class Writer
	{
	public:
		Writer(std::string file_name);

		void write(std::string message);
		void write(std::string message, bool append);
		void writeActionTable(const State& cont_state, const std::vector<std::string>& state_keys, std::map<std::pair<std::string,int>, double>& qtable); //, const TileCodingHM* tc_ptr
		void writeActionTable(const State& cont_state, const std::vector<double>& q_values);
	protected:
		std::string m_file;
		double getQOfStateActionPair( std::map<std::pair<std::string,int>, double>& qtable, const std::vector<std::string>& state_keys, int action);
	};

#endif
