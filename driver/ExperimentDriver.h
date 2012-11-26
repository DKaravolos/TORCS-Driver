#pragma once

#ifndef EXPERIMENT_DRIVER_H_
#define EXPERIMENT_DRIVER_H_

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>

#include "CarState.h"
#include "CarControl.h"
#include "WrapperBaseDriver.h"
#include "RLDriver.h"

//Include drivers that could be used in an experiment
#include "TCDriver.h"
#include "QDriver.h"
#include "CaclaDriver.h"

//To set experiment parameters, the class must know the Experiment class
#include "../rlcpp/Experiment.h"

class ExperimentDriver : public WrapperBaseDriver
{
public:
	ExperimentDriver();
	~ExperimentDriver();

	// SimpleDriver implements a simple and heuristic controller for driving
	virtual CarControl wDrive(CarState cs);

	// Print a shutdown message 
	virtual void onShutdown();
	
	// Print a restart message 
	virtual void onRestart();

	// Initialization of the desired angles for the rangefinders
	virtual void init(float *angles);

private:
	RLDriver* mp_driver;
	int m_driver_type;
	bool m_save_data;
	//std::vector<std::string> m_exploration_type;
	//std::string m_path_of_log;
	int g_curr_experiment;

	//Experiment parameters
	int m_nr_of_experiments;
	std::vector<int> m_steps;
	std::vector<int> m_runs;
	std::vector<double> m_learning_rates;
	std::vector<double> m_gammas;
	std::vector<double> m_taus;
	std::vector<double> m_epsilons;
	std::vector<double> m_sigmas;
	std::vector<std::string> m_dirs;

	//functions
	void selectFirstDriver();
	//void setNewDriver(const int& driver_nr);
	void setNewParameters(const int& driver_nr);
	void readExperimentParameters(const std::string& file);
	void addParameter(const std::string& type, std::stringstream& parameter, std::vector<double>& stored_values);
	void addParameter(const std::string& type, std::stringstream& parameter, std::vector<int>& stored_values);
	void addParameter(const std::string& type, std::stringstream& parameter, std::vector<std::string>& stored_values);
	void autoCompleteParameters();
	void autoCompleteParameter(std::vector<double>& parameter_vector);
	void autoCompleteParameter(std::vector<int>& parameter_vector);
	void autoCompleteParameter(std::vector<std::string>& parameter_vector);
	//void changeLogDir(const std::string& folder);
};


#endif //EXPERIMENT_DRIVER_H_