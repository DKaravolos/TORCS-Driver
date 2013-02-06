#include "ExperimentDriver.h"

using namespace std;

ExperimentDriver::ExperimentDriver()
{
	//Get parameters from a file
	cout << "Which file contains the parameters for the experiment?\n";
	string file;
	cin >> file;
	readExperimentParameters(file);
	g_curr_experiment = 0;
	
	cout << "Would you like to save the data structure? (y/n)\n";
	char save;
	cin >> save;
	m_save_data = (save == 'y');

	//Ask user which driver to test
	selectFirstDriver();
}

ExperimentDriver::~ExperimentDriver()
{
	delete mp_driver;
}

CarControl ExperimentDriver::wDrive(CarState cs)
{
	return mp_driver->wDrive(cs);
}

void ExperimentDriver::init(float* angles)
{
	cout << "CALLING THE INIT FUNCTION!\n";
	mp_driver->init(angles);
	if(mp_driver->getLearningStep() <=0)
	{
		setNewParameters(g_curr_experiment);

		RLInterface* curr_interface = mp_driver->getInterface();
		cout << "Changed driver. There should be new parameters now.\n";
		//Check parameter settings
		cout << "Epsilon: " << curr_interface->getExperiment()->getEpsilon() << endl;
		cout << "Gamma: " << curr_interface->getExperiment()->getGamma() << endl;
		cout << "Learning Rate: " << curr_interface->getExperiment()->getLearningRate() << endl;
		cout << "Tau: " << curr_interface->getExperiment()->getTau() << endl;
	}

}

void ExperimentDriver::onShutdown()
{
	mp_driver->onShutdown();
}

void ExperimentDriver::onRestart()
{
	mp_driver->onRestart();
	cout << "Steps done: " << mp_driver->getLearningStep() + 1 << endl;
	cout << "Steps required: " << m_steps[g_curr_experiment]* m_runs[g_curr_experiment] << endl;

	if(mp_driver->getLearningStep() + 1 == m_steps[g_curr_experiment]* m_runs[g_curr_experiment])
	{
		++g_curr_experiment;

		if(g_curr_experiment == m_nr_of_experiments)
		{
			cout << m_nr_of_experiments << " experiments done! Quitting.\n";
			#ifdef WIN32
				char q;
				cin >> q;
				exit(0); //I don't know any other way to stop the game.
			#endif
		} else {
			setNewDriver(g_curr_experiment);
		}
	}
}

void ExperimentDriver::selectFirstDriver()
{
	cout << "Which driver would you like to use for your experiment?\n";
	cout << "0 = TCDriver, 1 = CaclaDriver \n"; //, 2 = QDriver\n";
	cin >> m_driver_type;
	switch(m_driver_type)
	{
		case 0:
			mp_driver = new TCDriver(m_dirs[0],m_steps[0],m_runs[0],m_save_data);
			break;
		case 1:
			mp_driver = new CaclaDriver(m_dirs[0],m_steps[0],m_runs[0],m_save_data);
			cout << "NOT IMPLEMENTED!\n";
			break;
		case 2:
			//mp_driver = new QDriver();
			cout << "NOT IMPLEMENTED!\n";
			break;
		default:
			cout << "\nDriver not implemented. Try again.\n";
			cin.clear();
			selectFirstDriver();
	}
}

void ExperimentDriver::readExperimentParameters(const string& parameter_file)
{
	//Let's open the file
	string line;
	ifstream is;
	is.open(parameter_file);
	if(is.is_open())
	{
		try
		{
			while(!is.eof()) //We could place the getline in the while-condition, but an eof-check seems nicer
			{
				//Get line from file
				getline(is, line); 

				//Temporary variables
				stringstream parameter(line);
				string parameter_type;

				//Classify the line by the first item
				parameter >> parameter_type;
				if(parameter_type.compare("steps") == 0)
				{
					addParameter(parameter_type, parameter, m_steps);
				}
				else if (parameter_type.compare("runs") == 0)
				{
					addParameter(parameter_type, parameter, m_runs);
				}
				else if (parameter_type.compare("experiments") == 0)
				{
					parameter >> m_nr_of_experiments; //sometimes you want to run the same settings several times
				}
				else if (parameter_type.compare("learningrate") == 0)
				{
					addParameter(parameter_type, parameter, m_learning_rates);
				}
				else if (parameter_type.compare("gamma") == 0)
				{
					addParameter(parameter_type, parameter, m_gammas);
				}
				else if (parameter_type.compare("directory") == 0)
				{
					addParameter(parameter_type, parameter, m_dirs);
				}
				else if (parameter_type.compare("exploration") == 0)
				{
					string exp_type;
					parameter >> exp_type;
					if (exp_type.compare("boltzmann") == 0)
					{
						cout << "Boltzmann: ";
						addParameter(parameter_type, parameter, m_taus);
					} else if (exp_type.compare("egreedy") == 0)
					{
						cout << "E-greedy: ";
							addParameter(parameter_type, parameter, m_epsilons);
					} else if (exp_type.compare("gaussian") == 0)
					{
						cout << "Gaussian: ";
							addParameter(parameter_type, parameter, m_sigmas);
					} else
						cerr << "Uh o. Unknown exploration type encountered.\n";
				}
				else if (parameter_type.compare("eta") == 0)
				{
					addParameter(parameter_type, parameter, m_etas);
				}
			}

			cout << "\nYou have defined "<< m_nr_of_experiments << " experiments. Is that correct? (y/n) \n";
			char ans;
			cin >> ans;
			if(ans == 'n')
				exit(-100);
						
		} catch (iostream::failure e)
		{
			cerr << "Failure during reading experiment parameters from file.\n";
			cerr << e.what();
		}
		is.close();

	} else {
		cerr <<  "Can't open experiment parameter file. Quitting\n";
		#ifdef WIN32
			char end_program;
			cin >> end_program;
		#endif
		exit(-21) ;
	}

	autoCompleteParameters();
}

void ExperimentDriver::addParameter(const string& parameter_type, stringstream& parameter, vector<double>& parameter_vector)
{
	double input_value;
	cout << "Reading the '" << parameter_type << "' parameter:\t\t";
	//Add the rest of the line to the parameter vector
	while(parameter >> input_value){
		parameter_vector.push_back(input_value);
		cout << input_value << "  ";
	}
	cout << endl;
}

void ExperimentDriver::addParameter(const string& parameter_type, stringstream& parameter, vector<int>& parameter_vector)
{
	int input_value;
	cout << "Reading the '" << parameter_type << "' parameter:\t\t";
	//Add the rest of the line to the parameter vector
	while(parameter >> input_value){
		parameter_vector.push_back(input_value);
		cout << input_value << "  ";
	}
	cout << endl;
}

void ExperimentDriver::addParameter(const string& parameter_type, stringstream& parameter, vector<string>& parameter_vector)
{
	string input_value;
	cout << "Reading the '" << parameter_type << "' parameter:\t\t";
	//Add the rest of the line to the parameter vector
	while(parameter >> input_value){
		parameter_vector.push_back(input_value);
		cout << input_value << "  ";
	}
	cout << endl;
}
//This function allows us to use element-access of vectors
void ExperimentDriver::autoCompleteParameters()
{
	unsigned int l_experiments = unsigned(m_nr_of_experiments); //to avoid warnings of compiler
	if( m_steps.size() < l_experiments)
		autoCompleteParameter(m_steps);

	if( m_runs.size() < l_experiments)
		autoCompleteParameter(m_runs);

	if( m_learning_rates.size() < l_experiments)
		autoCompleteParameter(m_learning_rates);

	if( m_gammas.size() < l_experiments)
		autoCompleteParameter(m_gammas);
	
	if( m_dirs.size() < l_experiments)
		autoCompleteParameter(m_dirs);

	//It is not necessary to define at least one tau and one epsilon.
	//Either one will do, so the other vector can be empty.
	//Note:Added gaussian exploration
	if (m_taus.size() > 0 && m_taus.size() < l_experiments)
		autoCompleteParameter(m_taus);
	if (m_epsilons.size() > 0 && m_epsilons.size() < l_experiments)
		autoCompleteParameter(m_epsilons);
	if (m_sigmas.size() > 0 && m_sigmas.size() < l_experiments)
		autoCompleteParameter(m_sigmas);
}

//Fills the supplied parameter vector with it's last element until it has a size of nr_of_experiments.
//This includes downsizing and throwing away the last elements if the vector is larger than nr_of_experiments.
void ExperimentDriver::autoCompleteParameter(vector<double>& parameter_vector)
{
	double last_element = parameter_vector[parameter_vector.size()-1];
	parameter_vector.resize(m_nr_of_experiments,last_element);
}

//Fills the supplied parameter vector with it's last element until it has a size of nr_of_experiments.
//This includes downsizing and throwing away the last elements if the vector is larger than nr_of_experiments.
void ExperimentDriver::autoCompleteParameter(vector<int>& parameter_vector)
{
	int last_element = parameter_vector[parameter_vector.size()-1];
	parameter_vector.resize(m_nr_of_experiments,last_element);
}

//Fills the supplied parameter vector with it's last element until it has a size of nr_of_experiments.
//This includes downsizing and throwing away the last elements if the vector is larger than nr_of_experiments.
void ExperimentDriver::autoCompleteParameter(vector<string>& parameter_vector)
{
	string last_element = parameter_vector[parameter_vector.size()-1];
	parameter_vector.resize(m_nr_of_experiments,last_element);
}

/*void ExperimentDriver::changeLogDir(const string& dir)
{
 //logs are changed in constructor
}*/

void ExperimentDriver::setNewDriver(const int& driver_nr)
{
	//Create a new driver
	delete mp_driver;
	switch(m_driver_type)
	{
		case 0:
			mp_driver = new TCDriver(m_dirs[driver_nr],m_steps[driver_nr],m_runs[driver_nr],m_save_data);
			break;
		case 1:
			mp_driver = new CaclaDriver(m_dirs[driver_nr],m_steps[driver_nr],m_runs[driver_nr],m_save_data);
			//cout << "NOT IMPLEMENTED!\n";
			break;
		case 2:
			//mp_driver = new CaclaDriver();
			cout << "NOT IMPLEMENTED!\n";
			break;
	}
}

void ExperimentDriver::setNewParameters(const int& driver_nr)
{
	//Get acces to experiment parameters through the learning interface
	RLInterface* lp_interface = mp_driver->getInterface();
	Experiment* lp_experiment = lp_interface->getExperiment();
	
	lp_experiment->setGamma(m_gammas[driver_nr]);
	lp_experiment->setLearningRate(m_learning_rates[driver_nr]);

	//Hard-coded option for TileCoding driver. It is too much of a fuss to incorporate eta in all drivers. 
	//because other drivers will not be implemented anymore.
	if(m_driver_type == 0){
		lp_interface->setEta(m_etas[driver_nr]);
	}
	if ((m_epsilons.size() > 0 && m_taus.size() > 0)   ||
		(m_epsilons.size() > 0 && m_sigmas.size() > 0) ||
		(m_sigmas.size() > 0 && m_taus.size() > 0))
	{
		cerr << "ERROR: MORE THAN ONE WAY OF EXPLORATION DEFINED! I DON'T KNOW WHAT TO CHOOSE!!!\n";
		exit(-1);
	} else if(m_epsilons.size() > 0)
	{
		//set variable
		lp_experiment->setEpsilon(m_epsilons[driver_nr]);

		//make sure the right exploration type is used
		lp_experiment->setBoltzmann(false);
		lp_experiment->setEGreedy(true);
		lp_experiment->setGaussian(false);

		////delete the used item
		//vector<double>::iterator it = m_epsilons.begin() + driver_nr;
		//m_epsilons.erase(it);
	} else if(m_taus.size() > 0)
	{
		//set variable value
		lp_experiment->setTau(m_taus[driver_nr]);

		//Make sure the right exploration type is used
		lp_experiment->setBoltzmann(true);
		lp_experiment->setEGreedy(false);
		lp_experiment->setGaussian(false);

		////delete the used item
		//vector<double>::iterator it = m_taus.begin() + driver_nr;
		//m_taus.erase(it);
	}else if(m_gammas.size() > 0)
	{
		//set variable value
		lp_experiment->setSigma(m_gammas[driver_nr]);

		//Make sure the right exploration type is used
		lp_experiment->setBoltzmann(false);
		lp_experiment->setEGreedy(false);
		lp_experiment->setGaussian(true);

		////delete the used item
		//vector<double>::iterator it = m_gammas.begin() + driver_nr;
		//m_gammas.erase(it);
	} else 
	{
		cerr << "ERROR: Can't find a tau or epsilon. Using last known variable.\n" << endl;
	}
}