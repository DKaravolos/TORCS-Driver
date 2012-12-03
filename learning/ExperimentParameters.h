#ifndef EXPERIMENT_PARAMETERS
#define EXPERIMENT_PARAMETERS

struct ExperimentParameters 
{
    int episode;
    int step;
    int result;
    double rewardSum;
	bool endOfEpisode;
	int storePer;
	bool train;
	bool first_time_step;
};

#endif //EXPERIMENT_PARAMETERS
