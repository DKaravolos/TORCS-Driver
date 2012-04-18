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
};

#endif EXPERIMENT_PARAMETERS