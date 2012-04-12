#pragma once
#include<string>
#include<vector>

struct ExperimentParameters 
{
    int nExperiments, nAlgorithms;
    int nSteps, nEpisodes, nMaxStepsPerEpisode, nResults ;
    int nTrainSteps, nTrainEpisodes, nMaxStepsPerTrainEpisode, nTrainResults, trainStorePer ;
    int nTestSteps, nTestEpisodes, nMaxStepsPerTestEpisode, nTestResults, testStorePer ;
    int stateDimension, actionDimension ;
    bool discreteStates, discreteActions, endOfEpisode ;
    bool storePerStep, storePerEpisode ;
    bool boltzmann, egreedy, gaussian ;
    int nLearningRates ;
        
    const char * parameterFile ;
    std::string parameterPath ;
    std::string algorithmName ;
    std::vector< std::string > algorithms ;
    std::vector< double > taus ;
    std::vector< double > epsilons ;
    std::vector< double > sigmas ;
    std::string learningRateDecreaseType ;
    double * learningRate ;
    std::vector< std::vector < double > >  learningRates ;
    std::vector< double >  gammas ;
    double tau, epsilon, sigma, gamma ;
        
    bool train ;
};