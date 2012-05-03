#include <iostream>
#include <string>

#include "rlcpp\Qlearning.h"
#include "rlcpp\cNeuralNetwork.h"
#include "learning\TorcsWorld.h"

int main()
{
	TorcsWorld* world = new TorcsWorld();
	Qlearning* algorithm = new Qlearning("TorcsWorldCfg", world);
	cout <<"Writing QNN to file ...\n";
	algorithm->writeQNN("test_QNN");
	cout << "Done.\n";

	delete algorithm;

	cout << "Creating new Qlearning with NN files\n";
	Qlearning* algorithm2 = new Qlearning("TorcsWorldCfg", world, "test_QNN");
	cout << "Done.\n";

	delete algorithm2;
	delete world;

	char end;
	cin >> end;
	return 0;
}

