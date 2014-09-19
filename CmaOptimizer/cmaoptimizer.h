#ifndef _CMAOPTIMIZER_H_
#define _CMAOPTIMIZER_H_

#include <vector>

class CmaOptimizer
{

public:
	CmaOptimizer()
		: stopBit(0), maxIteration(1000), fitFunc(0),
		readyToRun(false), hasLowerBounds(false),
		hasUpperBounds(false),
		dim(0), lambda(0), running(0)
	{}
	
	// lambda : init # of samples
	// base : 4+(int)(3*log(N))
	void setOptimizer(
		double (*fitFunc)(double const *x, int dim),
		int dim,
		int lambda,
		std::vector<double> &xstart, 
		std::vector<double> &stddev, 
		std::vector<double> &lowerBounds,
		std::vector<double> &upperBounds
		);
	double (*fitFunc)(double const *x, int dim);
	bool isFeasible(double const *x, int dim);
	double boundaryPenalty(double const *x, int dim);
	void run();
	void run_boundary();

	void pause();
	void stop();
	void resume();
	void hehe();

	void printSolution(const double *x, int dim);
	void printSolution();
	void saveSolution(char *filename);
	void loadSolution(char *filename);

	int stopBit; // 0: stop 1: run 2: pause
	int maxIteration;
	int running; // 0: not running 1: running
	bool readyToRun;
	bool hasLowerBounds;
	bool hasUpperBounds;
	int dim;
	int lambda;
	std::vector<double>xstart;
	std::vector<double>stddev;
	std::vector<double>lowerBounds;
	std::vector<double>upperBounds;

	std::vector<double>solution;
};

#endif
