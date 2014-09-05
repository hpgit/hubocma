#ifndef _CMAOPTIMIZER_H_
#define _CMAOPTIMIZER_H_

#include <vector>

class CmaOptimizer
{

public:
	CmaOptimizer(){maxIteration = 3000; fitFunc = 0; readyToRun = false;}
	
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

	void printSolution(const double *x, int dim);
	void printSolution();
	void saveSolution(char *filename);
	void loadSolution(char *filename);

	int maxIteration;
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

#endif _CMAOPTIMIZER_H_
