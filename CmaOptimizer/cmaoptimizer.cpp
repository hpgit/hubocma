#include "CmaOptimizer.h"
#include <cma/cmaes_interface.h>
#include <cma/boundary_transformation.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <ctime>
#include <cfloat>
#include <cstdlib>
#include <cstring>

bool CmaOptimizer::isFeasible(double const *x, int dim)
{
	if(hasLowerBounds)
	{
		for(int i = 0; i < dim; i++)
		{
			if(x[i] < lowerBounds[i])  
				return false;
		}
	}
	if(hasUpperBounds)
	{
		for(int i = 0; i < dim; i++)
		{
			if(x[i] > upperBounds[i])  
				return false;
		}
	}
	return true;
}

double CmaOptimizer::boundaryPenalty(double const *x, int dim)
{
	double penalty = 0;
	if(hasLowerBounds)
	{
		for(int i = 0; i < dim; i++)
		{
			if(x[i] < lowerBounds[i])  
				penalty += lowerBounds[i] - x[i];
		}
	}
	if(hasUpperBounds)
	{
		for(int i = 0; i < dim; i++)
		{
			if(x[i] > upperBounds[i])  
				penalty += x[i] - upperBounds[i];
		}
	}	

	return 1000000*(penalty * penalty);
	//return std::exp(penalty);
}

void CmaOptimizer::setOptimizer(
	double (*fitFunc)(double const *x, int dim),
	int dim,
	int lambda,
	std::vector<double> &xstart, 
	std::vector<double> &stddev, 
	std::vector<double> &lowerBounds,
	std::vector<double> &upperBounds
	)
{
	this->solution.clear();
	this->xstart.clear();
	this->stddev.clear();
	this->lowerBounds.clear();
	this->upperBounds.clear();
	this->fitFunc = fitFunc;
	this->dim = dim;
	this->lambda = lambda;
	if(xstart.size()>0)this->xstart.assign(xstart.begin(), xstart.end());
	if(stddev.size()>0)this->stddev.assign(stddev.begin(), stddev.end());
	if(lowerBounds.size()>0)
	{
		this->lowerBounds.assign(lowerBounds.begin(), lowerBounds.end());
		hasLowerBounds = true;
	}
	if(upperBounds.size()>0)
	{
		this->upperBounds.assign(upperBounds.begin(), upperBounds.end());
		hasUpperBounds = true;
	}
	if(dim > 0)
		readyToRun = true;
}

void CmaOptimizer::run()
{
	if (readyToRun == false)
	{
		std::cout << "CMA is not ready. Please set this CmaOptimizer." << std::endl;
		return;
	}
	cmaes_t evo; /* an CMA-ES type struct or "object" */
	evo.version = NULL;
	double *arFunvals, *xfinal, *const*pop;
	unsigned long dimension;
	int i; 
	std::ofstream fout;
	fout.open("cmaResults.txt", std::ios::app);
	char buf[60];
	time_t tt;
	time(&tt);
	strcpy(buf, ctime(&tt));
	fout << buf << std::endl;

	/* Initialize everything into the struct evo, 0 means default */
	//arFunvals = cmaes_init(&evo, 0, NULL, NULL, 0, 0, "cmaes_initials.par");
	
	if(xstart.size() > 0 && stddev.size() > 0)
		arFunvals = cmaes_init(&evo, dim, xstart.data(), stddev.data(), 0, lambda, "non");
	else if(xstart.size() > 0 && stddev.size() == 0)
		arFunvals = cmaes_init(&evo, dim, xstart.data(), NULL, 0, lambda, "non");
	else if(xstart.size() == 0 && stddev.size() > 0)
		arFunvals = cmaes_init(&evo, dim, NULL, stddev.data(), 0, lambda, "non");
	else 
		arFunvals = cmaes_init(&evo, dim, NULL, NULL, 0, lambda, "non");

	dimension = (unsigned long)cmaes_Get(&evo, "dimension");
	
	printf("New Cma Starts\n");
	//cmaes_ReadSignals(&evo, "cmaes_signals.par");  /* write header and initial values */
	evo.sp.stopMaxIter = maxIteration;

	int generation = 0;
	double minarFunvals = DBL_MAX;
	
	/* Iterate until stop criterion holds */
	while(!cmaes_TestForTermination(&evo))
	  { 
	    /* generate lambda new search points, sample population */
	    pop = cmaes_SamplePopulation(&evo); /* do not change content of pop */
	
	    /* transform into bounds and evaluate the new search points */
	    for (i = 0; i < cmaes_Get(&evo, "lambda"); ++i) {
	      arFunvals[i] = fitFunc(pop[i], dimension); /* evaluate */
		  /* bundary penalty : made by hwangpil */
		  arFunvals[i] += boundaryPenalty(pop[i], dimension);
		  if(minarFunvals > arFunvals[i])
			  minarFunvals = arFunvals[i];
	    }
		if( generation++ % 10 == 0)
			//printf("generation : %3.d\tminFunvals : %lf\n", generation, minarFunvals);
				//fout<<generation++ << " " << minarFunvals<<std::endl;
				std::cout<<generation << " " << minarFunvals<<std::endl;
	
	    /* update the search distribution used for cmaes_SampleDistribution() */
	    cmaes_UpdateDistribution(&evo, arFunvals);  /* assumes that pop[i] has not been modified */
	
	    /* read instructions for printing output or changing termination conditions */ 
	    fflush(stdout); /* useful in MinGW */
	  }
	fout<<generation << " " << minarFunvals<<std::endl;
	std::cout << generation << " " << minarFunvals << std::endl;

	//printf("Stop:\n%s\n",  cmaes_TestForTermination(&evo)); /* print termination reason */
	//cmaes_WriteToFile(&evo, "all", "allcmaes.dat");         /* write final results */
	/* get best estimator for the optimum, xmean */
	xfinal = cmaes_GetNew(&evo, "xbestever");
	
	for(i=0; i<dimension; i++)
		solution.push_back(xfinal[i]);
	
	/* and finally release memory */
	fout.close();
	cmaes_exit(&evo); /* release memory */
	free(xfinal);

	readyToRun = false;
}

void CmaOptimizer::run_boundary()
{
	if(readyToRun == false)
		return;
	cmaes_t evo; /* an CMA-ES type struct or "object" */
	evo.version = NULL;
	boundary_transformation_t boundaries;
	double *arFunvals, *x_in_bounds, *const*pop;
	//double lowerBounds[] = {1.0, DBL_MAX / -1e2}; /* last number is recycled for all remaining coordinates */
	//double upperBounds[] = {3, 2e22};
	int nb_bounds = lowerBounds.size(); /* numbers used from lower and upperBounds */
	unsigned long dimension;
	int i; 

	std::ofstream fout;
	fout.open("cmaResults.txt", std::ios::app);
	char buf[60];
	time_t tt;
	time(&tt);
	strcpy(buf, ctime(&tt));
	fout << buf << std::endl;
	/* initialize boundaries, be sure that initialSigma is smaller than upper minus lower bound */
	boundary_transformation_init(&boundaries, lowerBounds.data(), upperBounds.data(), nb_bounds);

	/* Initialize everything into the struct evo, 0 means default */
	//arFunvals = cmaes_init(&evo, 0, NULL, NULL, 0, 0, "cmaes_initials.par");
	
	if(xstart.size() > 0 && stddev.size() > 0)
		arFunvals = cmaes_init(&evo, dim, xstart.data(), stddev.data(), 0, lambda, "non");
	else if(xstart.size() > 0 && stddev.size() == 0)
		arFunvals = cmaes_init(&evo, dim, xstart.data(), NULL, 0, lambda, "non");
	else if(xstart.size() == 0 && stddev.size() > 0)
		arFunvals = cmaes_init(&evo, dim, NULL, stddev.data(), 0, lambda, "non");
	else 
		arFunvals = cmaes_init(&evo, dim, NULL, NULL, 0, lambda, "non");

	dimension = (unsigned long)cmaes_Get(&evo, "dimension");
	
	//printf("%s\n", cmaes_SayHello(&evo));
	printf("New Cma Starts\n");
	
	x_in_bounds = cmaes_NewDouble(dimension); /* calloc another vector */
	//cmaes_ReadSignals(&evo, "cmaes_signals.par");  /* write header and initial values */

	int generation = 0;

	evo.sp.stopMaxIter = maxIteration;
	double minarFunvals=DBL_MAX;
	
	/* Iterate until stop criterion holds */
	while(!cmaes_TestForTermination(&evo))
	  { 
	    /* generate lambda new search points, sample population */
	    pop = cmaes_SamplePopulation(&evo); /* do not change content of pop */
	
	    /* transform into bounds and evaluate the new search points */
	    for (i = 0; i < cmaes_Get(&evo, "lambda"); ++i) {
	      boundary_transformation(&boundaries, pop[i], x_in_bounds, dimension);
	      /* this loop can be omitted if is_feasible is invariably true */
	      while(!isFeasible(x_in_bounds, dimension)) { /* is_feasible needs to be user-defined, in case, and can change/repair x */
	          cmaes_ReSampleSingle(&evo, i); 
	          boundary_transformation(&boundaries, pop[i], x_in_bounds, dimension);
	      }
		  /* evaluate */
		  arFunvals[i] = fitFunc(x_in_bounds, dimension);
		  /*
		  while( _isnan(arFunvals[i] = fitFunc(x_in_bounds, dimension)))
		  {
	          cmaes_ReSampleSingle(&evo, i); 
	          boundary_transformation(&boundaries, pop[i], x_in_bounds, dimension);
		  }
		  */
		  if(minarFunvals > arFunvals[i])
			  minarFunvals = arFunvals[i];
	    }
		if( generation++ % 10 == 0)
			//printf("generation : %3.d\tminFunvals : %lf\n", generation, minarFunvals);
				//fout<<generation++ << " " << minarFunvals<<std::endl;
				std::cout<<generation << " " << minarFunvals<<std::endl;
	
	    /* update the search distribution used for cmaes_SampleDistribution() */
	    cmaes_UpdateDistribution(&evo, arFunvals);  /* assumes that pop[i] has not been modified */
	
	    /* read instructions for printing output or changing termination conditions */ 
	    //cmaes_ReadSignals(&evo, "cmaes_signals.par");
	    fflush(stdout); /* useful in MinGW */
	  }
	std::cout << generation << " " << minarFunvals << std::endl;
	fout<<generation << " " << minarFunvals<<std::endl;

	//printf("Stop:\n%s\n",  cmaes_TestForTermination(&evo)); /* print termination reason */
	//cmaes_WriteToFile(&evo, "all", "allcmaes.dat");         /* write final results */
	/* get best estimator for the optimum, xmean */
	boundary_transformation(&boundaries,
			  (double const *) cmaes_GetPtr(&evo, "xbestever"), /* "xbestever" might be used as well */
			  x_in_bounds, dimension);
	
	/* do something with final solution x_in_bounds */
	
	//printf("%lf", x_in_bounds[0]);
	for(i=0; i<dimension; i++)
		solution.push_back(x_in_bounds[i]);
	
	/* and finally release memory */
	fout.close();
	cmaes_exit(&evo); /* release memory */
	boundary_transformation_exit(&boundaries); /* release memory */
	free(x_in_bounds);
}

void CmaOptimizer::saveSolution(char *filename)
{
	std::ofstream fout;
	std::ifstream fin;
	std::stringstream sstream;

	char data;

	fin.open(filename);
	while(fin.get(data)) sstream.put(data);
	fin.close();

	fout.open(filename, std::ios::out);
	fout.seekp(std::ios::beg);
	fout << solution.size() << std::endl;
	for(int i=0; i < solution.size(); i++)
		fout << solution.at(i) << " ";
	fout << std::endl;

	while(sstream.get(data)) fout.put(data);

	fout.close();
}

void CmaOptimizer::loadSolution(char *filename)
{
	std::ifstream fin;
	int dim;
	double sol;
	fin.open(filename);
	fin >> dim;
	for(int i=0; i<dim; i++)
	{
		fin >> sol;
		solution.push_back(sol);
	}
	fin.close();
}

void CmaOptimizer::printSolution(const double *x, int dim)
{
	for(int i=0; i<dim; i++)
		printf("%lf ", x[i]);
	printf("\n");
}


/* test code 
static double func(double const *x, int dim)
{
	return (x[0]-1)*(x[0]-2);
}

int main(int argc, char ** argv)
{
	std::vector<double> empty;
	CmaOptimizer c;
	c.setOptimizer(func, 1, empty, empty, empty, empty);
	c.run();

	printf("%lf", c.solution.at(0));

	double x[1];
	scanf_s("%lf", x);
	return 0;
}
*/
