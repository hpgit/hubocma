#include "hubocmathread.h"

HuboCmaThread::HuboCmaThread()
{
	cma = 0;
}

void HuboCmaThread::run()
{
	if (cma !=0)
		cma->run_boundary();
}

void HuboCmaThread::setCmaOptimizer(CmaOptimizer *_cma)
{
	cma = _cma;
}