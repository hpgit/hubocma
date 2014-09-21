#ifndef HUBOCMATHREAD_H
#define HUBOCMATHREAD_H

#include <QThread>
#include <CmaOptimizer.h>

class HuboCmaThread : public QThread
{
public:
	HuboCmaThread();
	CmaOptimizer *cma;

	void setCmaOptimizer(CmaOptimizer *_cma);
private:
	void run();
};

#endif //HUBOCMATHREAD_H