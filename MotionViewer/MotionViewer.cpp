#include "MotionViewer.h"

using std::cout;
using std::endl;





int main(int argc, char **argv)
{
	MotionViewer *form2;
	form2 = new MotionViewer();
	MotionGLWindow *gl = new MotionGLWindow(0, 0, form2->w(), form2->h()-30);
	
	form2->end();
	form2->show();
	
	return Fl::run();
}