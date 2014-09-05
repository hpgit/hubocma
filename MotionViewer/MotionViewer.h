

#ifndef WIN32
	#define WIN32
#endif

#include "MotionWindow.h"
#include <FL/Fl_Gl_Window.H>


class MotionViewer : public Fl_Window
{
public:
	// TODO:
	// 모션 클래스를 만들어서 constructor 파라미터로 넣게 할 것
	//MotionViewer(Motion *motion, int w = 800, int h = 600, const char *l = NULL) : Fl_Gl_Window(w, h, l){}
	MotionViewer(int w = 1024, int h = 768, const char *l = NULL) : Fl_Window(w, h, l){}

protected:
	int frame_rate;
};