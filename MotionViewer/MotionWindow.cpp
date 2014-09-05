#include "MotionWindow.h"

//MotionGLWindow class

MotionGLWindow::MotionGLWindow(int x, int y, int w, int h, const char *l)
	: Fl_Gl_Window(x, y, w, h, l)
{
	camera = Camera();

	press[0] = press[1] = press[2] = 0;
}

void MotionGLWindow::initialize_gl()
{
	init_gl_color();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective( 45., float(w())/float(h()), 0.1, 1000.);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void MotionGLWindow::init_gl_color()
{
	GLfloat light_ambient[] =  {0.0, 0.0, 0.0, 1.0};
	GLfloat light_diffuse[] =  {1.0, 1.0, 1.0, 1.0};
	GLfloat light_specular[] =  {1.0, 1.0, 1.0, 1.0};
	GLfloat light_position[] =  {1.0, 1.0, 1.0, 0.0};

	GLfloat mat_specular[] = {1.0, 1.0, 1.0, 1.0};
	GLfloat mat_shininess = 70.0;

	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular);
	glMaterialf(GL_FRONT, GL_SHININESS, mat_shininess);

	GLfloat ambient[] = {0.7, 0.7, 0.7, 1};
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	//glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);

	glEnable(GL_NORMALIZE);

	glEnable(GL_POINT_SMOOTH);
}

void MotionGLWindow::draw_ground()
{
	int count = 0;
	glBegin(GL_QUADS);
	for (int i = -80; i < 81; ++i) {
		for (int j = -80; j < 81; ++j) {
			if (count % 2 == 0)
				glColor3d(0.88, 0.88, 0.88);
				//glColor3d(0.90, 0.90, 0.90);
			else
				glColor3d(0.93, 0.93, 0.93);
				//glColor3d(0.95, 0.95, 0.95);

			glNormal3d(0.,0., 1.);

			glVertex3f(j, 0, i);
			glVertex3f(j, 0, i+1);
			glVertex3f(j+1, 0, i+1);
			glVertex3f(j+1, 0, i);
			count += 1;
		}
	}
	glEnd();
}

void MotionGLWindow::setupFloor()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_STENCIL_TEST);
	glStencilFunc(GL_ALWAYS,0x1,0x1);
	glStencilOp(GL_REPLACE,GL_REPLACE,GL_REPLACE);
	glStencilMask(0x1);		// only deal with the 1st bit
}

void MotionGLWindow::draw_axis()
{
	glBegin(GL_LINES);
	glColor3f(1,0,0);
	glVertex3f(1.0,0,0);
	glVertex3f(0,0,0);
	glColor3f(0,1,0);
	glVertex3f(0,1.0,0);
	glVertex3f(0,0,0);
	glColor3f(0,0,1);
	glVertex3f(0,0,1.0);
	glVertex3f(0,0,0);
	glEnd();
}

int MotionGLWindow::handle(int event)
{
	int returned = 0;
	int mouseX = Fl::event_x();
	int mouseY = Fl::event_y();
	int pushButton = Fl::event_button();

	if(event == FL_PUSH && pushButton >=1 && pushButton <=3)
	{
		press[pushButton] = 1;
		mousePrevX = mouseX;
		mousePrevY = mouseY;
		returned = 1;
	}
	else if ( event == FL_DRAG )
	{
		int mousedX = mouseX - mousePrevX;
		int mousedY = -(mouseY - mousePrevY);

		if( pushButton == 2 )
		{
			camera.setPosOnSphereAfterScreenRotate(mousedX, mousedY, w(), h());
			camera.setTransforming();
		}
		if( pushButton == 3)
		{
			camera.addPosAfterScreenShift(mousedX, mousedY, h(), M_PI/4.);
			camera.setTransforming();
		}
		returned = 1;

	}
	else if ( event == FL_RELEASE && pushButton >= 1 && pushButton <= 3 )
	{
		camera.invalidate();
		press[pushButton] = 0;
		camera.unsetTransforming();

		returned = 1;
	} 
	
	else if (event == FL_MOUSEWHEEL)
	{
		double dist = camera.distance;
		camera.setDistance((8.0+Fl::event_dy())*dist/8.0);
		returned = 1;
	}
	
	if(returned ==1)
	{
		redraw();
		return 1;
	}

	return Fl_Gl_Window::handle(event);
}

void MotionGLWindow::draw()
{
	if (!valid()) {
		valid(1);
		initialize_gl();
	}

	glClearColor(1,1,1,0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glLoadIdentity();
	camera.camera_lookat();

	glDisable(GL_LIGHTING);
	setupFloor();
	draw_ground();
	draw_axis();	

	glColor3d(0., 0., 0.);

	glEnable(GL_LIGHTING);
	_draw();
}

void MotionGLWindow::_draw()
{
}
