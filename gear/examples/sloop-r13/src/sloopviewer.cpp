#include <GL/freeglut.h>
#include <iostream>
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <vector>
#include "mysystem_loop_single.h"

using namespace std;

#define KEY_SPACE 32
#define KEY_ESC 27
#define KEY_ENTER 13

// system color
GLfloat textcolor[] = {0,0,0,1};
GLfloat backgroundcolor[] = {1,1,1,1};
//GLfloat backgroundcolor[] = {0,0,0,1};
GLfloat lightpos0[] = {0,0,4,1};
GLfloat lightpos1[] = {1,1,4,1};
GLfloat lightpos2[] = {-1,-1,4,1};
GLfloat lightambient0[] = {0.1,0.1,0.1,1};
GLfloat lightambient1[] = {0.1,0.1,0.1,1};
GLfloat lightambient2[] = {0.1,0.1,0.1,1};
GLfloat lightspecular[] = {1.0,1.0,1.0,1};

// mysystem
MySystem *pmysystem = NULL;
float mscale = 1; // model scale (this will be set later to be the radius of the estimated bounding shpere of the character)
bool bshowboundingbox = false;
glsubDrawType drawtype = GLSUB_SOLID;

// simulation mode
bool bsimul=false;	
double current_time = 0; // this is also used fZor replay
double max_simul_time = 100;
double simul_stepsize = 0.0002;
int simul_save_freq = 30; // data saving frequency
int max_counter_save = int(1./(double(simul_save_freq)*0.001));
int counter_save=0;
char str_simul_info[300];
double start_simul_time=0, end_simul_time=0;

// replay mode
bool breplay=false;	
int current_frame_idx=0; // current replay frame index
double speed=1.0; // replay speed
int render_freq=33;
int num_skip_frames = int(double(simul_save_freq)/double(render_freq)*speed-1); // number of frames to be skipped when replay

// viewer state
Vec3 spoint_prev;
SE3 mvmatrix, mvmatrix_prev; // view change
double &xcam = mvmatrix[12], &ycam = mvmatrix[13]; // view translate
double &sdepth = mvmatrix[14]; // zoom in/out
float fovy=64.0f, zNear=0.001f, zFar=10000.0f;
float aspect = 5.0f/4.0f;
long winsizeX, winsizeY;
int downX, downY;
bool bRotateView = false, bTranslateView = false, bZoomInOut = false, bSliderControl = false, bShowUsage = false;
bool bfullscreen = false;
GLfloat light0Position[] = {1, 1, 1, 1};
bool bperspective = false;

// slider control position
int sliderXi=20, sliderXf=520, sliderX=sliderXi;
int sliderY=10; // vertical position (from the bottom line of the window)

// declaration
void StepSimulation(void);
void StopSimulation(void);
void StartSimulation(void);

void PlotString(void *font, int x, int y, const char *string)
{        
	// switch to projection mode	
	glMatrixMode(GL_PROJECTION);	
	glPushMatrix();	
	// reset matrix	
	glLoadIdentity();	
	//Set the viewport	
	glViewport(0, 0, winsizeX, winsizeY);
	// set a 2D orthographic projection	
	glOrtho(0, winsizeX, winsizeY, 0, -100, 100);	
	glMatrixMode(GL_MODELVIEW);	
	glPushMatrix();	
	glLoadIdentity();
	glTranslatef(0.0f,0.0f,99.9f);
	// draw
	glRasterPos2i(x,y);
	glutBitmapString( font, (const unsigned char *)string );
	// restore
	glMatrixMode(GL_PROJECTION);	
	glPopMatrix ();	
	glMatrixMode(GL_MODELVIEW);	
	glPopMatrix ();
}  

void DrawTextInfo(void)
{
	char str[300];
	if ( bsimul ) {
		sprintf(str, "t = %7.4f", current_time);
	} else if ( pmysystem->sysdata.size() > 0 ) {
		sprintf(str, "t = %7.4f, tf = %7.4f", current_time, pmysystem->sysdata.m_list_time.back());
	} else {
		sprintf(str, "t = %7.4f", current_time);
	}
	PlotString(GLUT_BITMAP_HELVETICA_12, 20, 20, str);

	char str2[] = "shift + left-click : start/pause simulation with center of mass guidance, \nSPACE : start/pause replay, r : reset time, d : solid/wire rendering, ESC : exit";
	glColor4fv(textcolor);
	PlotString(GLUT_BITMAP_HELVETICA_12, 20, winsizeY-45, str2);

	// draw a slider
	int h = (sliderXf-sliderXi)/10;
	for (int x=sliderXi; x<sliderXf; x+=h) {
		PlotString(GLUT_BITMAP_HELVETICA_12, x, winsizeY-sliderY, "|------");
	}
	PlotString(GLUT_BITMAP_HELVETICA_12, sliderXf, winsizeY-sliderY, "|");
	PlotString(GLUT_BITMAP_HELVETICA_12, sliderX, winsizeY-sliderY, "[^]");
	sprintf(str, "(%d%%)", 100*(sliderX-sliderXi)/(sliderXf-sliderXi));
	PlotString(GLUT_BITMAP_HELVETICA_12, sliderXf+15, winsizeY-sliderY, str);
}

void DrawModel(void) 
{
	if ( pmysystem == NULL ) return;
	pmysystem->render(false);
}

void TestFunc(void)
{
	//pmysystem->test_func();
	cout << mvmatrix << endl;
}

void ReadSimulData(void)
{
	if ( pmysystem->sysdata.size() == 0 ) {
		return;
	}
	if ( !pmysystem->sysdata.read_data_from_memory(current_time, current_frame_idx) ) {
		std::cout << "failed in reading data: frame index = " << current_frame_idx << std::endl;
		return;
	}
	pmysystem->update_kinematics();

	if ( pmysystem->sysdata.size() > 0 ) {
		sliderX = int( (float)sliderXi + (float)(sliderXf-sliderXi) * (float)current_frame_idx / (float)(pmysystem->sysdata.size()-1) );
	} else {
		sliderX = sliderXi;
	}
}

void StopSimulation(void)
{
	glutIdleFunc(NULL);
	gReal elapsed_time = toc();
	end_simul_time = current_time;
	std::cout << "computation time for " << end_simul_time - start_simul_time << "sec simulation = " << elapsed_time << " sec" << std::endl;
	bsimul = false;
	current_frame_idx = pmysystem->sysdata.size()-1;
	ReadSimulData();
}

void StartSimulation(void)
{
	pmysystem->sysdata.truncate_data_on_memory(current_frame_idx);
	pmysystem->sysdata.write_data_into_memory(current_time);
	counter_save = 0;
	bsimul = true;
	start_simul_time = current_time;
	tic();
	glutIdleFunc(StepSimulation);
}

void StepSimulation(void)
{
	int cnt=0;
	while ( cnt++ < max_counter_save ) {
		bool bsuccess = true;
		if ( !pmysystem->stepSimulation() ) {
			cout << "simulation stopped! (t = " << current_time << " sec)" << endl;
			bsuccess = false;
		}
		current_time += pmysystem->step_size;
		if ( !bsuccess || current_time > max_simul_time ) {
			if ( current_time > max_simul_time ) {
				cout << "reached maximum simulation time (" << max_simul_time << "sec)!" << endl;
			}
			pmysystem->sysdata.write_data_into_memory(current_time);
			StopSimulation();
			glutPostRedisplay();
			return;
		}
	}
	pmysystem->sysdata.write_data_into_memory(current_time);
}

void StopReplay(void)
{
	breplay = false;
}

void StartReplay(void)
{
	breplay = true;
}

void ResetTime(void) 
{
	current_frame_idx = 0;
	ReadSimulData();
	glutPostRedisplay();
}

void Timer(int extra) 
{
	if ( breplay ) {
		current_frame_idx += 1 + num_skip_frames;
		if ( current_frame_idx >= pmysystem->sysdata.size() ) {
			breplay = false;
			current_frame_idx = pmysystem->sysdata.size()-1;
		}
		ReadSimulData();
		glutPostRedisplay();
	}
	if ( bsimul ) {
		glutPostRedisplay();
		//glutTimerFunc(int(1000./10.), Timer, 0);
		//return;
	}
	if ( breplay && bsimul ) {
		breplay = bsimul = false;
	}
	glutTimerFunc(int(1000./30.), Timer, 0);
}

void ReshapeCallback(int width, int height) 
{
	winsizeX = width;
	winsizeY = height;
	aspect = (float)winsizeX/(float)winsizeY;
	glViewport(0, 0, winsizeX, winsizeY);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glutPostRedisplay();
}

void DisplayCallback(void) 
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	if ( bperspective ) {
		gluPerspective(fovy, aspect, zNear, zFar);
	} else {
		double s = -sdepth;
		glOrtho(-aspect*s, aspect*s, -s, s, -100, 100);
	}
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glMultMatrixd(mvmatrix.GetArray());
	DrawModel();
	DrawTextInfo();
	glutSwapBuffers();
	glutPostRedisplay(); 
	glClearColor(backgroundcolor[0],backgroundcolor[1],backgroundcolor[2],backgroundcolor[3]);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 
}

void KeyboardCallback(unsigned char ch, int x, int y) 
{
	string filepath;
	vector<string> filefilternames, filefilterpatterns;

	switch (ch) {
	case KEY_ESC:// exit program
		cout << "exit program" << endl;
		exit(0);
		break;
	case KEY_ENTER:  // start/pause simulation
		if ( bsimul ) {
			StopSimulation();
		} else {
			if ( breplay ) {
				StopReplay();
			} 
			StartSimulation();
		}
		break;
	case KEY_SPACE: // play/pause replay
		if ( breplay ) {
			StopReplay();
		} else {
			if ( bsimul ) {
				StopSimulation();
			}
			if ( current_frame_idx == pmysystem->sysdata.size()-1 ) {
				ResetTime();
			}
			StartReplay();
		}
		break;
	case 'd':
		if ( drawtype == GLSUB_SOLID ) {
			drawtype = GLSUB_WIRE;
		} else {
			drawtype = GLSUB_SOLID;
		}
		for (size_t i=0; i<pmysystem->pLinks.size(); i++) {
			pmysystem->pLinks[i]->surface.setDrawType(drawtype);
			for (size_t j=0; j<pmysystem->pLinks[i]->surfaces_for_rendering_only.size(); j++) {
				pmysystem->pLinks[i]->surfaces_for_rendering_only[j]->setDrawType(drawtype);
			}
		}
		for (size_t i=0; i<pmysystem->pObjects.size(); i++) {
			pmysystem->pObjects[i]->surface.setDrawType(drawtype);
			for (size_t j=0; j<pmysystem->pObjects[i]->surfaces_for_rendering_only.size(); j++) {
				pmysystem->pObjects[i]->surfaces_for_rendering_only[j]->setDrawType(drawtype);
			}
		}
		break;
	case 'i':
		cout << "num of links = " << pmysystem->pLinks.size() << endl;
		break;
	case 'r':
		ResetTime();
		break;
	case 't':
		TestFunc();
		break;
	case 'v':
		bperspective = !bperspective;
		break;
	} 

	glutPostRedisplay(); 
}

void SpecialKeyboardCallback(int key, int x, int y) 
{
	bool bctrlpressed = false, bshiftpressed = false, baltpressed = false; // note that multiple ctrl/shift/alt key pressing at the same time cannot be handled here.
	if (glutGetModifiers() == GLUT_ACTIVE_CTRL) { 
		bctrlpressed = true;
	}

	switch (key) {

	case GLUT_KEY_RIGHT: // right: step forward
		break;
	case GLUT_KEY_LEFT: // left: step backward
		break;
	case GLUT_KEY_DOWN: // down: speed down
		break;
	case GLUT_KEY_UP: // up: speed up
		break;
	}
	
	glutPostRedisplay(); 
}

Vec3 get_pos_sp(int x, int y)
{
	Vec3 p;
	p[0] = 2.0 * double(x) / double(winsizeX) - 1.0;
	p[1] = 1.0 - 2.0 * double(y) / double(winsizeY);
	p[2] = p[0] * p[0] + p[1] * p[1];
	if ( p[2] < 1.0 ) p[2] = sqrt(1.0 - p[2]);
	else { p[2] = sqrt(p[2]); p[0] /= p[2]; p[1] /= p[2]; p[2] = 0.0; }
	return p;
}

Vec3 GetOGLPos(int x, int y)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    double winX, winY, winZ;
    GLdouble posX, posY, posZ;
 
    glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
    glGetDoublev( GL_PROJECTION_MATRIX, projection );
    glGetIntegerv( GL_VIEWPORT, viewport );
 
    winX = (float)x;
    winY = (float)viewport[3] - (float)y;
	winZ = 0;
    //glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
 
    gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);
 
    return Vec3(posX, posY, posZ);
}

void SetTargetPosition(int x, int y)
{
	if ( bperspective ) {
		cout << "warning:: perspective mode not supported!" << endl;
		return;
	}
	Vec3 p = GetOGLPos(x,y);
	Vec3 e = Vec3(-mvmatrix[2], -mvmatrix[6], -mvmatrix[10]);
	Vec3 q = Vec3(0,0,0), n = Vec3(1,0,0);
	if ( fabs(Inner(e,n)) > 1E-6 ) {
		Vec3 pp = p - ( Inner(p-q,n)/Inner(e,n) ) * e;
		pmysystem->m_desired_position = pp;
	} else {
		cout << "warning:: projection failed!" << endl;
	}
	//cout << pmysystem->m_desired_position << endl;
}

void MouseCallback(int button, int state, int x, int y) 
{
	bool bshiftpressed = false;
	if (glutGetModifiers() == GLUT_ACTIVE_SHIFT) { 
		bshiftpressed = true;
	}

	// start/stop simulation by clicking shift + left click down/up
	if ( bshiftpressed && (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) && !bsimul ) { 
		SetTargetPosition(x,y);
		StartSimulation();
		glutPostRedisplay();
		return;
	} else if ( (button == GLUT_LEFT_BUTTON) && (state == GLUT_UP) && bsimul ) {
		StopSimulation();
		glutPostRedisplay();
		return;
	}

	downX = x; downY = y;
	bRotateView = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) && y < winsizeY-50 );
	bTranslateView = ((button == GLUT_MIDDLE_BUTTON) &&  (state == GLUT_DOWN));
	bZoomInOut = ((button == GLUT_RIGHT_BUTTON) &&  (state == GLUT_DOWN));
	bSliderControl = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN) && y > winsizeY-sliderY-15);

	if ( bRotateView ) {
		mvmatrix_prev = mvmatrix;
		spoint_prev = get_pos_sp(x, y);
	}

	// wheel (zoom in/out)
	if ( (button != GLUT_LEFT_BUTTON) && (button != GLUT_MIDDLE_BUTTON) && (button != GLUT_RIGHT_BUTTON) ) {
		if ( button==3 ) { // wheel up --> zoom out
			sdepth -= 0.1f*mscale;
		} 
		if ( button==4 ) { // wheel down --> zoom in
			sdepth += 0.1f*mscale;
		}
	}

	// slider control
	if ( bSliderControl ) {
		sliderX = x; 
		if ( sliderX < sliderXi ) { sliderX = sliderXi; }
		if ( sliderX > sliderXf ) { sliderX = sliderXf; }
		current_frame_idx = int( float(pmysystem->sysdata.size()-1) * float(sliderX-sliderXi) / float(sliderXf-sliderXi) );
		if ( current_frame_idx > pmysystem->sysdata.size()-1 ) { current_frame_idx = pmysystem->sysdata.size()-1; }
		if ( current_frame_idx < 0 ) { current_frame_idx = 0; }
		ReadSimulData();
	}

	glutPostRedisplay();
}
 
void MotionCallback(int x, int y) 
{
	if ( bsimul ) {
		SetTargetPosition(x,y);
	}

	if ( bRotateView ) {
		Vec3 spoint = get_pos_sp(x, y);
		double theta = acos( Inner(spoint_prev, spoint) );
		Vec3 n = Cross(spoint_prev, spoint);
		if ( Norm(n) > 1e-6 ) {
			Vec3 w = ~(mvmatrix_prev.GetRotation()) * n;
			mvmatrix.SetRotation(mvmatrix_prev.GetRotation() * Exp(w));
		}
	}
	if (bTranslateView) { float den = 30; xcam += (float)(x-downX)/den*mscale; ycam += (float)(downY-y)/den*mscale; } // translate
	if (bZoomInOut) { float den = 30; sdepth -= (float)(downY-y)/den*mscale;  } // zoom in/out
	downX = x; downY = y;

	// slider control
	if ( bSliderControl ) {
		sliderX = x; 
		if ( sliderX < sliderXi ) { sliderX = sliderXi; }
		if ( sliderX > sliderXf ) { sliderX = sliderXf; }
		current_frame_idx = int( float(pmysystem->sysdata.size()-1) * float(sliderX-sliderXi) / float(sliderXf-sliderXi) );
		if ( current_frame_idx > pmysystem->sysdata.size()-1 ) { current_frame_idx = pmysystem->sysdata.size()-1; }
		if ( current_frame_idx < 0 ) { current_frame_idx = 0; }
		ReadSimulData();
	}

	glutPostRedisplay();
}
 
void InitGL() 
{
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 480);
	glutCreateWindow("sloop");
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);
	glClearColor(backgroundcolor[0],backgroundcolor[1],backgroundcolor[2],backgroundcolor[3]);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHTING);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHT2);
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos0);
	glLightfv(GL_LIGHT1, GL_POSITION, lightpos1);
	glLightfv(GL_LIGHT2, GL_POSITION, lightpos2);
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightambient0);
	glLightfv(GL_LIGHT1, GL_AMBIENT, lightambient1);
	glLightfv(GL_LIGHT2, GL_AMBIENT, lightambient2);
	glLightfv(GL_LIGHT0, GL_SPECULAR , lightspecular);
	glLightfv(GL_LIGHT1, GL_SPECULAR , lightspecular);
	glLightfv(GL_LIGHT2, GL_SPECULAR , lightspecular);
	glEnable(GL_CULL_FACE);
	glutReshapeFunc(ReshapeCallback);
	glutDisplayFunc(DisplayCallback);
	glutKeyboardFunc(KeyboardCallback);
	glutSpecialFunc(SpecialKeyboardCallback);
	glutMouseFunc(MouseCallback);
	glutMotionFunc(MotionCallback); 
	glutTimerFunc(render_freq, Timer, 0);
}

void PrintCommandLineUsage()
{
}

// parse command line input and save it to fileloadinfo
void ParseCommand(int argc, char **argv) 
{
	string dir;
#ifdef _WIN32
	dir = string("../../mesh");
#else
	dir = string("mesh");
#endif

	pmysystem = new MySystemSingleLoop;
	if ( !pmysystem->create(dir.c_str()) ) {
		cerr << "failed in creating system" << endl;
	}

	//mvmatrix.SetRotation(SO3(0,0,1,1,0,0,0,1,0));
	sdepth = -pmysystem->getRadiusOfMinimalEnclosingSphere() / tan(0.25f*fovy/180.f*3.14f);
	mscale = (float)pmysystem->getRadiusOfMinimalEnclosingSphere();
	mvmatrix = SE3(RMatrix("0.255771   0.966095   0.0352377  -0.0666667 ; -0.346923   0.0577025   0.936117  -0.586667 ; 0.902345  -0.251656   0.349919  -1.92904 ;  0   0   0   1").GetPtr());
}

int main(int argc, char **argv) 
{
	glutInit(&argc, argv);
	InitGL();
	ParseCommand(argc, argv);
	glutMainLoop(); 
	return 0; 
}
 