#include "pch.h"
#include <sstream>
#include <string>
#include <iostream>

#include "Simulation.h"
#include "GL_Related.h"

using namespace std;

SimBasic* simb = new SimBasic(vector3f(0.0f, -9.81f, 0.0f));
float timeElapsed = 0;

double eyex = 0.0, eyey = 0.0, eyez = 45.0;

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
	Application			application;
	GL_Window			window;
	Keys				keys;
	BOOL				isMessagePumpActive;
	MSG					msg;
	DWORD				tickCount;

	application.className = "OpenGL";
	application.hInstance = hInstance;


	ZeroMemory(&window, sizeof(GL_Window));
	window.keys = &keys;
	window.init.application = &application;
	window.init.title = (char*)"Gizem Kayar Simulation Window";
	window.init.width = 800;
	window.init.height = 600;
	window.init.bitsPerPixel = 16;
	window.init.isFullScreen = FALSE;

	ZeroMemory(&keys, sizeof(Keys));

	if (RegisterWindowClass(&application) == FALSE)
	{

		MessageBox(HWND_DESKTOP, "Error Registering Window Class!", "Error", MB_OK | MB_ICONEXCLAMATION);
		return -1;
	}

	g_isProgramLooping = TRUE;
	g_createFullScreen = window.init.isFullScreen;
	while (g_isProgramLooping)
	{
		window.init.isFullScreen = g_createFullScreen;
		if (CreateWindowGL(&window) == TRUE)
		{
			if (Initialize(&window, &keys) == FALSE)
			{
				TerminateApplication(&window);
			}
			else
			{
				isMessagePumpActive = TRUE;
				while (isMessagePumpActive == TRUE)
				{
					if (PeekMessage(&msg, window.hWnd, 0, 0, PM_REMOVE) != 0)
					{

						if (msg.message != WM_QUIT)
						{
							DispatchMessage(&msg);
						}
						else
						{
							isMessagePumpActive = FALSE;
						}
					}
					else
					{
						if (window.isVisible == FALSE)
						{
							WaitMessage();
						}
						else
						{
							tickCount = GetTickCount();
							Update(tickCount - window.lastTickCount);
							window.lastTickCount = tickCount;
							Draw();
							SwapBuffers(window.hDC);
						}
					}
				}
			}

			DestroyWindowGL(&window);
		}
		else
		{
			MessageBox(HWND_DESKTOP, "Error Creating OpenGL Window", "Error", MB_OK | MB_ICONEXCLAMATION);
			g_isProgramLooping = FALSE;
		}
	}

	UnregisterClass(application.className, application.hInstance);
	return 0;
}

void Update(DWORD ms)
{
	if (g_keys->keyDown[VK_ESCAPE] == TRUE)
		TerminateApplication(g_window);


	float sec = ms / 1000.0f;
	timeElapsed += sec;
	simb->simulate();
}

void drawCube(vector3f color)
{
	glBegin(GL_QUADS);

	float x = color.getX();
	float y = color.getY();
	float z = color.getZ();

	// Front Face
	glColor3f(x, y, z);
	glNormal3f(0, 0, 1);
	glVertex3f(-0.1f, -0.1f, 0.1f);
	glVertex3f(0.1f, -0.1f, 0.1f);
	glVertex3f(0.1f, 0.1f, 0.1f);
	glVertex3f(-0.1f, 0.1f, 0.1f);

	// Back Face
	glColor3f(x, y, z);
	glNormal3f(0, 0, -1);
	glVertex3f(-0.1f, -0.1f, -0.1f);
	glVertex3f(-0.1f, 0.1f, -0.1f);
	glVertex3f(0.1f, 0.1f, -0.1f);
	glVertex3f(0.1f, -0.1f, -0.1f);

	// Top Face
	if (x > 0.5f)
		glColor3f(x - 0.5f, y, z);
	else if (x != 0)
		glColor3f(x - 0.3f, y, z);
	else
		glColor3f(x + 0.3f, y, z);
	glNormal3f(0, 1, 0);
	glVertex3f(-0.1f, 0.1f, -0.1f);
	glVertex3f(-0.1f, 0.1f, 0.1f);
	glVertex3f(0.1f, 0.1f, 0.1f);
	glVertex3f(0.1f, 0.1f, -0.1f);

	// Bottom Face
	if (x > 0.5f)
		glColor3f(x - 0.5f, y, z);
	else if (x != 0)
		glColor3f(x - 0.3f, y, z);
	else
		glColor3f(x + 0.3f, y, z);
	glNormal3f(0, -1, 0);
	glVertex3f(-0.1f, -0.1f, -0.1f);
	glVertex3f(0.1f, -0.1f, -0.1f);
	glVertex3f(0.1f, -0.1f, 0.1f);
	glVertex3f(-0.1f, -0.1f, 0.1f);

	// Right face
	if (y > 0.5f)
		glColor3f(x, y - 0.5f, z);
	else if (x != 0)
		glColor3f(x, y - 0.3f, z);
	else
		glColor3f(x, y + 0.3f, z);
	glNormal3f(1, 0, 0);
	glVertex3f(0.1f, -0.1f, -0.1f);
	glVertex3f(0.1f, 0.1f, -0.1f);
	glVertex3f(0.1f, 0.1f, 0.1f);
	glVertex3f(0.1f, -0.1f, 0.1f);

	// Left Face
	if (y > 0.5f)
		glColor3f(x, y - 0.5f, z);
	else if (x != 0)
		glColor3f(x, y - 0.3f, z);
	else
		glColor3f(x, y + 0.3f, z);
	glNormal3f(-1, 0, 0);
	glVertex3f(-0.1f, -0.1f, -0.1f);
	glVertex3f(-0.1f, -0.1f, 0.1f);
	glVertex3f(-0.1f, 0.1f, 0.1f);
	glVertex3f(-0.1f, 0.1f, -0.1f);

	glEnd();
}

void drawWalls()
{
	glBegin(GL_QUADS);

	//left
	glColor3f(0.5f, 0.5f, 0.5f);
	glVertex3f(-10, 10, 10);
	glVertex3f(-10, 10, -10);
	glVertex3f(-10, -10, -10);
	glVertex3f(-10, -10, 10);

	//right
	glColor3f(0.5f, 0.5f, 0.5f);
	glVertex3f(10, 10, -10);
	glVertex3f(10, 10, 10);
	glVertex3f(10, -10, 10);
	glVertex3f(10, -10, -10);

	//far
	glColor3f(0.9f, 0.9f, 0.9f);
	glVertex3f(-10, 10, -10);
	glVertex3f(10, 10, -10);
	glVertex3f(10, -10, -10);
	glVertex3f(-10, -10, -10);

	//bottom
	glColor3f(0.8f, 0.8f, 0.8f);
	glVertex3f(-10, -10, -10);
	glVertex3f(10, -10, -10);
	glVertex3f(10, -10, 10);
	glVertex3f(-10, -10, 10);

	//top
	glColor3f(0.8f, 0.8f, 0.8f);
	glVertex3f(-10, 10, -10);
	glVertex3f(10, 10, -10);
	glVertex3f(10, 10, 10);
	glVertex3f(-10, 10, 10);


	glEnd();

}
void Draw(void)
{

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0, 0, 45, 0, 0, 0, 0, 1, 0);


	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	vector3f halfWorld = simb->halfWorld;

	drawWalls();


	for (int a = 0; a < simb->noPt; a++)
	{
		vector3f pos = simb->particleList[a].getPosition();
		vector3f color = simb->particleList[a].getColor();

		glTranslatef(pos.getX(), pos.getY(), pos.getZ());
		drawCube(color);
		glTranslatef(-pos.getX(), -pos.getY(), -pos.getZ());
	}


	glColor3ub(100, 100, 100);
	glPrint(-5.0f, -14, 0, "Time elapsed (seconds): %.2f", timeElapsed);

	glColor3ub(100, 0, 0);
	glPrint(-5.0f, 13, 0, "Simple Gravity System");
	glPrint(-5.0f, 12, 0, "System Kinetic Energy: %.2f", simb->kinEn);
	glPrint(-5.0f, 11, 0, "System Potential Energy: %.2f", simb->potEn);
	glPrint(-5.0f, 10, 0, "System Total Energy: %.2f", (simb->kinEn + simb->potEn));

	glFlush();
}








