#include <sstream>
#include <string>
#include <iostream>

#include "Simulation.h"
#include "GL_Related.h"

using namespace std;

SimSpring* simsp = new SimSpring();
float timeElapsed = 0;
bool drawSprings = true;
bool drawTriangles = false;

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
	//window.init.title = "Gizem Kayar Simulation Window";	
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
	if (g_keys->keyDown['S'] == TRUE)
	{
		drawSprings = true;
		drawTriangles = false;
	}
	if (g_keys->keyDown['T'] == TRUE)
	{
		drawSprings = false;
		drawTriangles = true;
	}


	float sec = ms / 1000.0f;
	timeElapsed += sec;
	simsp->simulate();
}

void drawCube(vector3f color)
{
	glBegin(GL_QUADS);

	float x = color.getX();
	float y = color.getY();
	float z = color.getZ();
	
	glColor3f(x, y, z);
	// Front Face
	glNormal3f(0, 0, 1);
	glVertex3f(-0.05f, -0.05f, 0.05f);
	glVertex3f(0.05f, -0.05f, 0.05f);
	glVertex3f(0.05f, 0.05f, 0.05f);
	glVertex3f(-0.05f, 0.05f, 0.05f);

	// Back Face
	glNormal3f(0, 0, -1);
	glVertex3f(-0.05f, -0.05f, -0.05f);
	glVertex3f(-0.05f, 0.05f, -0.05f);
	glVertex3f(0.05f, 0.05f, -0.05f);
	glVertex3f(0.05f, -0.05f, -0.05f);

	// Top Face
	glNormal3f(0, 1, 0);
	glVertex3f(-0.05f, 0.05f, -0.05f);
	glVertex3f(-0.05f, 0.05f, 0.05f);
	glVertex3f(0.05f, 0.05f, 0.05f);
	glVertex3f(0.05f, 0.05f, -0.05f);

	// Bottom Face
	glNormal3f(0, -1, 0);
	glVertex3f(-0.05f, -0.05f, -0.05f);
	glVertex3f(0.05f, -0.05f, -0.05f);
	glVertex3f(0.05f, -0.05f, 0.05f);
	glVertex3f(-0.05f, -0.05f, 0.05f);

	// Right face
	glNormal3f(1, 0, 0);
	glVertex3f(0.05f, -0.05f, -0.05f);
	glVertex3f(0.05f, 0.05f, -0.05f);
	glVertex3f(0.05f, 0.05f, 0.05f);
	glVertex3f(0.05f, -0.05f, 0.05f);

	// Left Face
	glNormal3f(-1, 0, 0);
	glVertex3f(-0.05f, -0.05f, -0.05f);
	glVertex3f(-0.05f, -0.05f, 0.05f);
	glVertex3f(-0.05f, 0.05f, 0.05f);
	glVertex3f(-0.05f, 0.05f, -0.05f);

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

	gluLookAt(0, 0, 45, 0, 0, 0 , 0, 1, 0);
	 

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	
	vector3f halfWorld = simsp->halfWorld;
	
	//glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	drawWalls();


	for (int a = 0; a < simsp->mesh.particles.size(); a++)
	{
		vector3f pos = simsp->mesh.particles[a].getPosition();
		vector3f color = simsp->mesh.particles[a].getColor();

		glTranslatef(pos.getX(), pos.getY(), pos.getZ());
		drawCube(color);
		glTranslatef(-pos.getX(), -pos.getY(), -pos.getZ());
	}

	if (drawSprings)
	{
		for (int s = 0; s < simsp->mesh.springs.size(); s++)
		{
			Spring  sp = simsp->mesh.springs[s];
			vector3f position1 = simsp->mesh.particles[sp.p1].getPosition();
			vector3f position2 = simsp->mesh.particles[sp.p2].getPosition();

			glColor3f(0, 0, 1);
			glLineWidth(1);
			glBegin(GL_LINES);
			glVertex3f(position1.getX(), position1.getY(), position1.getZ());
			glVertex3f(position2.getX(), position2.getY(), position2.getZ());
			glEnd();
		}
	}

	if (drawTriangles)
	{
		for (int t = 0; t < simsp->mesh.surfaceTriangles.size(); t++)
		{
			Triangle tri = simsp->mesh.surfaceTriangles[t];
			vector3f position1 = simsp->mesh.particles[tri.p1].getPosition();
			vector3f position2 = simsp->mesh.particles[tri.p2].getPosition();
			vector3f position3 = simsp->mesh.particles[tri.p3].getPosition();
			glColor3f(1, 0, 0);
			glLineWidth(1);
			glBegin(GL_TRIANGLES);
			glNormal3f(tri.normal.getX(), tri.normal.getY(), tri.normal.getZ());
			glVertex3f(position1.getX(), position1.getY(), position1.getZ());
			glVertex3f(position2.getX(), position2.getY(), position2.getZ());
			glVertex3f(position3.getX(), position3.getY(), position3.getZ());
			glEnd();
		}
	}
	


	glColor3ub(200, 255, 200);									
	glPrint(-5.0f, -14, 0, "Time elapsed (seconds): %.2f", timeElapsed);
	glPrint(0.0f, -12, 0, "S_key: draw springs, T_key: draw triangles");
	
	glPrint(-5.0f, 13, 0, "Simple Gravity System");
	glPrint(-5.0f, 12, 0, "System Kinetic Energy: %.2f", simsp->kinEn);
	glPrint(-5.0f, 11, 0, "System Potential Energy: %.2f", simsp->potEn);
	glPrint(-5.0f, 10, 0, "System Spring Energy: %.2f", (simsp->sprEn));
	glPrint(-5.0f, 9, 0, "System Total Energy: %.2f", (simsp->kinEn+simsp->potEn));

	glFlush();													
}








