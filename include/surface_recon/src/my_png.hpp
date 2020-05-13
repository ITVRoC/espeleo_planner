#ifndef MY_PNG
#define MY_PNG

#include <stdio.h>
#include <malloc.h>
#include <string>
#include <iostream>

#include <png.h>
#include <string.h>

using namespace std;

class my_png
{

	public:
	unsigned char* buffer;
	int width,height;

		my_png()
	{
		buffer = NULL;
		width = 0;
		height = 0;
	}

	my_png(int w, int h)
	{
		buffer = new unsigned char [w*h*3];
		width = w;
		height = h;

		memset(buffer,127,(w*h*3)*sizeof(unsigned char));
	}

	unsigned char& at(int r, int c, int ch);
	void write(string filename);

};

#endif