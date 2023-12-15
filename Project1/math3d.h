#pragma once
/* SIMPLE MATHS LIBRARY
PREPARED BY GIZEM KAYAR FOR COMPUTER GRAPHICS COURSE/NEW YORK UNIVERSITY
YEAR: 2022
*/

#include <iostream>
using namespace std;

class vector3f
{
private:
	float x, y, z;
public:
	//constructor
	vector3f()
	{
		clear();
	}

	vector3f(float xx, float yy, float zz)
	{
		x = xx;
		y = yy;
		z = zz;
	}

	//dectructor
	~vector3f() { }

	//Setters and getters
	float getX() { return x; }
	float getY() { return y; }
	float getZ() { return z; }
	void setX(float _x) { x = _x; }
	void setY(float _y) { y = _y; }
	void setZ(float _z) { z = _z; }

	//Important functions
	void clear()
	{
		x = y = z = 0.0;
	}

	void setUp(float a, float b, float c)
	{
		x = a;
		y = b;
		z = c;
	}
	vector3f addition(vector3f v)
	{
		float q, w, r;
		q = x + v.getX();
		w = y + v.getY();
		r = z + v.getZ();

		vector3f s;
		s.setUp(q, w, r);
		return s;
	}

	vector3f operator+(vector3f& v)
	{
		vector3f newVec;
		newVec.setUp(x + v.getX(), y + v.getY(), z + v.getZ());
		return newVec;
	}
	vector3f operator-(vector3f& v)
	{
		vector3f newVec;
		newVec.setUp(x - v.getX(), y - v.getY(), z - v.getZ());
		return newVec;
	}
	void operator +=(vector3f& v)
	{
		x += v.getX();
		y += v.getY();
		z += v.getZ();
	}
	vector3f operator / (float d)
	{
		return vector3f(x / d, y / d, z / d);
	}
	vector3f& operator /= (float d)
	{
		x /= d;
		y /= d;
		z /= d;
		return *this;
	}
	vector3f operator * (float d)
	{
		return vector3f(x * d, y * d, z * d);
	}
	vector3f& operator= (vector3f v)
	{
		x = v.getX();
		y = v.getY();
		z = v.getZ();

		return *this;
	}

	float length()
	{
		return sqrtf(x*x + y * y + z * z);
	}
	float lengthSquare()
	{
		return x * x + y * y + z * z;
	}

	void makeUnit()
	{
		float s = length();
		if (s != 0)
		{
			x /= s;
			y /= s;
			z /= s;
		}

	}
	vector3f returnUnit()
	{
		float s = length();
		if (s != 0)
			return vector3f(x / s, y / s, z / s);
		else
			return vector3f(0, 0, 0);
	}

	void print()
	{
		cout << "x is " << x << " y is " << y << " z is " << z << "\n";
	}

};
