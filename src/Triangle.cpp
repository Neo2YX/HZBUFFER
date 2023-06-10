#include "Triangle.h"
#include <algorithm>
#include <array>
#include <iostream>

Triangle::Triangle() {
	Vertex[0] << 0, 0, 0, 1;
	Vertex[1] << 0, 0, 0, 1;
	Vertex[2] << 0, 0, 0, 1;

	Color[0] << 0.0, 0.0, 0.0;
	Color[1] << 0.0, 0.0, 0.0;
	Color[2] << 0.0, 0.0, 0.0;

}

Triangle::Triangle(Triangle* t)
{
	for (int i = 0; i < 3; ++i)
	{
		Vertex[i] = t->Vertex[i];
		Normal[i] = t->Normal[i];
		Color[i] = t->Color[i];
	}
}

void Triangle::setVertex(int ind, Eigen::Vector4f v)
{
	Vertex[ind] = v;
}

void Triangle::setNormal(int ind, Eigen::Vector3f n)
{
	Normal[ind] = n;
}

void Triangle::setColor(int ind, float r, float g, float b)
{
	if ((r < 0.0) || (r > 255.) ||
		(g < 0.0) || (g > 255.) ||
		(b < 0.0) || (b > 255.)) {
		std::cout << "[ERROR]: invalid color value" << std::endl;
		exit(-1);
	}
	Color[ind][0] = (float)r / 255.;
	Color[ind][1] = (float)g / 255.;
	Color[ind][2] = (float)b / 255.;
}


void Triangle::setNormals(const std::array<Eigen::Vector3f, 3>& normals)
{
	Normal[0] = normals[0];
	Normal[1] = normals[1];
	Normal[2] = normals[2];
}

void Triangle::setColors(const std::array<Eigen::Vector3f, 3>& colors)
{
	setColor(0, colors[0][0], colors[0][1], colors[0][2]);
	setColor(1, colors[1][0], colors[1][1], colors[1][2]);
	setColor(2, colors[2][0], colors[2][1], colors[2][2]);
}

std::array<Eigen::Vector4f, 3> Triangle::toVector4() const
{
	std::array<Eigen::Vector4f, 3> res;
	std::transform(std::begin(Vertex), std::end(Vertex), res.begin(), [](auto& vec) { return Eigen::Vector4f(vec.x(), vec.y(), vec.z(), 1.f); });
	return res;
}

