#pragma once

#include "Eigen/Eigen"


class Triangle {
public:
	Eigen::Vector4f Vertex[3];
	Eigen::Vector3f Color[3];
	Eigen::Vector3f Normal[3];

	Triangle();
	Triangle(Triangle* t);

	Eigen::Vector4f a() const { return Vertex[0]; }
	Eigen::Vector4f b() const { return Vertex[1]; }
	Eigen::Vector4f c() const { return Vertex[2]; }

	void setVertex(int ind, Eigen::Vector4f v);
	void setNormal(int ind, Eigen::Vector3f n);
	void setColor(int ind, float r, float g, float b); 

	void setNormals(const std::array<Eigen::Vector3f, 3>& normals);
	void setColors(const std::array<Eigen::Vector3f, 3>& colors);
	std::array<Eigen::Vector4f, 3> toVector4() const;
};