#pragma once
#include <float.h>
#include "Eigen/Eigen"
#include "Triangle.h"



struct AABB
{
	float xmin, xmax, ymin, ymax, zmin, zmax;

	AABB(float xmi, float xma, float ymi, float yma, float zmi, float zma) : xmin(xmi), xmax(xma), ymin(ymi), ymax(yma), zmin(zmi), zmax(zma) {}

	AABB() : xmin(-FLT_MAX), xmax(FLT_MAX), ymin(-FLT_MAX), ymax(FLT_MAX), zmin(-FLT_MAX), zmax(FLT_MAX) {}

	AABB(Eigen::Vector3f minV, Eigen::Vector3f maxV) : xmin(minV.x()), xmax(maxV.x()), ymin(minV.y()), ymax(maxV.y()), zmin(minV.z()), zmax(maxV.z()) {}

	bool IsMeshIn(Eigen::Vector4f* t)
	{
		for (int i = 0; i < 3; ++i)
		{
			if (t[0].x() > xmax || t[0].x() <= xmin) return false;
			if (t[0].y() > ymax || t[0].y() <= ymin) return false;
			if (t[0].z() > zmax || t[0].z() <= zmin) return false;
			return true;
		}
	}

	Eigen::Vector3f centerPoint()
	{
		return Eigen::Vector3f((xmax + xmin) / 2.f, (ymax + ymin) / 2.f, (zmax + zmin) / 2.f);
	}
	float c2e()
	{
		return (xmax - xmin) / 2.f;
	}

	float midZ()
	{
		return (zmax - zmin) / 2.f;
	}
};

struct OctreeNode
{
	AABB BoundingBox;
	std::vector<Triangle*> TList;
	std::vector< std::array<Eigen::Vector3f, 3>> viewPos;

	OctreeNode* Children[8];

	OctreeNode()
	{
		for (int i = 0; i < 8; ++i)
		{
			Children[i] = nullptr;
		}
	}
};

class Octree
{
public:
	OctreeNode* root = nullptr;

	Octree() {}
	~Octree() { DeleteTree(); }
	void Init(std::vector<Triangle*> TriangleList, Eigen::Matrix4f model, Eigen::Matrix4f view, Eigen::Matrix4f projection);

	void DeleteTree()
	{
		if (root) {
			delete root;
			root = nullptr;
		}
	}
};