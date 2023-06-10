#pragma once
#include "Renderer.h"

struct TriangleTableNode;
struct EdgeTableNode;
struct DEdgeTableNode; //活化边表



namespace Render {
	class ScanLineZBuffer : public Renderer
	{
	public:
		std::vector<std::vector<TriangleTableNode>> TriangleTable;

		ScanLineZBuffer(int w, int h) : Renderer(w, h) {}

		virtual void draw(std::vector<Triangle*>& TriangleList) override;

		void ConstructTriangleTable(std::vector<Triangle*>& TriangleList);
	};

}

struct TriangleTableNode {
	float a, b, c, d;  //多边形所在平面的方程系数 ax+by+cz+d=0
	int id; //多边形的编号
	int dy; //多边形跨越的扫描线数目
	EdgeTableNode* edgeTable;

	Eigen::Vector3f Color;
};

struct EdgeTableNode {
	int x; //边上端点的x坐标
	float dx; //相邻两条扫描线交点的x坐标差（-1/k）
	int dy; //边跨越的扫描线数目
	float z; //边上端点的深度值
};

struct DEdgeTableNode {
	float leftX, leftDX;
	int leftDY;

	float rightX, rightDX;
	int rightDY;

	float leftZ;
	float dzx, dzy; //沿X，Y方向的深度增量

	TriangleTableNode* triangleNode;

	Eigen::Vector3f Color;
};

