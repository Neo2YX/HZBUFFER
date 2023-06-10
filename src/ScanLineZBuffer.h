#pragma once
#include "Renderer.h"

struct TriangleTableNode;
struct EdgeTableNode;
struct DEdgeTableNode; //��߱�



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
	float a, b, c, d;  //���������ƽ��ķ���ϵ�� ax+by+cz+d=0
	int id; //����εı��
	int dy; //����ο�Խ��ɨ������Ŀ
	EdgeTableNode* edgeTable;

	Eigen::Vector3f Color;
};

struct EdgeTableNode {
	int x; //���϶˵��x����
	float dx; //��������ɨ���߽����x����-1/k��
	int dy; //�߿�Խ��ɨ������Ŀ
	float z; //���϶˵�����ֵ
};

struct DEdgeTableNode {
	float leftX, leftDX;
	int leftDY;

	float rightX, rightDX;
	int rightDY;

	float leftZ;
	float dzx, dzy; //��X��Y������������

	TriangleTableNode* triangleNode;

	Eigen::Vector3f Color;
};

