#pragma once
#include "Renderer.h"
#include "Octree.h"

namespace Render {
	/*******************简单模式*******************/
	class HZBuffer : public Renderer
	{
	public:
		std::vector<float*> depthBuffers;
		std::vector<std::pair<int, int>> bufferSize;
		int bufferHeight = 0;
		int num = 0;

		HZBuffer(int w, int h);

		virtual void clear(Buffers buff) override;
		virtual void rstTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos) override;
		void updateHZBuffer(int x, int y, float z);

		virtual void print() override;
		
	};

	/*******************完整模式**********************/
	class OctHZBuffer : public HZBuffer
	{
	public:
		Octree octree;
		int num2 = 0;

		OctHZBuffer(int w, int h) : HZBuffer(w, h) {}

		virtual void draw(std::vector<Triangle*>& TriangleList) override;
		void drawTreeNode(OctreeNode* TNode, int depth);
		virtual void print() override;
	};


}