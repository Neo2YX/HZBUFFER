#include "HZBuffer.h"
#include <iostream>

static bool isInTriangle(int x, int y, const Eigen::Vector4f* vertices) {
	Eigen::Vector3f v[3];
	for (int i = 0; i < 3; i++)
		v[i] = { vertices[i].x(),vertices[i].y(), 1.0 };
	Eigen::Vector3f f0, f1, f2;
	f0 = v[1].cross(v[0]);
	f1 = v[2].cross(v[1]);
	f2 = v[0].cross(v[2]);
	Eigen::Vector3f p(x, y, 1.);
	if ((p.dot(f0) * f0.dot(v[2]) > 0) && (p.dot(f1) * f1.dot(v[0]) > 0) && (p.dot(f2) * f2.dot(v[1]) > 0))
		return true;
	return false;
}

static auto ToVec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

Render::HZBuffer::HZBuffer(int w, int h)
{
	frameBuffer = (unsigned char*)malloc(sizeof(unsigned char) * 4 * w * h);
	width = w;
	height = h;

	while (w)
	{
		float* ZBuff = (float*)malloc(w * w * sizeof(float));
		depthBuffers.push_back(ZBuff);
		bufferSize.push_back(std::make_pair(w * w, w));
		w /= 2;
	}
	bufferHeight = depthBuffers.size();
}

void Render::HZBuffer::clear(Buffers buff)
{
	if ((buff & Render::Buffers::Color) == Render::Buffers::Color)
	{
		for (int ind = 0; ind < width * height; ind++) {
			frameBuffer[ind * 4] = 0;
			frameBuffer[ind * 4 + 1] = 0;
			frameBuffer[ind * 4 + 2] = 0;
			frameBuffer[ind * 4 + 3] = 255;
		}
	}
	if ((buff & Render::Buffers::Depth) == Render::Buffers::Depth)
	{
		for (int i = 0; i < depthBuffers.size(); ++i)
		{
			float* b = depthBuffers[i];
			for (int j = 0; j < bufferSize[i].first; ++j)
			{
				b[j] = std::numeric_limits<float>::infinity();
				
			}
		}
	}
}

static bool IsHeight(int h, int xmax, int xmin, int ymax, int ymin)
{
	xmax /= pow(2.0, h + 1);
	xmin /= pow(2.0, h + 1);
	ymax /= pow(2.0, h + 1);
	ymin /= pow(2.0, h + 1);
	if ((xmax != xmin) || (ymax != ymin)) return false;
	return true;
}


static int GetHeight(int xmax, int xmin, int ymax, int ymin, int& x, int& y)
{
	int xh = 0, yh = 0;
	while (xmax != xmin) {
		xmax /= 2;
		xmin /= 2;
		xh++;
	}
	while (ymax != ymin) {
		ymax /= 2;
		ymin /= 2;
		yh++;
	}
	x = xmax;
	y = ymax;

	int height = 0;
	if (xh > yh)
	{
		height = xh;
		for (int i = 0; i < height - yh; ++i) {
			y /= 2;
		}
	}
	else
	{
		height = yh;
		for (int i = 0; i < height - xh; ++i) {
			x /= 2;
		}
	}
	return height;
}

float myMax(float z1, float z2, float z3, float z4)
{
	float maxV = z1;
	maxV = std::max(maxV, z2);
	maxV = std::max(maxV, z3);
	maxV = std::max(maxV, z4);
	return maxV;
}

void Render::HZBuffer::rstTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& viewPos)
{
	auto v = t.toVector4();
	int xmin = 0;
	int xmax = 0;
	int ymin = 0;
	int ymax = 0;
	float xminf = t.Vertex[0].x();
	float xmaxf = t.Vertex[0].x();
	float yminf = t.Vertex[0].y();
	float ymaxf = t.Vertex[0].y();
	float zmin = t.Vertex[0].z();
	float zmax = t.Vertex[0].z();

	for (int i = 0; i < 3; i++)
	{
		if (v[i].x() < xminf) xminf = t.Vertex[i].x();
		if (v[i].x() > xmaxf) xmaxf = t.Vertex[i].x();
		if (v[i].y() < yminf) yminf = t.Vertex[i].y();
		if (v[i].y() > ymaxf) ymaxf = t.Vertex[i].y();

		if (v[i].z() < zmin) zmin = t.Vertex[i].z();
	}
	xmin = xminf;
	xmax = xmaxf + 1;
	ymin = yminf;
	ymax = ymaxf + 1;

	int x, y;
	int height = GetHeight(xmax, xmin, ymax, ymin, x, y);
	if (zmin > depthBuffers[height][y * bufferSize[height].second + x])
	{
		num++; return;
	}
	if (v[0].y() > v[1].y()) v[0].swap(v[1]);
	if (v[1].y() > v[2].y()) v[1].swap(v[2]);
	if (v[0].y() > v[1].y()) v[0].swap(v[1]);


	Eigen::Vector2i vert[3];
	for (int i = 0; i < 3; i++) {
		vert[i].x() = v[i].x();
		vert[i].y() = v[i].y();
	}

	int th = vert[2].y() - vert[0].y() + 1;
	
	for (int j = vert[0].y(); j <= vert[1].y(); ++j)
	{
		int temph = vert[1].y() - vert[0].y() + 1;
		float step1 = (float)(j - vert[0].y()) / temph;
		float step2 = (float)(j - vert[0].y()) / th;
		Eigen::Vector2i v1;
		v1.x() = vert[0].x() + (vert[1].x() - vert[0].x()) * step1;
		v1.y() = vert[0].y() + (vert[1].y() - vert[0].y()) * step1;
		Eigen::Vector2i v2;
		v2.x() = vert[0].x() + (vert[2].x() - vert[0].x()) * step2;
		v2.y() = vert[0].y() + (vert[2].y() - vert[0].y()) * step2;
		float z1 = v[0].z() + (v[1].z() - v[0].z()) * step1;
		float z2 = v[0].z() + (v[2].z() - v[0].z()) * step2;
		if (v1.x() > v2.x()) {
			std::swap(z1, z2);
			v1.swap(v2);
		}
		float dz = (z2 - z1) / (v2.x() - v1.x());
		float z = z1;
		for (int i = v1.x(); i <= v2.x(); ++i)
		{
			if (z < depthBuffers[0][i + j * width])
			{
				auto [alpha, beta, gamma] = computeBarycentric2D(i + 0.5, j + 0.5, t.Vertex);
				depthBuffers[0][i + j * width] = z;
				auto interpolated_color = interpolate(alpha, beta, gamma, t.Color[0], t.Color[1], t.Color[2], 1);
				auto interpolated_normal = interpolate(alpha, beta, gamma, t.Normal[0], t.Normal[1], t.Normal[2], 1).normalized();
				auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, viewPos[0], viewPos[1], viewPos[2], 1);
				fragmentShaderLoad payload(interpolated_color, interpolated_normal);
				payload.viewPos = interpolated_shadingcoords;
				auto pixel_color = fragmentShader(payload);
				set_pixel(Eigen::Vector2i(i, j), pixel_color);

				updateHZBuffer(i, j, z);
			}
			z += dz;
		}
	}
	for (int j = vert[1].y(); j <= vert[2].y(); ++j)
	{
		int temph = vert[2].y() - vert[1].y() + 1;
		float step1 = (float)(j - vert[1].y()) / temph;
		float step2 = (float)(j - vert[0].y()) / th;
		Eigen::Vector2i v1;
		v1.x() = vert[1].x() + (vert[2].x() - vert[1].x()) * step1;
		v1.y() = vert[1].y() + (vert[2].y() - vert[1].y()) * step1;
		Eigen::Vector2i v2;
		v2.x() = vert[0].x() + (vert[2].x() - vert[0].x()) * step2;
		v2.y() = vert[0].y() + (vert[2].y() - vert[0].y()) * step2;
		float z1 = v[1].z() + (v[2].z() - v[1].z()) * step1;
		float z2 = v[0].z() + (v[2].z() - v[0].z()) * step2;
		if (v1.x() > v2.x()) {
			std::swap(z1, z2);
			v1.swap(v2);
		}
		float dz = (z2 - z1) / (v2.x() - v1.x());
		float z = z1;
		for (int i = v1.x(); i <= v2.x(); ++i)
		{
			if (z < depthBuffers[0][i + j * width])
			{
				auto [alpha, beta, gamma] = computeBarycentric2D(i + 0.5, j + 0.5, t.Vertex);
				depthBuffers[0][i + j * width] = z;
				auto interpolated_color = interpolate(alpha, beta, gamma, t.Color[0], t.Color[1], t.Color[2], 1);
				auto interpolated_normal = interpolate(alpha, beta, gamma, t.Normal[0], t.Normal[1], t.Normal[2], 1).normalized();
				auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, viewPos[0], viewPos[1], viewPos[2], 1);
				fragmentShaderLoad payload(interpolated_color, interpolated_normal);
				payload.viewPos = interpolated_shadingcoords;
				auto pixel_color = fragmentShader(payload);
				set_pixel(Eigen::Vector2i(i, j), pixel_color);
				updateHZBuffer(i, j, z);
				
			}
			z += dz;
		}
	}
	
}

void Render::HZBuffer::updateHZBuffer(int x, int y, float z)
{
	for (int i = 1; i < bufferHeight; ++i)
	{
		if (x % 2 == 1) x--;
		if (y % 2 == 1) y--;

		depthBuffers[i][(y / 2) * bufferSize[i].second + x / 2] = myMax(
			depthBuffers[i - 1][y * bufferSize[i - 1].second + x],
			depthBuffers[i - 1][y * bufferSize[i - 1].second + x + 1],
			depthBuffers[i - 1][(y + 1) * bufferSize[i - 1].second + x],
			depthBuffers[i - 1][(y + 1) * bufferSize[i - 1].second + x + 1]
		);
		x /= 2;
		y /= 2;
	}
}

void Render::HZBuffer::print()
{
	std::cout << "消隐数量：" << num;
}


void Render::OctHZBuffer::draw(std::vector<Triangle*>& TriangleList)
{
	octree.Init(TriangleList, model, view, projection);

	drawTreeNode(octree.root, 1);

	octree.DeleteTree();
}

void Render::OctHZBuffer::drawTreeNode(OctreeNode* TNode, int depth)
{
	if (TNode == nullptr) return;

	int mxBox = 0.5 * width * (TNode->BoundingBox.xmin + 1.f);
	int myBox = 0.5 * height * (TNode->BoundingBox.ymin + 1.f);


	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;
	float zmin = TNode->BoundingBox.zmin * f1 + f2;

	for (int i = 0; i < bufferHeight - depth; ++i)
	{
		mxBox /= 2;
		myBox /= 2;
	}
	int h = bufferHeight - depth;
	if (zmin > depthBuffers[h][myBox * bufferSize[h].second + mxBox])
	{
		num2++;
		return; 
	}

	for (int i = 0; i < 8; ++i) drawTreeNode(TNode->Children[i], depth + 1);

	auto& TriangleList = TNode->TList;
	for (int i = 0; i < TriangleList.size(); ++i)
	{
		rstTriangle(TriangleList[i], TNode->viewPos[i]);
	}
}

void Render::OctHZBuffer::print()
{
	std::cout << "八叉树消隐个数：" << num2 << std::endl;
	std::cout << "层次ZBuffer消隐个数：" << num << std::endl;
}


