#pragma once

#include "Eigen/Eigen"
#include <algorithm>
#include "Shader.h"
#include "Triangle.h"
#include <optional>

#define MY_PI 3.141592

namespace Render 
{
	//buffer 顺序控制
	enum class Buffers {
		Color = 1,
		Depth = 2
	};

	inline Buffers operator|(Buffers a, Buffers b)
	{
		return Buffers((int)a | (int)b);
	}

	inline Buffers operator&(Buffers a, Buffers b)
	{
		return Buffers((int)a & (int)b);
	}

	enum class RenderType {
		Line,
		Triangle
	};

	struct VerBuffer { //顶点缓冲区
		int VB_id = 0;
	};
	struct IndBuffer { //索引缓冲区
		int IB_id = 0;
	};
	struct ColorBuffer {
		int CB_id = 0;
	};

	class Renderer 
	{
	public:
		Renderer() {}
		Renderer(int width, int height);
		VerBuffer LoadVertex(const std::vector<Eigen::Vector3f>& vertices);
		IndBuffer LoadIndex(const std::vector<Eigen::Vector3i>& indices);
		ColorBuffer LoadColor(const std::vector<Eigen::Vector3f>& colors);
		ColorBuffer LoadNormal(const std::vector<Eigen::Vector3f>& normals);

		//设置变换矩阵
		void set_model(const Eigen::Matrix4f& m);
		void set_view(const Eigen::Matrix4f& v);
		void set_projection(const Eigen::Matrix4f& p);


		void set_vertex_shader(std::function<Eigen::Vector3f(vertexShaderLoad)> vert_shader);
		void set_fragment_shader(std::function<Eigen::Vector3f(fragmentShaderLoad)> frag_shader);

		void set_pixel(const Eigen::Vector2i& point, const Eigen::Vector3f& color);

		virtual void clear(Buffers buff);

		virtual void draw(std::vector<Triangle*>& TriangleList);

		virtual void rstTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& world_pos);

		Eigen::Matrix4f model;
		Eigen::Matrix4f view;
		Eigen::Matrix4f projection;

		int normal_id = -1;

		std::map<int, std::vector<Eigen::Vector3f>> VerBuf;
		std::map<int, std::vector<Eigen::Vector3i>> IndBuf;
		std::map<int, std::vector<Eigen::Vector3f>> ColBuf;
		std::map<int, std::vector<Eigen::Vector3f>> NorBuf;

		bool haveTex = false;

		std::function<Eigen::Vector3f(fragmentShaderLoad)> fragmentShader;
		std::function<Eigen::Vector3f(vertexShaderLoad)> vertexShader;

		unsigned char* frameBuffer;
		virtual unsigned char* GetFrameBuffer() { return frameBuffer; }
		std::vector<float> depthBuffer;
		int GetIndex(int x, int y);

		int width, height;

		int NextId = 0;
		int GetNextId() { return NextId++; }


		
		virtual void print() {};
	};




	
}
struct ScreenPoint {
	ScreenPoint() : x(0), y(0), z(std::numeric_limits<float>::infinity()) {}
	ScreenPoint(int a, int b, float c) : x(a), y(b), z(c) {}
	int x, y;
	float z; //深度值
};

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Eigen::Vector4f* v) {
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

