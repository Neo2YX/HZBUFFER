#include "Renderer.h"



Render::VerBuffer Render::Renderer::LoadVertex(const std::vector<Eigen::Vector3f>& vertices)
{
	auto id = GetNextId();
	VerBuf.emplace(id, vertices);

	return { id };
}

Render::IndBuffer Render::Renderer::LoadIndex(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = GetNextId();
	IndBuf.emplace(id, indices);

	return { id };
}

Render::ColorBuffer Render::Renderer::LoadColor(const std::vector<Eigen::Vector3f>& colors)
{
	auto id = GetNextId();
	ColBuf.emplace(id, colors);

	return { id };
}

Render::ColorBuffer Render::Renderer::LoadNormal(const std::vector<Eigen::Vector3f>& normals)
{
	auto id = GetNextId();
	NorBuf.emplace(id, normals);

	return { id };
}



auto ToVec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}



bool isInTriangle(int x, int y, const Eigen::Vector4f* vertices) {
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



void Render::Renderer::draw(std::vector<Triangle*>& TriangleList)
{
	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mvp = projection * view * model;
	for (const auto& t : TriangleList)
	{
		Triangle Tri = *t;
		
		std::array<Eigen::Vector4f, 3> mm{
			(view * model * t->Vertex[0]),
			(view * model * t->Vertex[1]),
			(view * model * t->Vertex[2]),
		};

		std::array<Eigen::Vector3f, 3> viewspacePos;

		std::transform(mm.begin(), mm.end(), viewspacePos.begin(), [](auto& v) {
			return v.template head<3>();
			});

		Eigen::Vector4f v[] = {
				mvp * t->Vertex[0],
				mvp * t->Vertex[1],
				mvp * t->Vertex[2]
		};

		for (auto& vec : v) {
			vec.x() /= vec.w();
			vec.y() /= vec.w();
			vec.z() /= vec.w();
		}

		Eigen::Matrix4f invTrans = (view * model).inverse().transpose();
		Eigen::Vector4f n[] = {
				invTrans * ToVec4(t->Normal[0], 0.0f),
				invTrans * ToVec4(t->Normal[1], 0.0f),
				invTrans * ToVec4(t->Normal[2], 0.0f)
		};

		for (auto& vert : v)
		{
			vert.x() = 0.5 * width * (vert.x() + 1.0);
			vert.y() = 0.5 * height * (vert.y() + 1.0);
			vert.z() = vert.z() * f1 + f2;
		}

		for (int i = 0; i < 3; ++i)
		{
			Tri.setVertex(i, v[i]);
		}

		for (int i = 0; i < 3; ++i)
		{
			Tri.setNormal(i, n[i].head<3>());
		}

		Tri.setColor(0, 148, 121.0, 92.0);
		Tri.setColor(1, 148, 121.0, 92.0);
		Tri.setColor(2, 148, 121.0, 92.0);

		rstTriangle(Tri, viewspacePos);
	}
}



static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
	auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
	auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

	u /= weight;
	v /= weight;

	return Eigen::Vector2f(u, v);
}

void Render::Renderer::rstTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& viewPos)
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
	for (int i = 0; i < 3; i++)
	{
		if (v[i].x() < xminf) xminf = t.Vertex[i].x();
		if (v[i].x() > xmaxf) xmaxf = t.Vertex[i].x();
		if (v[i].y() < yminf) yminf = t.Vertex[i].y();
		if (v[i].y() > ymaxf) ymaxf = t.Vertex[i].y();
	}
	xmin = xminf;
	xmax = xmaxf + 1;
	ymin = yminf;
	ymax = ymaxf + 1;

	for (int i = xmin; i < xmax; i++)
	{
		for (int j = ymin; j < ymax; j++)
		{
			if (isInTriangle(i + 0.5, j + 0.5, t.Vertex))
			{
				//Depth interpolated
				auto [alpha, beta, gamma] = computeBarycentric2D(i + 0.5, j + 0.5, t.Vertex);

				float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
				float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
				zp *= Z;

				if (zp < depthBuffer[GetIndex(i, j)])
				{
					depthBuffer[GetIndex(i, j)] = zp;
					auto interpolated_color = interpolate(alpha, beta, gamma, t.Color[0], t.Color[1], t.Color[2], 1);
					auto interpolated_normal = interpolate(alpha, beta, gamma, t.Normal[0], t.Normal[1], t.Normal[2], 1).normalized();
					auto interpolated_shadingcoords = interpolate(alpha, beta, gamma, viewPos[0], viewPos[1], viewPos[2], 1);
					fragmentShaderLoad payload(interpolated_color, interpolated_normal);
					payload.viewPos = interpolated_shadingcoords;
					auto pixel_color = fragmentShader(payload);
					set_pixel(Eigen::Vector2i(i, j), pixel_color);
				}
			}

		}
	}
}

void Render::Renderer::set_model(const Eigen::Matrix4f& m)
{
	model = m;
}

void Render::Renderer::set_view(const Eigen::Matrix4f& v)
{
	view = v;
}

void Render::Renderer::set_projection(const Eigen::Matrix4f& p)
{
	projection = p;
}

void Render::Renderer::set_vertex_shader(std::function<Eigen::Vector3f(vertexShaderLoad)> vert_shader)
{
	vertexShader = vert_shader;
}

void Render::Renderer::set_fragment_shader(std::function<Eigen::Vector3f(fragmentShaderLoad)> frag_shader)
{
	fragmentShader = frag_shader;
}

void Render::Renderer::clear(Buffers buff)
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
		std::fill(depthBuffer.begin(), depthBuffer.end(), std::numeric_limits<float>::infinity());
	}
}

Render::Renderer::Renderer(int w, int h)
{
	frameBuffer = (unsigned char*)malloc(sizeof(unsigned char) * 4 * w * h);
	depthBuffer.resize(w * h);
	width = w;
	height = h;
}

void Render::Renderer::set_pixel(const Eigen::Vector2i& point, const Eigen::Vector3f& color)
{
	auto ind = point.y() * width + point.x();
	frameBuffer[ind*4] = (unsigned char)(int)color[0];
	frameBuffer[ind * 4 + 1] = (unsigned char)(int)color[1];
	frameBuffer[ind * 4 + 2] = (unsigned char)(int)color[2];
	frameBuffer[ind * 4 + 3] = 255;
}


int Render::Renderer::GetIndex(int x, int y)
{
	return (height - 1 - y) * width + x;
}




/**************depthbuffers***************/
/*

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

static float myMax(float z1, float z2, float z3, float z4)
{
	float maxV = z1;
	maxV = std::max(maxV, z2);
	maxV = std::max(maxV, z3);
	maxV = std::max(maxV, z4);
	return maxV;
}

void Render::Renderer::rstTriangle(const Triangle& t, const std::array<Eigen::Vector3f, 3>& viewPos)
{
	auto v = t.toVector4();
	for (auto& vert : v)
	{
		if (vert.x() < 0 || vert.x() >= width || vert.y() < 0 || vert.y() >= height)
			return;
	}
	int xmin = 0;
	int xmax = 0;
	int ymin = 0;
	int ymax = 0;
	float xminf = t.Vertex[0].x();
	float xmaxf = t.Vertex[0].x();
	float yminf = t.Vertex[0].y();
	float ymaxf = t.Vertex[0].y();
	float zmin = t.Vertex[0].z();

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
	if (zmin > depthBuffers[height][y * bufferSize[height].second + x]) return;
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


				for (int h = 1; h < bufferHeight; ++h)
				{
					int  ti = i, tj = j;

					if (ti % 2 == 1) ti--;
					if (tj % 2 == 1) tj--;

					depthBuffers[h][(tj / 2) * bufferSize[h].second + ti / 2] = myMax(
						depthBuffers[h - 1][tj * bufferSize[h - 1].second + ti],
						depthBuffers[h - 1][tj * bufferSize[h - 1].second + ti + 1],
						depthBuffers[h - 1][(tj + 1) * bufferSize[h - 1].second + ti],
						depthBuffers[h - 1][(tj + 1) * bufferSize[h - 1].second + ti + 1]
					);

					x /= 2;
					y /= 2;
				}
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


				for (int h = 1; h < bufferHeight; ++h)
				{
					int  ti = i, tj = j;

					if (ti % 2 == 1) ti--;
					if (tj % 2 == 1) tj--;

					depthBuffers[h][(tj / 2) * bufferSize[h].second + ti / 2] = myMax(
						depthBuffers[h - 1][tj * bufferSize[h - 1].second + ti],
						depthBuffers[h - 1][tj * bufferSize[h - 1].second + ti + 1],
						depthBuffers[h - 1][(tj + 1) * bufferSize[h - 1].second + ti],
						depthBuffers[h - 1][(tj + 1) * bufferSize[h - 1].second + ti + 1]
					);

					x /= 2;
					y /= 2;
				}
			}
			z += dz;
		}
	}
}

Render::Renderer::Renderer(int w, int h)
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

void Render::Renderer::clearDepth()
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



*/