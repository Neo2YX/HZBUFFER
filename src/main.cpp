#include "Window.h"
#include <iostream>
#include "Triangle.h"
#include "Shaders.h"

#include "Dependencies/OBJ_Loader.h"
#include <ctime>

#include "CmdUI.h"

inline double DEG2RAD(double deg) { return deg * MY_PI / 180; }

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
	Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, -eye_pos[0],
		0, 1, 0, -eye_pos[1],
		0, 0, -1, -eye_pos[2],
		0, 0, 0, 1;

	Eigen::Matrix4f inverseZ;
	inverseZ << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 1;

	view = inverseZ * translate * view;

	return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
	Eigen::Matrix4f rotation;
	angle = angle * MY_PI / 180.f;
	rotation << cos(angle), 0, sin(angle), 0,
		0, 1, 0, 0,
		-sin(angle), 0, cos(angle), 0,
		0, 0, 0, 1;

	Eigen::Matrix4f scale;
	scale << 2.5, 0, 0, 0,
		0, 2.5, 0, 0,
		0, 0, 2.5, 0,
		0, 0, 0, 1;

	Eigen::Matrix4f translate;
	translate << 1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1;

	return translate * rotation * scale;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
	float zNear, float zFar)
{
	Eigen::Matrix4f projection;
	float top = -tan(DEG2RAD(eye_fov / 2.0f) * abs(zNear));
	float right = top * aspect_ratio;

	projection << zNear / right, 0, 0, 0,
		0, zNear / top, 0, 0,
		0, 0, (zNear + zFar) / (zNear - zFar), (2 * zNear * zFar) / (zFar - zNear),
		0, 0, 1, 0;
	return projection;
}

UI::ModelInfo modelInfo[4] = {
		{Eigen::Vector3f(0,2.5,8), 180},
		{Eigen::Vector3f(0,0,10), 150},
		{Eigen::Vector3f(0,0.2,1), 30},
		{Eigen::Vector3f(0,0,10), 0}
};

Render::Renderer renderer(WIDTH, HEIGHT);
Render::ScanLineZBuffer scanLineZbuffer(WIDTH, HEIGHT);
Render::HZBuffer hZbuffer(WIDTH, HEIGHT);
Render::OctHZBuffer octhZbuffer(WIDTH, HEIGHT);

clock_t start, end;

int main()
{

	std::vector<Triangle*> TriangleList;

	objl::Loader Loader;
	auto [ZBuffType, modelType] = UI::drawMainUI();

	//加载模型
	std::cout << "开始加载模型" << std::endl;
	int verticesNum = 0;
	start = clock();
	bool loadout;
	UI::MODEL model;
	switch (modelType)
	{
	case 1:
		loadout = Loader.LoadFile("models/spot_triangulated_good.obj");
		model = UI::MODEL::SPOT;
		if (!loadout) {
			std::cout << "加载模型失败，请确保程序运行位置与models文件夹在同一位置并检查模型文件" << std::endl;
			std::cout << "输入任意字符退出程序：";
			char c;
			std::cin >> c;
			return 0;
		}
		break;
	case 2:
		loadout = Loader.LoadFile("models/bunny.obj");
		model = UI::BUNNY;
		if (!loadout) {
			std::cout << "加载模型失败，请确保程序运行位置与models文件夹在同一位置并检查模型文件" << std::endl;
			std::cout << "输入任意字符退出程序：";
			char c;
			std::cin >> c;
			return 0;
		}
		break;
	case 3:
		loadout = Loader.LoadFile("models/helmet.obj");
		model = UI::HELMET;
		if (!loadout) {
			std::cout << "加载模型失败，请确保程序运行位置与models文件夹在同一位置并检查模型文件" << std::endl;
			std::cout << "输入任意字符退出程序：";
			char c;
			std::cin >> c;
			return 0;
		}
		break;
	case 4:
		loadout = Loader.LoadFile("models/miku.obj");
		model = UI::MIKU;
		if (!loadout) {
			std::cout << "加载模型失败，请确保程序运行位置与models文件夹在同一位置并检查模型文件" << std::endl;
			std::cout << "输入任意字符退出程序：";
			char c;
			std::cin >> c;
			return 0;
		}
		break;
	default:
		loadout = Loader.LoadFile("models/spot_triangulated_good.obj");
		model = UI::SPOT;
		if (!loadout) {
			std::cout << "加载模型失败，请确保程序运行位置与models文件夹在同一位置并检查模型文件" << std::endl;
			std::cout << "输入任意字符退出程序：";
			char c;
			std::cin >> c;
			return 0;
		}
		break;
	}
	for (auto mesh : Loader.LoadedMeshes)
	{
		verticesNum += mesh.Vertices.size();
		for (int i = 0; i < mesh.Vertices.size(); i += 3)
		{
			Triangle* t = new Triangle();
			for (int j = 0; j < 3; j++) {
				t->setVertex(j, Eigen::Vector4f(mesh.Vertices[i + j].Position.X, mesh.Vertices[i + j].Position.Y, mesh.Vertices[i + j].Position.Z, 1.0));
				t->setNormal(j, Eigen::Vector3f(mesh.Vertices[i + j].Normal.X, mesh.Vertices[i + j].Normal.Y, mesh.Vertices[i + j].Normal.Z));
			}
			TriangleList.push_back(t);
		}
	}
	end = clock();
	std::cout << "加载结束，用时：" << (float)(end - start) / CLOCKS_PER_SEC << "s" << std::endl << std::endl << std::endl;
	std::cout << "模型顶点数：" << verticesNum << "   三角面片数：" << TriangleList.size() << std::endl;
	InitWindow(WIDTH,HEIGHT);
	std::function<Eigen::Vector3f(fragmentShaderLoad)> active_shader = phong_fragment_shader;
	while (1) {
		Render::Renderer* r;
		switch (ZBuffType)
		{
		case 1:
			r = &renderer;
			break;
		case 2:
			r = &scanLineZbuffer;
			break;
		case 3:
			r = &hZbuffer;
			break;
		case 4:
			r = &octhZbuffer;
			break;
		default:
			r = &renderer;
			break;
		}

		r->set_vertex_shader(vertex_shader);
		r->set_fragment_shader(active_shader);

		start = clock();
		while (myWindow->window_close_ == false)
		{

			r->clear(Render::Buffers::Color);
			r->clear(Render::Buffers::Depth);

			r->set_model(get_model_matrix(modelInfo[model].angle));
			r->set_view(get_view_matrix(modelInfo[model].EyePos));
			r->set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

			r->draw(TriangleList);

			DrawWindow(r->GetFrameBuffer());
			//r->print();

			end = clock();
			std::cout << "绘制10帧用时：" << (float)(end - start) / CLOCKS_PER_SEC * 1000 << "ms" << std::endl << std::endl << std::endl;
			break;
		}
		ZBuffType =  UI::drawMenuUI();
		if (!ZBuffType) break;
	}
	CloseWindow();

	return 0;
}