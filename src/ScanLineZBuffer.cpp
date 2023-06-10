#include "ScanLineZBuffer.h"


auto toVec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static Eigen::Vector3f Interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}




void Render::ScanLineZBuffer::ConstructTriangleTable(std::vector<Triangle*>& TriangleList)
{
	TriangleTable.clear();
	TriangleTable.resize(height);

	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mv = view * model; //将顶点转换为视点坐标
	Eigen::Matrix4f mvp = projection * view * model; // 将顶点转换为标准设备化坐标

	Eigen::Vector4f viewDirction(0.f, 0.f, 1.f, 0.f);
	//此处将法线视为三角面片的顶点法线（默认三点法线相同
	int TriID = -1;
	for (auto t : TriangleList)
	{
		TriID++;//三角形编号
		//法向量变换为视点坐标
		Eigen::Matrix4f invTrans = (view * model).inverse().transpose();
		Eigen::Vector4f n[] = {
				invTrans * toVec4(t->Normal[0], 0.0f),
				invTrans * toVec4(t->Normal[1], 0.0f),
				invTrans * toVec4(t->Normal[2], 0.0f)
		};
		auto interpolated_normal = Interpolate(1, 1, 1, n[0].head(3), n[1].head(3), n[2].head(3), 3).normalized();

		if (interpolated_normal.dot(viewDirction.head(3)) <= 0) continue; //背部剔除

		//顶点坐标变换为世界坐标
		Eigen::Vector4f vW[] = {
				model * t->Vertex[0],
				model * t->Vertex[1],
				model * t->Vertex[2]
		};
		//齐次化
		for (auto& vec : vW) {
			vec.x() /= vec.w();
			vec.y() /= vec.w();
			vec.z() /= vec.w();
		}

		TriangleTableNode TNode;
		
		TNode.id = TriID;

		//顶点坐标变换为视点坐标
		std::array<Eigen::Vector4f, 3> vV = {
				mv * t->Vertex[0],
				mv * t->Vertex[1],
				mv * t->Vertex[2]
		};
		std::array<Eigen::Vector3f, 3> viewspacePos;

		std::transform(vV.begin(), vV.end(), viewspacePos.begin(), [](auto& v) {
			return v.template head<3>();
			});

		
		//将顶点变为标准设备化坐标
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

		

		//计算三角形的光照颜色（顶点Phong模型
		
		auto interpolated_shadingcoords = Interpolate(1, 1, 1, viewspacePos[0], viewspacePos[1], viewspacePos[2], 3);
		fragmentShaderLoad payload(Eigen::Vector3f(0.58f, 0.4745f, 0.36f), interpolated_normal);
		payload.viewPos = interpolated_shadingcoords;
		TNode.Color = fragmentShader(payload);


		TNode.a = interpolated_normal.x();
		TNode.b = interpolated_normal.y();
		TNode.c = interpolated_normal.z();
		TNode.d = -interpolated_normal.dot(vV[0].head(3));
		
		// 变为屏幕坐标
		ScreenPoint vert[3];
		for (int i = 0; i < 3; i++) {
			vert[i].x = (int)(0.5 * width * (v[i].x() + 1.0));
			vert[i].y = (int)(0.5 * height * (v[i].y() + 1.0));
			vert[i].z = v[i].z() * f1 + f2;
		}

		//对顶点进行排序
		if (vert[0].y > vert[1].y) std::swap(vert[0], vert[1]);
		if (vert[1].y > vert[2].y) std::swap(vert[1], vert[2]);
		if (vert[0].y > vert[1].y) std::swap(vert[0], vert[1]);

		TNode.dy = vert[2].y - vert[0].y;

		TNode.edgeTable = new EdgeTableNode[3];
		EdgeTableNode e20;
		e20.x = vert[2].x;
		e20.z = vert[2].z;
		e20.dy = vert[2].y - vert[0].y;
		e20.dx = -(float)(vert[2].x - vert[0].x) / e20.dy;
		TNode.edgeTable[0] = e20;
		EdgeTableNode e21;
		e21.x = vert[2].x;
		e21.z = vert[2].z;
		e21.dy = vert[2].y - vert[1].y;
		e21.dx = -(float)(vert[2].x - vert[1].x) / e21.dy;
		TNode.edgeTable[1] = e21;
		EdgeTableNode e10;
		e10.x = vert[1].x;
		e10.z = vert[1].z;
		e10.dy = vert[1].y - vert[0].y;
		e10.dx = -(float)(vert[1].x - vert[0].x) / e10.dy;
		TNode.edgeTable[2] = e10;

		TriangleTable[vert[2].y].push_back(TNode);
	}
}

void Render::ScanLineZBuffer::draw(std::vector<Triangle*>& TriangleList)
{
	ConstructTriangleTable(TriangleList);

	std::list<DEdgeTableNode> DEdgeTable;

	for (int i = height - 1; i >= 0; --i)
	{
		for (TriangleTableNode& t : TriangleTable[i])
		{
			DEdgeTableNode ENode;
			ENode.dzx = -t.a / t.c;
			ENode.dzy = t.b / t.c;
			ENode.Color = t.Color;
			ENode.triangleNode = &t;

			if (t.edgeTable[1].dx > t.edgeTable[0].dx)
			{
				ENode.leftX = t.edgeTable[0].x;
				ENode.leftDX = t.edgeTable[0].dx;
				ENode.leftDY = t.edgeTable[0].dy;
				ENode.rightX = t.edgeTable[1].x;
				ENode.rightDX = t.edgeTable[1].dx;
				ENode.rightDY = t.edgeTable[1].dy;

				ENode.leftZ = t.edgeTable[0].z;
			}
			else {
				ENode.leftX = t.edgeTable[1].x;
				ENode.leftDX = t.edgeTable[1].dx;
				ENode.leftDY = t.edgeTable[1].dy;
				ENode.rightX = t.edgeTable[0].x;
				ENode.rightDX = t.edgeTable[0].dx;
				ENode.rightDY = t.edgeTable[0].dy;

				ENode.leftZ = t.edgeTable[1].z;
			}

			DEdgeTable.push_back(ENode);
		}

		for (DEdgeTableNode& eNode : DEdgeTable)
		{
			float depthVal = eNode.leftZ;
			for (int j = std::max((int)eNode.leftX, 0); j <= std::min(int(eNode.rightX) - 1, width - 1); j++)
			{
				if (depthVal < depthBuffer[GetIndex(j, i)])
				{
					set_pixel(Eigen::Vector2i(j, i), eNode.Color);
					depthBuffer[GetIndex(j, i)] = depthVal;
				}
				depthVal += eNode.dzx;
			}
		}

		for (auto itr = DEdgeTable.begin(); itr != DEdgeTable.end();)
		{
			itr->leftDY--;
			itr->rightDY--;

			if (itr->leftDY < 0 && itr->rightDY < 0)
			{
				itr = DEdgeTable.erase(itr);
			}
			else {
				TriangleTableNode* TNode = itr->triangleNode;

				if (itr->leftDY < 0) {
					itr->leftX = TNode->edgeTable[2].x;
					itr->leftDX = TNode->edgeTable[2].dx;
					itr->leftDY = TNode->edgeTable[2].dy - 1;
				}
				if (itr->rightDY < 0) {
					itr->rightX = TNode->edgeTable[2].x;
					itr->rightDX = TNode->edgeTable[2].dx;
					itr->rightDY = TNode->edgeTable[2].dy - 1;
				}

				itr->leftX += itr->leftDX;
				itr->rightX += itr->rightDX;
				itr->leftZ += itr->dzy + itr->dzx * itr->leftDX;
				itr++;
			}
		}
	}
}


