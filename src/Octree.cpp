#include "Octree.h"
#define WIDTH 512
#define HEIGHT 512

static auto ToVec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Eigen::Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
	return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

void Octree::Init(std::vector<Triangle*> TriangleList, Eigen::Matrix4f model, Eigen::Matrix4f view, Eigen::Matrix4f projection)
{
	root = new OctreeNode;
	//root->BoundingBox = AABB(-1.f, 1.f, -1.f, 1.f, -0.986f, -0.98f);
	root->BoundingBox = AABB(-1.f, 1.f, -1.f, 1.f, -0.986f, -0.98f);
	float f1 = (50 - 0.1) / 2.0;
	float f2 = (50 + 0.1) / 2.0;

	Eigen::Matrix4f mv = view * model; //将顶点转换为视点坐标
	Eigen::Matrix4f mvp = projection * view * model; // 将顶点转换为标准设备化坐标

	Eigen::Vector4f viewDirction(0.f, 0.f, 1.f, 0.f);

	for (const auto& t : TriangleList)
	{
		Triangle* Tri = new Triangle(*t);

		std::array<Eigen::Vector4f, 3> mm{
				(mv * t->Vertex[0]),
				(mv * t->Vertex[1]),
				(mv * t->Vertex[2]),
		};

		std::array<Eigen::Vector3f, 3> viewspacePos;

		std::transform(mm.begin(), mm.end(), viewspacePos.begin(), [](auto& v) {
			return v.template head<3>();
			});
		//法向量变换为视点坐标
		Eigen::Matrix4f invTrans = (view * model).inverse().transpose();
		Eigen::Vector4f n[] = {
				invTrans * ToVec4(t->Normal[0], 0.0f),
				invTrans * ToVec4(t->Normal[1], 0.0f),
				invTrans * ToVec4(t->Normal[2], 0.0f)
		};
		auto interpolated_normal = interpolate(1, 1, 1, n[0].head(3), n[1].head(3), n[2].head(3), 3).normalized();

		if (interpolated_normal.dot(viewDirction.head(3)) <= 0) continue; //背部剔除

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

		OctreeNode* TNode = root;
		int depth = 10;
		while (depth--)
		{
			if (!depth)
			{
				
				for (auto& vert : v)
				{
					vert.x() = 0.5 * WIDTH * (vert.x() + 1.0);
					vert.y() = 0.5 * HEIGHT * (vert.y() + 1.0);
					vert.z() = vert.z() * f1 + f2;
				}

				for (int i = 0; i < 3; ++i)
				{
					Tri->setVertex(i, v[i]);
				}

				for (int i = 0; i < 3; ++i)
				{
					Tri->setNormal(i, n[i].head<3>());
				}

				Tri->setColor(0, 148, 121.0, 92.0);
				Tri->setColor(1, 148, 121.0, 92.0);
				Tri->setColor(2, 148, 121.0, 92.0);

				TNode->TList.push_back(Tri);
				TNode->viewPos.push_back(viewspacePos);
				break;
			}

			AABB& boundingBox = TNode->BoundingBox;
			Eigen::Vector3f centerPoint = boundingBox.centerPoint();
			float r = boundingBox.c2e();
			float zr = boundingBox.midZ();

			AABB ChildAABB;

			ChildAABB = AABB(centerPoint, centerPoint + Eigen::Vector3f(r, r, zr));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[0] == nullptr)
				{
					TNode->Children[0] = new OctreeNode;
					TNode->Children[0]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[0];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(-r, 0, 0), centerPoint + Eigen::Vector3f(0, r, zr));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[1] == nullptr)
				{
					TNode->Children[1] = new OctreeNode;
					TNode->Children[1]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[1];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(-r, -r, 0), centerPoint + Eigen::Vector3f(0, 0, zr));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[2] == nullptr)
				{
					TNode->Children[2] = new OctreeNode;
					TNode->Children[2]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[2];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(0, -r, 0), centerPoint + Eigen::Vector3f(r, 0, zr));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[3] == nullptr)
				{
					TNode->Children[3] = new OctreeNode;
					TNode->Children[3]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[3];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(0, 0, -zr), centerPoint + Eigen::Vector3f(r, r, 0));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[4] == nullptr)
				{
					TNode->Children[4] = new OctreeNode;
					TNode->Children[4]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[4];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(-r, 0, -zr), centerPoint + Eigen::Vector3f(0, r, 0));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[5] == nullptr)
				{
					TNode->Children[5] = new OctreeNode;
					TNode->Children[5]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[5];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(-r, -r, -zr), centerPoint );
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[6] == nullptr)
				{
					TNode->Children[6] = new OctreeNode;
					TNode->Children[6]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[6];
				continue;
			}

			ChildAABB = AABB(centerPoint + Eigen::Vector3f(0, -r, -zr), centerPoint + Eigen::Vector3f(r, 0, 0));
			if (ChildAABB.IsMeshIn(v))
			{
				if (TNode->Children[7] == nullptr)
				{
					TNode->Children[7] = new OctreeNode;
					TNode->Children[7]->BoundingBox = ChildAABB;
				}
				TNode = TNode->Children[7];
				continue;
			}


			

			for (auto& vert : v)
			{
				vert.x() = 0.5 * WIDTH * (vert.x() + 1.0);
				vert.y() = 0.5 * HEIGHT * (vert.y() + 1.0);
				vert.z() = vert.z() * f1 + f2;
			}

			for (int i = 0; i < 3; ++i)
			{
				Tri->setVertex(i, v[i]);
			}

			for (int i = 0; i < 3; ++i)
			{
				Tri->setNormal(i, n[i].head<3>());
			}

			Tri->setColor(0, 148, 121.0, 92.0);
			Tri->setColor(1, 148, 121.0, 92.0);
			Tri->setColor(2, 148, 121.0, 92.0);

			TNode->TList.push_back(Tri);
			TNode->viewPos.push_back(viewspacePos);
			break;
		}
	}
}
