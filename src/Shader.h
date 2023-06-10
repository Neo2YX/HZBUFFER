#pragma once

#include "Eigen/Eigen"


struct fragmentShaderLoad
{
	fragmentShaderLoad()
	{
	}

	fragmentShaderLoad(const Eigen::Vector3f& col, const Eigen::Vector3f& nor) :
		color(col), normal(nor) {}


	Eigen::Vector3f viewPos;
	Eigen::Vector3f color;
	Eigen::Vector3f normal;
};

struct vertexShaderLoad
{
	Eigen::Vector3f position;
};

