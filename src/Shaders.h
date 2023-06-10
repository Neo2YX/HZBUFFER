#pragma once

#include "Shader.h"

Eigen::Vector3f vertex_shader(const vertexShaderLoad& payload)
{
	return payload.position;
}

Eigen::Vector3f normal_fragment_shader(const fragmentShaderLoad& payload)
{
	Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() + Eigen::Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
	Eigen::Vector3f result;
	result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
	return result;
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
	auto costheta = vec.dot(axis);
	return (2 * costheta * axis - vec).normalized();
}

struct light
{
	Eigen::Vector3f position;
	Eigen::Vector3f intensity;
};

float myMax(float a, float b) {
	return a > b ? a : b;
}



Eigen::Vector3f phong_fragment_shader(const fragmentShaderLoad& payload)
{
	Eigen::Vector3f ka = Eigen::Vector3f(0.005, 0.005, 0.005);
	Eigen::Vector3f kd = payload.color;
	Eigen::Vector3f ks = Eigen::Vector3f(0.7937, 0.7937, 0.7937);

	auto l1 = light{ {20, 20, 20}, {500, 500, 500} };
	auto l2 = light{ {-20, 20, 0}, {500, 500, 500} };

	std::vector<light> lights = { l1, l2 };
	Eigen::Vector3f amb_light_intensity{ 10, 10, 10 };
	Eigen::Vector3f eye_pos{ 0, 0, 10 };

	float p = 150;

	Eigen::Vector3f color = payload.color;
	Eigen::Vector3f point = payload.viewPos;
	Eigen::Vector3f normal = payload.normal;

	Eigen::Vector3f result_color = { 0, 0, 0 };
	for (auto& light : lights)
	{
		float r2 = (light.position - point).dot((light.position - point));
		Eigen::Vector3f l = (light.position - point).normalized();
		Eigen::Vector3f n = normal.normalized();
		Eigen::Vector3f v = (eye_pos - point).normalized();
		Eigen::Vector3f h = (l + v).normalized();
		Eigen::Vector3f ambient = ka * amb_light_intensity[0];
		Eigen::Vector3f diffuse = (kd * light.intensity[0] / r2) * myMax(0.0f, n.dot(l));
		Eigen::Vector3f specular = (ks * light.intensity[0] / r2) * std::pow(myMax(0.f, (n.dot(h))), p);

		result_color += (ambient + diffuse + specular);
	}

	return result_color * 255.f;
}



