#pragma once
#include "Eigen/Eigen"
#include "ScanLineZBuffer.h"
#include "HZBuffer.h"


#define WIDTH 512
#define HEIGHT 512



namespace UI {
	enum MODEL {
		MIKU = 0,
		SPOT = 1,
		BUNNY = 2, 
		HELMET = 3
	};

	enum RENDERER {
		RENDERER = 0,
		SCANLINEZBUFF = 1,
		HZBUFF = 2,
		OCTHZBUFF = 3
	};

	struct ModelInfo {
		Eigen::Vector3f EyePos;
		float angle;
	};

	

	std::tuple<int, int> drawMainUI();

	int inputInt(int min, int max);
	int drawMenuUI();


}

