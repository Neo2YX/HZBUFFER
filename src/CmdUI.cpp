#include "CmdUI.h"
#include <sstream>
#include <iostream>
#include <tuple>

using namespace std;

tuple<int, int> UI::drawMainUI()
{
	cout << "***********************HZBuffer*************************" << endl;
	cout << "                                     贺云鹏 22121031" << endl;
	cout << "********************************************************" << endl;
	cout << "请选择ZBuffer算法：" << endl;
	cout << "1 普通包围盒ZBuffer     2 扫描线ZBuffer    3 层次ZBuffer     4 带八叉树的层次ZBUffer" << endl;
	cout << "请输入序号：";
	int ZBufferNum = inputInt(1, 4);
	cout << "请选择要加载的模型：" << endl;
	cout << "1 spot(5k)     2 bunny     3 helmet(15k)     4 miku(174k)";
	cout << "请输入序号：";
	int ModelType = inputInt(1, 4);

	return { ZBufferNum, ModelType };
}

string inputLine()
{
	string result;
	getline(cin, result);
	return result;
}

int UI::inputInt(int min, int max)
{
	while (1) {
		stringstream ss;
		ss << inputLine();
		int result;
		if (ss >> result)
		{
			char c;
			if (ss >> c) cout << "输入包含非法字符：" << c << endl;
			else if (result > max || result < min) cout << "输入的数字范围必须是" << min << "~" << max << endl;
			else return result;
		}
		else cout << "必须输入数字！" << endl;
		cout << "请重新输入：" << endl;
	}
}

int UI::drawMenuUI()
{
	cout << "********************************************************" << endl;
	cout << "请选择接下来的操作：" << endl;
	cout << "1 使用普通包围盒ZBuffer算法继续绘制" << endl;
	cout << "2 使用扫描线ZBuffer算法继续绘制" << endl;
	cout << "3 使用层次ZBuffer算法继续绘制" << endl;
	cout << "4 使用带八叉树的层次ZBUffer算法继续绘制" << endl;
	cout << "0 退出系统" << endl;
	cout << "请输入序号：";
	return inputInt(0, 4);
}
