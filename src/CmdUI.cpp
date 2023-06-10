#include "CmdUI.h"
#include <sstream>
#include <iostream>
#include <tuple>

using namespace std;

tuple<int, int> UI::drawMainUI()
{
	cout << "***********************HZBuffer*************************" << endl;
	cout << "                                     ������ 22121031" << endl;
	cout << "********************************************************" << endl;
	cout << "��ѡ��ZBuffer�㷨��" << endl;
	cout << "1 ��ͨ��Χ��ZBuffer     2 ɨ����ZBuffer    3 ���ZBuffer     4 ���˲����Ĳ��ZBUffer" << endl;
	cout << "��������ţ�";
	int ZBufferNum = inputInt(1, 4);
	cout << "��ѡ��Ҫ���ص�ģ�ͣ�" << endl;
	cout << "1 spot(5k)     2 bunny     3 helmet(15k)     4 miku(174k)";
	cout << "��������ţ�";
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
			if (ss >> c) cout << "��������Ƿ��ַ���" << c << endl;
			else if (result > max || result < min) cout << "��������ַ�Χ������" << min << "~" << max << endl;
			else return result;
		}
		else cout << "�����������֣�" << endl;
		cout << "���������룺" << endl;
	}
}

int UI::drawMenuUI()
{
	cout << "********************************************************" << endl;
	cout << "��ѡ��������Ĳ�����" << endl;
	cout << "1 ʹ����ͨ��Χ��ZBuffer�㷨��������" << endl;
	cout << "2 ʹ��ɨ����ZBuffer�㷨��������" << endl;
	cout << "3 ʹ�ò��ZBuffer�㷨��������" << endl;
	cout << "4 ʹ�ô��˲����Ĳ��ZBUffer�㷨��������" << endl;
	cout << "0 �˳�ϵͳ" << endl;
	cout << "��������ţ�";
	return inputInt(0, 4);
}
