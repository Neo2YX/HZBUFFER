#pragma once
#include <windows.h>
#include <tchar.h>
#include <assert.h>

struct Window
{
	int width_{ 800 };
	int height_{ 600 };
	HWND hwnd_{ nullptr };
	HDC hdc_{ nullptr };
	HBITMAP hbm_old_{ nullptr };
	HBITMAP hbm_{ nullptr };
	unsigned char* framebuffer_{ nullptr };
	bool window_close_{ false };
	char keys[512];
};

extern Window* myWindow;

void InitWindow(int w = 800, int h = 600);
void CloseWindow();
void msgDispatch();
void DrawWindow(unsigned char* framebuffer);
static LRESULT CALLBACK msgCallback(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam);