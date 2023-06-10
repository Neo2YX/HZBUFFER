#include "Window.h"

Window* myWindow = NULL;

void InitWindow(int w, int h)
{
	myWindow = new Window;
	WNDCLASS wc = { CS_BYTEALIGNCLIENT, (WNDPROC)msgCallback, 0, 0, 0,
		NULL, NULL, NULL, NULL, _T("RenderWindow") };
	BITMAPINFO bi = { { sizeof(BITMAPINFOHEADER), w, -h, 1, 32, BI_RGB,
		w * h * 4, 0, 0, 0, 0 } };
	RECT rect = { 0, 0, w, h };
	int wx, wy, sx, sy;
	LPVOID ptr;
	HDC hDC;

	wc.hbrBackground = (HBRUSH)GetStockObject(BLACK_BRUSH);
	wc.hInstance = GetModuleHandle(NULL);
	wc.hCursor = LoadCursor(NULL, IDC_ARROW);
	RegisterClass(&wc);

	myWindow->hwnd_ = CreateWindow(_T("RenderWindow"), _T("HZBuffer"),
		WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
		0, 0, 0, 0, NULL, NULL, wc.hInstance, NULL);
	assert(myWindow->hwnd_ != NULL);

	myWindow->window_close_ = 0;
	hDC = GetDC(myWindow->hwnd_);
	myWindow->hdc_ = CreateCompatibleDC(hDC);
	ReleaseDC(myWindow->hwnd_, hDC);

	myWindow->hbm_ = CreateDIBSection(myWindow->hdc_, &bi, DIB_RGB_COLORS, &ptr, 0, 0);
	assert(myWindow->hbm_ != NULL);

	myWindow->hbm_old_ = (HBITMAP)SelectObject(myWindow->hdc_, myWindow->hbm_);
	myWindow->framebuffer_ = (unsigned char*)ptr;
	myWindow->width_ = w;
	myWindow->height_ = h;

	AdjustWindowRect(&rect, GetWindowLong(myWindow->hwnd_, GWL_STYLE), 0);
	wx = rect.right - rect.left;
	wy = rect.bottom - rect.top;
	sx = (GetSystemMetrics(SM_CXSCREEN) - wx) / 2;
	sy = (GetSystemMetrics(SM_CYSCREEN) - wy) / 2;
	if (sy < 0) sy = 0;
	SetWindowPos(myWindow->hwnd_, NULL, sx, sy, wx, wy, (SWP_NOCOPYBITS | SWP_NOZORDER | SWP_SHOWWINDOW));
	SetForegroundWindow(myWindow->hwnd_);

	ShowWindow(myWindow->hwnd_, SW_NORMAL);
	msgDispatch();

	memset(myWindow->framebuffer_, 0, w * h * 4);
}

void CloseWindow()
{
	if (myWindow->hdc_) {
		if (myWindow->hbm_old_) {
			SelectObject(myWindow->hdc_, myWindow->hbm_old_);
			myWindow->hbm_old_ = NULL;
		}
		DeleteDC(myWindow->hdc_);
		myWindow->hdc_ = NULL;
	}
	if (myWindow->hbm_) {
		DeleteObject(myWindow->hbm_);
		myWindow->hbm_ = NULL;
	}
	if (myWindow->hwnd_) {
		CloseWindow(myWindow->hwnd_);
		myWindow->hwnd_ = NULL;
	}
	free(myWindow);
}

void msgDispatch()
{
	MSG msg;
	while (1) {
		if (!PeekMessage(&msg, NULL, 0, 0, PM_NOREMOVE)) break;
		if (!GetMessage(&msg, NULL, 0, 0)) break;
		DispatchMessage(&msg);
	}
}

void DrawWindow(unsigned char* framebuffer)
{
	for (int i = 0; i < myWindow->height_; ++i)
	{
		for (int j = 0; j < myWindow->width_; ++j)
		{
			//将RGBA格式改为BGRA格式
			myWindow->framebuffer_[(i * myWindow->width_ + j) * 4 ] = framebuffer[(i * myWindow->width_ + j) * 4 + 2];
			myWindow->framebuffer_[(i * myWindow->width_ + j) * 4 + 1] = framebuffer[(i * myWindow->width_ + j) * 4 + 1];
			myWindow->framebuffer_[(i * myWindow->width_ + j) * 4 + 2] = framebuffer[(i * myWindow->width_ + j) * 4];
			myWindow->framebuffer_[(i * myWindow->width_ + j) * 4 + 3] = framebuffer[(i * myWindow->width_ + j) * 4 + 3];
		}
	}

	HDC hDC = GetDC(myWindow->hwnd_);
	BitBlt(hDC, 0, 0, myWindow->width_, myWindow->height_, myWindow->hdc_, 0, 0, SRCCOPY);
	ReleaseDC(myWindow->hwnd_, hDC);
	msgDispatch();
}

LRESULT CALLBACK msgCallback(HWND hwnd, UINT msg, WPARAM wparam, LPARAM lparam)
{
	switch (msg)
	{
	case WM_CLOSE: myWindow->window_close_ = true; break;
	default: return DefWindowProc(hwnd, msg, wparam, lparam);
	}
	return 0;
}