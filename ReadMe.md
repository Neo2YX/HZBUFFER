# 计算机图形学上机4作业说明文档

贺云鹏  22121031

---
## 1 功能实现
本次作业共实现了四种ZBuffer算法。
+ 普通包围盒ZBuffer。 将每个三角面片取屏幕上的包围盒，再对包围盒的每个像素判断是否在面片中，如果是，则判断深度值
+ 扫描线ZBuffer
+ 层次ZBuffer
+ 带八叉树的层次ZBuffer

## 2 编程环境
操作系统：Windows11家庭中文版
程序编译使用Cmake， C++17

编译命令：
```
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make

//运行
start ./bin/HZBuffer.exe
```

##### 使用软件注意事项：
+ **编译项目时应确保项目目录不包含中文**
+ 运行程序环境应包括models文件夹，可以在命令行中使用`start ./bin/HZBuffer.exe`，或者将models文件夹复制到bin目录直接双击exe程序打开
+ 在切换ZBuffer算法时可能会出现黑屏的现象，再次绘制即可显示

## 3 文件结构
+ `Renderer.h` 绘制类，用于将模型绘制于512*512的窗口中。包含了**mvp坐标空间转化、光栅化、深度测试、颜色插值**等框架。其中，深度测试使用的为**普通包围盒ZBuffer算法**，即对每一个三角面片，计算其在屏幕空间的包围盒，对于包围盒内每一个像素点，判断其是否在三角面片内，再根据深度值判断是否需要着色。
+ `ScanLineZBuffer.h`、`HZBuffer.h` 继承了Renderer类，复写了深度测试算法。`ScanLineZBuffer`类实现了扫描线ZBuffer算法。`HZBuffer.h`中`HZBuffer`类继承`Renderer`类，实现了简单模式的层次ZBuffer算法。`HZBuffer.h`中`OctHZBuffer`类继承`HZBuffer`类，实现了完整模式的ZBuffer算法
+ `Triangle.h`、`Shader.h`、`BoundingBox.h`、`Octree.h` 数据结构类，定义了三角面片、着色器、三维包围盒、八叉树的数据结构
+ `CmdUI.h`、`Window.h` 图形显示及用户界面
+ 第三方库使用：
  + Eigen C++线性代数函数库
  + `OBJ_Loader.h` 模型加载库，来自Games101框架代码
  + `stb_image.h`  图像解码库，用于读入图像

