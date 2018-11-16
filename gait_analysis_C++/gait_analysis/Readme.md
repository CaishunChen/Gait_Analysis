# 使用指南  
## C++版本
### 编译
* 编译版本: >= C++11
* 编译环境: Ubuntu 16.04
* 编译方式: 
`cd gait_analysis`  
`make`  
* 清除二进制文件  
`make clean`
* 运行程序  
`cd gait_analysis/build`
`./main`
### 调参
* `cd gait_analysis/include`
* 打开`config.h`文件
* 所有调参的备注均已注明  

## Python 版本
### 编译
* 编译版本: Python 3.6.7
* 依赖包: numpy、matplotlib、Scipy
* 编译方式: 同普通Python程序
### 调参 
    由于Python 语法简单，故不作具体的配置文件，只需要按着备注一步一步即可找到C++中对应的待调参数

## 备注
* 两个版本无本质差别，但是python代码中没有计算步速摆速的过程；
* Python 代码便于可视化数据及计算结果；
* C++ 是正式发布程序，由于时间仓促暂时未加入左右脚判断整体内外翻的计算

### 作者: Slade_lu
    
