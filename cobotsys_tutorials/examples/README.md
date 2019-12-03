### 例程使用简介

#### 运行例程的方法
- 1.可将例程放入cobotsys_sdk目录，在命令行中编译。

例如：运行例程testCamera，参照https://github.com/COBOTOS/CobotSys/blob/master/cobotsys_tutorials/README.md 进行编译，编译完成后，在cobotsys_sdk/install/x86-64-install/devel/test中会有对应工程的文件夹,进入该文件夹，可以看到相应的可执行性程序，之后./testCamera即可运行例程。

- 2.使用clion打开例程

进入cobotsys_sdk目录，执行source setenv.sh设置环境变量，在同一终端下进入clion安装目录，./clion.sh即可打开clion，在clion中打开需要运行的例程，加载CMakeLists.txt后，即可编译运行。

##### 注意：例程的二级目录需取名为test！

#### 包含的例程
机器人相关例程：如COBOTLINK_Robot

力传感器相关例程：如COBOTLINK_FTSensor

相机使用相关例程：如COBOTLINK_CAMERA

其他例程：如Vtk.Test、OSG.Test

可结合相应的接口函数理解例程：https://github.com/COBOTOS/CobotSys/blob/master/cobotsys_tutorials/README.md
