**cobotsys_tutorials**


[TOC]


# 1 安装说明

1. 将sdk安装包，如sdk-release-Ubuntu-16-2019-11-11-14-18-18.run安装到Home文件夹下

打开终端

```sh
 ./sdk-release-Ubuntu-16-2019-11-11-14-18-18.run
```



 #2 开发说明

## 2.1 Cobotsys SDK编译工程介绍

在开发的过程中，我们需要经常去写CMakeList.txt文件，不同模块之间CMakeList也存在大量相似的部分。基于cobotsys 3.0开发平台将相似的部分简化成模板，变化的地方用变量来定义。开发者使用时只需要增加模板即可，极大减少了代码行数。

### 1 CMakeList.txt编写

简化后的CMakeList.txt 如下：

```cmake
#cmake版本不低于3.7
cmake_minimum_required(VERSION 3.7)
#模块依赖
set(COBOTSYS_DEPENDED_MODULES xxx)
#开源的第三方
set(COBOTSYS_DEPENDED_3RD_MODULES xxx)
#使用的模板
include($ENV{SDK_PATH}/Tiger/Tiger.Make/cmake/template/test.executable.cmake)
```

开发者主要使用2种模板，针对应用可以用qt.executable.cmake模板，编译完成后可执行程序生成在bin目录下。针对测试程序，可以用test.executable.cmake模板，编译完成后可执行程序生成在test/对应模块目录下。

下载附件压缩包，解压放置到SDK目录下，压缩包中分别有使用qt模板和使用test模板的样例代码。

### 2 编译

安装完成后cobotsys_sdk为一级目录，进去后XXX为二级目录，再进去后XXX.XXX.XXX为三级目录。

打开终端，进入cobotsys_sdk目录，执行source setenv.sh设置环境变量，再执行. go.sh编译前需执行一遍对目录进行扫描。

执行make+二级目录名，即可编译整个二级目录。如附件中的示例，执行make Chicken即可整体编译。

执行make+三级目录名，即可仅编译三级目录。如附件中的示例，执行make Chicken.Example.Robot即可仅编译该文件夹。

### 3 先删除再编译

针对三级目录可以执行，对当前模块删除后重新编译。如附件中的示例，执行make Chicken.Example.Robot_Force，即可删除后重新编译。

# Robot API 

## connect

```c++
RobotStatus connect(std：：string robotAddress,const std::string& name = "")
```

通过机器人IP地址连接机器人。

参数：

[in]robotAddress 机器人IP地址

返回：

返回机器人状态RobotStatus，可参考6.2.1 常见结构中的机器人状态枚举类。

示例：

```c++
//创建机器人工厂对象时,使用此头文件 RobotDriverFactoryInterface.h
  #include <RobotDriverFactoryInterface.h>
  
//创建接口对象时,使用此头文件 RobotDriverInterface.h
  #include <RobotDriverInterface.h>
  #include <boost/shared_ptr.hpp>
  
//统一的日志管理,使用此头文件
  #include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
  #include <logger/GlogConfig.h>

//如使用日志配置,需使用此命名空间
  using namespace TIGER_COMMON_NS_API;
  
//机器人驱动使用此命名空间
  using namespace WOLF_ROBOTDRIVER_NS_API;
  
  int main()
  {
  //使用日志配置时使用,必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建机器人工厂对象
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  
  
  / 通过工厂对象创建接口对象！
   Cobotsys所有业务功能模块基于服务抽象,机器人服务模块被编译成动态库通过工厂方法实例化,通过工厂方法可以加载不同的实现!
   例:此处参数填写ur3,就是加载ur机器人驱动；再如,填写denso_vs087,就是加载denso_vs087机器人的驱动！
  /
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver(“ur3”);
  
  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  	LOG_INFO << "Robot connect successfully!";
  }
  
  return 0;
  }
```



## disconnect

```c++
RobotStatus disconnect(const std::string& name = "")
```

断开与机器人的连接。

参数：

无

返回：

返回机器人状态RobotStatus，可参考6.2.1 常见结构中的机器人状态枚举类。

示例：

```c++
#include <RobotDriverFactoryInterface.h>
#include <RobotDriverInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

#include<logger/GlogConfig.h>

using namespace TIGER_COMMON_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;

int main()
{
    //使用日志配置时使用,必须要有配置文件 glogconfig.xml 
    GlogConfig::config(__COBOTSYS_MODULE_NAME__);
    
    //创建机器人工厂对象
    boost::shared_ptr <RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
    
    //通过工厂对象创建接口对象,示例中加载的为ur机器人插件 
    boost::shared_ptr <RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");
   
    //通过接口对象调用connect
    

    if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
        LOG_INFO << "Robot connect successfully!";
    }
    //通过接口对象调用disconnect
    if(RobotStatus::Disconnect == robotDriver->disconnect()){
        LOG_INFO << "Robot disconnect successfully!";
    }
    return 0;
}
```



## getRobotStatus

```c++
RobotStatus getRobotStatus(const std::string& name = "")
```

获取机器人当前的状态信息。

参数：

无

返回：

返回机器人状态RobotStatus，可参考6.2.1 常见结构中的机器人状态枚举类。

示例：

```c++
#include <RobotDriverFactoryInterface.h>
#include <RobotDriverInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件glogconfig.xml

#include <logger/GlogConfig.h>

using namespace TIGER_COMMON_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;

int main(){   
    //使用日志配置时使用,且必须要有配置文件 glogconfig.xml   
    GlogConfig::config(__COBOTSYS_MODULE_NAME__); 
    
    //创建机器人工厂对象
    boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();  
    
    //通过工厂对象创建接口对象,示例中加载的为ur机器人插件 
    boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");  
    
    //通过接口对象调用connect
    if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
LOG_INFO << "Robot connect successfully!";  
    }
    
    //通过接口对象调用getRobotStatus    
    RobotStatus robotStatus;   

    robotStatus = robotDriver->getRobotStatus();    
    //将获取到的机器人状态输出    
    LOG_INFO << "Robot status:" << RobotStatusEnum2String(robotStatus);   

    //通过接口对象调用disconnect  

    if(RobotStatus::Disconnect == robotDriver->disconnect()){   
        LOG_INFO << "Robot disconnect successfully!";  
    }
    return 0;
    
}
```





## getRobotJoints

```c++
std::vector<double> getRobotJoints(const std::string& name = "")
```

获取机器人当前关节角信息，单位弧度。

参数：

无

返回：

当前机器人关节角弧度值

示例：

```c++
#include <RobotDriverFactoryInterface.h>
#include <RobotDriverInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

using namespace TIGER_COMMON_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;

int main(){
    //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
    GlogConfig::config(__COBOTSYS_MODULE_NAME__);
    
    //创建机器人工厂对象   
    boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();   
    
    //通过工厂对象创建接口对象,示例中加载的为ur机器人插件   
    boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");    
    
    //通过接口对象调用connect  
    if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
        LOG_INFO << "Robot connect successfully!";
    }
    
    //获取机器人关节角,获取到的关节角单位为弧度
    std::vector<double> joints = robotDriver->getRobotJoints();  
    
    //打印获取到的关节角    
    LOG_INFO << "Joints:" << joints;   
    
    //通过接口对象调用disconnect   
    if(RobotStatus::Disconnect == robotDriver->disconnect()){
        LOG_INFO << "Robot disconnect successfully!"; 
    }    
    return 0;
}
```



## getRobotJointVels

```c++
std::vector<double> getRobotJointVels(const std::string& name = "")
```

分别获取机器人每个关节的速度，单位：弧度/s。

参数：

无

返回：

当前机器人每个关节角的速度

示例：

```c++
#include <RobotDriverFactoryInterface.h>
#include <RobotDriverInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include<logger/GlogConfig.h>

using namespace TIGER_COMMON_NS_API;
using namespace WOLF_ROBOTDRIVER_NS_API;

int main(){
    //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
    GlogConfig::config(__COBOTSYS_MODULE_NAME__);
    
    //创建机器人工厂对象
    boost::shared_ptr <RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
    
    //通过工厂对象创建接口对象,示例中加载的为ur机器人插件 
    boost::shared_ptr <RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");
    
    //通过接口对象调用connect
    if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
        LOG_INFO << "Robot connect successfully!";
    }
    //获取机器人关节角速度,可分别获取到机器人每个关节的速度
    std::vector<double> vels = robotDriver->getRobotJointVels();
    
    //打印获取到的关节角速度
    LOG_INFO << "Robot joint vels::" << vels;
    
    //通过接口对象调用disconnect
    if(RobotStatus::Disconnect == robotDriver->disconnect()){
       LOG_INFO << "Robot disconnect successfully!";
    }
    return 0;
}
```



## getRobotTcp

```c++
std::vector<double> getRobotTcp(const std::string& name = "")
```

获取机器人当前末端位置的姿态，即x，y，z，Rx，Ry，Rz，xyz单位为m，RxRyRz单位为弧度。

参数：

无

返回：

当前机器人末端位姿（位置+姿态，即x，y，z，Rx，Ry，Rz）

示例：

```c++
  #include <RobotDriverFactoryInterface.h>
  #include <RobotDriverInterface.h>
  #include <boost/shared_ptr.hpp>
  #include <logger/Logger.h>

  //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

  #include <logger/GlogConfig.h>

  using namespace TIGER_COMMON_NS_API;
  using namespace WOLF_ROBOTDRIVER_NS_API;

  int main()
  {
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);

  //创建机器人工厂对象
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();

  //通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  }

  //获取机器人末端姿态,即xyzRPY
  std::vector<double> tcp = robotDriver->getRobotTcp();

  //将获取到末端位姿打印出来,xyz单位为m,RPY单位为弧度
  LOG_INFO << "Tcp pos:" << tcp;

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }

  return 0;
  }
```

## moveArm

```c++
int moveArm(const std::vector<ArmRobotWayPoint>& movePath,const std::string& name = "")
```

控制机械臂做不同运动类型的运动，设置机器人IO，机器人延时。

参数：

[in]数据结构ArmRobotWayPoint，该结构体参见6.2.1

返回：

返回0表示控制机器人手臂运动成功，失败返回-1

示例：

```c++
  #include <math.h>
  #include <LocalStorageRepositoryAPIKey.h>
  #include <RobotDriverFactoryInterface.h>
  #include <RobotDriverInterface.h>
  #include <boost/shared_ptr.hpp>
  #include <logger/Logger.h>

  //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

  #include <logger/GlogConfig.h>

  using namespace TIGER_COMMON_NS_API;
  using namespace WOLF_ROBOTDRIVER_NS_API;

  void moveJ(){

  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  } else{
  LOG_ERROR << "Robot connect failed!";
  return;
  }

  std::vector<ArmRobotWayPoint> movePath;
  ArmRobotWayPoint temp;

  //弧度和角度的转换
  double turnPi = 3.141592654/180;

  std::vector<double> joint1 = {0turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};
  std::vector<double> joint2 = {20turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};

  /
   让机器人moveJ运动到joint1,再运动到joint2!
   设置关节最大加速度为10,设置关节最大速度为100,设置交融半径为0
  /
  temp.motionType = ArmRobotMotionType::MoveJ;
  temp.acc=10.00;
  temp.vel=100.00;
  temp.blendRaidus = 0.00;
  temp.jointPosition = joint1;
  movePath.push_back(temp);
  temp.jointPosition = joint2;
  movePath.push_back(temp);

  //通过接口对象调用MoveArm
  int ret = robotDriver->moveArm(movePath);
  LOG_INFO << "Robot move result=" << ret;

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }

  }

  void moveL(){

  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  } else{
  LOG_ERROR << "Robot connect failed!";
  return;
  }

  std::vector<ArmRobotWayPoint> movePath;
  ArmRobotWayPoint temp;

  double turnPi = 3.141592654/180;

  std::vector<double> joint1 = {0turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};
  std::vector<double> joint2 = {20turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};

  /
   让机器人moveL运动到joint1,再运动到joint2!
   设置关节最大加速度为10,设置关节最大速度为100,设置交融半径为0
  /
  temp.motionType = ArmRobotMotionType::MoveL;
  temp.acc=10.00;
  temp.vel=100.00;
  temp.blendRaidus = 0.00;
  temp.jointPosition = joint1;
  movePath.push_back(temp);
  temp.jointPosition = joint2;
  movePath.push_back(temp);

  int ret = robotDriver->moveArm(movePath);
  LOG_INFO << "Robot move result=" << ret;

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }
  }

  void servoJ(){
  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  } else{
  LOG_ERROR << "Robot connect failed!";
  return;
  }

  //获取机器人当前关节角
  std::vector<double> currentpos = robotDriver->getRobotJoints();

  std::vector<ArmRobotWayPoint> movePath;
  ArmRobotWayPoint temp;

  double turnPi = 3.141592654/180;

  std::vector<double> t_pos(6,0);
  t_pos= currentpos;

  //让机器人关节角1使用servoJ做±20°正弦曲线运动
  for(double t=0;t<6;t=t+0.008)
  {
  t_pos.at(0) =currentpos.at(0) + 20M_PI/180sin(23.14t);
  LOG_INFO<<"t_pos: "<<t_pos;
  temp.jointPosition = t_pos;
  temp.motionType = ArmRobotMotionType::ServoJ;
  temp.acc=5;
  temp.vel=10;
  temp.blendRaidus = 0;
  movePath.push_back(temp);
  LOG_INFO << robotDriver->moveArm(movePath);
  movePath.clear();
  usleep(8000);
  }

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }
  }

  void setIO(){
  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  } else{
  LOG_ERROR << "Robot connect failed!";
  return;
  }

  std::vector<ArmRobotWayPoint> movePath;
  ArmRobotWayPoint temp;

  double turnPi = 3.141592654/180;

  std::vector<double> joint1 = {0turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};
  std::vector<double> joint2 = {20turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};

  /
   控制机器人moveJ运动到Joint1
   设置IO端口为Set状态
   控制机器人moveJ运动到Joint2
   控制IO端口为Reset状态
  /
  temp.motionType = ArmRobotMotionType::MoveJ;
  temp.acc=10.00;
  temp.vel=100.00;
  temp.isFixed=0;
  temp.blendRaidus = 0.00;
  temp.jointPosition = joint1;
  movePath.push_back(temp);

  //设置IO端口为Set状态
  temp.motionType = ArmRobotMotionType::SetIO;
  temp.ioIndex=LION_LSR_NS_API::hw_plc_io_vibrationcylinder1;
  temp.ioStatus=DigitIoStatus::Set;
  movePath.push_back(temp);

  movePath.push_back(temp);
  temp.jointPosition = joint2;
  movePath.push_back(temp);

  //设置IO端口为Reset状态
  temp.motionType = ArmRobotMotionType::SetIO;
  temp.ioIndex=LION_LSR_NS_API::hw_plc_io_vibrationcylinder1;
  temp.ioStatus=DigitIoStatus::Reset;

  int ret = robotDriver->moveArm(movePath);
  LOG_INFO << "Robot move result=" << ret;

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }
  }

  void delay(){
  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  } else{
  LOG_ERROR << "Robot connect failed!";
  return;
  }

  std::vector<ArmRobotWayPoint> movePath;
  ArmRobotWayPoint temp;

  double turnPi = 3.141592654/180;

  std::vector<double> joint1 = {0turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};
  std::vector<double> joint2 = {20turnPi,-90turnPi,0turnPi,-90turnPi,0turnPi,0turnPi};

  /
   控制机器人moveJ运动到Joint1
   在当前位姿延时3秒
   控制机器人moveJ运动到Joint2
  /
  temp.motionType = ArmRobotMotionType::MoveJ;
  temp.acc=10.00;
  temp.vel=100.00;
  temp.isFixed=0;
  temp.blendRaidus = 0.00;
  temp.jointPosition = joint1;

  temp.motionType = ArmRobotMotionType::Delay;
  temp.time=3;
  movePath.push_back(temp);

  movePath.push_back(temp);
  temp.jointPosition = joint2;
  movePath.push_back(temp);

  int ret = robotDriver->moveArm(movePath);
  LOG_INFO << "Robot move result=" << ret;

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }
  }

  int main()
  {
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);

  //让机器人做moveJ运动
  moveJ();

  //让机器人做moveL运动
  moveL();

  //让机器人做servoJ运动
  servoJ();

  //设置机器人IO
  setIO();

  //机器人在运动过程中延时
  delay();

  return 0;
  }
```



## getIOStatus

```c++
DigitIoStatus getIOStatus(const std：：string& ioPort,const std::string& name = "")
```

获取机器人指定IO端口的状态，Set/Reset。机器人IO端口为可配置项，默认已预置的IO端口可参见，

/cobotsys_sdk/install/x86-64-install/devel/include/Lion.LocalStorageRepository.API目录下的LocalStorageRepositoryAPIKey.h文件，如下图所示。

参数：

[in]ioPort 机器人IO端口

返回：

机器人IO端口状态Set/Reset。DigitIoStatus为枚举类，可参见6.2.1

示例：

```c++
  #include <LocalStorageRepositoryAPIKey.h>
  #include <RobotDriverFactoryInterface.h>
  #include <RobotDriverInterface.h>
  #include <boost/shared_ptr.hpp>
  #include <logger/Logger.h>

  //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

  #include <logger/GlogConfig.h>

  using namespace TIGER_COMMON_NS_API;
  using namespace WOLF_ROBOTDRIVER_NS_API;

  int main()
  {
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);

  //创建机器人工厂对象
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();

  //通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  }

  //获取IO状态
  DigitIoStatus digitIoStatus;
  digitIoStatus = robotDriver->getIOStatus(LION_LSR_NS_API::hw_plc_io_vibrationcylinder1);

  //将获取到的IO状态打印出来
  LOG_INFO << "IO status:" << DigitIoStatusEnum2String(digitIoStatus);

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }

  return 0;
  }
```



## setIOStatus

```c++
int setIOStatus(const std：：string& ioPort，const DigitIoStatus& ioStatus,const std::string& name = "")
```

设置机器人指定端口Set/Reset。

参数：

[in]ioPort 机器人IO端口

[in]ioStatus 机器人IO状态。DigitIoStatus为枚举类，可参见6.2.1

返回：

返回0表示设置IO成功，失败返回-1

示例：

```c++
  #include <LocalStorageRepositoryAPIKey.h>
  #include <RobotDriverFactoryInterface.h>
  #include <RobotDriverInterface.h>
  #include <boost/shared_ptr.hpp>
  #include <logger/Logger.h>

  //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

  #include <logger/GlogConfig.h>

  using namespace TIGER_COMMON_NS_API;
  using namespace WOLF_ROBOTDRIVER_NS_API;

  int main()
  {
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);

  //创建机器人工厂对象
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();

  //通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //通过接口对象调用connect
  if(RobotStatus::Idle == robotDriver->connect("192.168.10.113")){
  LOG_INFO << "Robot connect successfully!";
  }

  //设置IO端口为Set状态
  int ret1 = robotDriver->setIOStatus(LION_LSR_NS_API::hw_plc_io_vibrationcylinder1,DigitIoStatus::Set);
  if(0 == ret1){
  LOG_INFO << "Set IO successfully!";
  }

  //设置IO端口为Reset状态
  int ret2 = robotDriver->setIOStatus(LION_LSR_NS_API::hw_plc_io_vibrationcylinder1,DigitIoStatus::Reset);
  if(0 == ret2){
  LOG_INFO << "Set IO successfully!";
  }

  //通过接口对象调用disconnect
  if(RobotStatus::Disconnect == robotDriver->disconnect()){
  LOG_INFO << "Robot disconnect successfully!";
  }

  return 0;
  }
```

## registerListener

```c++
int registerListener(RobotDriverListenerInterface：：pointer listenerInterface,const std::string& name = "")
```

注册监听。需要创建监听器，监听器创建可参考6.3.2。

参数：

[in]监听器对象

返回：

返回0表示注册成功，失败返回-1

示例：

```c++
  #include <RobotDriverFactoryInterface.h>

  #include <RobotDriverInterface.h>

  #include <boost/shared_ptr.hpp>

  #include <logger/Logger.h>

  //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

  #include <logger/GlogConfig.h>

  //加载监听函数头文件

  #include "robotListener.h"

  using namespace TIGER_COMMON_NS_API;
  using namespace WOLF_ROBOTDRIVER_NS_API;

  int main()
  {
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);

  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //创建监听者
  boost::shared_ptr<robotListener> listener = boost::shared_ptr<robotListener>(new robotListener());

  //注册监听,打印注册监听返回结果
  int ret1 = robotDriver->registerListener(listener);
  if(0 == ret1){
  LOG_INFO << "Robot register Listener successfully!";
  }

  //注销监听,打印注销监听返回结果
  int ret2 = robotDriver->unRegisterListener(listener);
  if(0 == ret2){
  LOG_INFO << "Robot unregister Listener successfully!";
  }

  return 0;
  }
```

## unRegisterListener

```c++
int unRegisterListener(RobotDriverListenerInterface：：pointer listenerInterface,const std::string& name = "")
```

注销监听。

参数：

[in]监听器对象

返回：

返回0表示注册成功，失败返回-1

示例：

```c++
  #include <RobotDriverFactoryInterface.h>

  #include <RobotDriverInterface.h>

  #include <boost/shared_ptr.hpp>

  #include <logger/Logger.h>

  //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

  #include <logger/GlogConfig.h>

  //加载监听函数头文件

  #include "robotListener.h"

  using namespace TIGER_COMMON_NS_API;
  using namespace WOLF_ROBOTDRIVER_NS_API;

  int main()
  {
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);

  //创建机器人工厂对象,通过工厂对象创建接口对象,示例中加载的为ur机器人插件
  boost::shared_ptr<RobotDriverFactoryInterface> factory = RobotDriverFactoryInterface::create();
  boost::shared_ptr<RobotDriverInterface> robotDriver = factory->createRobotDriver("ur3");

  //创建监听者
  boost::shared_ptr<robotListener> listener = boost::shared_ptr<robotListener>(new robotListener());

  //注册监听,打印注册监听返回结果
  int ret1 = robotDriver->registerListener(listener);
  if(0 == ret1){
  LOG_INFO << "Robot register Listener successfully!";
  }

  //注销监听,打印注销监听返回结果
  int ret2 = robotDriver->unRegisterListener(listener);
  if(0 == ret2){
  LOG_INFO << "Robot unregister Listener successfully!";
  }

  return 0;
  }
```

## 数据结构

ArmRobotMotionTypeEnum.h

机械臂运动类型。

枚举值：

MoveJ = 0， 机械臂做MoveJ运动

MoveL， 机械臂做MoveL运动

SetIO， 设置机器人IO状态

Delay， 延时

ServoJ， 机械臂做servoJ运动

MoveJGO， 预留

MoveLGO， 预留

MoveAbsJ 预留



ArmRobotWayPointStruct.h

机械臂Waypoint结构体

  std::vector<double> dcartPosition          机器人末端法兰位姿{x，y，z，Rx，Ry，Rz}，X、Y、Z单位为毫米，Rx、Ry、Rz单位为弧度
  std::vector<double> jointPosition          机器人6个关节角，单位弧度。
  ArmRobotMotionType motionType                  机器人运动类型，参见上一个枚举
  double acc                                     最大加速度百分比
  double vel                                     最大速度百分比
  double blendRaidus                             交融半径
  double time                                    机器人延时时间，单位秒
  std：：string ioIndex                          机器人IO端口
  DigitIoStatus ioStatus                         机器人IO状态
  double payload                                 负载
  int goValue                                    
  bool isFixed                                   
  std::vector<int> confdata                  
  int singArea                                   
  int confStatus                                 
  int interruptType                              
  RobotParamExtendType type                      
  std::vector<int> length                    
  std::vector< std::vector<char>> value   

DigitInputIoPortEnum.h

数字输入IO端口

枚举值：

  Port_Gripper = 1    数字输入IO端口1
  Port_Sucker = 2     数字输入IO端口2
  Port_Connetor = 3   数字输入IO端口3
  Port_Cylinder = 4   数字输入IO端口4
  Port_Pressure = 5   数字输入IO端口5
  Port_Alarm = 6      数字输入IO端口6
  Port_Reserve1 = 7   数字输入IO端口7
  Port_Reserve2 = 8   数字输入IO端口8
  Port_Reserve3 = 9   数字输入IO端口8

DigitIoPortEnum.h

数字IO端口

枚举值：

Port_Gripper = 1， 数字输出IO端口0

Port_Sucker = 2， 数字输出IO端口1

Port_Connetor = 3， 数字输出IO端口2

Port_Cylinder = 4， 数字输出IO端口3

Port_Pressure = 5， 数字输出IO端口4

Port_Alarm = 6， 数字输出IO端口5

Port_Reserve1 = 7， 数字输出IO端口6

Port_Reserve2 = 8， 数字输出IO端口7

Port_Reserve3 = 9 数字输出IO端口8

DigitIoStatusEnum.h

数字IO状态

枚举值：

Reset = 1， IO端口设置为Reset

Set = 2 IO端口设置为Set

DigitOutputIoPortEnum.h

数字输出IO端口

枚举值：

Port_Gripper = 1， 数字输出IO端口0

Port_Sucker = 2， 数字输出IO端口1

Port_Connetor = 3， 数字输出IO端口2

Port_Cylinder = 4， 数字输出IO端口3

Port_Pressure = 5， 数字输出IO端口4

Port_Alarm = 6， 数字输出IO端口5

Port_Reserve1 = 7， 数字输出IO端口6

Port_Reserve2 = 8， 数字输出IO端口7

Port_Reserve3 = 9 数字输出IO端口8

RobotExceptionEnum.h

机器人异常类别

枚举值：

RobotEmergencyStop = 1， 紧急停止

RobotEmergencyStopUnlocked = 2， 解除紧急停止

RobotProtectiveStop = 3， 保护性停止

RobotProtectiveStopUnlocked = 4 解除保护性停止

RobotInfoStruct.h

机器人信息结构体

std::vector<double> jointPosition; 机器人6个关节角，单位弧度

RobotMoveStruct.h

机器人运动的结构体

int curnentWayPoint; 当前机器人运动到第几个Waypoint点

int wayPointSize; 当前机器人运动轨迹Waypoint点总个数

RobotStatusEnum.h

机器人状态

枚举值：

Disconnect = 0， 机器人断开连接

RobotNameInexistence = 1， 机器人名称不存在

Connect = 2， 机器人已连接

Idle = 32， 机器人处于空闲状态

Running = 4， 机器人处于运动状态

ProtectiveStop = 8， 保护性停止

EmergencyStop = 16， 紧急停止

robotModeError = 64 机器人故障



## 监听函数

SDK提供了监听接口文件帮助用户快速创建监听器，机器人监听接口文件所在位置/cobotsys_sdk/install/x86-64-install/devel/include/Wolf.RobotDriver.API/RobotDriverListenerInterface.h

加载监听头文件实现一个机器人监听器。如表1和表2中的参考示例，当机器人运动时，onRobotInfo被调用，获取机器人的实时关节角；当机器人状态发生变化时，onRobotStatus被调用，可以获取机器人改变后的状态；当机器人输入IO状态发生变化时，onRobotIO被调用，可以获取所有IO端口和其对应的状态；当机器人处于运动状态时，onRobotMove被调用，可以获取当前waypoint点和waypoint总数。

```c++
  #ifndef COBOTOS_ROBOTLISTENER_H
  #define COBOTOS_ROBOTLISTENER_H

  #include <logger/Logger.h>
  #include <RobotDriverListenerInterface.h>
  #include <RobotDriverFactoryInterface.h>
  #include <RobotDriverInterface.h>

  using namespace WOLF_ROBOTDRIVER_NS_API;

  class robotListener:public RobotDriverListenerInterface
  {
  public:
  typedef DBusCxxPointer<robotListener> pointer;
  ~robotListener(){}
  robotListener();
    
  virtual void onRobotInfo(RobotInfo robotInfo,std::string name);

  virtual void onRobotException(RobotException robotException,std::string name);

  virtual void onRobotStatus(RobotStatus RobotStatus,std::string name);

  virtual void onRobotIO(std::map<std::string, DigitIoStatus> IO,std::string name);

  virtual void onRobotMove(RobotMove RobotMove,std::string name);

  };
#endif //COBOTOS_ROBOTLISTENER_H
```



```c++
  #include "robotListener.h"
  robotListener::robotListener() {

  }

  void robotListener::onRobotInfo(RobotInfo robotInfo,std::string name) {
  	if(!robotInfo.jointPosition.empty()){
  		for(unsigned int i=0;i<robotInfo.jointPosition.size();i++) {
  			LOG_INFO << "Joint" << i << "=" << robotInfo.jointPosition[i]/(3.1415926/180);
  		}
  	} else{
  			LOG_ERROR<<"Get robot info err!";
  	}
  }

  void robotListener::onRobotException(RobotException robotException,std::string name) {
  	LOG_INFO << "onRobotException:" << RobotExceptionEnum2String(robotException);
  }

  void robotListener::onRobotStatus(RobotStatus RobotStatus, std::string name){
  	LOG_INFO << "onRobotStatus:" << RobotStatusEnum2String(RobotStatus);
  }

  void robotListener::onRobotIO(std::map<std::string, DigitIoStatus> IO,std::string name)
  {
  	LOG_INFO << "onRobotIO:" << IO.size();
  }

  void robotListener::onRobotMove(RobotMove RobotMove,std::string name)
  {
  	LOG_INFO << "curnentWayPoint:" << RobotMove.curnentWayPoint;
  	LOG_INFO << "wayPointSize:" << RobotMove.wayPointSize;
  }
```



# ForceSensor API 

## Connect

```c++
ForceSensorStatus connect(const SensorInfo& sensorInfo, const std::string& name = "") 
```

连接力传感器 参数: 

[in] sensorInfo 力传感器信息 uri，uri 可以是 IP 地址也可以是串口。 返回: 

返回 ForceSensorStatus，connect 表示连接，disconnect 表示断开连接。 示例: 

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h 

#include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo);
LOG_INFO << "connect forceSensor result =" <<
ForceSensorStatusEnum2String(forceSensorStatus);
return 0; }
```

## Disconnect

PLCStatus disconnect(const std::string& name = "")
断开力传感器连接 参数:
无 返回:
返回 ForceSensorStatus，connect 表示连接，disconnect 表示断开连接。 示例:

示例: 

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo); LOG_INFO << "connect forceSensor result =" <<
ForceSensorStatusEnum2String(forceSensorStatus);
//断开 ForceSensor 连接
forceSensorStatus = forceSensor->disconnect();
LOG_INFO << "disconnect forceSensor result =" <<
ForceSensorStatusEnum2String(forceSensorStatus);
return 0;
}
```



## getSensorData

```c++
std::vector<double> getSensorData(const std::string& name = "")
```

获取力传感器真实数据
参数:
无。 返回:
返回 double 型的 vector，力矩 xyz 和扭矩 rpy。 示例:

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo);
LOG_INFO << "connect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
//获取 Data 数据
std::vector<double> forceSensorDatas =
forceSensor->getSensorData();
LOG_INFO << "forceSensorDatas.size() = " << forceSensorDatas.size()
<< "; forceSensorDatas = " << forceSensorDatas;
//断开 ForceSensor 连接
forceSensorStatus = forceSensor->disconnect(); LOG_INFO << "disconnect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
return 0;
}
```

## getSensorStatus

```c++
ForceSensorStatus getSensorStatus(const std::string& name = "")  //获取力传感器状态
```


参数:
无。 返回:
返回 ForceSensorStatus，connect 表示连接，disconnect 表示断开连接。 

示例: 

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API; //力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo);
LOG_INFO << "connect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
//获取 ForceSensor 状态
forceSensorStatus = forceSensor->getSensorStatus();
LOG_INFO << "forceSensor Status = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
//断开 ForceSensor 连接
forceSensorStatus = forceSensor->disconnect();
LOG_INFO << "disconnect forceSensor result = " << ForceSensorStatusEnum2String(forceSensorStatus);
return 0;
}
```

## setRobotEffectorName

```c++
bool setRobotEffectorName(const std::string& robotName,const std::string& effectorName,const std::string& name = "") //设置机器人末端执行器名
```

参数:
[in]robotName 机器人名。
[in]effectorName 末端执行器名。 返回:
返回 bool，true 表示成功，false 表示失败。 

示例:

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo);
LOG_INFO << "connect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
//设置机器人末端工具名
bool ret = forceSensor->setRobotEffectorName();
if(ret){
LOG_INFO << "Set Robot Effector Name successfully!" ;
}
//断开 ForceSensor 连接
forceSensorStatus = forceSensor->disconnect(); LOG_INFO << "disconnect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
return 0;
}
```

## getContactForce 

```c++
std::vector<double> getContactForce(const std::string& name = "") //获取接触力
```

需要先调用 setRobotEffectorName 和 calculateIntrinsicError 函数，是
为了让力传感器置零，获取接触力即为获取置零后的接触力。 

参数: 无。 

返回:返回 double 型的 vector，力矩 xyz 和扭矩 rpy。 

示例:

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo);
LOG_INFO << "connect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
//设置机器人末端工具名
bool ret1 = forceSensor->setRobotEffectorName();
if(ret){
LOG_INFO << "Set Robot Effector Name successfully!" ; }
//计算内部误差
bool ret2 = forceSensor->calculateIntrinsicError();
if(ret2){
LOG_INFO << "Calculate Intrinsic Error successfully!" ; }
//获取接触力
std::vector<double> contactForce = forceSensor->getContactForce();
LOG_INFO << "contactForce: " << contactForce;
//断开 ForceSensor 连接
forceSensorStatus = forceSensor->disconnect();
LOG_INFO << "disconnect forceSensor result = " << ForceSensorStatusEnum2String(forceSensorStatus);
return 0;
}
```

**calculateIntrinsicError** 

```c++
bool calculateIntrinsicError(const std::string& name = "")
```

计算内部误差。通过此函数使力传感器置零。
参数: 无 

返回: 返回 bool，true 表示成功，false 表示失败。 示例:

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h> #include <LocalStorageRepositoryAPIKey.h>

#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API; //力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象
boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory = ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//设置 SensorInfo SensorInfo sensorInfo;
sensorInfo.uri = "127.0.0.1";
ForceSensorStatus forceSensorStatus;
//连接 ForceSensor
forceSensorStatus = forceSensor->connect(sensorInfo);
LOG_INFO << "connect forceSensor result = " <<
ForceSensorStatusEnum2String(forceSensorStatus);
//设置机器人末端工具名
bool ret1 = forceSensor->setRobotEffectorName();
if(ret){
LOG_INFO << "Set Robot Effector Name successfully!" ;
}
//计算内部误差
bool ret2 = forceSensor->calculateIntrinsicError(); if(ret2){
LOG_INFO << "Calculate Intrinsic Error successfully!" ;
}
//断开 ForceSensor 连接
forceSensorStatus = forceSensor->disconnect();
LOG_INFO << "disconnect forceSensor result = " << ForceSensorStatusEnum2String(forceSensorStatus);
return 0;
}
```

**registerListener** 

```c++
int registerListener(ForceSensorListenerInterface::pointer listenerInterface,const std::string& name = "") //注册力传感器监听器
```

参数:
[in]listenerInterface 监听对象。 返回:
返回整型，0 表示成功，-1 表示失败。 示例:

参数:
[in]listenerInterface 监听对象。 返回:
返回整型，0 表示成功，-1 表示失败。 示例:



```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h #include <ForceSensorFactoryInterface.h> //创建力传感器接口对象时,使用此头文件 ForceSensorInterface.h

#include <ForceSensorInterface.h>

#include <LocalStorageRepositoryAPIKey.h> #include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件 #include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml #include <logger/GlogConfig.h>

#include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main()
{
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象 boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory
= ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//创建监听器、注册监听(这里已经实现了一个监听器 ForceSensorListener) boost::shared_ptr<forceSensorListener> listener =
boost::shared_ptr<forceSensorListener>(new ForceSensorListener());
int ret = forceSensor->registerListener(listener); if(0 == ret){
LOG_INFO << "Register listener successfully!"
}
return 0;
}
```

## unRegisterListener

```c++
int unRegisterListener(ForceSensorListenerInterface::pointer listenerInterface,const std::string& name = "") //注销力传感器监听 
```

参数:
[in]listenerInterface 监听对象。 

返回:
返回整型，0 表示成功，-1 表示失败。

 示例:

```c++
//创建力传感器工厂对象时,使用此头文件 ForceSensorFactoryInterface.h 
#include  <ForceSensorFactoryInterface.h> 

//创建力传感器接口对象时,使用此头文件 
#include <ForceSensorInterface.h> 
#include <LocalStorageRepositoryAPIKey.h>
#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件
#include <logger/Logger.h> //如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml

#include <logger/GlogConfig.h> #include "forceSensorListener.h"

//如使用日志配置,需使用此命名空间
using namespace TIGER_COMMON_NS_API;
//力传感器使用此命名空间
using namespace BAT_FORCESENSOR_NS_API;
int main() {
//使用日志配置时使用,必须要有配置文件 glogconfig.xml GlogConfig::config(__COBOTSYS_MODULE_NAME__);
//创建力传感器工厂对象 boost::shared_ptr<ForceSensorFactoryInterface> forceSensorFactory
= ForceSensorFactoryInterface::create();
//通过力传感器工厂对象创建力传感器对象 boost::shared_ptr<ForceSensorInterface> forceSensor =
forceSensorFactory->createForceSensor("OptoForceSensor");
//创建监听器
boost::shared_ptr<forceSensorListener> listener =
boost::shared_ptr<forceSensorListener>(new ForceSensorListener());
//注册监听
int ret1 = forceSensor->registerListener(listener);
if(0 == ret1){
LOG_INFO << "Register listener successfully!"
}
//注销监听
int ret2 = forceSensor->unRegisterListener(listener); if(0 == ret2){
LOG_INFO << "unRegister listener successfully!" }
return 0;
}
```

## 监听函数

SDK 提供了监听接口文件帮助用户快速创建监听器，ForceSensor 监听接口文件所在目录是 

cobotsys_sdk/install/x86-64-install/devel/include/Bat.ForceSensor.API/ForceSe nsorListenerInterface.h 

加载监听头文件实现一个 ForceSensor 监听器。

forceSensorListener.hpp:

```c++
#ifndef COBOTSYS_SDK_FORCESENSORLISTENER_H 
#define COBOTSYS_SDK_FORCESENSORLISTENER_H

#include <logger/Logger.h>
#include <ForceSensorListenerInterface.h>
#include <ForceSensorFactoryInterface.h>
#include <ForceSensorInterface.h>

using namespace BAT_FORCESENSOR_NS_API;
class forceSensorListener:public ForceSensorListenerInterface
{
public:
typedef DBusCxxPointer<forceSensorListener> pointer;
~forceSensorListener(){}
  
forceSensorListener();
  
virtual void onContactForce(std::vector<double> contactForce, const std::string& name)
{
	LOG_INFO << "onContactForce=" << contactForce;
}
  
virtual void onForceSensorData(std::vector<double> sensorData, const std::string& name)
{
	LOG_INFO << "onForceSensorData" << sensorData;
}
  
virtual void onForceSensorStatus(ForceSensorStatus sensorStatus, const std::string& name)
{
	LOG_INFO << "onForceSensorStatus" << (int)sensorStatus;
}; 

}

#endif //COBOTSYS_SDK_FORCESENSORLISTENER_H
```



forceSensorListener.cpp:

```c++
#include "forceSensorListener.h"

forceSensorListener::forceSensorListener(){
}
void forceSensorListener::onContactForce(std::vector<double>
contactForce,
const std::string& name)
{
	LOG_INFO << "onContactForce: " << contactForce; 
}

void forceSensorListener::onForceSensorData(std::vector<double> sensorData, const std::string& name)
{
	LOG_INFO << "onForceSensorData: " << sensorData;
}
void forceSensorListener::onForceSensorStatus(ForceSensorStatus sensorStatus, const std::string& name)
{
	LOG_INFO << "onForceSensorStatus: " <<
	ForceSensorStatusEnum2String(sensorStatus);
}
```

## 常见结构 

ForceSensorStatusEnum.h 力传感器状态枚举类 

Disconnected = 0; //力传感器断开连接状态

Connected = 1; //力传感器连接状态



SensorInfoStruct.h  力传感器信息结构体 

std::string uri;  //力传感器 IP 地址或串口地址 



# Camera API

## open

```c++
int open(const std::string cameraID,const std::string& name = "")
```

通过相机ID连接相机

参数：

[in]相机ID

返回：

返回0表示成功；错误的ID失败返回3；重复连接返回7；相机busy状态时连接返回6



示例：

```c++
//创建相机工厂对象时,使用此头文件 Camera3DFactoryInterface.h
#include <Camera3DFactoryInterface.h>

//创建相机接口对象时,使用此头文件 Camera3DInterface.h
#include <Camera3DInterface.h>

#include <boost/shared_ptr.hpp>
//统一的日志管理,使用此头文件

#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//如使用日志配置,使用此命名空间

using namespace TIGER_COMMON_NS_API;
//相机驱动使用此命名空间

using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象    
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  
  /*
  * 通过工厂对象创建接口对象！
  * Cobotsys所有业务功能模块基于服务抽象,相机服务模块被编译成动态库通过工厂方法实例化,通过工厂方法可以加载不同的实现!
  * 例:此处参数填写comatrix,就是加载comatrix相机驱动；再如,填写photoneo,就是加载photoneo相机的驱动！    */
  
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  
  //打印相机连接结果
  LOG_INFO << "Connect camera result=" << ret1;
  
  //使用相机接口对象调用close,断开相机连接
  int ret2 = camera->close();
  
  //打印断开相机连接结果
  LOG_INFO << "Disconnect camera result=" << ret2;
  
  return 0;
}
```



## close

```c++
int close(const std::string& name = "")
```

断开与相机的连接

参数：

无

返回：

返回0表示与相机的连接已断开，断开失败返回-1

示例：



```c++
//创建相机工厂对象时,使用此头文件 Camera3DFactoryInterface.h
#include <Camera3DFactoryInterface.h>

//创建相机接口对象时,使用此头文件 Camera3DInterface.h
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>

//统一的日志管理,使用此头文件
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

//如使用日志配置,使用此命名空间
using namespace TIGER_COMMON_NS_API;

//相机驱动使用此命名空间
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  //创建相机工厂对象
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  /*    
  * 通过工厂对象创建接口对象！
  * Cobotsys所有业务功能模块基于服务抽象,相机服务模块被编译成动态库通过工厂方法实例化,通过工厂方法可以加载不同的实现!
  * 例:此处参数填写comatrix,就是加载comatrix相机驱动；再如,填写photoneo,就是加载photoneo相机的驱动！     */
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  
  //打印相机连接结果
  LOG_INFO << "Connect camera result=" << ret1;
  
  //使用相机接口对象调用close,断开相机连接
  int ret2 = camera->close();
  
  //打印断开相机连接结果
  LOG_INFO << "Disconnect camera result=" << ret2;
	
  return 0;

}
```



## captureSync

```c++
std::vector<boost::shared_ptr<EAGLE_DATA_NS_API::VisionInputImage>> captureSync(const std::string& name = "")
```

同步捕获3D图片。一次捕获到3张图，2D图、深度图和点云图。

参数：无

返回：返回一个vector容器，容器中包含3张图片。图片使用boost智能指针包装，VisionInputImage结构体见6.2.3视觉数据。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//如使用日志配置,使用此命名空间
using namespace TIGER_COMMON_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create(); 
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  /*
  * 使用相机接口对象调用captureSync,捕获图片-同步
  * 默认同步捕获图片会捕获3张图,2D图、点云图和深度图
  */
  std::vector<boost::shared_ptr<VisionInputImage>> images = camera->captureSync();
  
  //打印图片张数
  LOG_INFO << "Pic count" << images.size();
  
  //使用相机接口对象调用close,断开相机连接
  int ret2 = camera->close();
  
  //打印断开相机连接结果
  LOG_INFO << "Disconnect camera result=" << ret2;

  return 0;
}
```



## captureAsync

int captureAsync(const std::string& name = "")

异步捕获3D图片。一次捕获到3张图，2D图、深度图和点云图。

参数：

无

返回：

函数执行成功返回0。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

//加载本地实现的回调函数
#include "Camera3DImageCallbackResponseAdapter.h"
//如使用日志配置,使用此命名空间

using namespace TIGER_COMMON_NS_API;

//捕获图片时需用到此命名空间
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!"; 
    return -1;
  }
  /*
  * 使用相机接口对象调用captureSync,捕获图片-异步
  * 实现自己的回调函数,使用相机接口设置回调
  */
  
  boost::function<void(boost::shared_ptr < VisionInputImage > image)> imageA = [](            boost::shared_ptr <VisionInputImage> image) {
    switch (image->type) {
      case ImageType::Color: {
        LOG_INFO << "Async capture for color!";
        break;
      }
      case ImageType::Depth: {
        LOG_INFO << "Async capture for depth!";
        break;
      }
      case ImageType::Cloud: {
        LOG_INFO << "Async capture for cloud!";
        break;
      }
      default:
        break;
    }
  };
  boost::function<void()> imageScanOverCallback = []() {
    LOG_ERROR << "Image scan over callback!";
  };
  
  boost::function<void()> imageScanErrorCallback = []() {
    LOG_ERROR << "Image scan error callback!";
  };
  
  //使用相机接口对象调用setCallback,设置回调,并打印结果
  int ret2 = camera->setCallback(
    boost::shared_ptr<Camera3DCallbackInterface>(
      new Camera3DImageCallbackResponseAdapter(imageA, imageScanOverCallback, imageScanErrorCallback)));
  
  LOG_INFO << "Set callback result=" << ret2;
  
  //使用相机对象调用captureAsync,捕获图片-异步
  int ret3 = camera->captureAsync();
  LOG_INFO << "Async capture result=" << ret3;
  sleep(3);
  
  //使用相机接口对象调用unsetCallback,取消设置回调,并打印结果
  int ret4 = camera->unsetCallback();
  LOG_INFO << "Unset callback result=" << ret4;
  
  //使用相机接口对象调用close,断开相机连接,并打印结果
  int ret5 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret5;

  return 0;
}
```



capture2DSync

```c++
boost::shared_ptr<EAGLE_DATA_NS_API::VisionInputImage> capture2DSync(const std::string& name = "")
```

仅捕获2D图片

参数：

无

返回：

返回一个用boost智能指针包装的VisionInputImage结构体，该结构体见6.2.3视觉数据。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

using namespace TIGER_COMMON_NS_API;
//捕获图片时需用到此命名空间
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  //使用接口对象调用capture2DSync,捕获2D图像
  boost::shared_ptr<EAGLE_DATA_NS_API::VisionInputImage> image = camera->capture2DSync();
  if(0 != image->image.get()){
    LOG_INFO << "Capture 2D image successfully!";
  }
  
  //使用相机接口对象调用close,断开相机连接
  int ret2 = camera->close();
  //打印断开相机连接结果
  
  LOG_INFO << "Disconnect camera result=" << ret2;
  return 0;

}
```



## setCamera3DParam

```c++
int setCamera3DParam(const Camera3DParam& camera3DParam,const std::string& name = "")
```

设置3D拍照的参数。需要设置增益、rgb、hdr。

参数：

[in]camera3DParam枚举，参见6.2.4相机驱动。需要的参数如下：

camera3DParam.gain：相机增益

camera3DParam.r：红

camera3DParam.g：绿

camera3DParam.b：蓝

camera3DParam.hdrEnable：是否启用hdr

返回：

返回0表示设置成功，失败返回-1。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

using namespace TIGER_COMMON_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  
  //使用相机接口对象调用setCamera3DParam,设置3D相机参数
  Camera3DParam camera3DParam;
  camera3DParam.r = 50;
  camera3DParam.g = 50;
  camera3DParam.b = 50;
  camera3DParam.gain = 100;
  camera3DParam.enableDepth = true;
  camera3DParam.hdrEnabled = true;
  int ret2 = camera->setCamera3DParam(camera3DParam);
  LOG_INFO << "Set 3D camera param result=" << ret2;
  
  //使用相机接口对象调用close,断开相机连接,打印结果
  int ret3 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret3;
  return 0;
}
    
```



**setCamera2DParam**

int setCamera2DParam(const Camera2DParam& camera2DParam,const std::string& name = "")

设置2D拍照的参数。需要设置帧率、曝光时间、增益。

参数：

[in]camera2DParam枚举，参见6.2.4相机驱动，需要的参数如下：

camera2DParam.gain，增益

camera2DParam.frameRate，帧率

camera2DParam.exposeTime，曝光时间

Ps： 帧率可用值 ： 75， 60， 30， 15， 5， 1 ，较低的帧率可以获得更长的曝光时间。曝光时间小于 1000000/帧率 (us)。

返回：

返回0表示设置成功，失败返回-1。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
using namespace TIGER_COMMON_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  /*
  * 使用相机接口对象调用setCamera2DParam,设置2D相机参数
  * gain:增益
  * frameRate:帧率
  * exposeTime:曝光时间
  * 帧率可用值 ： 75， 60， 30， 15， 5， 1 ，较低的帧率可以获得更长的曝光时间。曝光时间小于 1000000/帧率 (us)
  */
  Camera2DParam camera2DParam;
  camera2DParam.gain = 160;
  camera2DParam.frameRate = 75;
  camera2DParam.exposeTime = 10000;
  int ret2 = camera->setCamera2DParam(camera2DParam);
  LOG_INFO << "Set 2D camera param result=" << ret2;
  
  //使用相机接口对象调用close,断开相机连接,打印结果
  int ret3 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret3;

    return 0;
}
```



## capacity

int capacity(const std::string funName,const std::string& name = "")

确认相机是否有此功能，如果有就执行，如果没有就不执行。

参数

[in]功能名，如：getDistance

返回：

返回0表示有此能力，返回1表示没有此能力

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>

//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

//如使用日志配置,使用此命名空间
using namespace TIGER_COMMON_NS_API;

//捕获图片时需用到此命名空间
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;
int main(){
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  //使用相机接口对象调用capacity,输入功能函数名,确认相机是否有此能力
  int ret2 = camera->capacity("getDistance");
  LOG_INFO << "Camera capacity result=" << ret2;
  
  //使用相机接口对象调用close,断开相机连接,并打印结果
  int ret3 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret3;
  return 0;
}
```



## getDistance

```c++
int getDistance(float& distance,const std::string& name = "")
```

测量距离。

参数：

[out]距离，单位m

返回：

返回0表示成功，返回1表示失败



```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>

//如使用日志配置,使用此命名空间
using namespace TIGER_COMMON_NS_API;
//捕获图片时需用到此命名空间
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main()
{
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  
  //调用能力函数,确认该相机是否有获取距离这个功能
  camera->capacity("getDistance");
  /*
  * 使用相机接口对象调用getDistance,获取距离
  * 示例中加载的为comatrix相机,此相机没有获取距离这个功能,所以这里的getDistance不会被调用
  * 更改相机插件为DH相机,这个相机有获取距离这个功能,这里的getDistance就会被调用
  */
  
  float distance = 0;
  int ret2 = camera->getDistance(distance);
  LOG_INFO << "Distance result=" << distance << "m";
  
  //使用相机接口对象调用close,断开相机连接,并打印结果
  int ret3 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret3;
  return 0;

}
```

## registerListener

```c++
int registerListener(Camera3DListenerInterface：：pointer listenerInterface,const std::string& name = "")
```

注册监听，当连接相机成功时onCameraOpened被调用，当调用断开连接时onCameraClosed被调用。需要开发人员自己实现一个监听器，实现监听器可参考6.3.3创建相机驱动监听器。

参数：

[in]监听器对象

返回：

返回0表示注册监听成功，失败返回-1。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//加载本地实现的监听头文件
#include "Camera3DListener.h"

using namespace TIGER_COMMON_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main()
{
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //创建监听者
  boost::shared_ptr<Camera3DListener> listener = boost::shared_ptr<Camera3DListener>(new Camera3DListener());
  
  //使用相机接口对象调用registerListener,注册监听,并打印结果
  int ret1 = camera->registerListener(listener);
  LOG_INFO << "Register listener result=" << ret1;
  
  //使用相机接口对象调用unRegisterListener,注销监听,并打印结果
  int ret2 = camera->unRegisterListener(listener);
  LOG_INFO << "Unregister listener result=" << ret2;
  
  return 0;
}
```



## unRegisterListener

```c++
int unRegisterListener(Camera3DListenerInterface：：pointer listenerInterface,const std::string& name = "")
```

注销监听，注销后不再监听相机的连接和断开。

参数：

[in]监听器对象

返回：

返回0表示注销监听成功，失败返回-1。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//加载本地实现的监听头文件
#include "Camera3DListener.h"

using namespace TIGER_COMMON_NS_API;
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main()
{
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //创建监听者
  boost::shared_ptr<Camera3DListener> listener = boost::shared_ptr<Camera3DListener>(new Camera3DListener());
  
  //使用相机接口对象调用registerListener,注册监听,并打印结果
  int ret1 = camera->registerListener(listener);
  LOG_INFO << "Register listener result=" << ret1;
  
  //使用相机接口对象调用unRegisterListener,注销监听,并打印结果
  int ret2 = camera->unRegisterListener(listener);
  LOG_INFO << "Unregister listener result=" << ret2;
  return 0;
}
```



## setCallback

```c++
int setCallback(boost：：shared_ptr<Camera3DCallbackInterface> callbackInterface)
```

参数：[in]回调函数

返回：返回0表示设置回调成功，失败返回-1。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//加载本地实现的回调函数
#include "Camera3DImageCallbackResponseAdapter.h"

//如使用日志配置,使用此命名空间
using namespace TIGER_COMMON_NS_API;
//捕获图片时需用到此命名空间
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main()
{
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  
  /*
  * 使用相机接口对象调用captureSync,捕获图片-异步
  * 实现自己的回调函数,使用相机接口设置回调
  */
  boost::function<void(boost::shared_ptr < VisionInputImage > image)> imageA = [](
    boost::shared_ptr <VisionInputImage> image) {
    switch (image->type) {
      case ImageType::Color: {
        LOG_INFO << "Async capture for color!";
        break;
      }
      case ImageType::Depth: {
        LOG_INFO << "Async capture for depth!";
        break;
      }
      case ImageType::Cloud: {
        LOG_INFO << "Async capture for cloud!";
        break;
      }
      default:
        break;
    }
  };
  
  boost::function<void()> imageScanOverCallback = []() {
    LOG_ERROR << "Image scan over callback!";
  };
  boost::function<void()> imageScanErrorCallback = []() {
    LOG_ERROR << "Image scan error callback!";
  };
  
  //使用相机接口对象调用setCallback,设置回调,并打印结果
  int ret2 = camera->setCallback(
    boost::shared_ptr<Camera3DCallbackInterface>(
      new Camera3DImageCallbackResponseAdapter(imageA, imageScanOverCallback, imageScanErrorCallback)));
  LOG_INFO << "Set callback result=" << ret2;
  
  //使用相机对象调用captureAsync,捕获图片-异步
  int ret3 = camera->captureAsync();
  LOG_INFO << "Async capture result=" << ret3;
  sleep(3);
  
  //使用相机接口对象调用unsetCallback,取消设置回调,并打印结果
  int ret4 = camera->unsetCallback();
  LOG_INFO << "Unset callback result=" << ret4;
  
  //使用相机接口对象调用close,断开相机连接,并打印结果
  int ret5 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret5;
  
  return 0;
}
```

## unsetCallback

int unsetCallback()

参数：[in]回调函数

返回：返回0表示取消回调成功，失败返回-1。

示例：

```c++
#include <Camera3DFactoryInterface.h>
#include <Camera3DInterface.h>
#include <boost/shared_ptr.hpp>
#include <logger/Logger.h>
//如需要对日志进行配置,使用此头文件,且必须要有配置文件 glogconfig.xml
#include <logger/GlogConfig.h>
//加载本地实现的回调函数
#include "Camera3DImageCallbackResponseAdapter.h"

//如使用日志配置,使用此命名空间
using namespace TIGER_COMMON_NS_API;
//捕获图片时需用到此命名空间
using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;
using namespace boost;

int main()
{
  //使用日志配置时使用,且必须要有配置文件 glogconfig.xml
  GlogConfig::config(__COBOTSYS_MODULE_NAME__);
  //创建相机工厂对象,通过工厂对象创建接口对象！本示例中加载的为comatrix相机插件
  shared_ptr<Camera3DFactoryInterface> factory = Camera3DFactoryInterface::create();
  shared_ptr<Camera3DInterface> camera = factory->createCamera3D("comatrix");
  
  //使用相机接口对象调用open,输入参数为相机ID,连接相机
  int ret1 = camera->open("COMD0511019");
  if(0 == ret1){
    LOG_INFO << "Connect camera successfully!";
  } else{
    LOG_ERROR << "Connect camera failed!";
    return -1;
  }
  
  /*
  * 使用相机接口对象调用captureSync,捕获图片-异步
  * 实现自己的回调函数,使用相机接口设置回调
  */
  boost::function<void(boost::shared_ptr < VisionInputImage > image)> imageA = [](
    boost::shared_ptr <VisionInputImage> image) {
    switch (image->type) {
    case ImageType::Color: {
      LOG_INFO << "Async capture for color!";
      break;
    }
      case ImageType::Depth: {
        LOG_INFO << "Async capture for depth!";
        break;
      }
      case ImageType::Cloud: {
        LOG_INFO << "Async capture for cloud!";
        break;
   }
      default:
        break;
    }
  };
  
  boost::function<void()> imageScanOverCallback = []() {
    LOG_ERROR << "Image scan over callback!";
  };
  boost::function<void()> imageScanErrorCallback = []() {
    LOG_ERROR << "Image scan error callback!";
  };
  
  //使用相机接口对象调用setCallback,设置回调,并打印结果
  int ret2 = camera->setCallback(
    boost::shared_ptr<Camera3DCallbackInterface>(
      new Camera3DImageCallbackResponseAdapter(imageA, imageScanOverCallback, imageScanErrorCallback)));
  LOG_INFO << "Set callback result=" << ret2;
  
  //使用相机对象调用captureAsync,捕获图片-异步
  int ret3 = camera->captureAsync();
  LOG_INFO << "Async capture result=" << ret3;
  sleep(3);
  
  //使用相机接口对象调用unsetCallback,取消设置回调,并打印结果
  int ret4 = camera->unsetCallback();
  LOG_INFO << "Unset callback result=" << ret4;
  
  //使用相机接口对象调用close,断开相机连接,并打印结果
  int ret5 = camera->close();
  LOG_INFO << "Disconnect camera result=" << ret5;

  return 0;
}
```



## 相机API数据结构

Camera2DParamStruct.h

Camera3DErrorCodeEnum.h

Camera3DParamStruct.h

Camera3DStatusEnum.h

CameraImageCallbackTypeEnum.h



**Camera2DParamStruct.h**

配置2D相机参数

枚举值：

  int gain; 增益

  int frameRate; 帧率

  int exposeTime; 曝光时间

  int reserve1; 预留1

  int reserve2; 预留2

int reserve3; 预留3



**Camera3DErrorCodeEnum.h**

3D相机错误码

枚举值：

  ERRORCODE_OK = 0， 正常

  ERRORCODE_ERROR， 错误

  ERRORCODE_DISCONNECTED， 相机断开连接

  ERRORCODE_DISCOVERY_TIMEOUT， 相机超时

  ERRORCODE_ID_MISMATCH， 相机ID不匹配

  ERRORCODE_ID_NOTFOUND， 找不到相机ID

  ERRORCODE_STATUS_BUSY， 相机状态忙

  ERRORCODE_STATUS_ERROR， 相机状态错误

ERRORCODE_UNKNONW 未知错误



**Camera3DParamStruct.h**

3D相机配置参数

枚举值：

  int r; 红

  int g; 绿

  int b; 蓝

int gain; 增益

bool enableDepth; 是否捕获深度图

bool hdrEnabled; 是否开启hdr

  int reserve1; 预留1

  int reserve2; 预留2

int reserve3; 预留3



**Camera3DStatusEnum.h**

3D相机状态

枚举值：

  CAMERA_STATUS_CLOSE = 0， 断开相机连接

  CAMERA_STATUS_SETUP， 

  CAMERA_STATUS_OPEN， 开始连接相机

  CAMERA_STATUS_CAPTURE， 相机捕获图片

  CAMERA_STATUS_CONNECTING， 正在连接相机

CAMERA_STATUS_CONNECTED 已连接相机



**CameraImageCallbackTypeEnum.h**

相机图片类型

枚举值：

  TYPE_2D = 100， 2D图片

  TYPE_POINT_CLOUD， 点云图

  TYPE_DEPTH， 深度图

  TYPE_INDEX， 图片索引

  TYPE_ERROR， 扫描错误

  TYPE_OVER， 扫描结束

  TYPE_UNKNOW 未知错误



Camera API 监听函数

SDK提供了监听接口文件帮助用户快速创建监听器，相机监听接口文件所在目录/cobotsys_sdk/install/x86-64-install/devel/include/Eagle.Camera3D.API/Camera3DListenerInterface.h。

加载监听头文件实现一个机器人监听器。如表1和表2中的参考示例，当相机连接成功时onCameraOpen被调用，当调用相机close接口时onCameraClosed被调用。

Camera3DListener.h

```c++
#ifndef EAGLECAMERA3DAPI_CAMERA3DLISTENER_H
#define EAGLECAMERA3DAPI_CAMERA3DLISTENER_H

#include <Camera3DListenerInterface.h>
#include <EagleCamera3DApi.h>

using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;

class Camera3DListener:public Camera3DListenerInterface {
  
  public:
  Camera3DListener(){}
  
  ~Camera3DListener(){}
  
  virtual void onCameraOpened();
  
  virtual void onCameraClosed();
};
#endif
```



Camera3DListener.cpp

```c++
#include <logger/Logger.h>
#include "Camera3DListener.h"

using namespace EAGLE_CAMERA3D_NS_API;

void Camera3DListener::onCameraOpened( ) {
  LOG_INFO<<"onCameraOpened be called！";
}

void Camera3DListener::onCameraClosed( ) {
  LOG_INFO<<"onCameraClosed be called！";
}
```



## CameraAPI 回调函数

SDK提供设置和取消回调（设置/取消回调见相机驱动接口setCallback/unsetCallback），需要用户自己实现回调函数。相机异步捕获时需要设置回调。如下所示提供的参考示例，为一个参考的相机驱动回调函数：

Camera3DImageCallbackResponseAdapter.h

```c++
#ifndef EAGLECAMERA3DAPI_CAMERA3DIMAGECALLBACKRESPONSEADAPTER_H
#define EAGLECAMERA3DAPI_CAMERA3DIMAGECALLBACKRESPONSEADAPTER_H

#include <EagleCamera3DApi.h>
#include <boost/function.hpp>

using namespace EAGLE_DATA_NS_API;
using namespace EAGLE_CAMERA3D_NS_API;

class Camera3DImageCallbackResponseAdapter : public Camera3DCallbackInterface {
public:

Camera3DImageCallbackResponseAdapter() {};

Camera3DImageCallbackResponseAdapter(
boost::function<void(boost::shared_ptr < VisionInputImage > image)> imageAvailableCallback,
boost::function<void()> imageScanOverCallback,
  boost::function<void()> imageScanErrorCallback)
  : _imageAvailableCallback(imageAvailableCallback),
  _imageScanErrorCallback(imageScanErrorCallback), _imageScanOverCallback(imageScanOverCallback) {};
  
  virtual ~Camera3DImageCallbackResponseAdapter() {};
  
  void captureCallback(boost::shared_ptr <VisionInputImage> image) {
    if (_imageAvailableCallback != nullptr) {
      _imageAvailableCallback(image);
    }
  };
  
  void imageScanOverCallback() {
    if (_imageScanOverCallback != nullptr) {
      _imageScanOverCallback();
    }
  };
  
  void imageScanErrorCallback() {
    if (_imageScanErrorCallback != nullptr) {
      _imageScanErrorCallback();
    }
  };
  
  public:
  boost::function<void(boost::shared_ptr < VisionInputImage > image)> _imageAvailableCallback;
  boost::function<void()> _imageScanOverCallback;
  boost::function<void()> _imageScanErrorCallback;
};

#endif //EAGLECAMERA3DAPI_CAMERA3DIMAGECALLBACKRESPONSEADAPTER_H
```

### 4 RoboticsLibrary机器人库
#### 4.1 介绍
1.RoboticsLibrary机器人库包含几个模块：  
  1)RobWork 该项目的核心部分，包括数学、运动学、轨迹规划等库;  
  2)RobWorkStudio 该项目的GUI，包括一些可视化库; 

#### 4.1.1 命名空间（Namespace）
RobWork的头文件分布在多个目录中，每个目录都有自己的命名空间。命名空间的结构反映了包含代码的目录。例如：  
  ```c  
  // Include header files:
  #include <rw/models/WorkCell.hpp>
  #include <rw/kinematics/Frame.hpp>
  using namespace rw::models; //Namespace for WorkCell included by #include<rw/models/WorkCell.hpp>
  using namespace rw::kinematics; //Namespace for Frame included by #include <rw/kinematics/Frame.hpp>
  ```  
  与Robworkstudio包相关的所有类都放置在命名空间rws中。

#### 4.1.2 链接库（Libraries）
所有RobWork的类都在一个名为rw的库中，rwlibs目录（对应rw库）下的子目录对应于不同的库。不同库可以rw_表示，比如rwlibs/xyz对应于名为rw_xyz的链接库，其命名空间为rwlibs::xyz。再举一例子：  
```
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
```  
为了编译上述头文件，需要link链接库rw_pathplanners。 

#### 4.2 序列化（Serialization）
RobWork有两种主要的序列化结构。  
首先，有一个XML DOM解析器接口，可以让您轻松地读写XML数据。这个接口在整个RobWork中用于读取和写入定义良好的数据交换格式。XML DOM解析器不应该被当作一个序列化类结构，而是一个被正确定义的储存与加载加工厂。XML DOM解析器作为当前解析格式的扩展机制特别有用，例如，用户可以编写一个插件，插入到现有的解析实现中，例如WorkCell格式。 
其次，有一个通用的序列化框架，它支持类的序列化。此框架支持对不同的储存类型进行序列化，并可用于序列化许多不同的格式，包括二进制和用户定义的格式。  

#### 4.2.1 XML DOM解析
XML DOM解析器可当作插件放入后端，因此像XML验证这样的功能可以取决于后端。boost property tree和xercess(两种XML DOM解析器)都可支持放入后端。先生成一个解析器对象：

```c
DOMParser::Ptr parser = DOMParser::make();
parser->setSchema(schemaFileName);
parser->load(instream);
DOMElem::Ptr root = parser->getRootElement();
```

root类现在可以帮助实现解析XML数据，我们假设XML文件如下：  
```c
<group someboolproperty="true" someintproperty="10">
  <elem-string> </elem-string>
  <elem-int> </elem-int>
  <elem-double-list>0.1 0.2 0.1</elem-double-list> 
</group>
```

则可以进行如下解析：
```c
  DOMELem::Ptr gelem = root->getElement("group"); // throws an error if "group" is not there
  bool boolprop = gelem->getAttributeValueAsBool("someboolproperty", true); // defaults to true
  int intprop = element->getAttributeValueAsInt("someintproperty", 2); // defaults to 2
  // iterate over all elements in group
  BOOST_FOREACH(DOMElem::Ptr child, gelem->getChildren()){
    if(child->isName("elem-string")){
        std::string str-value = child->getValue();
    } else if (child->isName("elem-int")) {
        int int-val = child->getValueAsInt();
    } else if (child->isName("elem-double-list")) {
        std::vector<double> double-val-list = child->getValueAsDoubleList(); // default seperator is space ' '          
    }
  }
```

#### 4.2.2 通用序列化框架（Generic serialization）
与上面描述的XML序列化不同，通用序列化不是集中控制的序列化。  
通用序列化有两个主要概念：存储器、可序列化。  
存储器（实际上就是类）是一个格式化数据的接口，这些数据可以从可序列化的对象/类中读取或写入，这种接口包含了输入存储和输出存储。可序列化类可以是从rw:：common:：serializable内部继承而来的，或者是从外部覆盖得到的：
```c
void rw::common::serialization::write(const THE_CLASS&, OutputArchive& oar);
void rw::common::serialization::read(THE_CLASS&, InputArchive& iar);
```
在任何情况下，都应该为单个可序列化类确定保存和加载方法。在这些方法中，应该对归档文件执行读/写操作，例如存储或恢复类的状态。使用归档文件非常简单：
```c
INIArchive ar(std::cout);
ar.write(1, "length");
ar.write(1.120, "width");
ar.write(5.120, "height");
produce:
length : 1
width : 1,120
height : 5.120
```
可以看到，标识符ID可以与要序列化的值相关联。ID能够实现类型检测，并且每个ID都需要预先定义。存储接口已经定义了一系列序列化基元（int,float,double,string）和由它们组成的向量。更多复杂的类型需要使用这些基元来组成自己的序列化。
最后，存储器（INIAchieve类）也定义了一种多元素输入的方法：
```c
void write(const KDTreeQ<VALUE_TYPE>& out, OutputArchive& oarchive) {
  oarchive.write(out._dim, "dim");
  oarchive.write((int)out._nodes->size(), "nrNodes");
  oarchive.write((boost::uint64_t)out._root, "rootId");
  RW_ASSERT(out._nrOfNodes==out._nodes->size());
  for(int i=0;i<out._nodes->size();i++){
      const TreeNode &node = (*out._nodes)[i];
      oarchive.write((boost::uint64_t)&node, "id");
      oarchive.write( node._axis, "axis");
      oarchive.write( node._deleted, "del");
      oarchive.write( node._left, "left");
      oarchive.write( node._right, "right");
      oarchive.write( node._kdnode->key, "Q");
      oarchive.write( node._kdnode->value, "value");
  }
}
```
存储器类型有两种类型：  
-rw::common::INIArchive 以INI格式打印  
-rw::common::BINArchive 以二进制压缩格式打印

#### 4.3 工作单元
工作单元是RobWork中的主要容器之一，工作单元应收集场景中所有元素或模型的状态。主要包括：    
-坐标系的运动学结构  
-所有设备状态  
-所有传感器状态  
-所有控制器状态  

#### 4.3.1 加载工作单元
RobWork支持工作单元定义为XML格式。  
下述程序加载从终端得到文件名的工作单元，如果加载工作单元失败，rw::loaders::workcellloader::load()函数将引发异常，程序将中止并显示错误消息。
```c
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/models/WorkCell.hpp>
using rw::loaders::WorkCellFactory;
using rw::models::WorkCell;
int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell>\n";
        return 1;
    }
    const std::string file = argv[1];
    WorkCell::Ptr workcell = WorkCellFactory::load(file);
    std::cout << "Workcell " << *workcell << " successfully loaded.\n";
    return 0;
}
```

#### 4.3.2 遍历工作单元的设备
一个工作单元包含了许多设备(rw::models::Device)，例如你可以用如下方法遍历工作单元的设备并且打印其名称：
```c
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <boost/foreach.hpp>
using namespace rw::models;
void printDeviceNames(const WorkCell& workcell)
{
    std::cout << "Workcell " << workcell << " contains devices:\n";
    BOOST_FOREACH(Device::Ptr device, workcell.getDevices()) {
        std::cout << "- " << device->getName() << "\n";
    }
}
```
可以使用rw::models::WorkCell::findDevice<type>(name)函数寻找具有特定名称的设备。  

#### 4.3.3 无状态模型
使用RobWork时，一个非常重要的方面是了解它对无状态模型的使用。为了说明状态模型和无状态模型的区别，我们给出了两个小代码示例：
```c
struct StateFull {
double getQ(){ return _q; }
void setQ(double q){ _q=q; }
double _q;
}
struct StateLess {
double getQ(State& state){ return state.getDouble(this); }
void setQ(double q, State& state){ state.setDouble(q, this); }
}
```
在第一个结构体(状态模型)中，Q值被当作一个成员变量存储在本地，而第二个结构体(无状态模型)的Q值被存储在不同的state类对象中。目前，Q值是如何被存储的并不重要，如果想进一步了解，查看rw::kinematics::State, rw::kinematics::StateData与rw::kinematics::StateStructure。  
无状态模型的优点在于多个线程或多个方法可以同时使用同一设备模型。例如，可视化方法可以在一种状态下可视化设备，而用户正在另一种状态下设置设备的配置。这有效地减少了与线程相关的问题，也限制了复制模型的需要。 
对于RobWork，无状态模型只有少数变量是真正被存储在本地的，所以它们也并不是完全无状态的。保存在本地的变量是动态变化的状态，例如机器人设备的配置，例如关节配置。更多的静态变量（如关节边界）仍然保存在本地对象中。 

#### 4.3.4 运动学树和状态量
工作单元的运动学结构使用坐标系树来表示(参照rw::kinematics::Frame)。运动学树的根坐标系被称作世界坐标系(rw::models::WorkCell::getWorldFrame())。每个坐标系都有自己的位置坐标(rw::math::Transform3D)，并且与其父坐标系的位置坐标有关，当改变分配给其父坐标系的位置坐标时，它可能会跟着变动。例如当一个设备的转动关节(查看rw::models::RevoluteJoint)作为一个坐标系，且其父坐标系有微小转动的时候也会跟着变动。除了旋转关节，RobWork还支持直动关节，且其值可能依赖于其他关节。  
在RobWork中，要注意的是坐标系的值并不是存储在frames中，而是明确地存储在rw::kinematics::State值类型中。给定工作单元的状态，可以使用rw::motics::frame::getTransform()计算相对于其父坐标系的转换。  
工作单元的坐标系一直被组织化地存储在运动学树中，但是一些特定坐标系的父坐标系会动态改变，这些坐标系被称为动态可连接坐标系(dynamically attachable frames，简称DAFs)。DAFs连接的父坐标系并不是存储在DAF里的，而是被存储在外部的rw::kinematics::State值类型中。因此，不同的State值可以对应于不同的树结构。给定工作单元的State，工作单元的父坐标系与子坐标系可以通过rw::kinematics::Frame::getParent()与rw::kinematics::Frame::getChildren()得到。  
由于坐标系的值以及DAFs所连接父坐标系的值都存储在工作单元之外，因此我们说工作单元是无状态模型。这使得一个工作单元和相关的数据可以在多个线程中并发使用，并且可以轻松地通信工作单元的整个状态。  
为了说明这些重要的思想，这个例子演示了如何打印工作单元的运动学树结构，对于每个坐标系，还显示了坐标系在空间中的位置：  
```c
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/Frame.hpp>
#include <string>
#include <boost/foreach.hpp>
using namespace rw::kinematics;
using rw::models::WorkCell;
using rw::math::Transform3D;
void printKinematicTree(
    Frame& frame,
    const State& state,
    const Transform3D<>& parentTransform,
    int level)
{
    const Transform3D<> transform = parentTransform * frame.getTransform(state);
    std::cout
        << std::string(level, ' ')
        << frame.getName()
        << " at "
        << transform.P()
        << "\n";
    BOOST_FOREACH(Frame& child, frame.getChildren(state)) {
        printKinematicTree(child, state, transform, level + 1);
    }
}
void printDefaultWorkCellStructure(const WorkCell& workcell)
{
    std::cout << workcell << "\n";
    printKinematicTree(
        *workcell.getWorldFrame(),
        workcell.getDefaultState(),
        Transform3D<>::identity(),
        0);
}
```
我们从这个例子中看到，给定一个State，直接计算工作单元中每个坐标系的转换。RobWork有一些实用程序，可以方便地在日常工作中计算正向运动学，如下面介绍的rw::kinematics::FKTable与rw::kinematics::FKRange。

#### 4.3.5 一组坐标系相对于世界坐标系的变换
rw::kinematics::FKTable计算正常状态下若干坐标系的正向运动学，正向运动学的坐标转换结果被存储在FKTable对象中，因此坐标系转换过程不需要重复进行。此示例显示如何有效地计算坐标系的变换：
```c
#include <rw/kinematics/FKTable.hpp>
#include <vector>
#include <boost/foreach.hpp>
using rw::math::Transform3D;
using namespace rw::kinematics;
std::vector<Transform3D<> > worldTransforms(
    const std::vector<Frame*>& frames, const State& state)
{
    FKTable fk(state);
    std::vector<Transform3D<> > result;
    BOOST_FOREACH(Frame* f, frames) {
        result.push_back(fk.get(*f));
    }
    return result;
}
```

#### 4.3.6 一对坐标系的相对变换
rw::kinematics::FKRange计算一对坐标系的相对变换。为了高效计算一对坐标系的相对变换，用来连接坐标系的运动学树路径必须计算。这个例子展示了rw::kinematics::FKRange的用法：
```c
#include <rw/kinematics/FKRange.hpp>
using rw::math::Transform3D;
using namespace rw::kinematics;
Transform3D<> frameToFrameTransform(
    const Frame& a, const Frame& b, const State& state)
{
    FKRange fk(&a, &b, state);
    return fk.get(state);
}
```
如果你重复计算一对坐标系的正向运动学以及运动学树的父子结构，你可以重新使用rw::kinematics::FKRange，这样就不需要重新计算连接坐标系的路径。例如，给定一对坐标系和一组状态，可以有效地计算与坐标系相关的相对变换，如下所示：
```c
#include <rw/kinematics/FKRange.hpp>
#include <rw/kinematics/State.hpp>
#include <boost/foreach.hpp>
#include <vector>
using rw::math::Transform3D;
using namespace rw::kinematics;
std::vector<Transform3D<> > frameToFrameTransforms(
    const Frame& a,
    const Frame& b,
    const State& tree_structure,
    const std::vector<State>& states)
{
    FKRange fk(&a, &b, tree_structure);
    std::vector<Transform3D<> > result;
    BOOST_FOREACH(const State& state, states) {
        result.push_back(fk.get(state));
    }
    return result;
}
```
frameToFrameTransform()结构体的使用函数为rw::kinematics::Kinematics::frameTframe()。

#### 4.3.7 动态连接坐标系和可移动坐标系
所谓动态连接坐标系(dynamically attachable frame，简作DAF)，即为父坐标系可改变的坐标系，用户可使用函数rw::kinematics::Frame::attachFrame()将坐标系重新连接至新的父坐标系。除了DAF自身，它可以连接至工作区中的任意坐标系。但是要注意的时必须避免将坐标系连接至它运动学树中属于其下级的子坐标系上，因为这会在运动学结构上生成循环。任何类型的坐标系都可以是DAF，用户可用以下方式检查坐标系是否是DAF：
```c
#include <rw/kinematics/Frame.hpp>
using namespace rw::kinematics;
bool isDaf(const Frame& frame)
{
    return frame.getParent() == NULL;
}
```
例如，DAF用于模拟抓取器抓取物品。该物品由DAF表示，最初连接到工作单元的某个坐标系上。当末端执行器执行抓取时，通过将物品坐标系连接到末端执行器坐标系来模拟物品的拾取。
如果DAF的父坐标系改变了，那么DAF的世界坐标信息也会跟着变换。当仿真抓取物体时，你不希望物体立刻改变空间位置，因此DAF也通常是可移动坐标系(rw::kinematics::MovableFrame)。可移动坐标系是可以相对于其父坐标系做任意坐标变换的坐标系。为了仿真抓取物体的过程，目标坐标系的父坐标系被设置为末端执行器，同时它们之间的相对坐标位置被设置成末端执行器与物体之间的位置关系。这个过程可以如下形式描述：
```c
#include <rw/kinematics/MovableFrame.hpp>
#include <rw/kinematics/FKRange.hpp>
#include <rw/math/Transform3D.hpp>
using rw::math::Transform3D;
using namespace rw::kinematics;
void gripMovableFrame(
    MovableFrame& item, Frame& gripper, State& state)
{
    FKRange fk(&gripper, &item, state);
    const Transform3D<> transform = fk.get(state);
    item.setTransform(transform, state);
    item.attachTo(&gripper, state);
}
```
这个函数接收工作单元的当前状态作为输入量，并更新此状态以反映物体的抓取。回想一下，坐标系本身是无状态的：DAF的连接关系及其相对转换的信息完全存储在状态中。
RobWork提供函数rw::kinematics::Kinematics::gripFrame()与rw::kinematics::Kinematics::gripMovableFrame()作为上述问题的实际解决方案。

#### 5 设备与设置
工作单元的算法通常不在坐标系的父子关系和值上进行操作，而是直接操作设备(rw::models::Device)和配置(rw::math::Q)。
设备(rw::models::Device)控制工作单元坐标系的子集，不同设备所控制的坐标系可能重叠，且一个设备可能包含一个或多个设备(rw::models::CompositeDevice)。比如说一个工厂应用工作单元拥有一个六自由度工业机器人设备和另一个控制机器人基座的二自由度设备。这两个设备可以合成一个八自由度设备(rw::models::CompositeDevice)。
配置(rw::math::Q)是指一个设备坐标系的值向量，配置支持标准向量操作，如加法、标量乘法、内积等。设备的配置空间是设备的一组有效配置。例如rw::models::Device类型，其配置空间总是盒形的且由包含上下角标的元组表示(查看rw::models::Device::QBox与rw::models::Device::getBounds())。
设备(rw::models::Device)的算法通常假定只有设备的配置被更改，而工作单元其余部分的状态(rw::kinematics::State)保持不变。例如，路径规划器可以返回一组关于路径的配置序列，以及其规划的公共工作单元状态。在编写或使用此类算法时，您通常会将设备的配置转换为工作单元的状态。这通常是使用rw::models::Device::setQ()与rw::models::Device::getQ()方法完成的。下例为将公共状态的配置序列转换为状态序列：
```c
#include <rw/models/Device.hpp>
#include <rw/math/Q.hpp>
#include <rw/kinematics/State.hpp>
#include <boost/foreach.hpp>
#include <vector>
using namespace rw::math;
using namespace rw::models;
using namespace rw::kinematics;
std::vector<State> getStatePath(
const Device& device,
const std::vector<Q>& path,
const State& common_state)
{
State state = common_state;
std::vector<State> result;
BOOST_FOREACH(const Q& q, path) {
    device.setQ(q, state);
    result.push_back(state);
}
return result;
}
```
其实际操作函数可以使用rw::models::Models::getStatePath()。
注意rw::models::Device::setQ()与rw::models::Device::getQ()并不在设备中保存配置：配置从状态值读取并写入，设备本身是无状态的。

#### 6 配置空间度量和其他度量
rw::math::Metric<X>是一个通用接口，其用来度量一组X类型值之间的距离。举个例子，路径规划算法通常需要度量配置之间距离的衡量指标。
RobWork中可用的衡量指标主要如下：
曼哈顿距离/Manhattan metric(rw::math::MetricFactory::makeManhattan(), rw::math::MetricFactory::makeWeightedManhattan())
欧几里得距离/Euclidean metric (rw::math::MetricFactory::makeEuclidean(), rw::math::makeWeightedEuclidean())
Infinity metric (rw::math::MetricFactory::makeInfinity(), rw::math::MetricFactory::makeWeightedInfinity())
这些内置度量方法可以被实例化为配置类型(rw::math::Q)或其他向量类型，例如rw::math::Vector3D和std::vector<double>。以下程序展示了3中不同度量方法的实例化与预期输出：
```c
#include <rw/math/Metric.hpp>
#include <rw/math/MetricFactory.hpp>
using rw::common::Ptr;
using namespace rw::math;
void metricExample()
{
typedef Vector3D<> V;
typedef Ptr<Metric<V> > VMetricPtr;
VMetricPtr m1 = MetricFactory::makeManhattan<V>();
VMetricPtr m2 = MetricFactory::makeEuclidean<V>();
VMetricPtr mInf = MetricFactory::makeInfinity<V>();
const V a(0, 0, 0);
const V b(1, 1, 1);
std::cout
    << m1->distance(a, b) << " is " << 3.0 << "\n"
    << m2->distance(a, b) << " is " << sqrt(3.0) << "\n"
    << mInf->distance(a, b) << " is " << 1 << "\n";
}
```

#### 6.1 碰撞检测-工作单元
rw::proximity功能包提供了碰撞检测的功能。当使用工作单元与坐标系的时候，首选的接口应为rw::proximity::CollisionDetector。对于每个坐标系，应该有零个或多个几何形状与其连接。当检查这两个坐标系是否彼此碰撞的时候，实际上是在测试他们的几何形状。注意，与同一坐标系相关联的两个几何形状，将不会进行碰撞检测。
在碰撞检测器内部，检测过程分为两步：
粗测试阶段：提供一个过滤器用来决定哪一对坐标系需要被精测试。
精测试阶段：执行几何图形之间的实际碰撞检查。
默认的粗测试阶段过滤器是rw::proximity::BasicFilterStrategy，其包含了在CollisionSetup(关联于rw::models::WorkCell)中被指定的规则。basicfilterstrategy维护一个要检查的坐标系对列表，可以在运行时通过include和exclude方法对其进行修改。
精测试阶段是通过函数rw::proximity::CollisionStrategy实施的，这可能将会调用类似于Yaobi或PQP的外部库。这些可能被调用的外部库都被放置于rwlibs::proximitystrategies包中。CollisionStrategy能够缓存碰撞模型，并维护一个坐标系与模型之间的关系图。
这个程序演示了如何为工作单元的默认碰撞设置构造碰撞检测器。然后，示例程序调用碰撞检测器，查看工作单元在初始状态下是否处于碰撞状态：
```c
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
using namespace rw::proximity;
using namespace rwlibs::proximitystrategies;
void collisionExample(rw::models::WorkCell& workcell)
{
    CollisionDetector detector(
    &workcell, ProximityStrategyYaobi::make());
    const bool collision =
    detector.inCollision(workcell.getDefaultState());
    std::cout
    << "Workcell "
    << workcell
    << " is in collision in its initial state: "
    << collision
    << "\n";
}
```

#### 6.2 增加/删减几何图形
rw::proximity::CollisionDetector中包含增加/删减模型工具用于在线修改碰撞检测目录。例如，如果在坐标系中检测到新对象，则可以通过以下方式添加和删除该对象： 
```c
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::geometry;
void addGeometryToDetector(CollisionDetector::Ptr cd, Frame* myframe, Geometry::Ptr myGeometry) 
{
    cd->addModel(myframe, mygeometry);
}
void removeGeometryFromDetector(CollisionDetector::Ptr cd, Frame* myframe, Geometry::Ptr myGeometry) 
{       
    cd->removeModel(myframe, mygeometry->getId())
}
```

#### 6.3 修改粗测试阶段过滤器
当仿真一个机器人从桌面抓取物体时需要修改粗测试阶段过滤器，如关闭机器人工具-物体碰撞检测且打开物体-桌面碰撞检测。
为了完成这些任务，我们需要以如下方式修改粗测试阶段过滤器：
```c
BasicFilterStrategy::Ptr broadphase = ownedPtr(new BasicFilterStrategy(workcell));
CollisionDetector::Ptr collisionDetector = ownedPtr(new CollisionDetector(workcell, ProximityStrategyYaobi::make(), broadphase));
...

//Tool frame of the robot
Frame* toolFrame = workcell->findFrame("Robot.TCP");
//Frame of the object picked up
Frame* objectFrame = workcell->findFrame("Object");
//Frame of the table on which the object previously was located.
Frame* tableFrame = workcell->findFrame("Table");
//Remove checking between objectFrame and toolFrame
broadphase->exclude(rw::kinematics::FramePair(objectFrame, toolFrame);
//Add checking between the objectFrame and the tableFrame
broadphase->include(rw::kinematics::FramePair(objectFrame, tableFrame);
```

#### 7 工作单元和配置空间约束
碰撞检测器(rw::proximity::CollisionDetector)是工作单元状态约束的一个实例。碰撞检查只是约束的一种形式，应用程序可以根据rw::proximity::CollisionDetector之外的其他类来实现它们的约束。
离散状态约束(rw::kinematics::State)的通用接口是rw::pathplanning::StateConstraint。检查是否满足状态约束的调用方法是rw::pathplanning::StateConstraint::inCollision()，方法的命名只是一种约定。约束不需要考虑工作单元的实际碰撞，用户可以从接口继承并实现他们想要的任何类型的约束。
路径规划器与其他类型规划器通常由配置(rw::math::Q)而不是工作单元状态(rw::kinematics::State)来进行操作。配置空间的离散约束接口是rw::pathplanning::QConstraint，并且检查约束是否满足的调用方法是rw::pathplanning::QConstraint::inCollision()。rw::pathplanning::StateConstraint以及rw::pathplanning::QConstraint提供构造函数和用于组合约束的函数。 
一个基于采样原理的路径规划器通常调用配置约束(rw::pathplanning::QConstraint)去验证各个配置。路径规划器通过边缘连接各个配置，并验证设备是否可以遵循边缘表示的路径。用于验证连接一对配置的配置空间路径的接口称为rw::pathplanning::QEdgeConstraint。接口上用于验证边缘的方法是rw::pathplanning::QEdgeConstraint::inCollision()。
给定一个配置约束(rw::pathplanning::QConstraint)，则一个边缘的约束(rw::pathplanning::QEdgeConstraint)可以用离散检测碰撞边缘来补全。当构造此类边缘约束(查阅rw::pathplanning::QEdgeConstraint::make())时，你可以为边缘的离散验证指定分辨率和度量，也可以使用默认值。
一个拥有边缘约束的配置约束被命名为规划约束器(rw::pathplanning::PlannerConstraint)。rw::pathplanning::PlannerConstraint::make()为简化标准碰撞检测约束的构造，提供了实用功能。
下面的程序为工作单元的第一个设备构造了一个碰撞检测器和相应的默认计划约束，程序调用规划约束器来检查边缘的配置空间从下角标到上角标是否可以遍历：
```c
#include <rw/models/Device.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
using rw::common::ownedPtr;
using rw::math::Q;
using namespace rw::models;
using rw::pathplanning::PlannerConstraint;
using rw::proximity::CollisionDetector;
using rwlibs::proximitystrategies::ProximityStrategyYaobi;
void constraintExample(WorkCell& workcell)
{
    Device::Ptr device = workcell.getDevices().front();
    const PlannerConstraint constraint = PlannerConstraint::make(
        ownedPtr( new CollisionDetector(
        &workcell, ProximityStrategyYaobi::make() ) ),
        device,
        workcell.getDefaultState());
    const Q start = device->getBounds().first;
    const Q end = device->getBounds().second;
    std::cout
        << "Start configuration is in collision: "
        << constraint.getQConstraint().inCollision(start)
        << "\n"
        << "End configuration is in collision: "
        << constraint.getQConstraint().inCollision(end)
        << "\n"
        << "Edge from start to end is in collision: "
        << constraint.getQEdgeConstraint().inCollision(start, end)
        << "\n";
}
```

#### 8 配置空间采样
配置空间采样对于路径规划及许多其他的规划算法都是很有用的工具。
配置空间采样的接口函数是rw::pathplanning::QSampler，该接口提供了约束函数，包括：
rw::pathplanning::QSampler::makeFinite():有限配置序列的确定性抽样。
rw::pathplanning::QSampler::makeUniform(): 随机均匀采样装置的配置。 
rw::pathplanning::QSampler::makeConstrained(): 约束过滤的采样。
这个例子展示了无碰撞配置采样器的构造。取样器调用设备配置空间的随机取样器，并根据配置应无碰撞的约束条件过滤这些配置。
```c
#include <rw/models/WorkCell.hpp>
#include <rw/models/Device.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/math/Q.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
using rw::common::ownedPtr;
using rw::math::Q;
using namespace rw::models;
using rw::proximity::CollisionDetector;
using namespace rw::pathplanning;
using rwlibs::proximitystrategies::ProximityStrategyYaobi;
void samplerExample(WorkCell& workcell)
{
    Device::Ptr device = workcell.getDevices().front();
    CollisionDetector::Ptr coldect = ownedPtr( new CollisionDetector(&workcell, ProximityStrategyYaobi::make()) );
    QConstraint::Ptr constraint = QConstraint::make(
        coldect,
        device,
        workcell.getDefaultState());
    QSampler::Ptr anyQ = QSampler::makeUniform(device);
    QSampler::Ptr cfreeQ = QSampler::makeConstrained(anyQ, constraint);
    for (int i = 0; i < 4; i++) {
        const Q q = cfreeQ->sample();
        std::cout
            << "Q(" << i << ") is in collision: "
            << constraint->inCollision(q) << "\n";
    }
}
```

#### 9 路径规划
rw::pathplanning::PathPlanner<From, To, Path> 是用于寻找Path类型路径的常规接口，并且其起点为From类型，目标点为To类型。
此接口的重要变化包括:
rw::pathplanning::QToQPlanner: 配置空间路径的标准规划，连接启动配置到目标配置。
rw::pathplanning::QToTPlanner: 规划将开始配置连接到任意结束配置的配置空间路径，其中空间约束表示rw::math::Transform3D<>类型的值。通常，这种类型的规划器会为设备查找路径，以使设备的工具结束于给定的目标位姿(换句话说，rw::pathplanning::QToTPlanner类型的规划器隐式地解决反向运动学问题)
rw::pathplanning::QToQSamplerPlanner:规划一个配置空间，从启动配置到表示目标区域的任意结束配置(由采样器rw::pathplanning::QSampler返回)
这三个规划器都由配置序列来表示结果路径(rw::trajectory::QPath)。
RobWork的路径规划器都被放置于rw_pathplanners库。下面的示例为工作单元的第一个设备实例化了一个路径规划器，并计划了多条路径，以实现工作单元的随机无冲突配置。完整的配置空间路径映射到相应的状态序列(rw::kinematics::State)，并写入一个文件，该文件可以使用PlayBack插件加载到RobWorkStudio中。该示例使用了前面几节中描述的配置空间采样和路径规划约束： 
配置空间采样
工作单元与配置空间约束
```c
#include <rw/models/WorkCell.hpp>
#include <rw/models/Models.hpp>
#include <rw/loaders/path/PathLoader.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/pathplanning/QToQPlanner.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
using rw::math::Q;
using namespace rw::models;
using rw::kinematics::State;
using namespace rw::pathplanning;
using rw::proximity::CollisionDetector;
using rw::loaders::PathLoader;
using rwlibs::proximitystrategies::ProximityStrategyYaobi;
using namespace rwlibs::pathplanners;
void plannerExample(WorkCell& workcell)
{
    // The common state for which to plan the paths.
    const State state = workcell.getDefaultState();
    // The first device of the workcell.
    Device::Ptr device = workcell.getDevices().front();
    CollisionDetector coldect(&workcell, ProximityStrategyYaobi::make());
    // The q constraint is to avoid collisions.
    QConstraint::Ptr constraint = QConstraint::make(&coldect, device, state);
    // the edge constraint tests the constraint on edges, eg. edge between two configurations
    QEdgeConstraintIncremental::Ptr edgeconstraint = QEdgeConstraintIncremental::makeDefault(
        constraint, device);
    // An SBL based point-to-point path planner.
    QToQPlanner::Ptr planner = SBLPlanner::makeQToQPlanner(
        SBLSetup::make(constraint, edgeconstraint, device));
    // A sampler of collision free configurations for the device.
    QSampler::Ptr cfreeQ = QSampler::makeConstrained(
        QSampler::makeUniform(device), constraint);
    // The start configuration for the path.
    Q pos = device->getQ(state);
    // Plan 10 paths to sampled collision free configurations.
    rw::trajectory::Path<Q> path;
    for (int cnt = 0; cnt < 10; cnt++) {
        const Q next = cfreeQ->sample();
        const bool ok = planner->query(pos, next, path);
        if (!ok) {
            std::cout << "Path " << cnt << " not found.\n";
            return;
        } else {
            pos = next;
        }
    }
    // Map the configurations to a sequence of states.
    const std::vector<State> states = Models::getStatePath(*device, path, state);
    // Write the sequence of states to a file.
    PathLoader::storeVelocityTimedStatePath(
        workcell, states, "ex-path-planning.rwplay");
}
```
上述例子中的路径规划器是基于SBL算法的，下面这个例子显示了更多可用路径规划器的实例化：
```c
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rwlibs/pathplanners/sbl/SBLPlanner.hpp>
#include <rwlibs/pathplanners/sbl/SBLSetup.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/arw/ARWPlanner.hpp>
using rw::models::Device;
using namespace rw::pathplanning;
using namespace rwlibs::pathplanners;
QToQPlanner::Ptr getQToQPlanner(
    Device::Ptr device,
    const PlannerConstraint constraint,
    const std::string& type)
{
    if (type == "SBL") {
        QConstraint::Ptr qconstraint = constraint.getQConstraintPtr();
        return SBLPlanner::makeQToQPlanner(SBLSetup::make(qconstraint, QEdgeConstraintIncremental::makeDefault(qconstraint,device), device));
    } else if (type == "RRT") {
        return RRTPlanner::makeQToQPlanner(constraint, device);
    } else if (type == "ARW") {
        return ARWPlanner::makeQToQPlanner(constraint, device);
    } else {
        return NULL;
    }
}
```
这些构造器函数的变化具有选项，例如控制规划器的配置空间探索。

#### 10 逆运动学
模块rw::invkin包含逆运动学(IK)求解器，IK求解器的几个主要类型如下：
rw::invkin::IterativeIK: 迭代的IK求解器。
rw::invkin::ClosedFormIK: 分析型IK求解器。
以上两种类型的IK求解器都将转换(rw::math::Transform3D<>)作为输入量，并返回设备从基底到末端执行器的配置。
迭代的IK解算器需要一个启动配置，并由此开始迭代搜索。根据启动配置和其他约束，IK解算器可能会失败，也可能成功地找到有效配置。
```c
#include <rw/invkin/JacobianIKSolver.hpp>
using rw::invkin::JacobianIKSolver;
using rw::kinematics::State;
using namespace rw::math;
using rw::models::Device;
void inverseKinematics(rw::common::Ptr<Device> device, const State& state, const Transform3D<>& target)
{
    JacobianIKSolver solver(device, state);
    std::vector<Q> solutions = solver.solve(target, state);
    for(Q q : solutions) {
        std::cout<<"Solution = "<<q<<std::endl;
    }
}
```
迭代求解的解可能取决于求解方法中给定状态下的启动配置。要搜索所有的解，迭代IK求解器可以被包装在rw::invkin::IKMetaSolver中，该求解器使用随机启动配置调用指定的次数。许多机器人都有奇异点，可以使用rw::invkin::AmbiguityResolver来解决相应问题。
IK求解器接口(rw::pathplanning::QIKSampler)在使用过程中隐藏了选择IK求解器的过程以及求解器的启动配置。下面的程序测试设备的默认迭代IK求解器。程序随机选择了十组从基底到末端执行器的设备正解。然后，程序使用默认的IK采样器检查是否为所有变换找到了IK解决方案。每个目标变换只使用少量的启动配置，因此，IK采样器可能不会总是找到一个IK解。如果IK采样器受到要求IK解必须无碰撞的约束，则仅找到目标变换的一个子集的解决方案。
```c
#include <rw/loaders/WorkCellFactory.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/QConstraint.hpp>
#include <rw/pathplanning/QIKSampler.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyYaobi.hpp>
#include <boost/foreach.hpp>
using rw::common::ownedPtr;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;
using namespace rw::math;
using namespace rw::models;
using namespace rw::pathplanning;
using rw::proximity::CollisionDetector;
using namespace rwlibs::proximitystrategies;
typedef std::vector<Transform3D<> > TransformPath;
TransformPath getRandomTargets(const Device& device, State state, int targetCnt)
{
    TransformPath result;
    QSampler::Ptr sampler = QSampler::makeUniform(device);
    for (int cnt = 0; cnt < targetCnt; cnt++) {
        device.setQ(sampler->sample(), state);
        result.push_back(device.baseTend(state));
    }
    return result;
}
void printReachableTargets(
    const TransformPath& targets,
    QIKSampler& ik)
{
    int i = 0;
    BOOST_FOREACH(const Transform3D<>& target, targets) {
        const Q q = ik.sample(target);
        std::cout << i << " " << (q.empty() ? "False" : "True") << "\n";
        ++i;
    }
}
void invkinExample(
    Device& device, const State& state, QConstraint& constraint)
{
    QIKSampler::Ptr ik_any = QIKSampler::make(&device, state, NULL, NULL, 25);
    QIKSampler::Ptr ik_cfree = QIKSampler::makeConstrained(ik_any, &constraint, 25);
    const TransformPath targets = getRandomTargets(device, state, 10);
    std::cout << "IK solutions found for targets:\n";
    printReachableTargets(targets, *ik_any);
    std::cout << "Collision free IK solutions found for targets:\n";
    printReachableTargets(targets, *ik_cfree);
}
int main(int argc, char** argv)
{
    if (argc != 2) {
        std::cout << "Usage: " << argv[0] << " <workcell-file>\n";
        exit(1);
    }
    WorkCell::Ptr workcell = WorkCellLoader::Factory::load(argv[1]);
    Device::Ptr device = workcell->getDevices().front();
    const State state = workcell->getDefaultState();
    CollisionDetector::Ptr detector = ownedPtr( new CollisionDetector(
        workcell, ProximityStrategyYaobi::make()) );
    QConstraint::Ptr constraint = QConstraint::make(detector, device, state);
    invkinExample(*device, state, *constraint);
}
```

#### 11 C++共享指针规则
RoboticsLibrary库广泛引用不同算法指针所提供的不可复制对象(比如接口提供的对象)，对象的所有权由共享指针类型rw::common::Ptr管理。如果一个对象需要访问一个不可复制的对象，该对象的构造函数通常将一个rw::common::Ptr类型作为参数。
共享指针指向的类，通常为该指针类型定义了一个便捷的使用方式。如果该类被定命名为T，则该指针类型名称将会是T::Ptr，且该指针类型将会是rw::common::Ptr<T>。
以下是这些指针类型的一些示例：
rw::math::QMetric::Ptr
rw::models::WorkCell::Ptr
rw::proximity::CollisionDetector::Ptr
rw::pathplanning::QSampler::Ptr
rw::pathplanning::QToQPlanner::Ptr
以下是这些对象的构造函数函数示例：
rw::math::MetricFactory::makeEuclidean()
rw::proximity::CollisionDetector::make()
rw::pathplanning::QSampler::makeUniform()
rwlibs::pathplanners::RRTPlanner::makeQToQPlanner()
rw::common::Ptr类型不同于标准共享指针实现，它允许指针引用堆栈分配的对象，或其他实体已完全拥有所有权的对象。为了更方便地利用这些对象，一个指针T可以隐式地转化为Ptr<T>，然而隐式构造的rw::common::Ptr类型并不拥有对象的所有权。如果rw::common::Ptr类型并不拥有实体，则你必须隐式地调用rw::common::ownedPtr()函数。以下例子展示了习惯用法：
```c
#include <rw/common/Ptr.hpp>
using namespace rw::common;
class T {};
typedef Ptr<T> TPtr;
TPtr makeT() { return ownedPtr(new T); }
```
在日常编程中，rw::common::Ptr类型的构造由各种对象的构造函数管理。只有当你需要编写自己的RobWork扩展接口的时候，才需要隐式地调用rw::common::ownedPtr()。

#### 12 机器人任务格式
RobWork包含一种抽象的任务格式，可用于表示、保存和加载任务。最基础的rwlibs::task::Task是模块化的，且可以将rw::math::Q或rw::math::Transform3D存储为任务。
RobWork中的任务基本上是一个2元组，可以描述为
Task={(Target)*, (Motion|Action|Task)*}
任务中的元素为
Target: 通常情况下分别使用rw::math::Transform3D、rw::math::Q来表示笛卡尔、机器人配置。
Motion: 描述目标之间的运动/过渡，目标可以在任意数量的运动中共享。
Action: 没有固定的解释，且可以用于具体说明事件，例如打开/关闭抓取器，需要图片或者作为同步点。
Task: 任务是递归的，子任务可以在多个任务之间共享。
下面的示例说明了如何构造一个小任务、打印出任务、将其保存到文件、重新加载并再次打印。
```c
#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/Motion.hpp>
#include <rwlibs/task/Action.hpp>
#include <rwlibs/task/Target.hpp>
#include <rwlibs/task/loader/TaskLoader.hpp>
#include <rwlibs/task/loader/TaskSaver.hpp>
#include <iostream>
using namespace rw::math;
using namespace rw::common;
using namespace rwlibs::task;
void printMotion(QMotion::Ptr motion) {
    switch (motion->motionType()) {
    case MotionType::P2P: {
        QP2PMotion::Ptr p2p = motion.cast<QP2PMotion>();
        std::cout<<"Got P2P Motion from "<<p2p->start()<<" to "<<p2p->end()<<std::endl;
        break; }
    case MotionType::Linear: {
        QLinearMotion::Ptr lin = motion.cast<QLinearMotion>();
        std::cout<<"Got Linear Motion from "<<lin->start()<<" to "<<lin->end()<<std::endl;
        break; }
    }
}
void printAction(Action::Ptr action) {
    std::cout<<"Got Action of type = "<<action->getId()<<std::endl;
}
void printTask(QTask::Ptr task) {
    std::vector<Entity::Ptr> entities = task->getEntities();
    for (std::vector<Entity::Ptr>::iterator it = entities.begin(); it != entities.end(); ++it) {
        Entity::Ptr entity = *it;
        switch (entity->entityType()) {
        case EntityType::Motion: {
            QMotion::Ptr motion = entity.cast<QMotion>();
            printMotion(motion);
            break; }
        case EntityType::Action: {
            Action::Ptr action = entity.cast<Action>();
            printAction(action);
            break; }
        case EntityType::Task: {
            QTask::Ptr task = entity.cast<QTask>();
            printTask(task);
            break; }
        }
    }
}
int main() {
    //Construct a Task
    QTask::Ptr task = ownedPtr(new QTask());
    rw::math::Q q1(1); q1(0) = 1;
    rw::math::Q q2(1); q2(0) = 2;
    rw::math::Q q3(1); q3(0) = 3;
    task->addTargetByValue(q1);
    task->addTargetByValue(q2);
    task->addTargetByValue(q3);
    std::vector<QTarget::Ptr>& targets = task->getTargets();
    task->addMotion(ownedPtr(new QP2PMotion(targets[0], targets[1])));
    task->addMotion(ownedPtr(new QLinearMotion(targets[1], targets[2])));
    task->addAction(ownedPtr(new Action(ActionType::On)));
    printTask(task);
    TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml");
    saver->save(task, "MyTask.xml");
    TaskLoader::Ptr loader = TaskLoader::Factory::getTaskLoader("xml");
    loader->load("MyTask.xml");
    QTask::Ptr task2 = loader->getQTask();
    printTask(task2);
    return 0;
}
```

#### 13 线程
在开发可以并行运行的代码时，鼓励使用rw::common::ThreadPool概念。ThreadPool为用户提供了一种更方便的方法去指定多线程应用的线程数量。通过向线程池添加工作而不是启动单独的线程，用户可以限制应用程序使用的资源。同时，应用程序能够有效地利用给定的资源。
下面的示例显示如何创建rw::common::ThreadPool，以及如何将工作添加到队列中。添加到池中的工作按添加的顺序进行处理。在本例中，可以从命令行中给出参数列表，第一个参数是要使用的线程数(在主线程旁边)，后面是任意数量的图像文件名。然后，该示例将根据用户给定的线程数，尝试并行加载图像文件。请注意，零线程是有效的，因为这将导致所有工作直接在主线程中执行。在这种情况下，工作将直接在addwork函数中执行。为了避免程序在工作完成之前结束，waitForEmptyQueue()函数将阻塞，直到没有更多的工作需要做。
在loadFile函数中isStopping()方法将被检测，如果rw::common::ThreadPool在工作完成之前停止，添加到池中的工作应定期检查isStopping()函数，以允许应用程序正常关闭，这在长时间运行的任务中尤其重要。
```c
#include <rw/common/ThreadPool.hpp>
#include <rw/loaders/ImageLoader.hpp>
using namespace rw::common;
using rw::loaders::ImageLoader;
using rw::sensor::Image;
static boost::mutex outMutex;
static boost::mutex imageMutex;
static std::vector<Image::Ptr> images;
void loadFile(ThreadPool* pool, const std::string &file) {
    {
        boost::mutex::scoped_lock lock(outMutex);
        std::cout << "Loading " << file << std::endl;
    }
    Image::Ptr image;
    try {
        image = ImageLoader::Factory::load(file);
    } catch (Exception&) {
        image = NULL;
    }
    if (pool->isStopping())
        return;
    {
        boost::mutex::scoped_lock lock(imageMutex);
        images.push_back(image);
    }
}
int main(int argc, const char* argv[]) {
    ThreadPool::Ptr pool = ownedPtr(new ThreadPool(std::atoi(argv[1])));
    for (int i = 2; i < argc; i++) {
        ThreadPool::WorkFunction work = boost::bind(&loadFile,_1,argv[i]);
        pool->addWork(work);
    }
    pool->waitForEmptyQueue();
    return 0;
}
```
在编写多线程应用程序时，通常需要添加分支到多个并行任务中，然后等待任务完成后组合结果。然后可能需要根据结果再次添加分支到新的并行任务中。而且，每个并行任务能够将其工作分解成更多的子任务。原则上，这可以发生在层次结构中，其中每个子任务可以有自己的并行子任务等等。
为了方便此类任务的编程，可以使用rw::common::ThreadTask，下面的示例与前面的示例相同，但通过使用rw::common::ThreadTask类型来实现。
考虑到LoadTask类是从rw::common::ThreadTask继承而来的，LoadTask类是用应该加载的文件名构造的，run函数的实际工作是加载了文件，然后可以使用getimage函数检索结果。
MainTask类也从rw::common::ThreadTask继承而来，该任务的主要职责是为程序输入文件启动LoadTask，该类在其运行函数中不做太多的工作。它只构造所有加载任务，并将其作为子任务添加到维护任务中。子任务一添加就开始运行，当每个子任务完成时，调用subTaskDone函数。这里只存储子任务的结果。这里只存储子任务的结果，如果需要，也可以根据所取得的结果启动新的任务。当不再有子任务运行时，将调用idle函数。idle函数是最后一次添加新子任务的机会，否则任务将结束并调用done函数。当done函数被调用了，则任务已完成，并且无法再添加更多的子任务。如果该任务有父任务，其父任务subTaskDone将被执行。可以更改rw::common::ThreadTask的行为，使其在调用idle函数后不会被调用。通过启用keep -alive选项，任务将在keep-alive选项被禁用或添加新的子任务被添加之前保持空闲状态，但是要小心使用这个选项，如果该任务没有结束，则其父任务也不会结束。
主要功能和以前一样，这次将创建一个MainTask，并使用ThreadPool去为其添加工作。当调用execute函数时，工作将添加到池中并开始执行。通过使用waitUntilDone函数，主循环将不会在MainTask完成之前结束。
请注意，多个单独的任务和worker函数可以同时使用同一个池。
由于ThreadPool能够以零线程运行（直接在主线程中执行），因此ThreadTask也是如此。如果使用零线程，整个任务将在执行返回时完成。但是不建议在ThreadTask中使用零线程，这将在程序内部产生巨大的递归深度，并且不可避免地会在某一时刻上产生堆栈大小的问题。
```c
#include <rw/common/ThreadPool.hpp>
#include <rw/common/ThreadTask.hpp>
#include <rw/loaders/ImageLoader.hpp>
using namespace rw::common;
using rw::loaders::ImageLoader;
using rw::sensor::Image;
static boost::mutex outMutex;
class LoadTask: public ThreadTask {
public:
    LoadTask(ThreadTask::Ptr parent, const std::string &file): ThreadTask(parent), _file(file) {};
    Image::Ptr getImage() const { return _image; }
    void run() {
        {
            boost::mutex::scoped_lock lock(outMutex);
            std::cout << "Loading " << _file << std::endl;
        }
        try {
            _image = ImageLoader::Factory::load(_file);
        } catch (Exception&) {
            _image = NULL;
        }
    }
private:
    const std::string _file;
    Image::Ptr _image;
};
class MainTask: public ThreadTask {
public:
    MainTask(ThreadPool::Ptr pool, const std::vector<std::string> &files): ThreadTask(pool), _files(files) {};
    std::vector<Image::Ptr> getImages() {
        boost::mutex::scoped_lock lock(_imageMutex);
        return _images;
    }
    void run() {
        for(const std::string &file : _files) {
            LoadTask::Ptr task = ownedPtr(new LoadTask(this,file));
            addSubTask(task);
        }
    }
    void subTaskDone(ThreadTask* subtask) {
        LoadTask* task = static_cast<LoadTask*>(subtask);
        boost::mutex::scoped_lock lock(_imageMutex);
        _images.push_back(task->getImage());
        // More subtasks could be added at this point
    }
    void idle() {
        boost::mutex::scoped_lock lock(outMutex);
        std::cout << "All images loaded!" << std::endl;
        // More subtasks could be added at this point
    }
    void done() {
        boost::mutex::scoped_lock lock(outMutex);
        std::cout << "Main Task ended!" << std::endl;
        // No subtasks can be added now
    }
private:
    const std::vector<std::string> _files;
    boost::mutex _imageMutex;
    std::vector<Image::Ptr> _images;
};
int main(int argc, const char* argv[]) {
    ThreadPool::Ptr pool = ownedPtr(new ThreadPool(std::atoi(argv[1])));
    std::vector<std::string> files;
    for (int i = 2; i < argc; i++)
        files.push_back(argv[i]);
    MainTask task(pool, files);
    task.execute();
    task.waitUntilDone();
    std::cout << "Images loaded: " << task.getImages().size() << std::endl;
    return 0;
}
```

#### 14 RobWorkStudio
RobWorkStudio的主要目标是为了完善RobWork工作单元的可视化功能，并维护一个能够方便用户快速安装功能的基础插件。
RobWorkStudio是一个简单方便的前端，其用于RobWork工作单元的可视化。
RobworkStudio是由丹麦南部大学马士基-麦金尼-莫勒研究所机器人学系开发的，该部门的重点研究方向是工业机器人及其应用。

#### 14.1 默认的RobWorkStudio插件
RobworkStudio中的插件都定义了功能，无论是本机插件还是用户定义的插件。
拖动插件/The Jog plugin：提供了在工作单元中拖动机器人的功能。
sec_rws_plugins_log The Log plugin：在RobWorkStudio中显示默认日志。
sec_rws_plugins_treeview The TreeView plugin：显示工作单元的坐标系结构。
sec_rws_plugins_lua The Lua plugin：提供了一个编写和执行lua脚本的简单编辑器。
sec_rws_plugins_planning The planning plugin：使用户能够调用规划器以及规划轨迹。
sec_rws_plugins_propertyview The propertyview plugin：属性视图可用于显示和编辑与工作单元中的坐标系相关联的属性。
sec_rws_plugins_playback The playback plugin：此插件允许录制和播放TimedStatePaths。
sec_rws_plugins_sensor The Sensors plugin：这个插件可以在工作单元中显示模拟摄像机和测距扫描仪的输出。

####  14.2 创建个人插件
如果想要创建个人插件，则可复制RobworkStudio内example文件夹中的示例插件。pluginUIapp提供了一个示例，其使用QT(GUI编辑器)来设计界面。pluginapp提供了一个不依赖于GUI构建工具的简单示例。
要编译插件，需要执行以下步骤：
编辑cmakelists.txt文件，确保变量“rw_root”和“rwstudio_root”指向RobWork和RobWorkStudio目录。
调用"cmake ."来生成build文件。
调用"cmake"来build插件。
一旦插件build完成，你需要告诉RobWorkStudio去加载它。这是通过编辑RobWorkStudio.ini文件来完成的。如果RobWorkStudio.ini文件不存在，可以从bin目录复制RobWorkStudio.ini.template。在模板文件中，您可能需要删除现有插件并添加以下内容：
```c
MyPlugin\DockArea=1
MyPlugin\Filename=libmyplugin
MyPlugin\Path=../../MyPlugin/
MyPlugin\Visible=false
```
确保MyPlugin\Path指向生成库的位置，并且MyPlugin\Filename是正确的，且不应添加任何文件扩展名（这是自动解决的）。
当你启动RobWorkStudio的时候，它会加载你的插件。

#### 14.3 提示
下面是一些小的有用的例子，可以从一个插件中使用。
获取RobWorkStudio实例中当前使用的碰撞检测器。
CollisionDetector *detector = getRobWorkStudio()->getCollisionDetector();

#### 14.4 插件之间的信息传输
RobWorkStudio有许多插件可以使用的事件。插件可以注册事件，例如：
getRobWorkStudio()->stateChangedEvent().add(boost::bind(&MyPlugin::stateChangedListener, this,_1), this);
它绑定MyPlugin的StateChangedListener方法来监听StateChanged事件。
要查阅有关不同事件的更多信息，请参阅RobWorkStudio api-doc。

#### 14.5 RobWorkStudio特定坐标系属性
通过XML和TUL工作单元文件格式的通用属性，RobWork允许向坐标系中添加用户特定的信息。本节将列出RobWorkStudio的特定属性，指仅对RobWorkStudio而非RobWork有意义的属性。

#### 14.5.1 相机属性
可以将描述相机针孔模型的属性添加到坐标系中，然后可以在RobWorkStudio中可视化相机视图，属性字符串如下：
"<Field of view Y> <width> <height>"
例子：
<Property name="Camera">60 640 480</Property>
您当前只能使用ctrl+键[1-9]在相机之间更改视图，1是默认的第三人称视图。
重要！
支持多个摄像头，但每个坐标系只能有一个摄像头属性！
宽度和高度没有真正的尺寸，而真正重要的是它们之间的比例。
相机在坐标系的负Z轴方向上查看。
视场以度为单位，在Y轴上定义。

#### 14.6 有用的例子
#### 14.6.1 从插件向工作单元中添加新的坐标系
该例子描述了如何将个人的坐标系通过用户插件添加至工作单元。
可以通过位于工作单元中的StateStructure实例向工作单元添加坐标系。重要的是要理解，向状态结构添加坐标系将改变工作单元的静态状态结构（动态状态是位于状态对象中的状态）。更改静态结构不会直接影响状态对象，也就是说，除了新添加的坐标系外，它们对所有坐标系都有效。有两种方法可以使旧状态对新坐标系有效，一种是用新状态分配旧状态，不过这也会覆盖在旧状态下保存的任何状态信息，比如tour robot的配置。如果要将信息保留在旧状态，并使其对新添加的坐标系有效，则需要对其进行升级。可以使用statestructure实例statestruct或其他state newstate升级state oldstate。以下是一个如何操作的示例：
```c
// using another state to upgrade
oldstate.upgradeTo(newstate); // oldstate is upgraded to the structure of the newstate
// using state structure to upgrade
oldstate = stateStruct.upgrade( oldstate );
```
下面是如何从自己的插件向工作单元添加新坐标系的示例
```c
State oldState; // this is your old state
Frame *newFrame = make_new_frame(); // create your frame
getRobWorkStudio()->getWorkcell()->getStructure()->addFrame(newFrame,parentFrame);
// now update the oldState with the new state
oldState = getRobWorkStudio()->getWorkCell()->getStructure()->upgradeState(oldState);
// now this is VERY important, remember to update the RobWorkStudio state
getRobWorkStudio()->setState(oldState);
```

#### 14.6.2 从插件添加画笔
这个例子展现了用户如何从robworkstudio插件向RobWork视图场景添加自己的画笔。首先，我们需要创建画笔，然后找到我们需要使用画笔的坐标系，最后将其添加至RWStudio的WorkCellGLDrawer。下面的小代码段展现了用户如何创建用于创建画笔的指定着色器，还可以使用DrawableFactory从文件或原始字符串（立方体、盒子、圆柱体等）创建画笔。接下来，在工作单元中搜索一个名为“myFrame”的坐标系。如果找到了坐标系，那么将在WorkCellGLDrawer (SceneGraph)中创建从myFrame到用户可绘制的映射。
```c
MyRender *renderObj = new MyRender( .. );
Drawable *drawableObj = new Drawable(boost::shared_ptr<Render>(renderObj));
Frame *myFrame = getRobWorkStudio()->getWorkCell()->findFrame("myFrame");
if(drawableFrame != NULL)
    getRobWorkStudio()->getWorkCellGLDrawer()->addDrawableToFrame(myFrame, drawableObj);
```

#### 14.6.3 从插件添加碰撞模型
```c
double scale = 1.0; // set a scale, actually not used in RobWork yet
Transform3D<> transform = makeMyTransform();
CollisionModelInfo info("myname", transform, scale);
Accessor::collisionModelInfo().get(*myFrame).push_back(info);
```

#### 14.6.4 从坐标系获得画笔
此代码段将把与坐标系frameWithDrawables相关的所有画笔复制到画笔向量组里。
```c
std::vector<Drawable*> drawables;
Frame *frameWithDrawables; // specify the frame where your drawables are placed
getWorkCellGLDrawer()->getAllDrawables(state, frameWithDrawables, drawables);
```
下面代码段将把与任意坐标系工作单元相关的所有画笔复制到画笔向量组里。
```c
std::vector<Drawable*> drawables;
getWorkCellGLDrawer()->getAllDrawables(state, getWorkCell(), drawables);
```