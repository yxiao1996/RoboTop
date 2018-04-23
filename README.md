# RoboTop: Top-Level Codes for Robocon

* 我们借助了来自MIT的duckietown项目的代码结构，你可以在以下的地址拜访他们的优秀工作：
  > https://github.com/duckietown/Software
  
* ### 代码结构

  * #### 主程序包（robocon）
  
    * 主程序包（robocon）包含顶层代码的总体启动文件（.launch）。
    
  * #### 通用工具（Infrastructure）
  
    * 通用工具（Infrastucture）包含项目的公用节点
      1. 遥控器节点：joy_mapper；
      2. 相机接口；
      3. 节点通信接口：robocon_msg；
      4. 状态机节点：fsm

  *  #### 循线控制（CCDcontrol）
  
     * 循线控制（CCDcontrol）包含实现循线功能的节点
       1. CCD解码节点：ccd_decoder;
 
  * #### 目标检测（objectdetection）
  
    * 目标检测（objectdetection）包含实现目标检测功能的节点
      1. 目标检测节点：detector；
  
  * #### 模板（template）
  
    * 程序包模板（template）包含编写ROS节点的模板
      1. ROS节点模板：pkg_name；
      
* ### 依赖项

  1. ROS kinetic；
  2. OpenCV；

Raspi Branch
