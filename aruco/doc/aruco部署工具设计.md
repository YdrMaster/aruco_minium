# aruco部署工具设计【临时】

整理现有资源，更新现有目录结构。实现在树莓派中自动化编译以及部署aruco程序。

## 目录结构
```
|_ run-aruco.sh         ———— 自启动脚本
|_ aruco.conf           ———— 配置参数项目
|_ bin/                 ———— 存放编译后的执行程序       
    |_ xap_aruco/
        |_ xap_aruco
        |_ ...
    |_ utils/
        |_ ...
    |_ utils_calibration/
        |_ aruco_calibration
        |_ ...
|_ camera_configs/      ———— 相机参数文件
    |_ cam1.yml
    |_ cam2.yml
    |_ ...

```
## 配置运行参数

可通过修改`aruco.conf` 文件，调整运行状态
1. `xap_app_name` 默认指定到自启动的主程序，需填写相对bin文件夹的路径。
1. `cal_app_name` 相机参数校准程序，需填写相对bin文件夹的路径。
2. `rtsp_server` 指定相机模组设备的名称，如admin:admin@8.8.8.8:9999/live 
3. `camera_config` 指定相机模组使用的参数文件，相应文件存放在 `camera_configs` 文件夹

示例如下：

```ini
xap_app_name=xap_aruco/xap_aruco
cal_app_name=utils_calibration/aruco_calibration
rtsp_server=admin:admin@8.8.8.8:8923/live
camera_config=xyz.yml
size=0.03
```

## 在树莓派上部署

1. 安装 `opencv-4.5.5` 版本，以及相关依赖，克隆本仓库到本地
2. 增加本仓库中 `run-aruco.sh` 运行权限：`chmod +x run-aruco.sh`
3. 生成新运行实例，若 `aruco` 未编译，会自动完成编译：`./run-aruco.sh install rtsp-0`
4. 若未出现问题，则会自动生成 `rtsp-0` 文件夹，并在其中生成 `aruco.conf` 文件。修改其中内容，满足需求
5. 进入到 `rtsp-0`文件夹，运行相机校准命令，`./run-aruco.sh cal`，获取相机参数，或者将参数文件复制到`camera_configs`中。
6. 增加到系统自动运行服务：`./run-aruco.sh install`
7. 重启树莓派或者手动运行：`./run-arco.sh start`

本脚本支持命令包括：
- `run-aruco.sh init [app-name]` 初始化目录环境，提供编译后输出目录
- `run-aruco.sh install` 增加设备自启动服务
- `run-aruco.sh start` 启动进程
- `run-aruco.sh stop`  停止本脚本以及aruco进程
- `run-aruco.sh restart`  重启本脚本和aruco进程，适用于修改配置文件后的更新
- `run-aruco.sh cal`    启动校准相机进程，根据配置文件自动输出到相应文件保存


## 监听机理

在脚本自动启动后，会读取配置文件中指定的相机模组数据，并通过 `ping` 指令判断相机模组是否上线。当满足上线条件后，会自动根据配置参数及命令行开关等运行 `xap_app_name` 指定的程序。同时还会使用 `ping` 指令判断相机是否在线。若发生相机下线情况，会自动关闭 `xap_app_name` 程序，并重复上面等待过程。
