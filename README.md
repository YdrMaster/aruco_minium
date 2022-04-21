# aruco_minium

裁剪 aruco demo，将 `utils/aruco_test_no_gui` 改为不循环读取一张图片。

- 关键步骤在 [`markerdetector_impl`](aruco/src/markerdetector_impl.cpp) `ATTENTION` 注释处
- 相机参数文件[`camera_param.yml`](test_files/camera_param.yml)位于test_file中
- 测试用二维码图片文件[`qr.png`](test_files/qr.png)位于test_file中
- `utils/aruco_test_no_gui`中涉及文件加载路径目前均需使用绝对路径，详见 [aruco_test_no_gui](aruco/utils/aruco_test_no_gui.cpp) 中`TODO`