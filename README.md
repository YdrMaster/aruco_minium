# aruco_minium

裁剪 aruco demo，将 `utils/aruco_test_no_gui` 改为不循环读取一张图片。

- 关键步骤在 [`markerdetector_impl`](aruco/src/markerdetector_impl.cpp) `ATTENTION` 注释处
- 缺相机参数文件，在 [aruco_test_no_gui](aruco/utils/aruco_test_no_gui.cpp) `TODO` 处
