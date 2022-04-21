### 交叉编译 OpenCV Cross compilation for riscv64 with musl libc
#### Install cross compilation tools:
```
wget https://musl.cc/riscv64-linux-musl-cross.tgz
tar xvf riscv64-linux-musl-cross.tgz
export PATH=/path/to/riscv64-linux-musl-cross/bin:$PATH
```
#### Getting OpenCV Source Code
```
git clone https://github.com/elliott10/opencv.git
```
#### Building OpenCV
```
mkdir -p opencv/build && cd opencv/build
cmake -DCMAKE_TOOLCHAIN_FILE=../platforms/linux/riscv64-musl-gcc.toolchain.cmake -DCMAKE_INSTALL_PREFIX=$PWD/install  ../
make -j8
make install
```
The binaries will be placed in this directory: `opencv/build/bin/`

### 交叉编译 aruco
```
mkdir build; cd  build ;
OpenCV_DIR=/path/to/opencv/build/ cmake -DOpenCV_DIR=/path/to/opencv/build/ \
-DCMAKE_TOOLCHAIN_FILE=../riscv64-musl.cmake -DCMAKE_INSTALL_PREFIX=$PWD/install  ../
make -j
```
生成riscv64的musl二进制程序于`aruco/build/utils/aruco_test`

---
fsdafsdsadda
For full info about the library please go to 
https://www.uco.es/investiga/grupos/ava/node/26
