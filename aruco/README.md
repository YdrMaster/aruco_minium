# 交叉编译 OpenCV Cross compilation for riscv64 with musl libc

## 基于 zCore

1. 在 zCore 目录编译并放置依赖库

   1. 编译 ffmpeg `cargo ffmpeg --arch riscv64`
   2. 编译 opencv `cargo opencv --arch riscv64`

2. 修改路径

   1. 修改 [Makefile](Makefile):1 `ZCORE := <zCore 目录>`
   2. 修改 [cmake](riscv64-musl.cmake):13 `/path/to/zCore` 改为 zCore 目录，找 ffmpeg，注意这个必须使用绝对路径

3. 编译 `make`

   生成 riscv64 的 musl 二进制程序于 [`build/utils`](build/utils/)。

## 基于 alpine docker 编译 aruco

```bash
docker pull riscv64/apline:edge
docker create -it --name alpine alpine:edge
docker start -ai alpine
apk update
apk install opencv-dev build-base cmake
cd aruco_minium/aruco
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$PWD/install  ../
make
```

---
For full info about the library please go to
<https://www.uco.es/investiga/grupos/ava/node/26>
