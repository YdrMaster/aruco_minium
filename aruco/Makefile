ZCORE := /path/to/zCore
RISCV64 := $(ZCORE)/ignored/target/riscv64
OpenCV_DIR := $(RISCV64)/opencv
PATH := $(PATH):$(RISCV64)/riscv64-linux-musl-cross/bin

build:
	mkdir -p build
	cd build \
	&& cmake \
	   -DOpenCV_DIR=$(OpenCV_DIR) \
	   -DCMAKE_TOOLCHAIN_FILE=../riscv64-musl.cmake \
	   -DCMAKE_INSTALL_PREFIX=$(PWD)/install  \
	   .. \
	&& make -j

.PHONY : clean
clean:
	rm -rf build

.PHONY : install
install: build
	cp build/utils/aruco_test_no_gui_stream $(ZCORE)/rootfs/riscv64/bin
	cp -d build/src/libaruco.so* $(ZCORE)/rootfs/riscv64/lib
	riscv64-linux-musl-strip -s $(ZCORE)/rootfs/riscv64/lib/libaruco.so

	mkdir -p $(ZCORE)/rootfs/riscv64/aruco_res
	rm -f $(ZCORE)/rootfs/riscv64/aruco_res/*.yml
	cp ../test_files/*.yml $(ZCORE)/rootfs/riscv64/aruco_res
