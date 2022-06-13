ZCORE := /path/to/zCore
RISCV64 := $(ZCORE)/ignored/target/riscv64
OpenCV_DIR := $(RISCV64)/opencv
PATH := $(PATH):$(RISCV64)/riscv64-linux-musl-cross/bin

.PHONY : build
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
