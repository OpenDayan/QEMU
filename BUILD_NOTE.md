# Build Note

## 依赖

``` bash
pip3 install tomli
sudo apt install gdb-multiarch
```

## 构建

``` bash
mkdir build
cd build
../configure --target-list=riscv64-softmmu
make -j16
```

## 运行

``` bash
./qemu-system-riscv64 -s -S -bios /workspace/npu-firmware/bl0/bootrom.elf -machine rvtng-v0 -nographic -m 4096
```

## 调试器挂载

```bash
gdb-multiarch -ex "target remote localhost:1234" bootrom.elf
```
