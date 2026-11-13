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

假定 QEMU 与 `dayan-base` 为同级目录，以下命令在 QEMU 的 `build/` 目录执行：

``` bash
./qemu-system-riscv64 -s -S -bios ../../dayan-base/device/bl0/bootrom.elf -machine dayan-v0 -nographic -m 4096
```

如果目录布局不同，请通过 `-bios` 指定实际的 Boot ROM 路径。

## 调试器挂载

```bash
gdb-multiarch -ex "target remote localhost:1234" bootrom.elf
```
