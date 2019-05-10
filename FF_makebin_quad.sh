#! /bin/sh

# clean

make px4fmu-v3_altaquad clean
make px4fmu-v3_altaquad

echo Removing old px4.bin
rm px4.bin 2> /dev/null
/home/px4dev/gcc-arm-none-eabi-7-2017-q4-major/bin/arm-none-eabi-objcopy -I elf32-little build/nuttx_px4fmu-v3_altaquad/nuttx_px4fmu-v3_default.elf -O binary px4.bin

echo Wrote $(pwd)/px4.bin


