#!/bin/sh
set -e
cd libopencm3-upstream
#make

mkdir -p ../libopencm3/lib
cp lib/libopencm3_efm32hg.a ../libopencm3/lib
cp lib/cortex-m-generic.ld ../libopencm3/lib/cortex-m-generic.ld

for dir in libopencmsis libopencmsis/dispatch libopencmsis/efm32/hg \
           libopencm3 \
           libopencm3/cm3 libopencm3/dispatch \
           libopencm3/efm32 libopencm3/efm32/common libopencm3/efm32/hg \
           libopencm3/usb libopencm3/usb/dwc
do
    mkdir -p ../libopencm3/include/$dir
    cp include/$dir/* ../libopencm3/include/$dir/ > /dev/null 2>&1|| true
done
