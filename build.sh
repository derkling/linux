#!/bin/bash

AOSP=/home/queper01/work/aosp

export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-android-

export DTC_EXT=$AOSP/prebuilts/misc/linux-x86/dtc/dtc
export DTC_OVERLAY_TEST_EXT=$AOSP/prebuilts/misc/linux-x86/libufdt/ufdt_apply_overlay

export KCFLAGS=-mno-android
export KBUILD_DIFFCONFIG=akari_diffconfig

make CONFIG_BUILD_ARM64_DT_OVERLAY=y O=./out sdm845-perf_defconfig
make CONFIG_BUILD_ARM64_DT_OVERLAY=y O=./out -j32

# We need to be careful with the os_version and the os_patch_level
mkbootimg \
    --kernel out/arch/arm64/boot/Image.gz-dtb \
    --os_version "8.0.0" --os_patch_level "2018-02-01" \
    --cmdline "androidboot.hardware=qcom video=vfb:640x400,bpp=32,memsize=3072000 msm_rtb.filter=0x237 ehci-hcd.park=3 lpm_levels.sleep_disabled=1 service_locator.enable=1 swiotlb=2048 androidboot.configfs=true androidboot.usbcontroller=a600000.dwc3 zram.backend=z3fold msm_drm.dsi_display0=dsi_panel_cmd_display:config0 buildvariant=userdebug" \
    --base 0x00000000 \
    --kernel_offset 0x00008000 \
    --tags_offset 0x00000100 \
    --pagesize 4096 \
    -o boot.img

mkdtimg create dtbo.img --page_size=4096 `find out/arch/arm64/boot/dts -name "*.dtbo"`
