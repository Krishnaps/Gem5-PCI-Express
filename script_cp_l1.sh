#!/bin/bash

build/ARM/gem5.fast -d m5out/ configs/example/fs.py --machine-type=VExpress_GEM5_V1 --disk-image=$HOME/t_gem5/gem5/aarch-system-2014-10/disks/aarch32-ubuntu-natty-headless_backup.img  --kernel=$HOME/t_gem5/vmlinux --dtb-filename=$PWD/system/arm/dt/armv8_gem5_v1_1cpu.dtb  --checkpoint-restore=$1 --restore-with-cpu=DerivO3CPU --cpu-type=DerivO3CPU --num-cpus=1 --caches --l2cache --cacheline_size=128 --cpu-clock=3.2GHz --mem-size=1GB  --switch_up_lanes=1 --lanes=1  
