#! /bin/bash

SAVE_DIR=$(pwd)

RM_DATAWORK="rm -R ./datawork"
RM_LOG="rm -R ./log"

BIN="../../../build_debug/uxas"

mkdir -p RUNDIR_GpsDenied
cd RUNDIR_GpsDenied
$RM_DATAWORK
$RM_LOG
$BIN -cfgPath ../cfg_GpsDenied.xml
