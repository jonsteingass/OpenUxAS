#! /bin/bash

SAVE_DIR=$(pwd)

RM_DATAWORK="rm -R ./datawork"
RM_LOG="rm -R ./log"

BIN="../../../build/uxas"

mkdir -p RUNDIR_GpsDenied_Straight
cd RUNDIR_GpsDenied_Straight
$RM_DATAWORK
$RM_LOG
$BIN -cfgPath ../cfg_GpsDenied_Straight.xml
