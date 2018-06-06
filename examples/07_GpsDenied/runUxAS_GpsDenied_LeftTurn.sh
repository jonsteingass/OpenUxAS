#! /bin/bash

SAVE_DIR=$(pwd)

RM_DATAWORK="rm -R ./datawork"
RM_LOG="rm -R ./log"

BIN="../../../build/uxas"

mkdir -p RUNDIR_GpsDenied_LeftTurn
cd RUNDIR_GpsDenied_LeftTurn
$RM_DATAWORK
$RM_LOG
$BIN -cfgPath ../cfg_GpsDenied_LeftTurn.xml
