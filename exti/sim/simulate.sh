#!/usr/bin/env sh

######################################################################
# @author      : kistenklaus (karlsasssie@gmail.com)
# @file        : simulate
# @created     : Dienstag Apr 09, 2024 02:23:13 CEST
#
# @description : 
######################################################################

cmake -Bbuild && make -C build && ./build/sim > result.csv && python plot.py



