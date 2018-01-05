#!/bin/bash

PYTHONPATH=../python:$PYTHONPATH \
./test_xworld3d \
    --x3_conf=../games/xworld3d/confs/navigation.json \
    --show_screen=1 \
    --pause_screen=1 \
    --context=1 \
    --task_groups_exclusive=1 \
    --x3_move_speed=20 
