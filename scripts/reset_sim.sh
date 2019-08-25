#!/bin/bash
set -e
set -x

rosservice call /gazebo/reset_simulation "{}"
gnome-terminal -e "python judge/judgeServer.py"
gnome-terminal -e "python judge/visualizeWindow.py"
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy
