#!/bin/bash

echo "Cleaning up"
find ~/PX4-Autopilot/ -name "*sdu_drone*" -delete

sed -i '/1003_sdu_drone.hil/d' /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt

echo "Symlink"
#ln -s /home/$USER/vd_workspace/src/vd_gazebo/init.d-posix/* /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s /home/$USER/eit_ws/src/eit_playground/init.d-posix/* /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/
ln -s /home/$USER/eit_ws/src/eit_playground/init.d/* /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/
ln -s /home/$USER/eit_ws/src/eit_playground/mixers/* /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/mixers/
ln -s /home/$USER/eit_ws/src/eit_playground/models/* /home/$USER/PX4-Autopilot/Tools/sitl_gazebo/models/

echo "CMakeLists changes"
sed -i '/1002_standard_vtol.hil/a \ \ 1003_sdu_drone.hil' /home/$USER/PX4-Autopilot/ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt
