# Lightly modified from https://docs.duckietown.org/DT19/AIDO/out/ros_baseline.html
FROM duckietown/rpi-duckiebot-base:master19

RUN ["cross-build-start"]

#### START CUSTOM CATKIN_WS ####
RUN /bin/bash -c "mkdir -p custom_ws/src/"

# Copy or init your packages in here
COPY mp3_lane_filter_ht custom_ws/src/mp3_lane_filter_ht
COPY amirfr.yaml /data/config/calibrations/kinematics/amirfr.yaml
RUN chmod +x custom_ws/src/mp3_lane_filter_ht/src/mp3_ht_lane_filter.py

RUN /bin/bash -c "cd custom_ws/src/"

# Do not change the below line! This ensures that your workspace is 
# overlayed on top of the Duckietown stack!  
# MAKE sure this line is present in the build: 
# This workspace overlays: /home/software/catkin_ws/devel;/opt/ros/kinetic
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_init_workspace && cd ../.."
RUN /bin/bash -c "source /home/software/catkin_ws/devel/setup.bash && catkin_make -j -C custom_ws/"
RUN /bin/bash -c "source /home/software/custom_ws/devel/setup.bash"

#### END CUSTOM CATKIN_WS ####

RUN ["cross-build-end"]
