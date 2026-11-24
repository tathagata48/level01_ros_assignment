# tools/ - analysis scripts used while debugging the starter code

These are not needed to run the robot; they are the checks used to find and verify the
bugs listed in `../BUGS_FIXED.txt`.

| Script | What it checks | Found |
|---|---|---|
| `mesh_frames.py` | where each URDF mesh is vs. where its frame is | IMU/dummy frames swapped (bug 10), lidar offset (bug 11), base_footprint offset (bug 7) |
| `map_vs_world.py` | Gazebo world walls drawn over the occupancy map | map frame == Gazebo world frame, so AMCL's initial pose = spawn pose (0, 5, 0) |
| `drive_test.py` | commanded vs. achieved motion from `/odom` | wrong wheel_separation (bug 6), base_footprint drift while spinning (bug 7) |

Requirements: Python 3 with numpy, pillow and pyyaml (`map_vs_world.py`), plus a sourced
ROS 2 workspace for `drive_test.py`.
