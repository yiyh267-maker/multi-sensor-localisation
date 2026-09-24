# Validation status

## Existing robot evidence

The owner supplied three hardware photographs and an earlier project report containing terminal and dashboard screenshots. These document an assembled platform, discovered ROS topics, sampled topic rates, filtered odometry output and indoor/outdoor dashboard states. The exact source revision of those runs is not recorded. See [EVIDENCE.md](EVIDENCE.md).

The material does not provide a ground-truth trajectory, repeated-trial statistics, position RMSE, a verified loop-closure event or an end-to-end latency measurement. A roughly closed visual trajectory alone does not establish loop closure or localisation accuracy. GNSS displacement shown by the dashboard is distance from its current reference point, not surveyed error or total travelled distance.

## Completed release checks

- Original project retained; the release copy excludes local environments, builds and Git history.
- Comment cleanup: 21 modified files checked for executable-AST or non-comment-content equivalence; five explanatory Python docstrings translated separately.
- Syntax compilation of 51 retained project Python files without running sensor or motor code.
- Key configuration: both Flask index routes checked with unset, blank and sample keys. HTTP 503 is returned for missing configuration; configured templates and static CSS render successfully.
- Map-key URL encoding, absence of the original key, and Git ignore handling for `.env`, environments and build products checked.
- Documentation image provenance and local links checked; release ZIP checked against its file manifest.

The isolated route checks do not start ROS2, Socket.IO broadcasting or external map services. They do not establish full dashboard operation on the robot.

## Deferred hardware verification

The equipment is currently unavailable. A fresh ARM64 build and full run of the prepared release have not been completed.

When the robot is available, record the OS/package versions, source commit and hardware configuration, then:

1. Build the selected packages in a fresh workspace and recreate the web environment.
2. Confirm serial mapping, camera encoding, joystick axes and wheel direction with the chassis secured.
3. Check topic rates and TF connectivity after startup and after a restart.
4. Test indoor/outdoor mode changes, valid/invalid GNSS fixes and stale sensor data.
5. Compare a measured indoor path and surveyed outdoor reference points with recorded estimates over repeated trials.
6. Verify loop-closure events from RTAB-Map data rather than fixed dashboard strings.
7. Preserve logs before running the existing stop/reset scripts.

Future measurements should be added with their procedure and raw data; the historical screenshots should remain clearly identified as historical evidence.
