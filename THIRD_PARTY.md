# Third-party code and licence inventory

The project combines original integration work, AI-assisted code and existing robotics software. This file records provenance visible in the supplied tree; it does not assign a new licence or replace upstream notices.

## Main components

| Component | Local evidence | Status |
| --- | --- | --- |
| RF2O | [Source README](Indoor/ros2_ws/src/rf2o_laser_odometry/README.md), author comments, [LICENSE](Indoor/ros2_ws/src/rf2o_laser_odometry/LICENSE) | Includes GPL version 3 text; manifest declares GPL v3. Author notices retained. |
| RPLIDAR driver | [LICENSE](LiDAR/ros2_ws/src/rplidar_ros/LICENSE), `package.xml` | RoboPeak/Slamtec copyright notice and BSD-style licence text retained. |
| Additional RPLIDAR copy | [LICENSE](LiDAR/src/rplidar_ros-dev-ros2/LICENSE) | Separate bundled vendor copy retained. |
| IMU serial library | `IMU/YbImuLib/` and `LIO2D/ros2_ws/src/ybimu_ros2/ybimu_ros2/YbImuLib/` | Hardware protocol implementation supplied with the project; no standalone licence file found in these directories. Provenance/terms require confirmation. |
| Other vendor packages and assets | `LiDAR/src/` | Multiple packages have placeholder licence fields. Gmapping manifests declare `CreativeCommons-by-nc-sa-2.0`. These declarations are recorded without assuming they cover every bundled file. |
| RTAB-Map and robot_localization | Main launch script and package use | External runtime dependencies; implementations are not included as this project's original work. |
| AMap and Socket.IO browser client | Script URLs in the dashboard template | External services/libraries. Existing map attribution is retained in evidence images. |

The full list of manifest declarations is in [PACKAGE_LICENSE_INVENTORY.md](docs/PACKAGE_LICENSE_INVENTORY.md).

## Metadata that remains unresolved

- `LIO2D/ros2_ws/src/ybimu_ros2/package.xml` has a placeholder licence, while its `setup.py` declares MIT.
- `LIO2D/ros2_ws/src/lio2d_bringup/package.xml` and many vendor packages have placeholder licence fields.
- The image converter manifest declares Apache-2.0; this is recorded as existing metadata, not independently verified authorship.
- A project-wide licence has not been selected. Existing MIT declarations on some packages do not establish that the whole collection is MIT-licensed.

These are publication-preparation items. Confirm the origin and redistribution terms of the affected code/assets before making the complete collection public. Original third-party notices and licence files remain unchanged. Nested Git metadata was excluded from the release copy; source files were retained as ordinary files.
