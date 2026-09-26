# Test results and screenshots

These screenshots come from my project report and show the terminal output and dashboard during testing. They retain the original Chinese interface, terminal text and map attribution.

## Indoor and outdoor dashboard

![Indoor trajectory example](images/indoor-trajectory.jpeg)

The indoor dashboard shows a roughly rectangular relative trajectory during a movement test, alongside IMU and LiDAR status indicators. The trajectory represents motion in a local coordinate frame.

![Outdoor GNSS mode](images/outdoor-gnss.jpeg)

An outdoor example showing the WGS84 fix, converted map coordinates and disabled indoor canvas. The displayed `Dist` is displacement from the browser's GNSS reference, not positioning error.

![Second outdoor capture](images/outdoor-gnss-second.jpeg)

A second outdoor capture shows the dashboard at another displayed position. The `age` value measures freshness relative to the bridge timestamp, rather than complete sensor-to-screen latency.

The [initial indoor dashboard](images/indoor-idle.png) shows the interface before the movement test. The vision status and feature/match fields include fixed display strings; they are interface placeholders rather than measured visual-processing results.

## Recorded topic rates

The ranges below summarise the **average rate** readings during the captured test intervals, rounded for readability.

| Topic | Visible average rates | Capture |
| --- | --- | --- |
| `/scan_reliable` | approximately 9.10–9.14 Hz | [LiDAR](images/lidar-rate.png) |
| `/imu/data_reliable` | approximately 49.95–50.04 Hz | [IMU](images/imu-rate.png) |
| `/fix` | approximately 20.00–20.15 Hz | [GNSS](images/gnss-rate.png) |
| `/odom_rf2o` | approximately 7.99–8.00 Hz | [RF2O](images/rf2o-rate.png) |

The IMU and RF2O captures include an initial topic-not-published warning followed by rate measurements. These messages are preserved. The GNSS publisher processes both GGA and RMC sentences, each of which can produce a `/fix` message. This is a plausible explanation for a roughly 20 Hz topic rate with a requested 10 Hz receiver configuration, but the screenshot does not independently verify that receiver setting.

<details>
<summary>View the four rate captures</summary>

![LiDAR topic frequency](images/lidar-rate.png)
![IMU topic frequency](images/imu-rate.png)
![GNSS topic frequency](images/gnss-rate.png)
![RF2O topic frequency](images/rf2o-rate.png)

</details>

## Startup and ROS output

| Capture | Recorded output |
| --- | --- |
| [Startup](images/startup.png) | Serial-port classification and launch-script progress |
| [Topic list](images/topics.png) | ROS2 topics discovered during the test |
| [EKF output](images/ekf-output.png) | Filtered odometry in `odom`, with child frame `base_link` |
| [RTAB-Map path](images/rtabmap-path.png) | A `/mapPath` message in the `map` frame |

The original source filenames and image hashes are recorded in [image_sources.json](image_sources.json). See [HARDWARE.md](HARDWARE.md) for the assembly photographs and [VALIDATION.md](VALIDATION.md) for the test summary, interpretation of results and further evaluation.
