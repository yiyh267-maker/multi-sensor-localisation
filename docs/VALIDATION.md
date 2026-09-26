# Testing and evaluation

## Completed system tests

I tested the assembled robot with the sensor drivers, localisation nodes and browser dashboard running together. The tests covered multi-sensor acquisition, ROS2 topic publishing, indoor relative trajectory display, outdoor GNSS positioning and switching between dashboard modes. Terminal output and dashboard screenshots from my project report are collected in [EVIDENCE.md](EVIDENCE.md).

| Test | Recorded result |
| --- | --- |
| Sensor acquisition | LiDAR, IMU and GNSS topics were published during the test runs. |
| Local motion estimation | RF2O published laser odometry, and the EKF produced filtered odometry in the `odom` frame. |
| RTAB-Map output | A `/mapPath` message was recorded in the `map` frame. |
| Indoor display | The dashboard displayed a roughly rectangular relative trajectory during a robot movement test. |
| Outdoor display | GNSS coordinates were shown as WGS84 values and converted GCJ-02 map positions. |
| Mode switching | Switching to outdoor mode disabled the indoor trajectory canvas. |

The sampled topic rates were approximately 9.1 Hz for LiDAR, 50 Hz for IMU, 8 Hz for RF2O odometry and 20 Hz for GNSS `/fix` messages. Both GGA and RMC sentences can generate `/fix` messages, so that publication rate differs from the receiver's independent position-update rate.

## Interpretation of results

The tests demonstrate system integration and the indoor/outdoor display workflow. Position accuracy, end-to-end latency and loop-closure performance were not quantitatively evaluated. A closed-looking trajectory is not itself evidence of a loop-closure correction. The dashboard's GNSS distance is displacement from its reference position, rather than positioning error or total distance travelled.

The report discusses corridor-related drift, GNSS reception, monocular vision constraints and serial-device discovery time. These observations and proposed improvements are summarised in the [README](../README.md#limitations-and-future-work).

## Further evaluation

- Compare estimated indoor trajectories with measured reference paths over repeated trials.
- Compare outdoor fixes with surveyed reference positions under different reception conditions.
- Measure sensor-to-dashboard latency and behaviour during stale or invalid sensor input.
- Record RTAB-Map loop-closure events and compare trajectories before and after correction.

For each experiment, record the hardware configuration, software versions, source commit and measurement procedure. Preserve the raw data and logs before using the stop or reset scripts, which delete previous run data.
