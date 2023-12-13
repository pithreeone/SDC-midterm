# __Self-Driving-Car: radar localization__
## 1. How to run the code.
launch the roslaunch file: localization.launch  
`roslaunch localization localization.launch`

## 2. Parameters
### 2.1 Node: radar
  - `~maximum_distance` (`double`, default: 500) - maximum distance in radar poind cloud. The distance larger than this value will be deleted.
  - `~intensity_thres` (`double`, default: 95) - The intensity of poind cloud lower than the value will be ignored.

### 2.2 Node: localization
  - `~filter_type` (`int`, default: 1) - Use Low pass filter or Kalman filter. 0: Low pass filter, 1: Kalman filter
  - `~scan_matching_type` (`int`, default: 0) - Use ICP or NDT. 0: ICP, 1: NDT
  - `~low_pass_filter_gain` (`double`, default: 1.0) - The parameter use in low pass filter
  - `~low_pass_filter_angle_gain` (`double`, default: 1.0) - The parameter use in low pass filter
