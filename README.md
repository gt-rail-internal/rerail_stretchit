# rerail_stretchit
ROS Workspace for Stretch Robot 2023

The software stack is built in ROS1 for the Stretch RE Robot

##### It has 4 main components
1. [Scene Segmentation](https://github.com/gt-rail-internal/rerail_stretchit_segmentation)
2. [Grasping objects](https://github.com/gt-rail-internal/rerail_stretchit_grasp)
3. [Manipulation](https://github.com/gt-rail-internal/rerail_stretchit_manipulation)
4. [Task Executor](https://github.com/gt-rail-internal/rerail_stretchit_task_execution)

### Setup

We will be using the vcstool to setup this ROS workspace. before you begin, please read the documentation [here](https://github.com/dirk-thomas/vcstool)

> NOTE: All the commands assume that you are in the workspace root. i.e. rerail_stretchit

1. Install Dependencies 

    `cd setup_scripts/`

    `chmod +x setup_ws.sh`

    `./setup_ws.sh`

    `cd ..`

2. Validate the repos:

    `cd src`

    `vcs validate < stretch.repos`

3. Clone all the repos: 

    `vcs import < stretch.repos`

4. Build the workspace:

    `source /opt/ros/noetic/setup.bash`

    `cd ..`

    `catkin_make`
    
5. Source the workspace:

    `source ./stable/devel/setup.bash`

6. Setting up the stretch_ros package
    1. [stretch_ros](https://github.com/hello-robot/stretch_ros):
    
        This is the official repository provided by Hello Robot for the Stretch Robot. We have already cloned this in the previous steps.

        1. Export the URDF

            Navigate to the cloned repository directory.
            Run the command to generate URDF by following the document [here](https://github.com/hello-robot/stretch_ros/tree/noetic/stretch_description#exporting-a-urdf)

        2. Robot Calibration and Regenerating URDF
            Follow [this link](https://github.com/hello-robot/stretch_ros/blob/noetic/stretch_calibration/README.md#calibrate-the-stretch-re1) to see the documentation for calibrating stretch Visualize the calibration and make sure that the  total_error printed on the command line is less than 0.05
            
            Now, [regenerate the URDF](https://github.com/hello-robot/stretch_ros/blob/noetic/stretch_calibration/README.md#generate-a-new-urdf-after-changing-the-tool) 

        
        Troubleshooting

        If you encounter an error during calibration, the below points can help:

        1. If you get a dependency mismatch error for `opencv-contrib-python` then upgrade the version of `opencv-contrib-python` by running:
            ```bash
            pip install opencv-contrib-python==4.8.0.76
            ```

        2. If need to perform quick calibration then modify the `head_calibration_options` as follows:
        - Set `data_to_use` to `'use_very_little_data'`.
        - Set `fit_quality` to `'fastest_lowest_quality'`.

            Note using the above setting may lead to undesired fit error like shown below:
            ```
            [ERROR] [1695165009.479201]: The fit error is very high: 0.05577298122727921 > 0.05 (fit_error > fit_error_threshold)
            ```

7. Start all the ros nodes using the stretchit launch file
```bash
roslaunch task_executor stretchit.launch
```

8. Run the demo task
```bash
rosrun task_executor run_task.py demo_task
```








