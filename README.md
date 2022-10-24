## VSCode tasks

In VSCode pressing `ctrl + shift + p` opens the command pallate, then type `Tasks: Run Task` to open the list of possible tasks to run. These are all defined in the `.vscode/tasks.json` file. They are the following:
- `Core` - Starts roscore so that programs can message eachother.
- `Build` - Builds all of the packages in the workspace.
- `Run` - Run an executable in one of the packages in the project.
- `Clean` - Deletes all build files.

## Creating new packages

Each package is a component of the robot software that runs independently of one another. When the `catkin build` is run, all packages in the `src` directory are built together. To create a new package, run `cakin create pkg package_name` in the `src` directory where `package_name` is the name of the new package. This will create a `CMakeLists.txt` and `package.xml` file that defines the configuration and build procedure for the package. Create a subfolder called `scripts` for python code in the package.

## Remote Control

The raspberry pi is configured for link-local connection over the ethernet port to enable direct ethernet connection over an ethernet cable. Plug in the ethernet cable and SSH to the pi with

    ssh t8@raspberrypi.local
    
Once connected to the pi, internet access can be enabled by connecting to a wireless network. This is done with the [`nmcli`](https://networkmanager.dev/docs/api/latest/nmcli.html) (network manager commandline interface) tool. To list all available wifi networks type

    nmcli connection show

To connect to a network type

    sudo nmcli dev wifi connect <network_name> password <password>

To disconnect from a network type

    sudo nmcli connection down <network_name>

## Calibrating the camera

The following commands calibrate the camera.

    rosrun ximea_ros ximea_demo
    rosrun camera_calibration cameracalibrator.py --size 9x6 --square 0.024 image:=/ximea_ros/ximea_31701651/image_raw camera:=/ximea_ros/ximea_31701651

0. Calibrate.
1. Save calibration to /tmp folder with Save Button
2. Copy calibration tar file into Desktop.
3. Unzip / extract the folder.
4. Move ost.yaml file into .ros/camera_info/ directory as ximea_SERIAL.yaml

## Dependancies

Upon booting a new pi, the following dependancies must be installed:

    pip install:
        scipy
        numpy
        rospy
        pigpio
        opencv
        

## Launching
Each time the pi is booted the usb memory limits must be disabled and the
gpio daemon started, that is performed with the `setup.bash` script. Run

    ./scripts/setup.bash

When first initialising the carousel package must be built with

    catkin build

The package defines 3 launch files for starting the robot arm.
1. `carousel.launch` waits for the cubes to stop in place before picking them up.
2. `carousel_throw.launch` waits for the cubes to stop in place, and throws them towards their dropoff locations.
3. `carousel_catch.launch` lets the cubes rotate into the gripper before placing them in their dropoff locations.

Any of these can be launched where `<launch_file>` is replaced with the above
filenames with

    source devel/setup.bash && roslaunch carousel <launch_file>

These launch files start the ximea camera package with the aruco detect file,
the dynamixel package and nodes required for the statemachine and supporting
data transformation and publishing.

## Credits & References.
The Ximea camera package "metr4202_ximea_ros" was taken from Miguel Valencia's github repo for METR4202, with
some adjustments made to the ximea_ros package. The ximea_color package was strictly used 