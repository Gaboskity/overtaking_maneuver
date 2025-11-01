# overtaking_maneuver

## Installing
Go to your workspace's src folder
``` bash
cd /to_your_ws/src
```
Clone the repository
``` bash
git clone git@github.com:csanadneukum/overtaking_maneuver.git
```
## Building
Go to the package's  src folder
``` bash
cd /to_your_ws/src/overtaking_maneuver/src
```
Make the python script executable
``` bash
chmod +x path_creator.py
```
Go back to your workspace
``` bash
cd /to_your_ws
```

``` bash
source devel/setup.bash
```

Build the package
``` bash
catkin_make --only-pkg-with-deps overtaking_maneuver
```
## Launching

``` bash
roslaunch overtaking_maneuver maneuver.launch
```