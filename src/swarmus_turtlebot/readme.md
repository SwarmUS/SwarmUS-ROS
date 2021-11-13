# SwarmUS implementation for TurtleBot3

This package contains a working implementation of a HiveMindBridge and various bringup files to be used on a TurtleBot3 
robot in order to implement a working robot swarm using the SwarmUS platform. This package can therefore be seen as 
an example of the way the SwarmUS is intended to be used.

## Setup of the embedded environment
The first step is to deploy this package to the Pi. There are a great many ways to do that. For simplicity's sake,
we're using FileZilla or any SCP or FTP tool. Copy the root of this folder (`swarmus_turtlebot`) to `~/catkin_ws/src/`.

The next step is to install the dependencies using _rosdep_ on the robot.

```bash
cd catkin_ws
rosdep update
apt update
rosdep install --from-paths src --ignore-src -r -y
```

Next, compile and install the HiveMindBridge library.

```bash
mkdir ~/dependencies
cd ~/dependencies
git clone https://github.com/SwarmUS/HiveMindBridge.git
cd HiveMindBridge
mkdir build
cd build
cmake ..
make -j 2 # Make sure to limit the number of jobs to avoid flooding the memory of the Pi
sudo make install
```

You may now build the `swarmus_turtlebot` package.

```bash
cd ~/catkin_ws
catkin_make -j 1 # Make sure to limit the number of jobs to avoid flooding the memory of the Pi
```

Finally, setup the Pi's network like shown in the _netplan_ configuration file included in this package's _tools_ 
folder (`tools/50-cloud-inti-yaml`).

## Using the TB with a HiveBoard
Two bringup files must be launched in order to use the TB with a HiveBoard. The first launchfile is just the ordinary 
bringup as provided by Turtlebot:

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
```

The second file is provided in this package:

```bash
roslaunch swarmus_turtlebot turtlebot.launch
```

Voilà, you may now connect the HiveBoard to the TB via Ethernet. A greet procedure should now succeed.

## Systemd service
You can use the provided swarmus-turtlebot.service to start the ros nodes. Make sure your bashrc sources the catkin_ws, export the required variables. Also make sure it can be executed even when non-interactive.

To add the service you can use those commands.
```sh
sudo cp swarmus-turtlebot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable swarmus-turtlebot
```
Then you can reboot or start the service manually using `sudo start enable swarmus-turtlebot`.


To be runable when non-interactive, you can remove those lines in your .bashrc if they are present.
```sh
case $- in
    *i*) ;;
      *) return;;
esac
```