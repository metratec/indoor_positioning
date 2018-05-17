# indoor_positioning

## I. About
This is a ROS package for the metraTec Indoor Positioning System. You can use the regular IPS for zone location of your robot 
or the IPS+ products for 3D-position-estimation using UWB ranging.

![ips_plus](screenshots/ips_plus.png?raw=true "IPS+ example")

The image shows an example of the indoor positioning functionality. Four RF beacons are placed in the environment that
localize a corresponding receiver in the vicinity. The IPS displays the current (user-defined) zone that the
receiver is in (green rectangle). Additionally, using the IPS+ position estimation, the actual 3D-position of 
the receiver is displayed (pink sphere).

## II. Installation

### From ROS binaries (preferred method):  
Simply run the following command where you change ```ROSDISTRO``` to whatever distribution you are using.
```
$ sudo apt-get install ros-ROSDISTRO-indoor-positioning
```

### From source:  
If you can't use the binary release, for example when you are using an older version of ROS, you can still manually 
build the package yourself.

First, clone the repository into the source folder of your catkin workspace:
```
$ git clone https://github.com/metratec/indoor_positioning.git YOUR_CATKIN_WS/src/indoor_positioning
```
Install python dependencies via:
```
$ pip install -r requirements.txt
```
Now you should be able to use the package like any other (catkin) package in your workspace.

## III. Usage
The ```receiver.py``` node handles communication with the receiver and publishes incoming messages. Therefore you have to run this node
in order to use the IPS or IPS+ node. Then you can run either the ```positioning.py``` node for IPS or the 
```positioning_plus.py``` node for IPS+ (or both, theoretically).

### Setup
To be able to use the nodes for zone location (IPS) or UWB positioning (IPS+) you have to setup your environment in a YAML configuration file.
Take a look at the ```config/zones_template.yml``` example for information about the structure and content of this config file.

### Launch files
There are two launch files you can use right away to test this package with default parameters. 
The ```ips.launch``` file starts the receiver and the IPS node for zone location. 
The ```ips_plus.launch``` file starts the receiver and the IPS+ node for UWB position estimation. 
For both you have to specify whether you use a TCP or USB receiver and how they are connected.

```
# launching the IPS node using a metraTec USB receiver
$ roslaunch indoor_positioning ips.launch rec_name:=port rec_value:=/dev/ttyUSB0
# launching the IPS+ node using a TCP receiver
$ roslaunch indoor_positioning ips_plus.launch rec_name:=host rec_value:=192.168.2.223  
```
If you want to use non-default parameters for any of the nodes you have to set them in the ROS parameter server or 
write your own launch files that set the respective parameters however you want. 
Take a look at the two launch files mentioned above as an example.

_Note:_ by setting the ```do_val``` parameter of the launch files to ```True``` you can simultaneously start rviz with
predefined settings to display the defined beacon positions, current zone and/or estimated receiver position.

### Nodes

#### receiver.py
Use this node to establish a connection with the metraTec IPS receiver or USB stick. You have to pass the USB port the
stick is connected to or the IP address of the regular receiver as a command line argument or as a private parameter
when you are using a launch file.

Running from command line:  
```
$ rosrun indoor_positioning receiver.py TYPE REQUIRED OPTIONAL
# For example:
$ rosrun indoor_positioning receiver.py usb /dev/ttyUSB0
$ rosrun indoor_positioning receiver.py tcp 192.168.2.223
```
##### Subscribed topics:
- ips/receiver/send (std_msgs/String):  
Message to be sent over the TCP or serial connection to the IPS receiver or USB stick

##### Published topics:
- ips/receiver/raw (indoor_positioning/StringStamped):  
Raw messages received from the receiver or USB stick

##### Parameters:
- ~host (string, default=None):  
IP address of the connected receiver

- ~port (with host: int, default=10001; without host: string, default=None):  
Port for the TCP connection if '~host' parameter is present, otherwise specifies the USB port of the stick

- ~baudrate (int, default=115200):  
Baudrate to use for serial communication with USB receiver

#### positioning.py (IPS)
Use this node to perform indoor zone location using the metraTec IPS tracking system. Prerequisites for using this node
is a running receiver-node that handles communication with the receiver and thus with the beacons in the vicinity.
Also, make sure that you have defined your zones correctly in the YAML config file.

##### Subscribed topics:
- ips/receiver/raw (indoor_positioning/StringStamped):  
Raw messages received by the UWB receiver

##### Published topics:
- ips/receiver/current_zone/name (indoor_positioning/StringStamped):  
Name of the zone the receiver is currently in

- ips/receiver/current_zone/polygon (geometry_msgs/PolygonStamped):  
Polygon comprising the current zone

- ips/receiver/zone_leave (indoor_positioning/StringStamped):  
Name of the zone that the receiver has left. Is published at the moment a zone-leave occurs

- ips/receiver/zone_enter (indoor_positioning/StringStamped):  
Name of the zone that the receiver has entered. Is published at the moment a zone-enter occurs

##### Parameters:
- ~config_file (string, default='config/zones.yml'):  
Path to the configuration file of zones and beacons relative to the package directory

- ~rate (double, default=1):  
The publishing rate in messages per second

- ~bcn_len (int, default=2*number_of_beacons):  
Buffer length for BCN messages
        
#### positioning_plus.py (IPS+)
Use this node to perform indoor positioning using the metraTec IPS+ tracking system. Prerequisites for using this node
is a running receiver-node that handles communication with the receiver and thus with the beacons in the vicinity.
Also, make sure that tf is broadcasting transformations between all the different coordinate frames used in the config
file and the coordinate frame of the receiver (which is specified via rosparam, see below).

##### Subscribed topics:
- ips/receiver/raw (indoor_positioning/StringStamped):  
Raw messages received by the UWB receiver

##### Published topics:
- ips/receiver/send (std_msgs/String):  
Message to be sent to the receiver, e.g. 'SRG <EID> \r' ranging request

- ips/receiver/position (geometry_msgs/PointStamped):  
Estimated position of the receiver after UWB ranging and trilateration

##### Parameters:
- ~config_file (string, default='config/zones.yml'):  
Path to the configuration file of zones and beacons relative to the package directory

- ~frame_id (string, default='map'):  
Coordinate frame the receiver position should be estimated in

- ~rate (double, default=0.1):  
The publishing rate in messages per second

- ~bcn_len (int, default=2*number_of_beacons):  
Buffer length for BCN messages

- ~srg_len (int, default=number_of_beacons):  
Buffer length for SRG messages

- ~min_beacons (int, default=4):  
Minimum number of beacons to be used for UWB ranging. Should be 3 (two possible points) or 4

- ~max_z (double, default=None):  
Maximum z-coordinate the receiver should have after ranging. Used as bounds for trilateration.

#### ips_map.py
This node publishes tf transforms between the frame_id set in the config file and the beacon positions in that zone.
You can use these transforms for visualization in rviz to make sure you have set up your environment correctly.

##### Published tf's:
- beacon.frame_id -> beacon.eid  
Transforms for all zones and beacons defined in the config file. Transforms are broadcasted from the zone frame_id to the position of the beacon in that frame

##### Parameters:
- ~config_file (string, default='config/zones.yml'):  
Path to the configuration file of zones and beacons relative to the package directory

- ~rate (double, default=0.1):  
The publishing rate of transforms in transforms per second
