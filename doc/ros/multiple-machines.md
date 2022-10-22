# ROS with multiple machines

See [Link](http://wiki.ros.org/ROS/Tutorials/MultipleMachines)


### Setup:

1. Source ros
2. Source workspace
3. Source ipconfig

## ROS core device

ipconfig:

```bash
# make sure its defined in "/etc/hosts" for resolving ip
ROS_HOSTNAME="<device_hostname>"
export ROS_MASTER_URI="http://$ROS_HOSTNAME:11311"
```

## Other devices that should connected to ROS network

ipconfig:

```bash
export ROS_IP="<your_ip>"

# make sure its defined in "/etc/hosts" for resolving ip
ROS_HOSTNAME="<host_name_other_device>"
export ROS_MASTER_URI="http://$ROS_HOSTNAME:11311"
```