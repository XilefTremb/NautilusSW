# Scripts

This folder contains the DVL script. To run it when opening you vm or starting from scrath, make sur your VM iis on the same subnet as the DVL. By default, his is 192.168.2.3 so anything on 192.168.2.X /24 is okay.

You also need to make the python file exectuable by running chmod -x dvl_sensor_node.py while in this folder

You the run it by running ros2 run nautilus_bringup dvl_sensor_node.py

You should check for the message at the beginning of the execution to see if it connected corretly. 

If it did, you should see info updates in that same terminal and can check in another terminal by using ros2 topic echo /dvl_pub

Windows application CeruleanTracker also shows all this data and auto configs the communication so you can use that to check if the dvl is working okay
