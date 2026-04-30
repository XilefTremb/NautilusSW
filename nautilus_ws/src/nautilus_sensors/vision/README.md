To run the vision in realtime, you need to run stream.py and yolo_pipeline.py
    - "ros2 run nautilus_sensors stream.py"
    - "ros2 run nautilus_sensors yolo_pipeline.py"

In simulation, you dont need to run stream.py because the topics are published by the simulation, so you just need to run yolo_pipeline_sim.py

    - "ros2 run nautilus_sensors yolo_pipeline_sim.py"

If you want to save images in real time, you need to add parameters when calling the function, because the saving is off by default, so run this instead
    - "ros2 run nautilus_sensors stream.py --ros-args -p save_images:=true"




