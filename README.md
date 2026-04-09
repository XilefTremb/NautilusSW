This is the Nautilus Software Github, it encompasses all of the code needed to run the virtual and real submarine. 

To run the simulation, you first need to build it correctly. Start by going into the NautilusWs/nauillus_ws folder. There you can run multiple commandes that have to do with git. After first cloning the github, you will be on branch dev. This repo is setup in a way where you cannot commit on the dev, you have to be on a branch to work. So firstly, run these commands : 
  - "git status" allows you to see what branch you are on and wether you are ahead or behind of the branch. Since you are on the dev and just cloned, you should be at the right spot.
  - "git pull" just to be certain you are on the most recent version.
  - "git checkout NSFW-XX" alows you to choose a branch to work on.
Once you are on the correct branch, you can start working with git :
  - "git add ." allows you to add all your changes to the current branch
  - "git status" once again allows you to see what files changed and what branch you are on and wether you are ahead or behind.
  - "git commit -m "XXXX"" allows you to commit all the changes you made to the branch. In the XXXX, you need to add a comment saying what you changed. Ideally, you should start it with NSFWXX which is the number of the branch.

To start the simulation, if it is your first time cloning the git, there are a few steps. First of all, make sure you run these three commands in the terminal you are running. 
  - "colcon build" to build the environment
  - "source install/setup.bash" to source the NautilusWs
  - "source ~/ardu_ws/install/setup.bash" to source the ardu_ws
Now you are working in the correct environment, there is one last step before you cn start using the simulation, which is to add a line in the bashrc that specifies the path of the models for the gazebo sim. To do so, you first need to open the bashrc running this command
  - "nano ~/.bashrc"
Then, using arrow keys, go to the bottom of the file and add this line
  - "export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:~/NautilusSW/nautilus_ws/src/nautilus_bringup/models/:${GZ_SIM_RESOURCE_PATH}"
Then, to save and exit, press these two keybinds in this order :
  - "Ctrl + 0" to write
  - "Ctrl + X" to exit

Now you have a correctly built environment. To run the simulation, you simply need to run this command :
  - "ros2 launch nautilus_bringup orca_comp.launch.py"
This should by itself open the gazebo simulation

There are also a few usefull tools that you can use to run additional code.
  - To visualise image topics, you can start the rqt_viewer with :
  - "ros2 run rqt_image_view rat_image_view"
  - To run any python file, you can also run this
  - "python3 /src/your_file_path/your_file.py" just put in the correct path.

Just remember that in any new terminal you open, you always need to start by running the "source install/setup.bash". Sourcing the ardu_ws is only necessary for the terminal that runs the simulation.





