**Cloning the repository:

Open the desired destiny in terminal. Then

$ git clone https://github.com/Sutadasuto/AutoNOMOS_Stardust

**Compiling the plugin for contolling the model in Gazebo:

In terminal, enter the Gazebo_plugin folder. Then

$ mkdir build
$ cd build
$ cmake ../
$ make

For further reference: http://gazebosim.org/tutorials?tut=plugins_hello_world

**Bashrc update:

In terminal

$ sudo  gedit ~/.bashrc

In the file opened by this command, add the next lines at the end:

source /home/sutadasuto/AutoNOMOS_Stardust/AutoNOMOS_simulation/devel/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/path/to/project/AutoNOMOS_Stardust/Gazebo_plugin/build

(Be sure to replace "/path/to/project" for the path where you downloaded the repo. Save the file and close it)

**Building the project:

Open the ~/AutoNOMOS_Stardust/AutoNOMOS_simulation folder in terminal. Then
$ catkin_make
$ source ~/.bashrc

OPTIONAL (HIGHLY RECOMMENDED): You can make the Rviz config of the AutoNOMOS_Stardust your default Rviz config. To do this, find the dafault.rviz file in the root of the repository, copy it and paste it in the next location: /home/user/.rviz (note that the .rviz folder is hidden). The system will ask you if you want to replace the file and you should accept; however, it is highly recommendable that you back up your original file first, so you can restore it later for future projects. If you don't wish to replace the config file, you should open this config file everytime you open Rviz to use it with the AutoNOMOS_Stardust project.

The system should be ready for use. In terminal, run

$ roslaunch autonomos_gazebo my_world.launch 

A Gazebo simulation should start, with the AutoNOMOS in the middle of a world with red blocks. In another tab of the terminal, list the ROS topics

$ rostopic list

Be sure the next topics are in list, as those are the ones in charge of moving the AutoNOMOS. If those topics are not display, there was a problem with the plugin:

/AutoNOMOS_mini/manual_control/steering
/AutoNOMOS_mini/manual_control/velocity

**Running the project

Open the ~/AutoNOMOS_Stardust/AutoNOMOS_simulation folder in terminal. In different tabs, run the next commands in the given order:

$ roscore
$ rosrun rviz rviz
$ roslaunch skycam skycam.launch
$ roslaunch autonomos_gazebo my_world.launch 

In the Rviz window, use the "2D Nav Goal" tool to select a destination for the AutoNOMOS.
