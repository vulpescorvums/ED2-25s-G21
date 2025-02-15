# Group_21-EGN4952C-Project
 Engineering Project for EGN4950C at FAU

1. Download Carla into zip file (https://github.com/carla-simulator/carla/releases/tag/0.9.15/)
2. Extract into any folder, you will have "WindowsNoEditor"
3. Open a cmd prompt in the "WindowsNoEditor" directory and type in " python -m venv carla_env " to create a folder to house all needed files
4. Type in " carla_env\Scripts\activate && cd PythonAPI && mkdir carla_testing "
	1. First command turns on the environment in the cmd
	2. The second changes the directory to PythonAPI
	3. Third creates the folder/directory
5. Type in " cd carla && python -m pip install --upgrade pip " (If there are warnings and errors, disable your proxy in settings via "Network and Internet -> Proxy -> Use a proxy server -> Off")
6. Type in " pip install -r requirements.txt " (If there is an error, install the package straight through the package file at https://github.com/DigitalGlobe/gbdxtools-windows-dependencies/blob/master/Shapely-1.6.4.post2-cp37-cp37m-win_amd64.whl with the command " pip install Shapely-1.6.4.post2-cp37-cp37m-win_amd64.whl ")
7. Type in " cd ../examples "
6. Type in " pip install -r requirements.txt "
8. Type in " cd ../carla_testing "
9. Place your Demo.py and carla_include.py files here and also create a folder named "out" with subfolders "camera", "depth_camera", "gnss", "lidar", and "lidar_plots"
10. Go back to the directory where the Demo.py is then open a second cmd prompt for step 11.
11. Open a second cmd prompt in the " WindowsNoEditor " directory and type in " CarlaUE4.exe -quality-level=Low " to run the sim and save your computer
12. When the sim is running and your have the cmd prompt with the carla_testing directory, type in " python carla_include && python Demo.py " to run the script. 
	(make sure the carla_env is activated still if its not working)
13. Two windows should pop up, one displaying real-time sensors and detectors and another displaying real-time GNSS data.
14. To turn off sim, exit out of both the pop-up camera and gnss displays and the sim should stop.
