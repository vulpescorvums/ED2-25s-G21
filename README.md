# Group_21-EGN4952C-Project
 Engineering Project for EGN4950C at FAU

1. Download Carla into zip file
2. Extract into any folder, you will have "WindowsNoEditor"
3. Open a cmd prompt in the "WindowsNoEditor" directory and type in " python -m venv carla_env " to create a folder to house all needed files
4. Type in " carla_env\Scripts\activate && cd PythonAPI && mkdir carla_testing "
	1. First command turns on the environment in the cmd
	2. The second changes the directory to PythonAPI
	3. Third creates the folder/directory
5. Type in " cd carla && python -m pip install --upgrade pip " 
6. Type in " pip install -r requirements.txt "
7. Type in " cd ../examples "
6. Type in " pip install -r requirements.txt "
8. Type in " cd ../carla_testing "
9. Place your " SensorManager.py " file here and also create a folder named "out" with subfolders "camera", "depth_camera", "gnss", "lidar", and "lidar_plots"
10. Go back to the directory where the test.py is then open a second cmd prompt for step 11.
11. Open a second cmd prompt in the " WindowsNoEditor " directory and type in " CarlaUE4.exe -quality-level=Low " to run the sim and save your computer
12. When the sim is running and your have the cmd prompt with the carla_test directory, type in " SensorManager.py " to run the script. 
	(make sure the carla_env is activated still if its not working)
13. A window should pop up, displaying real-time sensors and detectors.
14. To turn off sim, press " CTRL + C " in the CMD use to launch the " SensorManager.py " file.

## Bounding Boxes
- Red 3D Bounding boxes surround traffic lights, vehicles, and pedestrians
- Red 2D boxes cover traffic signs
- Blue 3D boxes cover vehicles that are within 50ft of the ego_vehicle's vicinity

![image](https://github.com/user-attachments/assets/02234dc9-2760-4fb5-aa3f-d229f0015330)

## Sensor Manager
- LiDAR sends out thousands of lasers per second to create a 3D map of the vehicle's surrounding area
- Depth Camera uses greyscale colouring to detect the distance of objects
- Semantic Segmentation Camera colour-coats different objects that the camera detects
- RGB Camera is a basic camera used to collect a live video feed
- Radar is used to detect if an object is moving away or closer to the vehicle.

![image](https://github.com/user-attachments/assets/9043b7e5-713e-4eaf-8b69-d2430f669d31)
