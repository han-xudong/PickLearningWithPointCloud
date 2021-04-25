# Learning to Pick with 3D Point Cloud
> Assignment Project #2: 30% | Due Monday, Apr 26

------

## Target
This experiment is mainly divided into two parts. They are 6D calibration and object grasping respectively. The 6D calibration is to establish the transformation basis of the camera coordinate system and the robot coordinate system with 3D point cloud information, that is, the hand-eye transformation matrix used to describe the relative spatial pose of the robot and the camera. On this basis, object grasping is to get the grasping coordinates through the 3D point cloud information by camera and control the movement of the robot to grasp.

## Hardware List
The hardware equipment required for this experiment is as follows:
- camera: Intel Realsense
- mechanical arm: Aubo i5
- calculation platform: MSI Trident, i7-10700, GTX1660 6G, 8G DDR4
- calibrators: 3D-printed nib (for calibration on a flat surface)
- clamping device:  pneumatic clamping jaw (air compressor and air valve)
- target objects:  plastic bottles, cans

## Algorithm
In this project, we mainly used the GeoGrasp algorithm.

GeoGrasp is an algorithm based on 3D point clouds, which originally was designed and implemented by Zapata-Impata et al. This is the main frame of the algorithm we used in this project. Traditionally, vision-based grasping systems proposed in the literature usually take multiple views to detect and identify the object in front of the robot. Once they recognize the object and its pose, they proceed to calculate potential contact points using stored 3D CAD models. Some recent solutions find these grasping configurations by using machine learning techniques trained on large data sets or in simulation. GeoGrasp, in contrast, only need one camera to operate. It also needs no training data since it’s based on geometry analysis. However, some unsupervised learning approaches including Clustering and PCA are utilized in the process.

The general procedure of the algorithm could be divided into 3 parts. Scene segmentation, locating grasping areas and ranking grasping points, as shown in figure below.

![image](https://github.com/MEE336-Red-Team/Learning_to_Pick_with_3D_Point_Cloud/blob/main/figure/framework_of_Geograsp.png)

The first step is to segment the potential objects out of the scene. Euclidean Cluster Extraction from the Point Cloud Library is used in the original algorithm. Other clustering methods like gaussian mixture model might behave better than simply calculating the Euclidean distances. However, it’s not guaranteed that the processing speed would satisfy the need of real-time usage.

After segmenting out the objects, we have to find the grasping areas for potential grasping points. To begin, filters are used to reduce noise on the surface of the object. Principle component analysis (PCA) is used to find the vector v, which approximate the orientation of the object. Cutting plane γ is found by calculating the centroid c of the point clouds. Cutting plane γ also need to be perpendicular to the vector v to guarantee that the grasping points is stable. The thickness of the cutting plane is decided empirically, 7mm. Opposite areas along the vector v is selected as grasping areas. To make the grasping areas more precise, initial grasping points are found by calculating the maximum and minimum in the areas, along the perpendicular axis. We then draw spheres around these two points, and set these as the final grasping areas.

The last step is to evaluate the contact points to determine the best grasping points. First, define θ be a grasp configuration whose contact points are one point q_i from each of the voxelized grasping areas. And then a ranking function is proposed to choose the best points by assessing the potential stability of the grasp, mainly considering the following four factors: 1) Distance to the cutting plane; 2) Curvature of the point; 3) Antipodal configuration; 4) Perpendicular grasp. Finally, the following ranking function is proposed to assess the potential stability of a grasp configuration:

![image](https://github.com/MEE336-Red-Team/Learning_to_Pick_with_3D_Point_Cloud/blob/main/figure/equation_1.png)
![image](https://github.com/MEE336-Red-Team/Learning_to_Pick_with_3D_Point_Cloud/blob/main/figure/equation_2.png)
![image](https://github.com/MEE336-Red-Team/Learning_to_Pick_with_3D_Point_Cloud/blob/main/figure/equation_3.png)

## Preparation
1. Connect the power supply and plug in the AUBO.
2. Turn the black knob in the main machine of AUBO from ON to the red emergency stop button upward to the right to unlock the emergency mode.
3. Turn the emergency stop button on the teaching device up to the right, open the teaching device and click the save button in the popover.
4. Connect the control box to the main machine with network cable.
5. Right-click on the desktop to open the terminal and enter the following code to open PyCharm.
```python
cd Downloads/pycharm-community-2020.3.3/bin
sh pycharm.sh
```
6. Enter the following code in the terminal to turn on the camera.
```
realsense-viewer
```

## Experiment

1. Install the calibration nib 1 on the flange at the end of the manipulator.
2. Click RGB Camera Off and open the Camera.
3. Adjust RealSense and Camera Shelves.
4. Start the manipulator arm.
5. Adjust the end of the code to [x, y, 0.04, 3.14, 0, 1.57] and run it.
6. Place the calibrator on the platform so that the two vertices of the calibrator and the calibration plate are on the same vertical line.
7. Adjust the end of the code to [0.3, 0.0, 0.5, 3.14, 0.0, 0.0] and run it. The robotic arm is then removed from view.
8. Slide the mouse to the vertex of the calibration object on the RealSense interface and read the pixel value (U, V) above.
9. Open configs/basic_config/ cail2d.yaml in PyCharm
10. Record the given (x, y) value and the (u,v) value read by the RealSense-Viewer into cail2d.yaml opened by PyCharm.
11. Repeat steps 5-9 for 4 times to get 4 groups of (x, y) values and their corresponding 4 groups of pixel values (u,v), that is, to complete data collection for 2D calibration.
12. According to the configuration file cail2D. Yaml to calculate the hand-eye matrix and run ME336-2021 spring/deepclaw/modules / * * / * * calibration Calibration2D. Py.
13. Modify two values in the figure, which correspond to pixel points in the camera coordinate system.
14. Open the real-sense corresponding to the pixel above and find a point, and place a calibrator on the desktop corresponding to the pixel.
15. Right-click and run Calibration2d.py to get two values in the run box.
16. Open auboi5Control.py and modify it according to the values obtained in the previous step.
17. Right click and run Auboi5Control.py to observe the movement of the manipulator arm.
18. Obtain the gap between the end of the manipulator calibration plate and the calibration object in the plane to check whether the error is reasonable.
19. Setup the gripper.
20. Open Realsense-Viewer and customize a rectangular area on the conveyor belt that is the camera recognition area.
21. Open the main.py file.
22. Modify crop_bounding = [y1, y2, x1, x2].
23. In Realsense-Viewer, read the pixel coordinates (x1,y1) on the upper left and (x2,y2) on the lower right. Modify crop_bounding = [y1, y2, x1, x2].
24. Place an empty bottle on the manipulator operating platform to ensure that it is within the visual field and that there is no other sundries in the center of the visual field.
25. Modify the z and adjust the grasping height.
26. Open main.py in the image, right-click and run. Note that the Realsense-viewer is off.
27. Observe the movement of the manipulator arm.

## Video
https://www.bilibili.com/video/BV1R64y1m7nk/

## Challenges and Solutions
- PyCharm: Always open PyCharm in the terminal in which the instructions are directed. If you open PyCharm in any other way, you will have a path error problem.
- Calibration plate problem: In the beginning, our calibration plate was broken.We use tape to hold it together and continue the 3D calibration.However, such calibration plates actually have a small bending, and the small errors will gradually accumulate and become larger in the process of iterative calibration, eventually resulting in a large deviation in the 3D calibration results.
- The offset of the calibration plate relative to the end flange of the manipulator in CAIL-3D file was not modified, which would cause the calibration to fail.
- You need to change the storage path of the computed results at the end of the code in EyeOnBase.py file.The previous code did not load the matrix results we calculated to the file that we read later, so we need to unify the file names before and after.
![image](https://github.com/MEE336-Red-Team/Learning_to_Pick_with_3D_Point_Cloud/blob/main/figure/EyeOnBase_Problem.png)
- Camera problem: When our camera is in STEREO Module state, the 3D view is opened. Its depth view has poor effect, with many black areas and missing information, which leads to the failure of our early calibration.
- Pay attention to the stability of the camera's USB interface.Sometimes the USB interface of the camera will be unstable, and the results shown below will appear.A new socket is needed.
![image](https://github.com/MEE336-Red-Team/Learning_to_Pick_with_3D_Point_Cloud/blob/main/figure/camera_USB_problem.jpg)
- When using Realsense-Viewer to customize a rectangular area during the 6D fetching process, the scope should not be too small, otherwise it is easy to cause the recognition target to exceed the recognition area.
- The position of the empty bottle will also affect the grasping result.If the mouth of the bottle is facing the base of the mechanical arm, it will lead to a large tilt of the clamp, which will not be able to accurately clamp the bottle.

## Conclusion
In this project, object recognition is the key to pick with images. we propose EfficientDet to recognize the object. To compare the performance, we ran three normal algorithms, yoloV4, yoloV5 and EfficientDet. First, we tried yoloV5 provided by the tutorial. And we also tried yoloV4, which has a higher accuracy in recognizing objects. But yoloV4 is much slower than yoloV5. When recognizing multiple images, yoloV5 has an average processing time of 7ms per image, that’s 140 FPS, and that is an incredible achievement. Considering we aim at grabing garbage, a faster solving program is exactely what we need. We also tried EfficientDet to detect the target objects. It is a little bit slowly than yoloV5, and has similay accuracy. But the problem is when we require more accuracy and apply different EfficientDet algorithms, the running time of the progrim is raising so fast that it becomes a great disadvantage. However, these algorithms all have a common regret, that is, the bounding frame cannot be tilted. When the object is tilted in the camera field of view, the bounding frame is still parallel to the x- and y-axis, which leads to the robot arm still grasping along the x- or y-axis. Although it is still effective for grasping plastic bottles and cans, it may cause problems for other items. Therefore, this is an optimization direction in the future.

After we find out the way to recognize the object, we provide the position to the Aubo i5 and grasp. After many experiments, the success rate of grasping plastic bottles and cans is very high. But paper and other thin items are difficult to grasp, is due to the structure of the claw restrictions. These are what we can improve on and hopefully will be addressed in Project 3.
