# Learning to Pick with 3D Point Cloud
> Assignment Project #2: 30% | Due Monday, Apr 26

------

## Target
This experiment is mainly divided into two parts. They are 6D calibration and object grasping respectively. The 6D calibration is to establish the transformation basis of the camera coordinate system and the robot coordinate system with 3D point cloud information, that is, the hand-eye transformation matrix used to describe the relative spatial pose of the robot and the camera. On this basis, object grasping is to get the grasping coordinates through the 3D point cloud information by camera and control the movement of the robot to grasp.

## Hardware List
The hardware equipment required for this experiment is as follows:
- camera: Intel Realsense D435
- mechanical arm: Aubo i5
- calculation platform: MSI Trident, i7-10700, GTX1660 6G, 8G DDR4, Ubuntu OS
- calibrators: 3D-printed L-shape Calibration Target with printed 4x4 Checkerboard
- clamping device: pneumatic clamping jaw (air compressor and air valve)
- target objects: plastic bottles, cans

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
1. Start the device, calibration, and walk the grid
2. Open the realsense-viewer, click Stereo Module, select four points and record, enter the plane_calculate.py, create the plane_model, and enter the main function
![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=M2FhMmI2YjhhNTVkMGI0ZjUxNWVjYTc0OGY3ZDQ4NmRfalJSemUzd0FCdTN2dDVrRnlMZUdQNlJrNnkwc2dvOVBfVG9rZW46Ym94Y250bVl5a2lQSmxNcEtOdmlzUlpkTHVoXzE2MTkzNjc5MTE6MTYxOTM3MTUxMV9WNA)
![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=OGQ0YmViN2RkMDgyNzE3N2ViNGFlZmExZGI3ZTBhMTVfbDVyM3lESEY1aUxCc2ZFTmYzUDk2TmE0SXpwc0hwM1pfVG9rZW46Ym94Y25vSjJ6ZkoxZmFzdFUwTzhqMEFOajFjXzE2MTkzNjc5MzM6MTYxOTM3MTUzM19WNA)
3. Install the grasper and pay attention to the safety height
![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=M2FiNDEyMmQyYzhiNGVjODQ1YmZjNzM4ZTk0ZmM5MjNfeG9ZbXNXNnJCODlxcEpMbGtRRzBabnRKS0s1dVFJYUNfVG9rZW46Ym94Y25FUlF4VTJiRVZFU2ZQTUZDcTRkMFlnXzE2MTkzNjc4NTg6MTYxOTM3MTQ1OF9WNA)
4. Read the crop_bounding from the realsense-viewer
![img](https://bionicdl.feishu.cn/space/api/box/stream/download/asynccode/?code=ZTIyYjAzYjBmZTFmNGE1Y2EyMTNkNTZhMjFhZTQ1NWZfTDZucjJVb0dkRm9ITFBBT2g2a2ZWVkV3ZEpxNFFHRDBfVG9rZW46Ym94Y25hSnh3UTNmRnU0Mk1kYTZrdndhWjdjXzE2MTkzNjc4MTA6MTYxOTM3MTQxMF9WNA)
5. Run the robot arm
6. Verify the algorithm by changing the position of the bottle

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
Compared with region proposal-based method, Geograsp do not have to collect multi-angle images to form 3D point cloud, which means fast and effective. Though general method may provide better accuracy, Geograsp is enough for garbage identification. Information 
