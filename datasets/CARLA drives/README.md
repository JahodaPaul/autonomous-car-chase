## CARLA Chasing dataset

### Dataset description
We have collected a database of 20 different drives with varying difficulty: 10 easy, and 10 challenging rides. The rides are about 1 minute long on average. We used the Tesla Model 3 and manually drove it around the city using connected steering wheel and pedals Hama uRage GripZ. As the car was driven, its location and orientation were recorded at every frame. This way, we have collected a database of 20 different drives with varying difficulty (in a sense of chasing the car). We used this database to evaluate the autonomous driving system. At the beginning of the following experiments, a chasing car was placed half of a meter behind the chased car. Then, we updated every frame the location and orientation of the chased car based on the saved coordinates in the dataset. Meanwhile, the chasing vehicle was driven autonomously and trying to maintain a desired distance from the pursued car.

We divide the dataset into two equal-sized sets based on difficulty. The first set contains simple drives in which the car was driven slowly and without sudden turning and braking. Any changes such as turns were slow. The average speed in these drives is 35.01 km/h.

On the other hand, the second set aims to test the algorithm to its limits. The drives include sudden braking, fast turns as well as cutting corners. Sometimes even drifts were performed. The average speed in these drives is 49.68 km/h with the fastest drive having an average speed of 63.29 km/h.

### How to use the dataset
After you have downloaded the dataset. Download [CARLA version 0.9.8](https://carla.org/2020/03/09/release-0.9.8/). Then, copy this directory into a CARLA_0.9.8/PythonAPI directory.

You can then start the process. First you get a new video and call the CARLA server in the directory CARLA_0.9.8
```
./CarlaUE4.sh
```

Afterwards, you open another terminal and call the following command while in directory CARLA_0.9.8/PythonAPI/CARLA drives
```
python3 synchronous_mode.py
```

This will start the first drive of the car chase. The chasing algorithm is not implemented in this file. The chasing car will drive straight.
![Imgur](https://i.imgur.com/d1XOkYX.png)


To test your own algorithm, change the following line:
```
                # Choose approriate steer and throttle here
                steer, throttle = 0, 1
                vehicle.apply_control(carla.VehicleControl(throttle=throttle,steer=steer))
```
I recommend using "bbox" variable (chased car 3D) box, "image_segmentation" that contains the semantic segmentation of the current image and the "img_rgb" that contains rgb image seen from the camera.


### Evaluation
Afterwards, Use the following command to evaluate the dataset
```
python3 AnalyseResults.py
```
It will calculate the percentage how long your system was able to chase another vehicle. You can also find average MAE, RMSE and number of collisions in res/results.txt
![Imgur](https://i.imgur.com/iSdHqDg.jpg)

### Example with our system
[Here](https://youtu.be/mAh8CcaAHgM) you can find an example of our full system chasing another vehicle. The drive is part of the difficult split set of the dataset.

