# Autonomous car chase
The repository presents a system that can autonomously chase another vehicle. The goal is to maintain a predefined distance between two cars. It has been tested in a CARLA simulator using CARLA Car Chasing dataset and with a RC cars. [Full video can be found here](https://www.youtube.com/watch?v=SxDJZUTOygA).

|&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;![Video of the car chase](https://i.imgur.com/RHbzHUF.gif)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; |
|-------|


## Main contributions
* Functional autonomous driving system capable of chasing another vehicle based on information from a single monocular RGB camera. The full system is found in [src](/src). The simplified version used with the RC cars can be found in [RC_Version](/RC_Version).
* [Dual-task neural](/dual-task%20network) network that can concurrently detect objects and perform coarse image segmentation.
 
Detection             |  Segmentation 2019
:-------------------------:|:-------------------------:
![detection](https://i.imgur.com/1y7BDH2.png)  |  ![segmentation](https://i.imgur.com/fMWX081.png)
* Three datasets (car chase dataset in CARLA, object detection dataset and semantic segmentation dataset) found in [datasets](/datasets) 
* Publication at European Conference on Computer Vision (ECCV 2020) ACVR workshop

## Citations
Please cite Autonomous Car Chasing paper in your publications if it helps your research.
```
@inproceedings{chase_Jahoda2020,
  author = {Pavel Jahoda and Jan Cech and Jiri Matas},
  title = {Autonomous Car Chasing},
  booktitle = {Proceedings of the European Conference on Computer Vision (ECCV) Workshops},
  year = {2020}
}
```


