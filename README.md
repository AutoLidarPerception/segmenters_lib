# segmenters_lib
The LiDAR segmenters library, for segmentation-based detection.

## TODO lists
### Ground Segmenters
- [x] PCL RANSAC
> Refer: [PCL: Plane model segmentation](http://pointclouds.org/documentation/tutorials/planar_segmentation.php#planar-segmentation)
- [x] GPF (Ground Plane Fitting), ICRA 2017
```bibtex
@inproceedings{zermas2017fast,
  title={Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={Robotics and Automation (ICRA), 2017 IEEE International Conference on},
  pages={5067--5073},
  year={2017},
  organization={IEEE}
}
```
- [ ] [linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation), IV 2010
```bibtex
@inproceedings{himmelsbach2010fast,
  title={Fast segmentation of 3d point clouds for ground vehicles},
  author={Himmelsbach, Michael and Hundelshausen, Felix V and Wuensche, H-J},
  booktitle={Intelligent Vehicles Symposium (IV), 2010 IEEE},
  pages={560--565},
  year={2010},
  organization={IEEE}
}
```
- [ ] [depth_clustering](https://github.com/PRBonn/depth_clustering), IROS 2016
```bintex
@inproceedings{bogoslavskyi2016fast,
  title={Fast range image-based segmentation of sparse 3D laser scans for online operation},
  author={Bogoslavskyi, Igor and Stachniss, Cyrill},
  booktitle={Intelligent Robots and Systems (IROS), 2016 IEEE/RSJ International Conference on},
  pages={163--169},
  year={2016},
  organization={IEEE}
}
```
- [ ] Scan Line Run, ICRA 2017, like Junior
```bibtex
@inproceedings{zermas2017fast,
  title={Fast segmentation of 3d point clouds: A paradigm on lidar data for autonomous vehicle applications},
  author={Zermas, Dimitris and Izzat, Izzat and Papanikolopoulos, Nikolaos},
  booktitle={Robotics and Automation (ICRA), 2017 IEEE International Conference on},
  pages={5067--5073},
  year={2017},
  organization={IEEE}
}
@incollection{montemerlo2009junior,
  title={Junior: The stanford entry in the urban challenge},
  author={Montemerlo, Michael and Becker, Jan and Bhat, Suhrid and Dahlkamp, Hendrik and Dolgov, Dmitri and Ettinger, Scott and Haehnel, Dirk and Hilden, Tim and Hoffmann, Gabe and Huhnke, Burkhard and others},
  booktitle={The DARPA Urban Challenge},
  pages={91--123},
  year={2009},
  publisher={Springer}
}
```
- [ ] Deep-learning: FCN, IV 2017
```bibtex
@inproceedings{caltagirone2017fast,
  title={Fast LIDAR-based road detection using fully convolutional neural networks},
  author={Caltagirone, Luca and Scheidegger, Samuel and Svensson, Lennart and Wahde, Mattias},
  booktitle={Intelligent Vehicles Symposium (IV), 2017 IEEE},
  pages={1019--1024},
  year={2017},
  organization={IEEE}
}
```
### Non-ground Segmenters
- [x] PCL Euclidean Cluster Extraction
> Refer: [PCL: Euclidean Cluster Extraction](http://pointclouds.org/documentation/tutorials/cluster_extraction.php#cluster-extraction)
- [x] Region-based Euclidean Cluster Extraction
```bibtex
@inproceedings{yan2017online,
  title={Online learning for human classification in 3d lidar-based tracking},
  author={Yan, Zhi and Duckett, Tom and Bellotto, Nicola},
  booktitle={Intelligent Robots and Systems (IROS), 2017 IEEE/RSJ International Conference on},
  pages={864--871},
  year={2017},
  organization={IEEE}
}
```
- [ ] Model-based Segmentation, ITSC 2017
```latex
@article{shin2017real,
  title={Real-time and accurate segmentation of 3-D point clouds based on Gaussian process regression},
  author={Shin, Myung-Ok and Oh, Gyu-Min and Kim, Seong-Woo and Seo, Seung-Woo},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  volume={18},
  number={12},
  pages={3363--3377},
  year={2017},
  publisher={IEEE}
}
```
- [ ] Probabilistic Framework, RSS 2016
```bibtex
@inproceedings{held2016probabilistic,
  title={A Probabilistic Framework for Real-time 3D Segmentation using Spatial, Temporal, and Semantic Cues.},
  author={Held, David and Guillory, Devin and Rebsamen, Brice and Thrun, Sebastian and Savarese, Silvio},
  booktitle={Robotics: Science and Systems},
  year={2016}
}
```

## How to use
`git clone` as a ROS package, with [common_lib](https://github.com/LidarPerception/common_lib) and [object_builders_lib](https://github.com/LidarPerception/object_builders_lib) as dependencies.


