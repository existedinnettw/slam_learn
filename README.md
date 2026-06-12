# SLAM overview

Simultaneous Localization and Mapping

* 2d SLAM
* 3d SLAM
* visual SLAM
  - rgbd

## goal

intermediate checkpoint

* 2D SLAM

final

* 3D SLAM

## index

1. [slam introduction](./slam_introduction.md)
2. [bayes filter and model](./bayes_filter_and_model.md)
2. KF series
   1. [KF](./KF.md)
   2. [EKF](./EKF.md)
      * [EKF-SLAM](./EKF_SLAM.md)
        * use EKF to solve SLAM problem
        * 2D SLAM as example
      * For small map, EKF is the most widely used SLAM algorithm.
   3. [Unscented Kalman Filter](./UKF.md)(optional)
   4. [Extended Information Filter](./EIF.md)
      * Necessary for SEIF
   5. [Sparse Extended Information Filter](./SEIF.md)
      * The only KF-based SLAM algorithm that can used in large map.
   6. summary by `slam09-kf-wrapup.pdf`
   * > Engineers still use EKF for simple systems with very few moving parts (like a basic line-following robot). It is also used when a system needs to combine data from multiple sensors instantly (like a car combining GPS and a speedometer).
2. grid map
2. scan matching
   * ICP (Iterative Closest Point)
   * It is `Front-Ends for Graph-Based SLAM` too.
3. [particle filter](./particle_filter.md) (optional)
   * FastSLAM
      * > Feature-Based SLAM with Particle Filters
   * > In industry, EKF-ICP (Extended Kalman Filter with Iterative Closest Point) is vastly more used than Particle-ICP. EKF-ICP provides real-time, highly efficient state estimation. Particle-ICP is rarely used in commercial systems because it is computationally too heavy.
4. [graph-based SLAM](./graph_based_slam.md)
   * least square optimization
   * > Graph SLAM is much more widely used in the robotics industry today (compare to EKF-ICP). Modern autonomous systems—from self-driving cars to warehouse robots—rely on graph-based algorithms to handle complex, large-scale environments and fix mapping errors.


## ref

* [Robot Mapping - WS 2022/23 ](http://ais.informatik.uni-freiburg.de/teaching/ws22/mapping/)
    * main learning resource
* [NTNU Simultaneous Localization and Mapping (SLAM) for Robotics](https://www.youtube.com/playlist?list=PLZ_sI4f41TGtsqgT6cMLCUCYOT7mCjBMM)
* [PythonRobotics](https://github.com/atsushisakai/pythonrobotics)
* [MIT16.485 - Visual Navigation for Autonomous Vehicles](https://vnav.mit.edu/)

---

* [cpp SLAM](https://conan.io/center/recipes?value=slam)


