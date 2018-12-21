# Robot Eyes

This package intents to create a modular system for RGB-D and/or RGB image processing for industrial scenarios. 

![#f03c15](https://placehold.it/15/f03c15/000000?text=+) *Disclaimer:*
This is a work in progress, the code is expected to change drastically, and no backwards compatibility is in consideration for the moment.

### Project Characteristics 
- [ ] Efficient instead of universality: We want to create a system that can process effectively a certain number of 'known' objects, i.e. we want to deploy robotic industrial applications meaning that the scenarios of the robot will not be totally unknown.
- [ ] Modulatiry.
- [ ] Real time operation.
 
## Contents 
* [Instalation](https://github.com/Danfoa/robot_eyes/wiki/Installation)
* [Starting Steps](https://github.com/Danfoa/robot_eyes/wiki/Starting-Steps)
* Examples
    * [Integrate RS200 with your robot](https://github.com/Danfoa/invite-robotics/wiki/Tutorial---3D-Sensor-Integration)
    * [Point Cloud pre processing](https://github.com/Danfoa/robot_eyes/wiki/PointCloud-pre-processing)
* Nodelets
    * Common
        * [CloudColorer]((https://github.com/Danfoa/robot_eyes/wiki/nodelets/CloudColorer))
* Nodes
    * Segmentation
        * [plane_segmenter](https://github.com/Danfoa/robot_eyes/wiki/plane_segmenter)
        * [cylinder_segmenter](https://github.com/Danfoa/robot_eyes/wiki/cylinder_segmenter)
    * Common
        * [cloud_colorer](https://github.com/Danfoa/robot_eyes/wiki/cloud_colorer)
