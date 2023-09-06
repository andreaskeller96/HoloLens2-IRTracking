# On-Device HoloLens 2 IR Tracking

This project contains the source code for tracking passive IR sphere markers using only the HoloLens 2 AHAT Sensor.


## Features
* Easy-to-use tracking of retro-reflective marker arrays using the HoloLens 2 research mode
* Simultaneous tracking of multiple markers (tested with up to 5, can theoretically support a lot more)
* Support for partial marker occlusion - define marker arrays with 4 or more spheres, keep tracking even when some are occluded (min. 3 visible at all times)
* Support for spherical markers and flat marker stickers
* Support for different marker times tracked simultaneously
* Filter 3D sphere world positions using Kalman Filters
* Filter marker array world position and rotation using low-pass filtering


## How to use precompiled library
Check out the Unity Sample application provided here: https://github.com/andreaskeller96/HoloLens2-IRTracking-Sample/


## How to build from source
1. Add zlib, libtiff, opencv412d to the project before compilation
2. Compile for ARM64 in release mode
3. Copy the resulting HL2IRToolTracking.winmd and HL2IRToolTracking.dll to your Unity Project's Assets/.../Plugins/WSA/ARM64/ folder
4. Copy the contents of the UnityBindings folder to your Unity Project's Assets/.../Scripts folder
5. Setup scene as in the sample here: https://github.com/andreaskeller96/HoloLens2-IRTracking-Sample/


## Thanks
Special thanks to Wenhao Gu for his hololens plugin project that this dll is based on: https://github.com/petergu684/HoloLens2-ResearchMode-Unity
This project also makes use of a number of awesome open source libraries, including:
* [opencv](https://github.com/opencv/opencv)
* [libtiff](https://gitlab.com/libtiff/libtiff)
* [zlib](https://github.com/madler/zlib)


## License and Citation

If you use this project, please cite the following BibTeX entries:

```BibTeX
@misc{keller2023hl2irtracking,
  author =       {Andreas Keller},
  title =        {HoloLens 2 Infrared Retro-Reflector Tracking},
  howpublished = {\url{https://github.com/andreaskeller96/HoloLens2-IRTracking}},
  year =         {2023}
}
```
A. Keller, HoloLens 2 Infrared Retro-Reflector Tracking. https://github.com/andreaskeller96/HoloLens2-IRTracking, 2023. [Online]. Available: https://github.com/andreaskeller96/HoloLens2-IRTracking

```bibtex
@ARTICLE{10021890,
  author={Martin-Gomez, Alejandro and Li, Haowei and Song, Tianyu and Yang, Sheng and Wang, Guangzhi and Ding, Hui and Navab, Nassir and Zhao, Zhe and Armand, Mehran},
  journal={IEEE Transactions on Visualization and Computer Graphics}, 
  title={STTAR: Surgical Tool Tracking using Off-the-Shelf Augmented Reality Head-Mounted Displays}, 
  year={2023},
  volume={},
  number={},
  pages={1-16},
  doi={10.1109/TVCG.2023.3238309}}

```
A. Martin-Gomez et al., “STTAR: Surgical Tool Tracking using Off-the-Shelf Augmented Reality Head-Mounted Displays,” IEEE Transactions on Visualization and Computer Graphics, pp. 1–16, 2023, doi: 10.1109/TVCG.2023.3238309.
