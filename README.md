# DLT
The implementation of 3D reconstruction using Direct Linear Transform (DLT). 
Dataset consists of generated 3D points and their projections.

## Quickstart

Guide how to use in your applications.

Before start you need to download all packages from the file `requirements.txt` before executing the code.

### Structure of repository

- `DLT\data\` contains example datasets used for testing the algorithm:
  - `images.txt` - image list with two lines of data per image;
  - `cameras.txt` - Camera list with one line of data per camera;
  - `points3D.txt` - 3D point list with one line of data per point;
  - `IMAGES\` contains images.

- `DLT\DLT\` contains two folders with main functions of the algorithm:
  - `generation` - data processing, preporation for the algorithm;
  - `projection` - projection calculating, main algorithm steps.

- `main.ipynb` - main part of code needed to launch the algorithm.
