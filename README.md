# DLT
The implementation of 3D reconstruction using Direct Linear Transform (DLT). 
Dataset consists of generated 3D points and their projections.

We reproduce the algorithm developed by Daniel Bardsley and Bai Li "3D Reconstruction Using the Direct Linear Transform with a Gabor Wavelet Based
Correspondence Measure".

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

## Development instructions

1. Before start one need to download all packages from the file `requirements.txt` before executing the code.
2. Run the main.ipynb using dataset from `DLT\data\` or create your own dataset.
3. 
