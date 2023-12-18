# DLT
The implementation of 3D reconstruction using Direct Linear Transform (DLT). 
Dataset consists of generated 3D points and their projections.

## Quickstart

Guide how to use in your applications.

Before start you need to download all packages from the file `requirements.txt` before executing the code.

### Structure of repository

- `DLT\data\` contains example datasets used for testing the algorithm.
- - `images.txt` - image list with two lines of data per image:
      {IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME
        POINTS2D[] as (X, Y, POINT3D_ID)}
- - `cameras.txt` - Camera list with one line of data per camera:
      {CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]}
- - `points3D.txt` - 3D point list with one line of data per point:
      {POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)}
- - `IMAGES\` contains images.
 
- 

