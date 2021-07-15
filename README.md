# Lane Finder for Dashcams
![Example](./images/3301626331308_.pic_hd.jpg?raw=true)
### Example Video:
<a href="https://www.youtube.com/watch?v=GFDZNolTKzY">YouTube Link 1</a>

<a href="https://www.youtube.com/watch?v=r7bV_Y1b_Ns">YouTube Link 2</a>

### Introduction
This application marks lanes on dashcam videos and calculates the position of the car/curvature of the lane

### Tools
Python OpenCV Library

### Implementation Details
1. Compute the camera calibration matrix and distortion coefficients given a set of chessboard images shot by the camera.

2. Apply a distortion correction to raw images (straight the curved lines).
![Example](./images/3311626331999_.pic_hd.jpg?raw=true)
3. Use color transforms, gradients, etc., to create a thresholded binary image.
![Example](./images/3321626332695_.pic_hd.jpg?raw=true)
4. Apply a perspective transform to rectify binary image ("birds-eye view").
![Example](./images/3331626332900_.pic.jpg?raw=true)
5. Detect lane pixels and fit to find the lane boundary.
![Example](./images/3351626333087_.pic.jpg?raw=true)
6. Determine the curvature of the lane and vehicle position with respect to center.
7. Warp the detected lane boundaries back onto the original image.
![Example](./images/3361626333167_.pic_hd.jpg?raw=true)
8. Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.
![Example](./images/3371626333194_.pic_hd.jpg?raw=true)
