# Triangle-rotations-Computer-Graphics
My solution to Games101 HW1. 
![ezgif com-animated-gif-maker](https://github.com/user-attachments/assets/d23625ca-96fc-4602-9fac-43a360d33881)

Press w and s to rotate triangle around axis [1, 1, 0]. 
- This rotation is done with Rodrigues' rotation formula

Press a and d to rotate the triangle around axis z [0, 0, 1].

The triangle's line is drawn with bresenham's line drawing algorithm. 

Used Eigen library.

The triangle is applied with
MVP transformation
- model transformation
- view/viewing transformation
- perspective projection transformation
- viewport transformation

# Build
You need OpenCV and Eigen library locally to build this project. 

If you're on Ubuntu, run 
1. sudo apt install libeigen3-dev
2. sudo apt install libopencv-dev

then, go into ./build/ directory or create one if it's not there and run
1. cmake ..
2. make
3. ./Rasterizer
4. Finished! Go play with the triangle! If you want it to rotate against another axis, modify rot_axis in main() in the main.cpp file!
