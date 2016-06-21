# tud_img_prep
Set of tools for preprocessing a camera video feed using OpenCV.

## Includes:

* Deinterlacing of analog video signals,  e.g. video feed acquired using an Easycap USB capture card. Different interlacing methods:
  * Half image
  * Interpolation  (several interpolation methods)
* Color format selection and conversion
* Equalization of the images
  * Manual equalization using brighness and contrast controls
  * Automatic using OpenCV EqualizeHistogram function
* Filtering
  * Median
  * Gaussian
  * Bilateral Filter

All the options can be changed in real time using Dynamic Reconfigure.

## Installation
Clone repository in your catkin worskspace:

    cd ~/catkin_ws/src
    git clone git@github.com:raultron/tud_img_prep.git

Compile using catkin:

    catkin build tud_img_prep


## Usage

    rosrun tud_img_prep img_prep

This package requires ROS camera topics properly published.

Default input topics:

    /cam/image_raw
    /cam/camera_info

Default output topics:

    /prep/cam/image_raw
    /prep/cam/camera_info

The default camera namespace can be changed using the parameter "camera_namespace" in a launch file or directly in the terminal.

For example, if you have a camera that publishes in:

    /my_camera/image_raw
    /my_camera/camera_info

Then the terminal command will be:

    rosrun tud_img_prep img_prep _camera_namespace:=/my_camera

And the ouput topics will be

    /prep/my_camera/image_raw
    /prep/my_camera/camera_info
