#Till Krüger’s algorithm

A line detection algorithm we tested was created by [Till-Julius Krüger](http://www.mi.fu-berlin.de/inf/groups/ag-ki/Theses/Completed-theses/Bachelor-theses/2015/Krueger/index.html
) as his bachelor thesis.

# Inverse Perspective Mapping

In order to correctly detect traffic lanes and situations on the road in general, the optimal input would be a vertical bird's eye image of the current situation. Since our camera is mounted almost horizontally on top of our car, the input image has a completely different perspective. Using homography projection we can transform the perspective of horizontal image into an equivalent vertical view, especially to make distorted borders of traffic lanes look parallel. This particular homography projection is called Inverse Perspective Mapping (IPMapping) and it's done by multiplication of the input image with a projection matrix. This matrix is calculated using the parameters of the used camera and differs for each model. The camera's horizontal and vertical focal length, horizontal and vertical optical center of the camera, the height of camera from the ground and angle of how much the camera is tilted towards the ground against horizontal position are the input parameters. The focal lengths and optical centers of our camera can be measured by ROS camera_calibration package, the height and angle by suitable tools. Once we have those values, we can build an inverse transformation matrix:

Where f_u and f_v are focal lengths, c_u and c_u are optical centres, and where camDeg is the angle and camHeight is the height of the camera.
Using this matrix, we could directly transform the image, but being a very complex task, we precalculate a matrix of the same size as the input. It maps each point of the input image to its new location in the output image. Then, with every camera frame we just go through the input image, look at the precalculated matrix and

# Edge detection

We include the IPMapper in our node instead of having it as separate node. We only transform only the part of the input image that contains the road. Before we begin scanning the image for edges, we (only once per execution) generate scanlines, i.e. line segments in which we will later look for the edges. Scanlines are horizontal, perpendicular to straight lines in road image, generated inside a predefined region of interest with predefined vertical distance. After receiving and transforming an image from the camera we walk along each of these scanlines and look for edges (significant transition from light to dark or dark to light grey). This is detected using kernel of size 5, where we subtract values in the left side of kernel and add those on right side of kernel (along with the same from previous and next row of image) to final sum, which corresponds to the gradient of this part of image. If the absolute value of this sum is higher than predefined threshold, we found an edge in the image at this position. Detected edges also have minimum distance in a scanline between them so we always detect only the most significant part of one edge. Finally we sort the lane marking edge points by their respective y coordinate.

# Lane width detection

TODO:

# Lane Markings

Detected edges are further filtered and paired so that each edge with a positive gradient value has its other edge with negative gradient. These pairs of edges represent the beginning and the end of white line on black road. The resulting lane markings are in the center of each pair of start and end points. Detected lane markings are then sorted into three groups, each one represents a region where the left, central and right traffic lane is expected to be. Note that with our camera Intel RealSense SR300 we almost never get any results in left region because of the camera’s viewing angle. The primary sorting criteria is proximity of lane marking to previously detected polynomials (one in each of these three positions). If not all lane polynomials were found in previous frame, the secondary criteria is proximity to polynomials which are moved using the nearest detected lane polynomial. Finally, if a line point is not near to a polynomial or no polynomials were available, the distance to predefined default vertical lines is considered.

# RANSAC

TODO:
    - poly valid haben wir verändert -> Keine Ahnung wie

Having these three groups of lane markings, next step is to find three polynomials which represent (are supported by) most of these lane markings. In each group we divide them into three equally-sized groups – the top, central and bottom one. Then we select one from each of these groups and create Newton’s polynomial supported by these three lane markings. We check if the polynomial is valid, i.e. it’s not far from previous polynomial or from default line and we count supporter proportion (how many of lane marking lie close enough to the polynomial). If the polynomial is valid and has high enough supporter proportion, we use it, otherwise we repeat this lane-creation process until we find one good enough or until we reach iteration count limit.

# Generate moved polynomials

RANSAC may not be able detect all three polynomials due to not having enough lane markings, not finding a valid polynomial during  or insufficient supporters being properly associated with the lane marking. To improve detection in the next frame we can shift the detected polynomials (if any) by the lane width to where we estimate the missing lane marking to be. This improves especially the grouping of the lane markings to a lane at the beginning of the next frame if the car is in a bend. The lane markings would be assigned to a wrong group since the vertical default lines cross different road lanes during a cure. Without proper assignment of the lane markings to a group RANSAC cannot detect any polynomials.

If two lane polynomials were found during RANSAC, the nearer polynomial is moved to the missing position (e.g. the central polynomial is shifted to the left lane if the central and right lane were found). We shift the polynomials by moving the interpolation points of the selected lane polynomial along the normal of the gradient at each interpolation point. The moved points are used to interpolate a new Newton polynomial.

# Steering angle

Since we generated moved polynomials for all three lanes we always use the detected / moved polynomial of the right lane. We then calculate the normal of the gradient of the right lane polynomial at a y value slightly offset from the lower image fram. We then shift the point of the right lane polynomial at the y position by half the lane width. Assuming the car is always in the middle of the image we substract half of the image width from the x coordinate of the shifted point. Finally we can use trigonometric relationships to calculate the steering angle.

# Debugging

## \#define's

Following \#define's can be uncommented / commented to enable / disable different debugging features:

PUBLISH_IMAGES - publish the image to a topic visible with rviz

SAVE_FRAME_IMAGES - save the current image as a .jpeg for easier frame-by-frame analysis, the images are storred at $HOME_DIR/.ros/

These \#define's must be commented when code shall be compiled on the car as they display images and show them on a screen.

SHOW_EDGE_WINDOW - display the edges found by the edge detection
SHOW_LANE_MARKINGS_WINDOW - display the possible lane markings created from the detected edges
SHOW_GROUPED_LANE_MARKINGS_WINDOW - display the grouped lane markings (color coded: left - red, center - green, right - blue, groupless - yellow)
SHOW_RANSAC_WINDOW - display the polynomials fitted by RANSAC
SHOW_ANGLE_WINDOW - display the normal on the (shifted) right polnomial, the target point, and the angle in which the car would steer

## Additional features

All previous debug windows have been moved to functions for overall cleaner code. Furthermore each window can now be resized.

Debugging utility functions:

The debugPaintPolynom() function paints a given Newton polynomial on the given image.

The debugPaintPoints() function paints a batch of points.

The debugWriteImg() writes the given image as a .jpeg to the given location.

# Ideas for improving performance

## Removing the IP Mapper

The IP Mapper relies on known camera height, angle and lens values.Any change in e.g. camera height will result in lane detection not working properly.By removing the IPMapper makes the code more robust and independent from underlying hardware. However this results in the need to deal with the optical distortion as lines marking the lanes are not parallel anymore.

We attempted the IP Mapper by firstly changing the horizontal default lines to arbitrary lines to handle the optical distortion on straight parts. The lane detection and polynomial fitting namely in tight curves is an issue, as not all lines are seen at any given time. Furthermore polynomials can not be shifted by one offset again due to the distortion. Therefore different offsets are needed for the top, middle, and bottom of the image. We tried to improve the automatic lane width detection to calculate the lane widths at the top and bottom image borders and calculate the value for the middle of the image by interpolation.

The most recent, experimental version is located at: https://github.com/tobiasschuelke/catkin_ws_user/tree/tobias/ip_mapper_tests

## Multithreading / GPGPU using OpenCL

There are some parts of the code which could benefit from multithreading:

* edge detection - spawn arbitrarily many threads, each thread walks along a subset off the scanlines
* lane marking sorting - spawn arbitrarily many threads, each thread performs the checks used to associate a point with the lane marking on the points detected by the edge detection
* RANSAC - one thread is used per lane to fit the polynomial

The SoC on the Odroid XU4 also features a Mali GPU which supports OpenCL 1.1 Full Profile. It may be possible to use the GPU to do the edge detection or perform RANSAC in parallel.

TODO: Was sollen diese Zeilen hier drunter???
-> Die waren schon da, keine Ahnung ==> stehen lassen!

Vaclav' Blahut

Faculty of Informatics

Masaryk University Brno

395963@mail.muni.cz
