#include "laneDetection.h"

using namespace std;

// save frames as images in ~/.ros/
#define SAVE_FRAME_IMAGES

// show windows with results of each step in pipeline of one frame
//#define SHOW_EDGE_WINDOW
//#define SHOW_LANE_MARKINGS_WINDOW
#define SHOW_GROUPED_LANE_MARKINGS_WINDOW
#define SHOW_RANSAC_WINDOW
#define SHOW_ANGLE_WINDOW

// publish ransac and grouped lane frames to show it in rviz
//#define PUBLISH_IMAGES

// try kernel width 5 for now
const static int kernel1DWidth = 5;

// params for pubRGBImageMsg
unsigned int headSequenceId = 0;
ros::Time headTimeStamp;
string rgbFrameId = "_rgb_optical_frame";
sensor_msgs::CameraInfoPtr rgbCameraInfo;

// frame number for saving image files
size_t frame = 0;

const uint32_t MY_ROS_QUEUE_SIZE = 1;
const double PI = 3.14159265;

double m1 = 0;
double m2 = 0;
double n1 = 0;
double n2 = 0;

FuPoint<double> p1Draw = FuPoint<double>();
FuPoint<double> p1DrawShifted = FuPoint<double>();
FuPoint<double> p2Draw = FuPoint<double>();
FuPoint<double> p2DrawShifted = FuPoint<double>();
FuPoint<double> p3Draw = FuPoint<double>();
FuPoint<double> p3DrawShifted = FuPoint<double>();

cLaneDetectionFu::cLaneDetectionFu(ros::NodeHandle nh) : nh_(nh), priv_nh_("~") {
    string node_name = ros::this_node::getName();

    ROS_INFO("Node name: %s", node_name.c_str());

    priv_nh_.param<string>(node_name + "/cameraName", cameraName, "/usb_cam/image_raw");

    priv_nh_.param<int>(node_name + "/minYRoiLeft", minYRoiLeft, 10);
    priv_nh_.param<int>(node_name + "/maxYRoiLeft", maxYRoiLeft, 10);
    priv_nh_.param<int>(node_name + "/minYRoiCenter", minYRoiCenter, 50);
    priv_nh_.param<int>(node_name + "/maxYRoiCenter", maxYRoiCenter, 60);
    priv_nh_.param<int>(node_name + "/minYRoiRight", minYRoiRight, 120);
    priv_nh_.param<int>(node_name + "/maxYRoiRight", maxYRoiRight, 130);

    priv_nh_.param<int>(node_name + "/camW", camW, 640);
    priv_nh_.param<int>(node_name + "/camH", camH, 480);
    priv_nh_.param<int>(node_name + "/projYStart", projYStart, 60);
    priv_nh_.param<int>(node_name + "/projImageH", projImageH, 360);
    priv_nh_.param<int>(node_name + "/projImageW", projImageW, 640);
    priv_nh_.param<int>(node_name + "/projImageHorizontalOffset", projImageHorizontalOffset, 0);
    priv_nh_.param<int>(node_name + "/roiTopW", roiTopW, 620);
    priv_nh_.param<int>(node_name + "/roiBottomW", roiBottomW, 620);

    priv_nh_.param<int>(node_name + "/maxYRoi", maxYRoi, 360);
    priv_nh_.param<int>(node_name + "/minYDefaultRoi", minYDefaultRoi, 10);
    priv_nh_.param<int>(node_name + "/minYPolyRoi", minYPolyRoi, 11);

    priv_nh_.param<int>(node_name + "/interestDistancePoly", interestDistancePoly, 10);
    priv_nh_.param<int>(node_name + "/interestDistanceDefault", interestDistanceDefault, 5);

    priv_nh_.param<int>(node_name + "/iterationsRansac", iterationsRansac, 38);
    priv_nh_.param<double>(node_name + "/proportionThreshould", proportionThreshould, 0.68);

    priv_nh_.param<int>(node_name + "/gradientThreshold", gradientThreshold, 10);
    priv_nh_.param<int>(node_name + "/nonMaxWidth", nonMaxWidth, 16);
    priv_nh_.param<int>(node_name + "/laneMarkingSquaredThreshold", laneMarkingSquaredThreshold, 800);

    priv_nh_.param<int>(node_name + "/angleAdjacentLeg", angleAdjacentLeg, 18);

    priv_nh_.param<int>(node_name + "/scanlinesVerticalDistance", scanlinesVerticalDistance, 2);
    priv_nh_.param<int>(node_name + "/scanlinesMaxCount", scanlinesMaxCount, 200);

    priv_nh_.param<int>(node_name + "/maxAngleDiff", maxAngleDiff, 14);

    priv_nh_.param<int>(node_name + "/polyY1", polyY1, 20);
    priv_nh_.param<int>(node_name + "/polyY2", polyY2, 50);
    priv_nh_.param<int>(node_name + "/polyY3", polyY3, 70);

    double f_u;
    double f_v;
    double c_u;
    double c_v;
    double camDeg;
    double camHeight;
    int camHHalf = camH / 2;

    priv_nh_.param<double>(node_name + "/f_u", f_u, 624.650635);
    priv_nh_.param<double>(node_name + "/f_v", f_v, 626.987244);
    priv_nh_.param<double>(node_name + "/c_u", c_u, 309.703230);
    priv_nh_.param<double>(node_name + "/c_v", c_v, 231.473613);
    priv_nh_.param<double>(node_name + "/camDeg", camDeg, 13.0);
    priv_nh_.param<double>(node_name + "/camHeight", camHeight, 36.2);

    //ipMapper = IPMapper(camW, camHHalf, f_u, f_v, c_u, c_v, camDeg, camHeight);

    projImageWHalf = projImageW / 2;

    polyDetectedLeft = false;
    polyDetectedCenter = false;
    polyDetectedRight = false;

    bestPolyLeft = make_pair(NewtonPolynomial(), 0);
    bestPolyCenter = make_pair(NewtonPolynomial(), 0);
    bestPolyRight = make_pair(NewtonPolynomial(), 0);

    laneMarkingsLeft = vector<FuPoint<int>>();
    laneMarkingsCenter = vector<FuPoint<int>>();
    laneMarkingsRight = vector<FuPoint<int>>();
    laneMarkingsNotUsed = vector<FuPoint<int>>();

    polyLeft = NewtonPolynomial();
    polyCenter = NewtonPolynomial();
    polyRight = NewtonPolynomial();

    supportersLeft = vector<FuPoint<int>>();
    supportersCenter = vector<FuPoint<int>>();
    supportersRight = vector<FuPoint<int>>();

    prevPolyLeft = NewtonPolynomial();
    prevPolyCenter = NewtonPolynomial();
    prevPolyRight = NewtonPolynomial();

    movedPolyLeft = NewtonPolynomial();
    movedPolyCenter = NewtonPolynomial();
    movedPolyRight = NewtonPolynomial();

    defaultXLeft = Line<int>(projImageH);
    defaultXCenter = Line<int>(projImageH);
    defaultXRight = Line<double>(projImageH);

    movedPointForAngle = FuPoint<double>();
    pointForAngle = FuPoint<double>();

    maxDistance = 4;

    lastAngle = 0;

    headTimeStamp = ros::Time::now();

    read_images_ = nh.subscribe(nh_.resolveName(cameraName), MY_ROS_QUEUE_SIZE, &cLaneDetectionFu::ProcessInput, this);

    publishAngle = nh.advertise<std_msgs::Float32>("/lane_model/angle", MY_ROS_QUEUE_SIZE);

    image_transport::ImageTransport image_transport(nh);

    imagePublisher = image_transport.advertiseCamera("/lane_model/lane_model_image", MY_ROS_QUEUE_SIZE);

#ifdef PUBLISH_IMAGES
    imagePublisherRansac = image_transport.advertiseCamera("/lane_model/ransac", MY_ROS_QUEUE_SIZE);
    imagePublisherLaneMarkings = image_transport.advertiseCamera("/lane_model/lane_markings", MY_ROS_QUEUE_SIZE);
#endif

    if (!rgbCameraInfo) {
        rgbCameraInfo.reset(new sensor_msgs::CameraInfo());
        rgbCameraInfo->width = projImageW;
        rgbCameraInfo->height = projImageH + 50;
    }

    //from camera properties and ROI etc we get scanlines (=line segments, úsečky)
    //these line segments are lines in image on whose we look for edges
    //the outer vector represents rows on image, inner vector is vector of line segments of one row, usualy just one line segment
    //we should generate this only once in the beginning! or even just have it pregenerated for our cam
    scanlines = getScanlines();

#ifdef SAVE_FRAME_IMAGES
    struct stat st;
    if (!stat("groupedLaneMarkings", &st))
        system("exec rm -r ./groupedLaneMarkings/*");
    else
        mkdir("groupedLaneMarkings", S_IRWXU);

    if (!stat("ransac", &st))
        system("exec rm -r ./ransac/*");
    else
        mkdir("ransac", S_IRWXU);

    if (!stat("polyRange", &st))
        system("exec rm -r ./polyRange/*");
    else
        mkdir("polyRange", S_IRWXU);
#endif
}

cLaneDetectionFu::~cLaneDetectionFu() {
}

void cLaneDetectionFu::ProcessInput(const sensor_msgs::Image::ConstPtr &msg) {
    // clear some stuff from the last cycle
    bestPolyLeft = make_pair(NewtonPolynomial(), 0);
    bestPolyCenter = make_pair(NewtonPolynomial(), 0);
    bestPolyRight = make_pair(NewtonPolynomial(), 0);

    supportersLeft.clear();
    supportersCenter.clear();
    supportersRight.clear();

    //use ROS image_proc or opencv instead of ip mapper?

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat image = cv_ptr->image.clone();
    Mat cutImage = image(cv::Rect(0, camH * 0.25f, camW, camH * 0.75f));

    //Mat remappedImage = ipMapper.remap(cutImage);
    //cv::Mat transformedImage =  remappedImage(cv::Rect((camW / 2) - projImageWHalf + projImageHorizontalOffset, projYStart, projImageW, projImageH)).clone();
    cv::Mat transformedImage = cutImage;

    cv::resize(cutImage, transformedImage, cv::Size(projImageW, projImageH));

    //cv::flip(transformedImage, transformedImage, 0);

    // scanlines -> edges (in each scanline we find maximum and minimum of kernel fn ~= where the edge is)
    // this is where we use input image!
    vector<vector<EdgePoint>> edges = cLaneDetectionFu::scanImage(transformedImage);

#ifdef SHOW_EDGE_WINDOW
    drawEdgeWindow(transformedImage, edges);
#endif

    // edges -> lane markings
    vector<FuPoint<int>> laneMarkings = cLaneDetectionFu::extractLaneMarkings(edges);

#ifdef SHOW_LANE_MARKINGS_WINDOW
    drawLaneMarkingsWindow(transformedImage, laneMarkings);
#endif

    // initialize defaultXLeft, defaultXCenter and defaultXRight values
    findLanePositions(laneMarkings);

    // assign lane markings to lanes
    buildLaneMarkingsLists(laneMarkings);

#if defined(PUBLISH_IMAGES) || defined(SHOW_GROUPED_LANE_MARKINGS_WINDOW)
    drawGroupedLaneMarkingsWindow(transformedImage);
    drawPolyRangeWindow(transformedImage);
#endif

    // try to fit a polynomial for each lane
    ransac();
    improveLaneWidth();
    // generate new polynomials based on polynomials found in ransac for lanes without ransac polynomial
    generateMovedPolynomials();

#if defined(PUBLISH_IMAGES) || defined(SHOW_RANSAC_WINDOW)
    drawRansacWindow(transformedImage);
#endif

    // calculate and publish the angle the car should drive
    pubAngle();

#if defined(PUBLISH_IMAGES) || defined(SHOW_ANGLE_WINDOW)
    drawAngleWindow(transformedImage);
#endif
}

/*
 *      Pipeline:
 */

/**
 * Compute scanlines. Each may consist of multiple segments, split at regions
 * that should not be inspected by the kernel.
 * @param side
 * @return vector of segments of scanlines, walk these segments with the kernel
 */
vector<vector<LineSegment<int>>> cLaneDetectionFu::getScanlines() {
    vector<vector<LineSegment<int>>> scanlines;

    vector<cv::Point> checkContour;
    checkContour.push_back(cv::Point(projImageWHalf - (roiBottomW / 2), maxYRoi - 1));
    checkContour.push_back(cv::Point(projImageWHalf + (roiBottomW / 2), maxYRoi - 1));
    checkContour.push_back(cv::Point(projImageWHalf + (roiTopW / 2), minYPolyRoi));
    checkContour.push_back(cv::Point(projImageWHalf - (roiTopW / 2), minYPolyRoi));

    int scanlineStart = 0;
    int scanlineEnd = projImageW;

    int segmentStart = -1;
    vector<LineSegment<int>> scanline;
    //i = y; j = x;
    for (int i = 1;
         (i / scanlinesVerticalDistance) < scanlinesMaxCount && i <= projImageH;
         i += scanlinesVerticalDistance) {
        scanline = vector<LineSegment<int>>();

        // walk along line
        for (int j = scanlineStart; j <= scanlineEnd; j++) {
            bool isInside = pointPolygonTest(checkContour, cv::Point(j, i), false) >= 0;

            // start new scanline segment
            if (isInside && j < scanlineEnd) {
                if (segmentStart == -1) segmentStart = j;
                // found end of scanline segment, reset start
            } else if (segmentStart != -1) {
                scanline.push_back(
                        LineSegment<int>(
                                FuPoint<int>(segmentStart, i),
                                FuPoint<int>(j - 1, i)
                        )
                );

                segmentStart = -1;
            }
        }
        // push segments found
        if (scanline.size()) {
            scanlines.push_back(scanline);
        }
    }
    return scanlines;
}

/**
 * Walk with prewitt/sobel kernel along all scanlines.
 * @param image
 * @return All edgePoints on side, sorted by scanlines.
 */
vector<vector<EdgePoint>> cLaneDetectionFu::scanImage(cv::Mat image) {
    //ROS_INFO_STREAM("scanImage() - " << scanlines.size() << " scanlines.");
    vector<vector<EdgePoint>> edgePoints;

    //const Image &image = getImage();
    //const ImageDimensions &imgDim = getImageDimensions();
    //const OmnidirectionalCameraMatrix &cameraMatrix = getOmnidirectionalCameraMatrix();

    // scanline length can maximal be image height/width
    int scanlineMaxLength = image.cols;

    // store kernel results on current scanline in here
    vector<int> scanlineVals(scanlineMaxLength, 0);

    // walk over all scanlines
    for (auto scanline : scanlines) {
        // set all brightness values on scanline to 0;
        fill(scanlineVals.begin(), scanlineVals.end(), 0);
        int offset = 0;
        if (scanline.size()) {
            offset = scanline.front().getStart().getY();
        }

        // scanline consisting of multiple segments
        // walk over each but store kernel results for whole scanline
        for (auto segment : scanline) {
            int start = segment.getStart().getX();
            int end = segment.getEnd().getX();

            // walk along segment
            for (int i = start; i < end - kernel1DWidth; i++) {
                int sum = 0;

                //cv::Mat uses ROW-major system -> .at(y,x)
                // use kernel width 5 and try sobel kernel
                sum -= image.at<uint8_t>(offset - 1, i);
                sum -= image.at<uint8_t>(offset - 1, i + 1);
                // kernel is 0
                sum += image.at<uint8_t>(offset - 1, i + 2);
                sum += image.at<uint8_t>(offset - 1, i + 4);

                sum -= 2 * image.at<uint8_t>(offset, i);
                sum -= 2 * image.at<uint8_t>(offset, i + 1);
                // kernel is 0
                sum += 2 * image.at<uint8_t>(offset, i + 2);
                sum += 2 * image.at<uint8_t>(offset, i + 4);

                sum -= image.at<uint8_t>(offset + 1, i);
                sum -= image.at<uint8_t>(offset + 1, i + 1);
                // kernel is 0
                sum += image.at<uint8_t>(offset + 1, i + 2);
                sum += image.at<uint8_t>(offset + 1, i + 4);


                // +4 because of sobel weighting
                sum = sum / (3 * kernel1DWidth + 4);
                //ROS_INFO_STREAM(sum << " is kernel sum.");
                if (abs(sum) > gradientThreshold) {
                    // set scanlineVals at center of kernel
                    scanlineVals[i + kernel1DWidth / 2] = sum;
                }
            }
        }

        // after walking over all segments of one scanline
        // do non-max-suppression
        // for both minima and maxima at same time
        // TODO: Jannis: find dryer way
        int indexOfLastMaximum = 0;
        int valueOfLastMaximum = 0;
        int indexOfLastMinimum = 0;
        int valueOfLastMinimum = 0;
        for (int i = 1; i < scanlineMaxLength - 1; i++) {
            // check if maximum
            if (scanlineVals[i] > 0) {
                if (scanlineVals[i] < scanlineVals[i - 1] or scanlineVals[i] < scanlineVals[i + 1]) {
                    scanlineVals[i] = 0;
                } else {
                    // this pixel can just survive if the next maximum is not too close
                    if (i - indexOfLastMaximum > nonMaxWidth) {
                        // this is a new maximum
                        indexOfLastMaximum = i;
                        valueOfLastMaximum = scanlineVals[i];
                    } else {
                        if (valueOfLastMaximum < scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMaximum] = 0;
                            indexOfLastMaximum = i;
                            valueOfLastMaximum = scanlineVals[i];
                        } else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
            // check if minimum
            if (scanlineVals[i] < 0) {
                if (scanlineVals[i] > scanlineVals[i - 1] or scanlineVals[i] > scanlineVals[i + 1]) {
                    scanlineVals[i] = 0;
                } else {
                    // this pixel can just survive if the next minimum is not too close
                    if (i - indexOfLastMinimum > nonMaxWidth) {
                        // this is a new minimum
                        indexOfLastMinimum = i;
                        valueOfLastMinimum = scanlineVals[i];
                    } else {
                        if (valueOfLastMinimum > scanlineVals[i]) {
                            // this is a new maximum
                            // drop the old maximum
                            scanlineVals[indexOfLastMinimum] = 0;
                            indexOfLastMinimum = i;
                            valueOfLastMinimum = scanlineVals[i];
                        } else {
                            scanlineVals[i] = 0;
                        }
                    }
                }
            }
        }
        // collect all the edgePoints for scanline
        vector<EdgePoint> scanlineEdgePoints;
        for (int i = 0; i < static_cast<int>(scanlineVals.size()); i++) {
            if (scanlineVals[i] != 0) {
                FuPoint<int> imgPos = FuPoint<int>(i, offset);

                FuPoint<Meter> relPos = FuPoint<Meter>();//offset, i);//cameraMatrix.transformToLocalCoordinates(imgPos);
                scanlineEdgePoints.push_back(EdgePoint(imgPos, relPos, scanlineVals[i]));
            }
        }
        edgePoints.push_back(move(scanlineEdgePoints));
    }
    // after walking along all scanlines
    // return edgePoints
    return edgePoints;
}


/**
 * uses Edges to extract lane markings
 */
vector<FuPoint<int>> cLaneDetectionFu::extractLaneMarkings(const vector<vector<EdgePoint>> &edges) {
    vector<FuPoint<int>> result;

    for (const vector<EdgePoint> &line : edges) {
        if (line.empty()) continue;

        for (
                auto edgePosition = line.begin(), nextEdgePosition = edgePosition + 1;
                nextEdgePosition != line.end();
                edgePosition = nextEdgePosition, ++nextEdgePosition
                ) {
            if (edgePosition->isPositive() and not nextEdgePosition->isPositive()) {
                FuPoint<int> candidateStartEdge = edgePosition->getImgPos();
                FuPoint<int> candidateEndEdge = nextEdgePosition->getImgPos();

                if ((candidateStartEdge - candidateEndEdge).squaredMagnitude() < laneMarkingSquaredThreshold) {
                    result.push_back(center(candidateStartEdge, candidateEndEdge));
                }
            }
        }
    }

    // sort the lane marking edge points
    sort(result.begin(), result.end(),
              [](FuPoint<int> a, FuPoint<int> b) {
                  return a.getY() > b.getY();
              });

    return result;
}

/**
 * Calculates x positions of the lanes. Lane markings of the first 10 rows from the bottom
 * of the image are used. Car must start between right and center lane because all lane
 * marking points in left half of the image are considered as possible lane edge points
 * of center lane (analog: right half of image for right lane).
 *
 * Lane marking points in range of other lane marking points are supporters because they form a line.
 * The x-value of found lane points with maximum supporters will be used. This ensures that non-lane
 * points are ignored.
 *
 * Start position of left lane is calculated after start positions of center and right lanes are found.
 */
void cLaneDetectionFu::findLanePositions(vector<FuPoint<int>> &laneMarkings) {
    if (upperLaneWidthUpdated) {
        return;
    }

    // defaultXLeft is calculated after center and right lane position is found
    if (laneWidth > 0 && defaultXLeft.isStartValueSet()) {
        if (polyDetectedCenter && (polyDetectedRight || polyDetectedLeft)) {
            for (int y = 0; y < projImageH / 3; y++) {
                double x = polyDetectedRight ? polyRight.at(y) : polyLeft.at(y);
                double centerX = polyCenter.at(y);

                if (x > 0 && x < projImageW && centerX > 0 && centerX < projImageW) {
                    upperLaneWidthUpdated = true;
                    upperLaneWidth = polyDetectedRight ? x - centerX : centerX - x;

                    double xBottom = polyDetectedRight ? polyRight.at(projImageH) : polyLeft.at(projImageH);
                    double centerXBottom = polyCenter.at(projImageH);
                    laneWidth = polyDetectedRight ? xBottom - centerXBottom : centerXBottom - xBottom;

                    defaultXCenter.setStart((int) polyCenter.at(projImageH));
                    defaultXCenter.setEnd((int) polyCenter.at(0));

                    if (polyDetectedLeft) {
                        defaultXLeft.setStart((int) polyLeft.at(projImageH));
                        defaultXLeft.setEnd((int) polyLeft.at(0));
                    } else {
                        defaultXLeft.setEnd((int) (polyCenter.at(0) - upperLaneWidth));
                    }

                    if (polyDetectedRight) {
                        defaultXRight.setStart(polyRight.at(projImageH));
                        defaultXRight.setEnd(polyRight.at(0));
                    } else {
                        defaultXRight.setEnd((int) (polyCenter.at(0) + upperLaneWidth));
                    }

                    // TODO: Only for testing
                    if (polyDetectedRight) {
                        double mRight = gradient(projImageH / 2, polyRight);
                        double mCenter = gradient(projImageH / 2, polyCenter);

                        double halfHeight = (double) projImageH / 2;
                        double x = defaultXRight.atY(halfHeight);
                        double y = defaultXRight.atX(x + 1);
                        double m = y - halfHeight;

                        m1 = 1 / mCenter;
                        m2 = mRight;

                        n1 = y - m1 * (x + 1);
                        n2 = projImageH / 2 - m2 * polyRight.at(projImageH / 2);

                        ROS_INFO("       Right Poly gradient: %f, center: %f, m: %f, halfHeight: %f", mRight, mCenter, m, halfHeight);
                    }

                    ROS_INFO("       findLane: end set to %d, %d, %d", defaultXLeft.atX(0), defaultXCenter.atX(0), defaultXRight.atX(0));

                    return;
                }
            }
        }
        return;
    }

    // counts how many lane marking points form a line with point in centerStart
    // at same index
    vector<int> centerSupporter;
    vector<int> rightSupporter;

    // possible start points of center lane
    vector<FuPoint<int> *> centerStart;
    vector<FuPoint<int> *> rightStart;

    for (int j = 0; j < laneMarkings.size(); j++) {
        FuPoint<int> *laneMarking = &laneMarkings.at(j);

        if (laneMarking->getY() > maxYRoi) {
            continue;
        }

        bool isSupporter = false;
        if (laneMarking->getX() < projImageWHalf + projImageHorizontalOffset) {
            for (int i = 0; i < centerStart.size(); i++) {
                if (isInRangeAndImage(*centerStart.at(i), *laneMarking)) {
                    isSupporter = true;
                    centerSupporter.at(i)++;

                    if (centerSupporter.at(i) > 5) {
                        goto defaultLineCalculation;
                    }

                    break;
                }
            }

            if (!isSupporter) {
                centerStart.push_back(laneMarking);
                centerSupporter.push_back(0);
            }
        } else {
            for (int i = 0; i < rightStart.size(); i++) {
                if (isInRangeAndImage(*rightStart.at(i), *laneMarking)) {
                    isSupporter = true;
                    rightSupporter.at(i)++;

                    if (rightSupporter.at(i) > 5) {
                        goto defaultLineCalculation;
                    }

                    break;
                }
            }

            if (!isSupporter) {
                rightStart.push_back(laneMarking);
                rightSupporter.push_back(0);
            }
        }
    }

    defaultLineCalculation:

    // use x-value of lane marking point with most (and at least 3) supporters
    if (centerStart.size() > 0) {
        vector<int>::iterator maxCenterElement = max_element(centerSupporter.begin(), centerSupporter.end());

        if (*maxCenterElement > 3) {
            int position = distance(centerSupporter.begin(), maxCenterElement);
            defaultXCenter.setStart(centerStart.at(position)->getX());

            ROS_INFO("       findLane: defaultXCenter start et to %d", centerStart.at(position)->getX());
        }
    }

    if (rightStart.size() > 0) {
        vector<int>::iterator maxRightElement = max_element(rightSupporter.begin(), rightSupporter.end());

        if (*maxRightElement > 3) {
            int position = distance(rightSupporter.begin(), maxRightElement);
            defaultXRight.setStart(rightStart.at(position)->getX());

            ROS_INFO("       findLane: defaultXRight start et to %d", rightStart.at(position)->getX());
        }
    }

    if (defaultXCenter.isStartValueSet() && defaultXRight.isStartValueSet()) {
        double bottomRightLaneX = defaultXRight.atY(projImageH);
        double bottomCenterLaneX = defaultXCenter.atY(projImageH);

        if (bottomRightLaneX != -1 && bottomCenterLaneX != -1) {
            laneWidth = (bottomRightLaneX - bottomCenterLaneX);
            defaultXLeft.setStart(bottomCenterLaneX - (int) laneWidth);
        }
    }
}

/**
 * Creates three vectors of lane marking points out of the given lane marking
 * point vector.
 *
 * A point has to lie within the ROI of the previously detected lane polynomial
 * or within the default ROI, if no polynomial was detected.
 * The lists are the input data for the RANSAC algorithm.
 *
 * @param laneMarkings  a vector containing all detected lane markings
 */
void cLaneDetectionFu::buildLaneMarkingsLists(const vector<FuPoint<int>> &laneMarkings) {
    laneMarkingsLeft.clear();
    laneMarkingsCenter.clear();
    laneMarkingsRight.clear();
    laneMarkingsNotUsed.clear();

    ROS_INFO("Frame: %d, laneMarkings: %d", frame, sizeof(laneMarkings));

    for (FuPoint<int> laneMarking : laneMarkings) {

        // check if lane marking point is near to found lane poly of ransac

        if (polyDetectedRight) {
            if (isInPolyRoi(polyRight, laneMarking)) {
                ROS_INFO("      -> polyRight");
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        if (polyDetectedCenter) {
            if (isInPolyRoi(polyCenter, laneMarking)) {
                ROS_INFO("      -> polyCenter");
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (polyDetectedLeft) {
            if (isInPolyRoi(polyLeft, laneMarking)) {
                ROS_INFO("      -> polyLeft");
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        // check if lane marking point is near to moved poly of ransac

        if (movedPolyLeft.isInitialized()) {
            if (isInPolyRoi(movedPolyLeft, laneMarking)) {
                ROS_INFO("      -> movedPolyLeft");
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        if (movedPolyCenter.isInitialized()) {
            if (isInPolyRoi(movedPolyCenter, laneMarking)) {
                ROS_INFO("      -> movedPolyCenter");
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (movedPolyRight.isInitialized()) {
            if (isInPolyRoi(movedPolyRight, laneMarking)) {
                ROS_INFO("      -> movedPolyRight");
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        // if ransac found a polynomial in last frame skip default lane comparison
        // TODO: could this be an issue?
        if (polyDetectedLeft || polyDetectedCenter || polyDetectedRight) {
            continue;
        }

        // no poly available from last frame, check if lane marking point is near to
        // default lane or near to already classified point (this way points are also
        // classified properly if car starts in a turn)

        if (laneMarkingsRight.size() > 0) {
            if (isInRangeAndImage(laneMarkingsRight.at(laneMarkingsRight.size() - 1), laneMarking)) {
                ROS_INFO("      -> in Range of right lane markings");
                laneMarkingsRight.push_back(laneMarking);
                continue;
            }
        }

        if (laneMarkingsCenter.size() > 0) {
            if (isInRangeAndImage(laneMarkingsCenter.at(laneMarkingsCenter.size() - 1), laneMarking)) {
                ROS_INFO("      -> in Range of center lane markings");
                laneMarkingsCenter.push_back(laneMarking);
                continue;
            }
        }

        if (laneMarkingsLeft.size() > 0) {
            if (isInRangeAndImage(laneMarkingsLeft.at(laneMarkingsLeft.size() - 1), laneMarking)) {
                ROS_INFO("      -> in Range of left lane markings");
                laneMarkingsLeft.push_back(laneMarking);
                continue;
            }
        }

        if (isInDefaultRoi(RIGHT, laneMarking)) {
            ROS_INFO("      -> in default ROI: right lane");
            laneMarkingsRight.push_back(laneMarking);
            continue;
        }

        if (isInDefaultRoi(CENTER, laneMarking)) {
            ROS_INFO("      -> in default ROI: center lane");
            laneMarkingsCenter.push_back(laneMarking);
            continue;
        }

        if (isInDefaultRoi(LEFT, laneMarking)) {
            ROS_INFO("      -> in default ROI: left lane");
            laneMarkingsLeft.push_back(laneMarking);
            continue;
        }

        laneMarkingsNotUsed.push_back(laneMarking);
    }
}

/**
 * Starts the RANSAC algorithm for detecting each of the three lane marking
 * polynomials.
 */
void cLaneDetectionFu::ransac() {
    polyDetectedLeft = ransacInternal(LEFT, laneMarkingsLeft, bestPolyLeft,
                                      polyLeft, supportersLeft, prevPolyLeft);

    polyDetectedCenter = ransacInternal(CENTER, laneMarkingsCenter,
                                        bestPolyCenter, polyCenter, supportersCenter, prevPolyCenter);

    polyDetectedRight = ransacInternal(RIGHT, laneMarkingsRight, bestPolyRight,
                                       polyRight, supportersRight, prevPolyRight);

    //polyDetectedLeft = false;
    //polyDetectedCenter = polyDetectedCenter && !upperLaneWidthUpdated;

    ROS_INFO("      Ransac poly detected: left = %s, center = %s, right = %s", polyDetectedLeft ? "true" : "false",
             polyDetectedCenter ? "true" : "false", polyDetectedRight ? "true" : "false");
}

/**
 * Detects a polynomial with RANSAC in a given list of lane marking edge points.
 *
 * @param position      The position of the wanted polynomial
 * @param laneMarkings  A reference to the list of lane marking edge points
 * @param bestPoly      A reference to a pair containing the present best
 *                      detected polynomial and a value representing the fitting
 *                      quality called proportion
 * @param poly          A reference to the polynomial that gets detected
 * @param supporters    A reference to the supporter points of the present best
 *                      polynomial
 * @param prevPoly      A reference to the previous polynomial detected at this
 *                      position
 * @return              true if a polynomial could be detected and false when not
 */
bool cLaneDetectionFu::ransacInternal(ePosition position,
                                      vector<FuPoint<int>> &laneMarkings,
                                      pair<NewtonPolynomial, double> &bestPoly, NewtonPolynomial &poly,
                                      vector<FuPoint<int>> &supporters, NewtonPolynomial &prevPoly) {

    if (laneMarkings.size() < 7) {
        ROS_INFO("      Ransac (lane %d) abort: too few lane markings", position);

        prevPoly = poly;
        poly.clear();
        return false;
    }

    vector<FuPoint<int>> tmpSupporters = vector<FuPoint<int>>();

    // vectors for points selected from the bottom, mid and top of the sorted
    // point vector
    vector<FuPoint<int>> markings1 = vector<FuPoint<int>>();
    vector<FuPoint<int>> markings2 = vector<FuPoint<int>>();
    vector<FuPoint<int>> markings3 = vector<FuPoint<int>>();

    bool highEnoughY = false;

    // Points are selected from the bottom, mid and top. The selection regions
    // are spread apart for better results during RANSAC
    for (vector<FuPoint<int>>::size_type i = 0; i != laneMarkings.size(); i++) {
        if (i < double(laneMarkings.size()) / 7) {
            markings1.push_back(laneMarkings[i]);
        } else if (i >= (double(laneMarkings.size()) / 7) * 3
                   && i < (double(laneMarkings.size()) / 7) * 4) {
            markings2.push_back(laneMarkings[i]);
        } else if (i >= (double(laneMarkings.size()) / 7) * 6) {
            markings3.push_back(laneMarkings[i]);
        }

        if (laneMarkings[i].getY() > 5) {
            highEnoughY = true;
        }
    }

    polyY3 = laneMarkings[0].getY();
    polyY1 = laneMarkings[laneMarkings.size() - 1].getY();
    polyY2 = polyY1 + ((polyY3 - polyY1) / 2);

    ROS_INFO("      Ransac: polyY1 - 3: %d, %d, %d", polyY1, polyY2, polyY3);

    //what is this for?
    if (position == CENTER) {
        if (!highEnoughY) {
            ROS_INFO("      Ransac abort: position center and !highEnoughY");

            prevPoly = poly;
            poly.clear();
            return false;
        }
    }

    // save the polynomial from the previous picture
    prevPoly = poly;

    for (int i = 0; i < iterationsRansac; i++) {

        // randomly select 3 different lane marking points from bottom, mid and
        // top
        int pos1 = rand() % markings1.size();
        int pos2 = rand() % markings2.size();
        int pos3 = rand() % markings3.size();

        FuPoint<int> p1 = markings1[pos1];
        FuPoint<int> p2 = markings2[pos2];
        FuPoint<int> p3 = markings3[pos3];

        double p1X = p1.getX();
        double p1Y = p1.getY();
        double p2X = p2.getX();
        double p2Y = p2.getY();
        double p3X = p3.getX();
        double p3Y = p3.getY();

        // clear poly for reuse
        poly.clear();

        // create a polynomial with the selected points
        poly.addData(p1X, p1Y);
        poly.addData(p2X, p2Y);
        poly.addData(p3X, p3Y);

        // check if this polynomial is not useful
        if (!polyValid(position, poly, prevPoly)) {
            ROS_INFO("      Ransac: poly is invalid");

            poly.clear();
            continue;
        }

        // count the supporters and save them for debugging
        int count1 = 0;
        int count2 = 0;
        int count3 = 0;

        // find the supporters
        tmpSupporters.clear();

        for (FuPoint<int> p : markings1) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count1++;
                tmpSupporters.push_back(p);
            }
        }

        for (FuPoint<int> p : markings2) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count2++;
                tmpSupporters.push_back(p);
            }
        }

        for (FuPoint<int> p : markings3) {
            if (horizDistanceToPolynomial(poly, p) <= maxDistance) {
                count3++;
                tmpSupporters.push_back(p);
            }
        }

        if (position != CENTER && count1 == 0 || count2 == 0 || count3 == 0) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly had no supporters in one of the regions");
            continue;
        }

        // calculate the proportion of supporters of all lane markings
        double proportion = (double(count1) / markings1.size()
                             + double(count2) / markings2.size()
                             + 3 * (double(count3) / markings3.size())) / 5;

        if (position != CENTER && proportion < proportionThreshould) {
            poly.clear();
            //DEBUG_TEXT(dbgMessages, "Poly proportion was smaller than threshold");
            continue;
        }

        // check if poly is better than bestPoly
        if (proportion >= bestPoly.second) {
            bestPoly = make_pair(poly, proportion);
            supporters = tmpSupporters;
        }
    }

    poly = bestPoly.first;

    if (poly.getDegree() == -1) {
        return false;
    }

    // TODO: update default lines?
    /*
    if (position == LEFT) {
        //defaultXLeft.setStartPoint(laneMarkings[laneMarkings.size() - 1]);
        //defaultXLeft.setEndPoint(laneMarkings[0]);

        defaultXLeft.setStart((int) poly.at(projImageH));
        defaultXLeft.setEnd((int) poly.at(0));
    } else if (position == CENTER) {
        //defaultXCenter.setStartPoint(laneMarkings[laneMarkings.size() - 1]);
        //defaultXCenter.setEndPoint(laneMarkings[0]);

        defaultXCenter.setStart((int) poly.at(projImageH));
        defaultXCenter.setEnd((int) poly.at(0));
    } else if (position == RIGHT) {
        //defaultXRight.setStartPoint(laneMarkings[laneMarkings.size() - 1]);
        //defaultXRight.setEndPoint(laneMarkings[0]);
        defaultXRight.setStart((int) poly.at(projImageH));
        defaultXRight.setEnd((int) poly.at(0));
    }*/

    return true;
}

void cLaneDetectionFu::improveLaneWidth() {
    if (!upperLaneWidthUpdated || !polyDetectedCenter) {
        return;
    }

    if (laneMarkingsCenter.empty())
        return;

    int centerX;
    if (!laneWidthFinal) {
        int bottom = projImageH - 5;
        centerX = laneMarkingsCenter.at(0).getY();

        if (centerX > bottom && isPartOfLine(false, laneMarkingsCenter, laneMarkingsCenter.at(0))) {
            if (polyDetectedRight && !laneMarkingsRight.empty() && laneMarkingsRight.at(0).getY() > bottom) {
                if (isPartOfLine(false, laneMarkingsRight, laneMarkingsRight.at(0))) {
                    laneWidthFinal = true;
                    laneWidth = polyRight.at(projImageH) - polyCenter.at(projImageH);

                    ROS_INFO("Lane width now final: %f", laneWidth);
                }
            }
            if (polyDetectedLeft && !laneMarkingsLeft.empty() && laneMarkingsLeft.at(0).getY() > bottom) {
                if (isPartOfLine(false, laneMarkingsLeft, laneMarkingsLeft.at(0))) {
                    laneWidthFinal = true;
                    laneWidth = polyCenter.at(projImageH) - polyLeft.at(projImageH);

                    ROS_INFO("Lane width now final: %f", laneWidth);
                }
            }
        }
    }

    if (!upperLaneWidthFinal) {
        int top = 5;
        centerX = laneMarkingsCenter.at(laneMarkingsCenter.size() - 1).getY();
        if (centerX < top && isPartOfLine(true, laneMarkingsCenter, laneMarkingsCenter.at(laneMarkingsCenter.size() - 1))) {
            if (polyDetectedRight && !laneMarkingsRight.empty() && laneMarkingsRight.at(laneMarkingsRight.size() - 1).getY() < top) {
                int rightX = laneMarkingsRight.at(laneMarkingsRight.size() - 1).getY();
                if (rightX > top && centerX > top) {
                    if (isPartOfLine(true, laneMarkingsRight, laneMarkingsRight.at(laneMarkingsRight.size() - 1))) {
                        upperLaneWidthFinal = true;
                        upperLaneWidth = polyRight.at(0) - polyCenter.at(0);

                        ROS_INFO("Upper lane width now final: %f", upperLaneWidth);
                    }
                }
            }
            if (polyDetectedLeft && !laneMarkingsLeft.empty() && laneMarkingsLeft.at(laneMarkingsLeft.size() - 1).getY() < top) {
                int leftX = laneMarkingsLeft.at(laneMarkingsLeft.size() - 1).getY();
                if (leftX > top && centerX > top) {
                    if (isPartOfLine(true, laneMarkingsLeft, laneMarkingsLeft.at(laneMarkingsLeft.size() - 1))) {
                        upperLaneWidthFinal = true;
                        upperLaneWidth = polyCenter.at(0) - polyLeft.at(0);

                        ROS_INFO("Upper lane width now final: %f", upperLaneWidth);
                    }
                }
            }
        }
    }
}

bool cLaneDetectionFu::isPartOfLine(bool invert, vector<FuPoint<int>> &laneMarkings, FuPoint<int> &p) {
    int supporter = 0;
    FuPoint<int> *currentPoint = &p;

    if (invert) {
        for (int i = laneMarkings.size() - 1; i >= 0; i--) {
            if (!isInRange(laneMarkings.at(i), *currentPoint))
                break;
            currentPoint = &laneMarkings.at(i);
            supporter++;
        }
    } else {
        for (int i = 0; i < laneMarkings.size(); i++) {
            if (!isInRange(laneMarkings.at(i), *currentPoint))
                break;
            currentPoint = &laneMarkings.at(i);
            supporter++;
        }
    }
ROS_INFO("supporter: %d invert: %d", supporter, invert);
    return supporter > 3;
}

/**
 * Shifts detected lane polynomials to the position of the undected lane polyominals,
 * so they can be used in the next cycle to group the lane markings.
 */
void cLaneDetectionFu::generateMovedPolynomials() {
    // TODO: could this be an issue? we clear the moved polys beforehand ...

    movedPolyLeft.clear();
    movedPolyCenter.clear();
    movedPolyRight.clear();

    isPolyMovedLeft = false;
    isPolyMovedCenter = false;
    isPolyMovedRight = false;

    if ((!polyDetectedLeft && !polyDetectedCenter && !polyDetectedRight)
        || (polyDetectedLeft && polyDetectedCenter && polyDetectedRight)) {
        return;
    }

    if (polyDetectedRight && !polyDetectedCenter) {
        isPolyMovedCenter = true;
        //shiftPolynomial(polyRight, movedPolyCenter, -laneWidth);
        shiftPolynomial(polyRight, movedPolyCenter, -.8f);

        if (!polyDetectedLeft) {
            isPolyMovedLeft = true;
            //shiftPolynomial(polyRight, movedPolyLeft, -2 * laneWidth);
            shiftPolynomial(polyRight, movedPolyLeft, -1.4f);
        }
    } else if (polyDetectedLeft && !polyDetectedCenter) {
        isPolyMovedCenter = true;
        //shiftPolynomial(polyLeft, movedPolyCenter, laneWidth);
        shiftPolynomial(polyLeft, movedPolyCenter, 0.6f);

        if (!polyDetectedRight) {
            isPolyMovedRight = true;
            //shiftPolynomial(polyLeft, movedPolyRight, 2 * laneWidth);
            shiftPolynomial(polyLeft, movedPolyRight, 1.4f);
        }
    } else if (polyDetectedCenter) {
        if (!polyDetectedLeft) {
            isPolyMovedLeft = true;
            //shiftPolynomial(polyCenter, movedPolyLeft, -laneWidth);
            shiftPolynomial(polyCenter, movedPolyLeft, -0.6);
        }
        if (!polyDetectedRight) {
            isPolyMovedRight = true;
            //shiftPolynomial(polyCenter, movedPolyRight, laneWidth);
            shiftPolynomial(polyCenter, movedPolyRight, .8f);
        }
    }
}

/**
  * Calculates the angle the car should drive to. Uses a point of the right (shifted) polynomial
  * at distance of angleAdjacentLeg away from the car and shifts it to the center between the right
  * and center lane in direction of the normal vector. The angle between the shifted point and the
  * car is published.
  */
void cLaneDetectionFu::pubAngle() {
    double dstMiddleCenter;
    double dstMiddleRight;
    double diff;
    double oppositeLeg;
    int y = projImageH - angleAdjacentLeg;

    /*
     * check if there were polynomials detected
     * if so, calculate the distances from the middle of the image to the center and right polynomials
     * else check if there were default lines detected
     * if so, calculate the distances from the middle of the image to the center and right polynomials
     * else abort
     */
    if ((polyDetectedCenter || isPolyMovedCenter) && (polyDetectedRight || isPolyMovedRight)) {
        gradientForAngle = polyDetectedRight ? gradient(y, polyRight.getInterpolationPointY(0), polyRight.getInterpolationPointY(1), polyRight.getCoefficients())
                                             : gradient(y, movedPolyRight.getInterpolationPointY(0), movedPolyRight.getInterpolationPointY(1), movedPolyRight.getCoefficients());

        dstMiddleCenter = abs(polyDetectedCenter ? projImageWHalf - polyCenter.at(y)
                                                 : projImageWHalf - movedPolyCenter.at(y));
        dstMiddleRight = abs(polyDetectedRight ? projImageWHalf - polyRight.at(y)
                                               : projImageWHalf - movedPolyRight.at(y));
    } else if (defaultXCenter.isInitialized() && defaultXRight.isInitialized()){
        dstMiddleCenter = abs(projImageWHalf - defaultXRight.atY(y));
        dstMiddleRight = abs(projImageWHalf - defaultXCenter.atY(y));
    } else {
        return;
    }

    /*
     * calculate the drift
     */
    diff = abs(dstMiddleRight - dstMiddleCenter);
    // drifting towards the left
    if (dstMiddleCenter < dstMiddleRight) {
        oppositeLeg = diff;
    }
    // drifting towards the right
    if (dstMiddleCenter > dstMiddleRight) {
        oppositeLeg = -diff;
    }

    /*
     * calculate steering angle
     */
    double adjacentLeg = y;
    double result = atan(oppositeLeg / adjacentLeg) * 180 / PI;

    /*
     * debug points
     *
     * Note: the oppositeLeg is calculated at the origin, remember to offset it properly by the car's position relative to the image when painting in the debug window
     */
    movedPointForAngle.setX(projImageWHalf + oppositeLeg);
    movedPointForAngle.setY(y);
    pointForAngle.setX(projImageWHalf);
    pointForAngle.setY(y);

    /*
     * filter too rash steering angles / jitter in polynomial data
     */
    ROS_ERROR("%f %f %d", result, lastAngle, maxAngleDiff);
    if (abs(result - lastAngle) > maxAngleDiff) {
        if (result - lastAngle > 0)
            result = lastAngle + maxAngleDiff;
        else
            result = lastAngle - maxAngleDiff;
    }

    lastAngle = result;

    std_msgs::Float32 angleMsg;

    angleMsg.data = result;

    publishAngle.publish(angleMsg);

}

#if 0
void cLaneDetectionFu::pubAngle() {
    /*
     * we didn't detect anything
     */
    if (!upperLaneWidthUpdated || (!polyDetectedRight && !isPolyMovedRight)) {
        return;
    }

    /*
     * 
     */
    int y = projImageH - angleAdjacentLeg;
    double xRightLane;
    if (polyDetectedRight) {
        xRightLane = polyRight.at(y);
        gradientForAngle = gradient(y, polyRight.getInterpolationPointY(0), polyRight.getInterpolationPointY(1), polyRight.getCoefficients());
    } else {
        xRightLane = movedPolyRight.at(y);
        gradientForAngle = gradient(y, movedPolyRight.getInterpolationPointY(0), movedPolyRight.getInterpolationPointY(1), movedPolyRight.getCoefficients());
    }

    /*
     * find the mid of the lane
     */
    double xRight = polyDetectedRight ? polyRight.at(y) : polyCenter.at(y);
    double xCenter = polyCenter.at(y);

    // if no polynomials were detected use previous values
    if (!polyDetectedCenter || !polyDetectedRight || laneMiddle <= 0) {
        //double offset = -1 * laneWidth / 2;
        shiftPoint(movedPointForAngle, -0.5f, (int) xRightLane, y);
    // if the center and right polynomial were detected calculate and store lane mid
    } else if (polyDetectedCenter && polyDetectedRight) {
        laneMiddle = (xRight - xCenter) / 2;

/* TODO: in the first case we shift movedPointForAngle, here we only set new values - why? */
        movedPointForAngle.setX(xCenter + laneMiddle);
        movedPointForAngle.setY(y);
    // otherwise use stored value
    } else {
        movedPointForAngle.setX(polyDetectedRight ? xRight - laneMiddle : xRight + laneMiddle);
        movedPointForAngle.setY(y);
    }

    pointForAngle.setX(xRightLane);
    pointForAngle.setY(y);

    /*
     * 
     */
    int centerXMovedCenter = defaultXCenter.atY(movedPointForAngle.getY());
    int centerXMovedRight = defaultXRight.atX(movedPointForAngle.getY());

    if (centerXMovedCenter == -1 || centerXMovedRight == -1) {
        return;
    }

    /*
     * calculate steering angle
     */
    double oppositeLeg = movedPointForAngle.getX() - (centerXMovedCenter + (centerXMovedRight - centerXMovedCenter) / 2);
    double adjacentLeg = projImageH - movedPointForAngle.getY();
    double result = atan(oppositeLeg / adjacentLeg) * 180 / PI;

    /*
     * filter too rash steering angles / jitter in polynomial data
     */
    if (abs(result - lastAngle) > maxAngleDiff) {
        if (result - lastAngle > 0)
            result = lastAngle + maxAngleDiff;
        else
            result = lastAngle - maxAngleDiff;
    }

    lastAngle = result;

    std_msgs::Float32 angleMsg;

    angleMsg.data = result;

    publishAngle.publish(angleMsg);
}
#endif


/*
 *      Utils:
 */

/**
 * Shifts a point by a given offset along the given gradient.
 * The sign of the offset dictates the direction of the shift relative to the offset
 *
 * @param p the shifted point
 * @param m the gradient of the polynomial at x
 * @param offset negative if shifting to the left, positive to the right
 * @param x the x coordinate of the original point
 * @param y the y coordinate of the original point
 */
void cLaneDetectionFu::shiftPoint(FuPoint<double> &p, double m, double offset, int x, int y) {
    /*
     * Depending on the sign of the gradient of the poly at the different
     * x-values and depending on which position we are, we have to add or
     * subtract the expected distance to the respective lane polynomial, to get
     * the wanted points.
     *
     * The calculation is done for the x- and y-components of the points
     * separately using the trigonometric ratios of right triangles and the fact
     * that arctan of some gradient equals its angle to the x-axis in degree.
     */
    if (m == 0) {
        p.setX(x - offset);
        p.setY(y - offset);
    } else if (m > 0) {
        p.setX(x - offset * sin(atan(-1 / m)));
        p.setY(y - offset * cos(atan(-1 / m)));
    } else {
        p.setX(x + offset * sin(atan(-1 / m)));
        p.setY(y + offset * cos(atan(-1 / m)));
    }
}

/*void cLaneDetectionFu::shiftPoint(FuPoint<double> &p, double offset, int x, int y) {
    double viewLaneWidthOffset = laneWidth - upperLaneWidth;
    double horizontalShiftLength = upperLaneWidth + viewLaneWidthOffset * (y / (double) projImageH);

    p.setY(y);
    p.setX(x + offset * horizontalShiftLength);

    //ROS_INFO("laneWidth: %f, upperLaneWidth: %f, viewLaneWidthOffset: %f, horizontalShiftLength: %f, x: %f", laneWidth, upperLaneWidth, viewLaneWidthOffset, horizontalShiftLength, x + offset * horizontalShiftLength);
}*/

/**
 * Shifts a polynomial by a given offset
 * The sign of the offset dictates the direction of the shift relative to the offset
 *
 * @param f the original polynomial
 * @param g the shifted polynomial
 * @param offset negative if shifting to the left, positive to the right
 */
void cLaneDetectionFu::shiftPolynomial(NewtonPolynomial &f, NewtonPolynomial &g, double offset) {
    /*FuPoint<double> shiftedPoint;
    double m;

    for (int i = 0; i < 3; i++) {
        m = gradient(f.getInterpolationPointX(i), f.getInterpolationPointY(0), f.getInterpolationPointY(1), f.getCoefficients());
        shiftPoint(shiftedPoint, m, offset, f.getInterpolationPointX(i), f.getInterpolationPointY(i));
        g.addData(shiftedPoint);
    }*/


    //TODO: could this be an issue?
    if (!upperLaneWidthUpdated) {
        return;
    }

    FuPoint<double> shiftedPoint;
    for (int i = 0; i < 3; i++) {
        double y = f.getInterpolationPointY(i);
        double m = gradient(y, f);

        ROS_INFO("m = %f, m2 = %f, mFinal = %f", m, m2, (m - m2));

        m = m - m2;

        double viewLaneWidthOffset = laneWidth - upperLaneWidth;
        double horizontalShiftLength = upperLaneWidth + viewLaneWidthOffset * (y / (double) projImageH);

        shiftPoint(shiftedPoint, m, offset * horizontalShiftLength, f.getInterpolationPointX(i), y);

        double heightDiff = abs(y - shiftedPoint.getY());
        if (heightDiff > 4) {
            double yLaneWidth = min(y, shiftedPoint.getY()) + heightDiff / 2;
            double horizontalShiftLength = upperLaneWidth + viewLaneWidthOffset * (yLaneWidth / (double) projImageH);
            shiftPoint(shiftedPoint, m, offset * horizontalShiftLength, f.getInterpolationPointX(i), y);
        }
        g.addData(shiftedPoint);

        ROS_INFO("Shift poly, %d, shifted point: (%f, %f)", i, shiftedPoint.getY(), shiftedPoint.getX());

        if (i == 0) {
            p1Draw.setX(f.getInterpolationPointX(i));
            p1Draw.setY(f.getInterpolationPointY(i));
            p1DrawShifted.setX(shiftedPoint.getX());
            p1DrawShifted.setY(shiftedPoint.getY());
        } else if (i == 1) {
            p2Draw.setX(f.getInterpolationPointX(i));
            p2Draw.setY(f.getInterpolationPointY(i));
            p2DrawShifted.setX(shiftedPoint.getX());
            p2DrawShifted.setY(shiftedPoint.getY());
        } else {
            p3Draw.setX(f.getInterpolationPointX(i));
            p3Draw.setY(f.getInterpolationPointY(i));
            p3DrawShifted.setX(shiftedPoint.getX());
            p3DrawShifted.setY(shiftedPoint.getY());
        }
    }
}

/**
 * Checks if edge point p is near to lane marking point lanePoint. Enables proper
 * assignment of edge points to lanes in first frames when the car starts if car
 * starts in front of a turn.
 *
 * @param lanePoint Point of a lane
 * @param p Point to check
 * @return True if p is near to lanePoint
 */
bool cLaneDetectionFu::isInRangeAndImage(FuPoint<int> &lanePoint, FuPoint<int> &p) {
    if (p.getY() < minYDefaultRoi || p.getY() > maxYRoi) {
        return false;
    }
    if (p.getY() > lanePoint.getY()) {
        return false;
    }

    return isInRange(lanePoint, p);
}

bool cLaneDetectionFu::isInRange(FuPoint<int> &lanePoint, FuPoint<int> &p) {
    double distanceX = abs(p.getX() - lanePoint.getX());
    double distanceY = abs(p.getY() - lanePoint.getY());

    return ((distanceX < interestDistancePoly) && (distanceY < interestDistancePoly));
}


/**
 * Calculates the horizontal distance between a point and the default line given
 * by its position.
 *
 * @param line  The position of the default line (LEFT, CENTER or RIGHT)
 * @param p     The given point
 * @return      The horizontal distance between default line and point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistanceToDefaultLine(ePosition &line, FuPoint<int> &p) {

    if (LEFT == line) {
        return defaultXLeft.horizDistance(p);
    }
    if (CENTER == line) {
        return defaultXCenter.horizDistance(p);
    }
    if (RIGHT == line) {
        return defaultXRight.horizDistance(p);
    }
    return 0;
}

/**
 * Calculates the horizontal distance between a point and a polynomial.
 *
 * @param poly  The given polynomial
 * @param p     The given point
 * @return      The horizontal distance between the polynomial and the point, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistanceToPolynomial(NewtonPolynomial &poly, FuPoint<int> &p) {
    double pY = p.getY();
    double pX = p.getX();

    double polyX = poly.at(pY);
    double distance = abs(pX - polyX);

    return distance;
}

/**
 * Method, that checks if a point lies within the default ROI of a position.
 *
 * @param position  The position of the default ROI
 * @param p         The given point, which is checked
 * @return          True, if the point lies within the default ROI
 */
bool cLaneDetectionFu::isInDefaultRoi(ePosition position, FuPoint<int> &p) {
    if (p.getY() < minYDefaultRoi || p.getY() > maxYRoi) {
        return false;
    } else {
        int distance = horizDistanceToDefaultLine(position, p);

        ROS_INFO("                      isInDefaultRoi: distance = %d", distance);

        return distance >= 0 && distance <= interestDistanceDefault;
    }
}

/**
 * Method, that checks if a point lies within the the ROI of a polynomial.
 *
 * @param poly      The polynomial, whose ROI is used
 * @param p         The point, which is checked
 * @return          True, if the point lies within the polynomial's ROI
 */
bool cLaneDetectionFu::isInPolyRoi(NewtonPolynomial &poly, FuPoint<int> &p) {
    if (p.getY() < minYPolyRoi || p.getY() > maxYRoi) {
        return false;
    } else if (horizDistanceToPolynomial(poly, p) <= interestDistancePoly) {
        return true;
    } else {
        return false;
    }
}

/**
 * Calculates the horizontal distance between two points.
 *
 * @param p1    The first point
 * @param p2    The second point
 * @return      The horizontal distance between the two points, horizontal distance = difference in X!!!
 */
int cLaneDetectionFu::horizDistance(FuPoint<int> &p1, FuPoint<int> &p2) {
    double x1 = p1.getX();
    double x2 = p2.getX();

    return abs(x1 - x2);
}

/**
 * Calculates the gradient of a polynomial at a given x value. The used formula
 * was obtained by the following steps:
 * - start with the polynomial of 2nd degree in newton basis form:
 *   p(x) = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 * - expand the equation and sort it by descending powers of x
 * - form the first derivative
 *
 * Applying the given x value then results in the wanted gradient.
 *
 * @param x         The given x value
 * @param interpolationPoint0Y    The first data point used for interpolating the polynomial
 * @param interpolationPoint1Y    The second data point used for interpolating the polynomial
 * @param coeffs    The coefficients under usage of the newton basis
 * @return          The gradient of the polynomial at x
 */
double cLaneDetectionFu::gradient(double x, double interpolationPoint0Y, double interpolationPoint1Y, vector<double> coeffs) {
    return (2 * coeffs[2] * x + coeffs[1])
           - (coeffs[2] * interpolationPoint1Y)
           - (coeffs[2] * interpolationPoint0Y);
}

double cLaneDetectionFu::gradient(double x, NewtonPolynomial poly) {
    return gradient(x, poly.getInterpolationPointY(0), poly.getInterpolationPointY(1), poly.getCoefficients());
}

/**
 * Method, that checks, if a polynomial produced during RANSAC counts as usable.
 *
 * @param position  The position of the polynomial, that is checked
 * @param poly      The polynomial, that is checked
 * @param prevPoly  The previous polynomial detected at this position
 * @return          True, if the polynomial counts as valid
 */
bool cLaneDetectionFu::polyValid(ePosition position, NewtonPolynomial poly, NewtonPolynomial prevPoly) {

    if (prevPoly.getDegree() != -1) {
        return isSimilar(poly, prevPoly);
    }
    if (position == RIGHT) {
        if (polyDetectedRight) {
            return isSimilar(poly, polyRight);
        }
        if (isPolyMovedRight) {
            return isSimilar(poly, movedPolyRight);
        }
    }
    if (position == CENTER) {
        if (polyDetectedCenter) {
            return isSimilar(poly, polyCenter);
        }
        if (isPolyMovedCenter) {
            return isSimilar(poly, movedPolyCenter);
        }
    }
    if (position == LEFT) {
        if (polyDetectedLeft) {
            return isSimilar(poly, polyLeft);
        }
        if (isPolyMovedLeft) {
            return isSimilar(poly, movedPolyLeft);
        }
    }

    return true;
}

/**
 * Checks if two polynomials are similar and do not vary too much from each other.
 */
bool cLaneDetectionFu::isSimilar(const NewtonPolynomial &poly1, const NewtonPolynomial &poly2) {
    /*FuPoint<int> p1 = FuPoint<int>(poly1.at(polyY1), polyY1);
    FuPoint<int> p2 = FuPoint<int>(poly2.at(polyY1), polyY1);

    if (horizDistance(p1, p2) > interestDistancePoly) { //0.05 * meters
        return false;
    }

    FuPoint<int> p3 = FuPoint<int>(poly1.at(polyY2), polyY2);
    FuPoint<int> p4 = FuPoint<int>(poly2.at(polyY2), polyY2);

    if (horizDistance(p3, p4) > interestDistancePoly) { //0.05 * meters
        return false;
    }

    FuPoint<int> p5 = FuPoint<int>(poly1.at(polyY3), polyY3);
    FuPoint<int> p6 = FuPoint<int>(poly2.at(polyY3), polyY3);

    if (horizDistance(p5, p6) > interestDistancePoly) { //0.05 * meters
        return false;
    }*/

    if (largeDistance(poly1, poly2, polyY1)) {
        return false;
    }

    if (largeDistance(poly1, poly2, polyY2)) {
        return false;
    }

    if (largeDistance(poly1, poly2, polyY3)) {
        return false;
    }

    return true;
}

bool cLaneDetectionFu::largeDistance(const NewtonPolynomial &poly1, const NewtonPolynomial &poly2, int y) {
    FuPoint<int> p1 = FuPoint<int>((int) poly1.at(y), y);
    FuPoint<int> p1Changed = FuPoint<int>(y, (int) poly1.at(y));

    double m = gradient(y, poly1.getInterpolationPointY(0), poly1.getInterpolationPointY(1), poly1.getCoefficients());
    //m = -1 / m;

    double yIntersection = intersection(p1Changed, m, poly2.getInterpolationPointY(0), poly2.getInterpolationPointY(1), poly2.getCoefficients());

    double xIntersection = poly2.at(y);
    FuPoint<int> p2 = FuPoint<int>((int) xIntersection, (int) yIntersection);


    int distanceX = abs(p1.getX() - p2.getX());
    int distanceY = abs(p1.getY() - p2.getY());
    ROS_INFO("           p1: (%d, %d), p2: (%d, %d), distance: (%d, %d), gradient: %f", p1.getX(), p1.getY(), p2.getX(), p2.getY(), distanceX, distanceY, m);


    return !isInRange(p1, p2);
}

/*
 *      debug functions
 */


void cLaneDetectionFu::drawEdgeWindow(Mat &img, vector<vector<EdgePoint>> edges) {

    Mat transformedImagePaintable = img.clone();
    cvtColor(transformedImagePaintable, transformedImagePaintable, CV_GRAY2BGR);
    for (int i = 0; i < (int) edges.size(); i++) {
        for (int j = 0; j < edges[i].size(); j++) {
            FuPoint<int> edge = edges[i][j].getImgPos();
            Point edgeLoc = Point(edge.getX(), edge.getY());
            circle(transformedImagePaintable, edgeLoc, 1, Scalar(0, 255, 255), -1);//edges[i][j].getValue()), -1);
        }
    }

    cv::namedWindow("ROI, scanlines and edges", WINDOW_NORMAL);
    cv::imshow("ROI, scanlines and edges", transformedImagePaintable);
    cv::waitKey(1);
}

void cLaneDetectionFu::drawLaneMarkingsWindow(Mat &img, vector<FuPoint<int>> &laneMarkings) {

    Mat transformedImagePaintable = img.clone();
    cv::cvtColor(transformedImagePaintable, transformedImagePaintable, CV_GRAY2BGR);
    for (int i = 0; i < (int) laneMarkings.size(); i++) {
        FuPoint<int> marking = laneMarkings[i];
        cv::Point markingLoc = cv::Point(marking.getX(), marking.getY());
        cv::circle(transformedImagePaintable, markingLoc, 1, cv::Scalar(0, 255, 0), -1);
    }

    cv::namedWindow("Lane Markings", WINDOW_NORMAL);
    cv::imshow("Lane Markings", transformedImagePaintable);
    cv::waitKey(1);
}

void cLaneDetectionFu::drawGroupedLaneMarkingsWindow(Mat &img) {

    Mat transformedImagePaintable = img.clone();
    cv::cvtColor(transformedImagePaintable, transformedImagePaintable, CV_GRAY2BGR);

    debugPaintPoints(transformedImagePaintable, Scalar(0, 0, 255), laneMarkingsLeft);
    debugPaintPoints(transformedImagePaintable, Scalar(0, 255, 0), laneMarkingsCenter);
    debugPaintPoints(transformedImagePaintable, Scalar(255, 0, 0), laneMarkingsRight);
    debugPaintPoints(transformedImagePaintable, Scalar(0, 255, 255), laneMarkingsNotUsed);

    if (defaultXLeft.isInitialized()) {
        cv::Point2d p1l(defaultXLeft.getStart().getX(), defaultXLeft.getStart().getY());
        cv::Point2d p2l(defaultXLeft.getEnd().getX(), defaultXLeft.getEnd().getY());
        cv::line(transformedImagePaintable, p1l, p2l, cv::Scalar(0, 0, 255));
    }

    if (defaultXCenter.isInitialized()) {
        cv::Point2d p1c(defaultXCenter.getStart().getX(), defaultXCenter.getStart().getY());
        cv::Point2d p2c(defaultXCenter.getEnd().getX(), defaultXCenter.getEnd().getY());
        cv::line(transformedImagePaintable, p1c, p2c, cv::Scalar(0, 255, 0));
    }

    if (defaultXRight.isInitialized()) {
        cv::Point2d p1r(defaultXRight.getStart().getX(), defaultXRight.getStart().getY());
        cv::Point2d p2r(defaultXRight.getEnd().getX(), defaultXRight.getEnd().getY());
        cv::line(transformedImagePaintable, p1r, p2r, cv::Scalar(255, 0, 0));
    }

    if (m2 > 0) {
        for (int y = 0; y < projImageH; y++) {
            int x1 = (int) ((y - n1) / m1);
            int x2 = (int) ((y - n2) / m2);

            ROS_INFO("x1: %d, x2: %d", x1, x2);

            cv::Point markingLoc = cv::Point(x1, y);
            cv::circle(transformedImagePaintable, markingLoc, 1, cv::Scalar(255, 255, 0), -1);

            cv::Point markingLoc2 = cv::Point(x2, y);
            cv::circle(transformedImagePaintable, markingLoc2, 1, cv::Scalar(0, 0, 255), -1);
        }
    }

    cv::Point2d p1(projImageWHalf - (roiBottomW / 2), maxYRoi - 1);
    cv::Point2d p2(projImageWHalf + (roiBottomW / 2), maxYRoi - 1);
    cv::Point2d p3(projImageWHalf + (roiTopW / 2), minYPolyRoi);
    cv::Point2d p4(projImageWHalf - (roiTopW / 2), minYPolyRoi);
    cv::line(transformedImagePaintable, p1, p2, cv::Scalar(0, 200, 0));
    cv::line(transformedImagePaintable, p2, p3, cv::Scalar(0, 200, 0));
    cv::line(transformedImagePaintable, p3, p4, cv::Scalar(0, 200, 0));
    cv::line(transformedImagePaintable, p4, p1, cv::Scalar(0, 200, 0));

#ifdef PUBLISH_IMAGES
    pubRGBImageMsg(transformedImagePaintable, imagePublisherLaneMarkings);
#endif

#ifdef SHOW_GROUPED_LANE_MARKINGS_WINDOW
    cv::namedWindow("Grouped Lane Markings", WINDOW_NORMAL);
    cv::imshow("Grouped Lane Markings", transformedImagePaintable);
    cv::waitKey(1);
#endif

#ifdef SAVE_FRAME_IMAGES
    debugWriteImg(transformedImagePaintable, "groupedLaneMarkings");
#endif

}

void cLaneDetectionFu::drawPolyRangeWindow(cv::Mat &img) {
    cv::Mat transformedImagePaintablePolyRange = img.clone();
    cv::cvtColor(transformedImagePaintablePolyRange, transformedImagePaintablePolyRange, CV_GRAY2BGR);

    // left poly = red, middle poly = green, right poly = blue
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(0, 0, 255), polyLeft, minYPolyRoi, maxYRoi, -interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(0, 0, 255), polyLeft, minYPolyRoi, maxYRoi, interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(0, 255, 0), polyCenter, minYPolyRoi, maxYRoi, -interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(0, 255, 0), polyCenter, minYPolyRoi, maxYRoi, interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(255, 0, 0), polyRight, minYPolyRoi, maxYRoi, -interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(255, 0, 0), polyRight, minYPolyRoi, maxYRoi, interestDistancePoly);

    // moved left poly = purple, moved middle poly = black, moved right poly = orange
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(139, 0, 139), movedPolyLeft, minYPolyRoi, maxYRoi, -interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(139, 0, 139), movedPolyLeft, minYPolyRoi, maxYRoi, interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(0, 0, 0), movedPolyCenter, minYPolyRoi, maxYRoi, -interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(0, 0, 0), movedPolyCenter, minYPolyRoi, maxYRoi, interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(51, 153, 255), movedPolyRight, minYPolyRoi, maxYRoi, -interestDistancePoly);
    debugPaintPolynom(transformedImagePaintablePolyRange, cv::Scalar(51, 153, 255), movedPolyRight, minYPolyRoi, maxYRoi, interestDistancePoly);

    debugPaintPoints(transformedImagePaintablePolyRange, Scalar(0, 0, 255), laneMarkingsLeft);
    debugPaintPoints(transformedImagePaintablePolyRange, Scalar(0, 255, 0), laneMarkingsCenter);
    debugPaintPoints(transformedImagePaintablePolyRange, Scalar(255, 0, 0), laneMarkingsRight);
    debugPaintPoints(transformedImagePaintablePolyRange, Scalar(0, 255, 255), laneMarkingsNotUsed);

#ifdef SHOW_GROUPED_LANE_MARKINGS_WINDOW
    cv::namedWindow("Poly range", WINDOW_NORMAL);
    cv::imshow("Poly range", transformedImagePaintablePolyRange);
    cv::waitKey(1);
#endif

#ifdef SAVE_FRAME_IMAGES
    debugWriteImg(transformedImagePaintablePolyRange, "polyRange");
#endif
}

void cLaneDetectionFu::drawRansacWindow(cv::Mat &img) {

    cv::Mat transformedImagePaintableRansac = img.clone();
    cv::cvtColor(transformedImagePaintableRansac, transformedImagePaintableRansac, CV_GRAY2BGR);

    // left poly = red, middle poly = green, right poly = blue
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(0, 0, 255), polyLeft, minYPolyRoi, maxYRoi, 0);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(0, 255, 0), polyCenter, minYPolyRoi, maxYRoi, 0);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(255, 0, 0), polyRight, minYPolyRoi, maxYRoi, 0);

    // moved left poly = purple, moved middle poly = black, moved right poly = orange
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(139, 0, 139), movedPolyLeft, minYPolyRoi, maxYRoi, 0);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(0, 0, 0), movedPolyCenter, minYPolyRoi, maxYRoi, 0);
    debugPaintPolynom(transformedImagePaintableRansac, cv::Scalar(51, 153, 255), movedPolyRight, minYPolyRoi, maxYRoi, 0);

    cv::Point start = cv::Point(p1Draw.getX(), p1Draw.getY());
    cv::Point end = cv::Point(p1DrawShifted.getX(), p1DrawShifted.getY());
    cv::line(transformedImagePaintableRansac, start, end, cv::Scalar(255, 255, 0));

    start = cv::Point(p2Draw.getX(), p2Draw.getY());
    end = cv::Point(p2DrawShifted.getX(), p2DrawShifted.getY());
    cv::line(transformedImagePaintableRansac, start, end, cv::Scalar(255, 255, 0));

    start = cv::Point(p3Draw.getX(), p3Draw.getY());
    end = cv::Point(p3DrawShifted.getX(), p3DrawShifted.getY());
    cv::line(transformedImagePaintableRansac, start, end, cv::Scalar(255, 255, 0));

    // laneWidth
    start = cv::Point(defaultXCenter.atY(projImageH - 5), projImageH - 5);
    end = cv::Point(defaultXCenter.atY(projImageH - 5) + laneWidth, projImageH - 5);
    cv::line(transformedImagePaintableRansac, start, end, cv::Scalar(255, 0, 255));

    // upperLaneWidth
    start = cv::Point(defaultXCenter.atY(5), 5);
    end = cv::Point(defaultXCenter.atY(5) + upperLaneWidth, 5);
    cv::line(transformedImagePaintableRansac, start, end, cv::Scalar(255, 0, 255));

#ifdef PUBLISH_IMAGES
    pubRGBImageMsg(transformedImagePaintableRansac, imagePublisherRansac);
#endif

#ifdef SHOW_RANSAC_WINDOW
    cv::namedWindow("RANSAC results", WINDOW_NORMAL);
    cv::imshow("RANSAC results", transformedImagePaintableRansac);
    cv::waitKey(1);
#endif

#ifdef SAVE_FRAME_IMAGES
        debugWriteImg(transformedImagePaintableRansac, "ransac");
#endif
}

void cLaneDetectionFu::drawAngleWindow(Mat &img) {
    frame++;

    Mat transformedImagePaintableLaneModel = img.clone();
    cvtColor(transformedImagePaintableLaneModel, transformedImagePaintableLaneModel, CV_GRAY2BGR);

    if (polyDetectedRight || isPolyMovedRight) {
        //double carPositionX = defaultXCenter.getEnd().getX() + (defaultXRight.getEnd().getX() - defaultXCenter.getEnd().getX()) / 2;
        double carPositionX = pointForAngle.getX();

        cv::Point pointLoc = cv::Point(carPositionX, projImageH);
        cv::circle(transformedImagePaintableLaneModel, pointLoc, 2, cv::Scalar(0, 0, 255), -1);

        cv::Point anglePointLoc = cv::Point(sin(lastAngle * PI / 180) * angleAdjacentLeg + carPositionX,
                                            projImageH - (cos(lastAngle * PI / 180) * angleAdjacentLeg));
        cv::line(transformedImagePaintableLaneModel, pointLoc, anglePointLoc, cv::Scalar(255, 255, 255));

        cv::Point startNormalPoint = cv::Point(pointForAngle.getX(), pointForAngle.getY());
        cv::circle(transformedImagePaintableLaneModel, startNormalPoint, 2, cv::Scalar(100, 100, 100), -1);

        cv::Point targetPoint = cv::Point(movedPointForAngle.getX(), movedPointForAngle.getY());
        cv::circle(transformedImagePaintableLaneModel, targetPoint, 2, cv::Scalar(0, 0, 255), -1);

        // TODO: what was this for?
        double m = -gradientForAngle;

        double n = movedPointForAngle.getY() - m * movedPointForAngle.getX();
        double x = 10;
        double y = m * x + n;
        //

        cv::Point endNormalPoint = cv::Point(movedPointForAngle.getX(), movedPointForAngle.getY());
        cv::line(transformedImagePaintableLaneModel, startNormalPoint, endNormalPoint, cv::Scalar(0, 0, 0));

    } else {
        cv::Point pointLoc = cv::Point(5, 5);
        cv::circle(transformedImagePaintableLaneModel, pointLoc, 3, cv::Scalar(0, 0, 255), 0);
    }

#ifdef PUBLISH_IMAGES
    pubRGBImageMsg(transformedImagePaintableLaneModel, imagePublisher);
#endif

#ifdef SHOW_ANGLE_WINDOW
    cv::namedWindow("Lane polynomial", WINDOW_NORMAL);
    cv::imshow("Lane polynomial", transformedImagePaintableLaneModel);
    cv::waitKey(1);
#endif
}

void cLaneDetectionFu::pubRGBImageMsg(cv::Mat &rgb_mat, image_transport::CameraPublisher publisher) {
    sensor_msgs::ImagePtr rgb_img(new sensor_msgs::Image);

    rgb_img->header.seq = headSequenceId;
    rgb_img->header.stamp = headTimeStamp;
    rgb_img->header.frame_id = rgbFrameId;

    rgb_img->width = rgb_mat.cols;
    rgb_img->height = rgb_mat.rows;

    rgb_img->encoding = sensor_msgs::image_encodings::BGR8;
    rgb_img->is_bigendian = 0;

    int step = sizeof(unsigned char) * 3 * rgb_img->width;
    int size = step * rgb_img->height;
    rgb_img->step = step;
    rgb_img->data.resize(size);
    memcpy(&(rgb_img->data[0]), rgb_mat.data, size);

    rgbCameraInfo->header.frame_id = rgbFrameId;
    rgbCameraInfo->header.stamp = headTimeStamp;
    rgbCameraInfo->header.seq = headSequenceId;

    publisher.publish(rgb_img, rgbCameraInfo);
}

void cLaneDetectionFu::debugPaintPolynom(cv::Mat &m, cv::Scalar color, NewtonPolynomial &p, int start, int end, int offset) {
    cv::Point point;
    for (int i = start; i < end; i++) {
        point = cv::Point(p.at(i) + offset, i);
        cv::circle(m, point, 0, color, -1);
    }
}

void cLaneDetectionFu::debugPaintPoints(cv::Mat &m, cv::Scalar color, vector<FuPoint<int>> &points) {
    for (FuPoint<int> point : points) {
        cv::Point pointLoc = cv::Point(point.getX(), point.getY());
        cv::circle(m, pointLoc, 1, color, -1);
    }
}

void cLaneDetectionFu::debugWriteImg(cv::Mat &image, string folder) {
    stringstream img;
    img << "./" << folder << "/" << frame << ".jpg";
    cv::imwrite(img.str(), image);
}



/*
 *      ROS:
 */



void cLaneDetectionFu::config_callback(line_detection_fu::LaneDetectionConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure Request");

    interestDistancePoly = config.interestDistancePoly;
    interestDistanceDefault = config.interestDistanceDefault;
    iterationsRansac = config.iterationsRansac;
    maxYRoi = config.maxYRoi;
    minYDefaultRoi = config.minYDefaultRoi;
    minYPolyRoi = config.minYPolyRoi;
    polyY1 = config.polyY1;
    polyY2 = config.polyY2;
    polyY3 = config.polyY3;
    maxAngleDiff = config.maxAngleDiff;
    projYStart = config.projYStart;
    roiTopW = config.roiTopW;
    roiBottomW = config.roiBottomW;
    proportionThreshould = config.proportionThreshould;
    gradientThreshold = config.gradientThreshold;
    nonMaxWidth = config.nonMaxWidth;
    laneMarkingSquaredThreshold = config.laneMarkingSquaredThreshold;
    angleAdjacentLeg = config.angleAdjacentLeg;
    scanlinesVerticalDistance = config.scanlinesVerticalDistance;
    scanlinesMaxCount = config.scanlinesMaxCount;

    /*defaultXLeft = Line<int>(
        FuPoint<int>(config.minYRoiLeft, minYPolyRoi),
        FuPoint<int>(config.maxYRoiLeft, maxYRoi)
    );
    defaultXCenter = Line<int>(
        FuPoint<int>(config.minYRoiCenter, minYPolyRoi),
        FuPoint<int>(config.maxYRoiCenter, maxYRoi)
    );
    defaultXRight = Line<int>(
        FuPoint<int>(config.minYRoiRight, minYPolyRoi),
        FuPoint<int>(config.maxYRoiRight, maxYRoi)
    );*/

/*
    defaultXLeft.getStart().setX(config.minYRoiLeft);
    defaultXLeft.getEnd().setX(config.maxYRoiLeft);

    defaultXCenter.getStart().setX(config.minYRoiCenter);
    defaultXCenter.getEnd().setX(config.maxYRoiCenter);

    defaultXRight.getStart().setX(config.minYRoiRight);
    defaultXRight.getEnd().setX(config.maxYRoiRight);
*/
    scanlines = getScanlines();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "cLaneDetectionFu");
    ros::NodeHandle nh;

    cLaneDetectionFu node = cLaneDetectionFu(nh);

    dynamic_reconfigure::Server<line_detection_fu::LaneDetectionConfig> server;
    dynamic_reconfigure::Server<line_detection_fu::LaneDetectionConfig>::CallbackType f;
    f = boost::bind(&cLaneDetectionFu::config_callback, &node, _1, _2);
    server.setCallback(f);

    while (ros::ok()) {
        ros::spinOnce();
    }
    return 0;
}



/*
 *      Unused (but may be useful in future)
 */



/**
 * Calculates the x value of the point where the normal of the tangent of a
 * polynomial at a given point p intersects with a second polynomial.
 *
 * The formula for the intersection point is obtained by setting equal the
 * following two formula:
 *
 * 1. the formula of the normal in point-slope-form:
 *     y - p_y = -(1 / m) * (x - p_x) which is the same as
 *           y = -(x / m) + (p_x / m) + p_y
 *
 * 2. the formula of the second polynomial of 2nd degree in newton basis form:
 *           y = c0 + c1(x - x0) + c2(x - x0)(x - x1)
 *
 * Expanding everything and moving it to the right side gives a quadratic
 * equation in the general form of 0 = ax^2 + bx + c, which can be solved using
 * the general quadratic formula x = (-b +- sqrt(b^2 - 4ac)) / 2a
 *
 * The three cases for the discriminant are taken into account.
 *
 * @param p         The point of the first poly at which its tangent is used
 * @param m         The gradient of the tangent
 * @param points    The data points used for interpolating the second polynomial
 * @param coeffs    The coeffs of the second polynomial with newton basis
 * @return          The x value of the intersection point of normal and 2nd poly
 */
/*double cLaneDetectionFu::intersection(FuPoint<double> &p, double &m,
                                      vector<FuPoint<int>> &points, vector<double> &coeffs) {
    double a = coeffs[2];
    double b = coeffs[1] - (coeffs[2] * points[1].getY())
               - (coeffs[2] * points[0].getY()) + (1.0 / m);
    double c = coeffs[0] - (coeffs[1] * points[0].getY())
               + (coeffs[2] * points[0].getY() * points[1].getY())
               - p.getY() - (p.getX() / m);

    double dis = pow(b, 2) - (4 * a * c);
    double x1 = 0;
    double x2 = 0;

    if (dis < 0) return -1;
    if (dis == 0) return -b / (2 * a);

    x1 = (-b + sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);
    x2 = (-b - sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);
    return fmax(x1, x2);
}*/

double cLaneDetectionFu::intersection(FuPoint<int> &p, double &m,
                                      double interpolationPoint0Y, double interpolationPoint1Y, vector<double> coeffs) {
    double a = coeffs[2] + 0.000000001f;
    double b = coeffs[1] - (coeffs[2] * interpolationPoint1Y)
               - (coeffs[2] * interpolationPoint0Y) + (1.0 / m);
    double c = coeffs[0] - (coeffs[1] * interpolationPoint0Y)
               + (coeffs[2] * interpolationPoint0Y * interpolationPoint1Y)
               - p.getY() - (p.getX() / m);

    double dis = pow(b, 2) - (4 * a * c);
    double x1 = 0;
    double x2 = 0;

    if (dis < 0) return -1;
    if (dis == 0) return -b / (2 * a);

    x1 = (-b + sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);
    x2 = (-b - sqrt(pow(b, 2) - (4 * a * c))) / (2 * a);

    double distanceX1 = abs(x1 - p.getX());
    double distanceX2 = abs(x2 - p.getX());

    return distanceX1 < distanceX2 ? x1 : x2;
}

/**
 * Calculates the gradient of a second polynomial at the point, at which the
 * normal of the tangent of the first polynomial at the given point
 * intersects with the second polynomial.
 *
 * @param x         The given x value of the point on the first polynomial
 * @param poly1     The first polynomial
 * @param interpolationPoint0Y    The first data point used for interpolating the second polynomial
 * @param interpolationPoint1Y    The second data point used for interpolating the second polynomial
 * @param coeffs1   The coeffs of the first poly using newton basis
 * @param coeffs2   The coeffs of the second poly using newton basis
 * @param m1        The gradient of the first poly at x
 * @return          The gradient of the second poly at the intersection point
 */
double cLaneDetectionFu::nextGradient(double x, NewtonPolynomial &poly1,
                                      double &interpolationPoint0Y, double &interpolationPoint1Y,
                                      vector<double> coeffs1, vector<double> coeffs2, double m1) {

    FuPoint<int> p = FuPoint<int>(x, poly1.at(x));
    double x2 = intersection(p, m1, interpolationPoint0Y, interpolationPoint1Y, coeffs2);

    return gradient(x2, interpolationPoint0Y, interpolationPoint1Y, coeffs2);
}

/**
 * Check two gradients for similarity. Return true if the difference in degree
 * is less than 10.
 *
 * @param m1    The first gradient
 * @param m2    The second gradient
 * @return      True, if the diffenence between the gradients is less than 10 degrees
 */
bool cLaneDetectionFu::gradientsSimilar(double &m1, double &m2) {
    double a1 = atan(m1) * 180 / PI;
    double a2 = atan(m2) * 180 / PI;

    return (abs(a1 - a2) < 10);
}

/**
 * Finds the position of the polynomial with the highest proportion.
 * @return The position of the best polynomial
 */
ePosition cLaneDetectionFu::maxProportion() {
    double maxVal = bestPolyLeft.second;

    if (bestPolyCenter.second > maxVal) {
        maxVal = bestPolyCenter.second;
        return CENTER;
    }

    if (bestPolyRight.second > maxVal) {
        return RIGHT;
    }

    return LEFT;
}
