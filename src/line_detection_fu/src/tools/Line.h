/*
 * Line.h
 * 
 * A line through two FuPoint's
 */

#ifndef LINE_H_
#define LINE_H_

#include <cstddef>
#include "FuPoint.h"

template<typename T>

/**
 * Line is used to model default lines which help to categorize nearby
 * lane markings to one lane. Start is at the bottom of the image, end
 * at the top.
 */
class Line {
public:

    Line() {}

    Line(T imageHeight) {
        this->imageHeight = imageHeight;
    }

    virtual ~Line() {};

    void setStart(T x) {
        startValueSet = true;
        start = FuPoint<T>(x, imageHeight);
    }

    void setEnd(T x) {
        end = FuPoint<T>(x, 0);

        init();
    }

    void setStartPoint(FuPoint<int> start) {
        this->start.setX(start.getX());
        this->start.setY(start.getY());
    }

    void setEndPoint(FuPoint<int> end) {
        this->end.setX(end.getX());
        this->end.setY(end.getY());

        init();
    }

    FuPoint<T> getStart() {
        return start;
    }

    FuPoint<T> getEnd() {
        return end;
    }

    /*
     * Returns the y value at the given x value
     */
    T atX(T x) {
        if (properties.isHorizontalConstant) {
            return start.getY();
        }
        if (properties.isVerticalConstant) {
            ROS_INFO("Line.h atX(): vertical constant line");
            return -1;
        }
            double t = (double) (x - start.getX()) / (double) (start.getX()-end.getX());
            double r = start.getY() + t * (start.getY() - end.getY());

            ROS_INFO("atX: t = %f, r = %f", t, r);
            return r;
    }

    /*
     * Returns the x value at the given y value
     */
    T atY(T y) {
        if (initialized) {
            if (properties.isHorizontalConstant) {
                ROS_INFO("Line.h atY(): horizontal constant line");
                return -1;
            }
            if (properties.isVerticalConstant) {
                return start.getX();
            }
            double t = (double) (y - start.getY()) / (double) (start.getY()-end.getY());
            double r = start.getX() + t * (start.getX() - end.getX());

            ROS_INFO("atY: t = %f, r = %f", t, r);
            return r;

        }
        if (startValueSet) {
            return start.getX();
        }
        return -1;
    }

    /*
     * Returns the horizontal distance to a given FuPoint, using the FuPoints y coordinate
     */
    T horizDistance(FuPoint<T> &p) {
        if (isInitialized()) {
            if (properties.isHorizontalConstant) {
                // point on line
                if (p.getY() == start.getY()) {
                    return 0;
                }
                ROS_INFO("Line.h horizDistance(): horizontal constant line");
                return -1;
            }

            ROS_INFO("Linie.h: m: %f, n: %f, x: %d, y: %d, atY: %d", m, n, p.getX(), p.getY(), atY(p.getY()));

            return std::abs(atY(p.getY()) - p.getX());
        }
        if (startValueSet) {
            return std::abs(std::abs(p.getX()) - std::abs(start.getX()));
        }
        return -1;
    }

    bool isInitialized() {
        return initialized;
    }

    bool isStartValueSet() {
        return startValueSet;
    }

private:
    FuPoint<T> start;
    FuPoint<T> end;
    struct {
        char isHorizontalConstant : 1;
        char isVerticalConstant : 1;
        char : 0; // padding
    } properties;
    // y = m * x + n
    double m;
    double n;
    bool initialized = false;
    bool startValueSet = false;
    T imageHeight;

    void init() {
        if (start == end) {
            ROS_ERROR("Line.h Constructor: start and end may not have the same coordinates!");
            return;
        }

        ROS_INFO("Line.h: start = (%d, %d), end = (%d, %d)", start.getX(), start.getY(), end.getX(), end.getY());
        properties.isVerticalConstant = 0;
        properties.isHorizontalConstant = 0;

        initialized = true;

        if (start.getX() == end.getX()) {
            properties.isVerticalConstant = 1;
            return;
        }
        if (start.getY() == end.getY()) {
            properties.isHorizontalConstant = 1;
            return;
        }
        //m = (double) (end.getY() - start.getY()) / (double) (end.getX() - start.getX());
        //double tmp = 1.f / ((((double) start.getX() / (double) end.getX()) - 1.f) + 0.0001f);
        //n = tmp * ((double) (end.getY() - start.getY()) + (double) end.getY());
    }
};

#endif /* LINE_H_ */
