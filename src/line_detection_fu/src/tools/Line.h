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
class Line
{
public:

	Line() {}

	Line(FuPoint<T> start, FuPoint<T> end):
		start(start),
		end(end)
	{
		if (start == end) {
			ROS_ERROR("Line.h Constructor: start and end may not have the same coordinates!");
			return;
		}
		if (start.getX() == end.getX()) {
			properties.isVerticalConstant = 1;
			return;
		}
		if (start.getY() == end.getY()) {
			properties.isHorizontalConstant = 1;
			return;
		}
		m = (double)(start.getX() - end.getX()) / (double)(start.getY() - end.getY());
		double tmp = 1.f / ((((double)end.getX() / (double)start.getX()) - 1.f) + 0.0001f);
		n = tmp * ((double)(start.getY() - end.getY()) + (double)start.getY());
	}

	virtual ~Line() {};


	FuPoint<T> getStart()
	{
		return start;
	}
	FuPoint<T> getEnd()
	{
		return end;
	}

	/*
	 * Returns the y value at the given x value
	 */
	T atX(T x)
	{
		if (properties.isHorizontalConstant) {
			return start.getY();
		}
		if (properties.isVerticalConstant) {
			ROS_INFO("Line.h atX(): vertical constant line");
			return (T) NULL;
		}
		return (m * x + n);
	}

	/*
	 * Returns the x value at the given y value
	 */
	T atY(T y)
	{
		if (properties.isHorizontalConstant) {
			ROS_INFO("Line.h atY(): horizontal constant line");
			return (T) NULL;
		}
		if (properties.isVerticalConstant) {
			return start.getX();
		}
		return (y - n) / m;
	}

	/*
	 * Returns the horizontal distance to a given FuPoint, using the FuPoints y coordinate
	 */
	T horizDistance(FuPoint<T> &p)
	{
		if (properties.isHorizontalConstant) {
			ROS_INFO("Line.h horizDistance(): vertical constant line");
			return (T) NULL;
		}
		return std::abs(p.getX() - atY(p.getY()));
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

};

#endif /* LINE_H_ */
