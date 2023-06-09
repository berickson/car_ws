#ifndef ACKERMAN_H
#define ACKERMAN_H

#include "geometry.h"

#include <math.h>
#include <string>
#include <sstream>
#include <iostream>
#include "geometry.h"

using namespace std;


class Ackermann {
public:
  struct Arc {
    double steer_radians;
    double arc_radians;
    double r;
    double arc_len;
    double curvature; // radians per unit length of arc
  };

  double w; // width of front wheelbase
  double l;
  double x; // x and y are at center of rear wheels
  double y;
  double heading;

  Ackermann(double front_wheelbase_width = 1.0, double wheelbase_length = 1.0, Point position=Point(0,0), Angle heading = Angle::radians(0));
  void reset(Point position  = Point(), Angle heading = Angle::radians(0));

  Point front_left_position() const;
  Point front_position() const;
  Point rear_position() const;


  void move_right_wheel(Angle outside_wheel_angle, double wheel_distance, double new_heading = NAN);
  void move_relative_to_heading(Point p);
  Arc arc_to_relative_location(double x,double y);
  string to_string() const;
};

// calculates an arc where the front wheel will travel to
// point x ahead and point y t left
void test_arc_to_relative_location(double l, double x, double y);
void arc_to_relative_location_tests();
void move_left_wheel_tests();

void test_ackerman();
void test_ackerman2();
void test_pose();

#endif // ACKERMAN_H
