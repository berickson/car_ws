#include "lookup_table.h"
#include <vector>
#include <array>
#include "geometry.h"
#include <iostream>
// include rclcpp for logging
#include "rclcpp/rclcpp.hpp"

LookupTable::LookupTable(vector<array<double, 2> > table)
  : table(table)
{

}

LookupTable::LookupTable(vector<double> flattened_table) {
  // Get the logger for "car"
  auto logger = rclcpp::get_logger("car");

  // ensure that the flattened table has an even number of elements
  if (flattened_table.size() % 2 != 0) {
    RCLCPP_ERROR(logger, "flattened_table must have an even number of elements");
    return;
  }
  // ensure that the flattened table has at least 2 elements
  if (flattened_table.size() < 2) {
    RCLCPP_ERROR(logger, "flattened_table must have at least 2 elements");
    return;
  }
  // ensure that the flattened table is sorted
  for (unsigned i = 0; i < flattened_table.size()-2; i+=2) {
    if (flattened_table[i] >= flattened_table[i+2]) {
      RCLCPP_ERROR(logger, "flattened_table must be sorted");
      return;
    }
  }

  for (unsigned i = 0; i < flattened_table.size(); i+=2) {
    table.push_back({flattened_table[i],flattened_table[i+1]});
  }
}

double LookupTable::lookup(double v) const {
  unsigned last = table.size()-1;
  if (v <= table[0][0]){
    return table[0][1];

  }
  if (v >= table[last][0]) {
    return table[last][1];
  }
  for(unsigned i = 0; i < last; i++) {
    if (v >= table[i][0] && v < table[i+1][0]){
      return interpolate(v,table[i][0],table[i][1],table[i+1][0],table[i+1][1]);
    }
  }
  return NAN; // error if we didnt find v

}

void test_lookup_table()
{
  LookupTable t(
  {
    {-2., 1200},
    {-1., 1250},
    {0.0,  1500},
    {0.1, 1645},
    {0.34, 1659},
    {0.85, 1679},
    {1.2, 1699},
    {1.71, 1719},
    {1.88, 1739},
    {2.22, 1759},
    {2.6, 1779},
    {3.0, 1799},
    {14.0, 2000}
  });

  vector<double> items = {-3,-2,0,2.1,4,13.9,16};
  for (auto item:items)
    cout << "lookup " << item << " returned " << t.lookup(item) << endl;
}
