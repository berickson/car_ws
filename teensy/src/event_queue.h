#pragma once
#include "rx_event.h"
#include "Arduino.h"

class EventQueue {
public:
  static const int size = 10;

  RxEvent events[size];

  void add(RxEvent new_event);
  bool matches(const RxEvent * pattern, int count);
  String to_string();
};
