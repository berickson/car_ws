#pragma once

class RxEvent {
public:
  char speed;
  char steer;
  char aux;

  RxEvent():RxEvent('0','0','0'){}
  RxEvent(char steer, char speed, char aux);
  bool equals(RxEvent other);
  bool is_bad();
};
