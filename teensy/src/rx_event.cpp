#include "rx_event.h"

RxEvent::RxEvent(char steer, char speed, char aux) {
  this->steer = steer;
  this->speed = speed;
  this->aux = aux;
}

bool RxEvent::equals(RxEvent other) {
  return speed == other.speed && steer == other.steer && aux == other.aux;
}

bool RxEvent::is_bad(){
  return speed == '?' || steer == '?';
}
