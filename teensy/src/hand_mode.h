#pragma once

#include "task.h"

class HandMode : public Task {
public:
  virtual void begin();
  virtual void end();
  virtual void execute();
  HandMode();
};
