#pragma once

#include "task.h"

class OffMode : public Task {
public:
  virtual void begin();
  virtual void end();
  virtual void execute();
  OffMode();
};
