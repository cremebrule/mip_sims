// Ringbuf class //

#include "ringbuf.h"

Ringbuf::Ringbuf() {this->num = 1;}
Ringbuf::Ringbuf(int total) {this->num = total;}
Ringbuf::~Ringbuf() {}
int Ringbuf::update_idx(void)
{
  this->idx = (this->idx + 1) % num;
  return idx;
}

// END Ringbuf class //
