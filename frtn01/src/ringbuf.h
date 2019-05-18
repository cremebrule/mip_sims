// Ringbuf class //

class Ringbuf
{
public:
  Ringbuf();
  Ringbuf(int total);
  ~Ringbuf();
  int update_idx(void);
  
private:
  int num;
  int idx = 0;
};

// END Ringbuf class //
