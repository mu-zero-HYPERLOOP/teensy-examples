#pragma once


// FIXME! Some hints about performance! 
// Here you are using a lot of floating ops, which are always slower than fixed point integer operations
// For example m_sum += initalValue will be relativly slow. 
// Additionally you are storing it in a float[WINDOW_SIZE] array, which is also a bit overkill
// Here is what i would change!
// store m_sum, m_readings in uint16_t (halfs memory) and perform only one floating division
// on getAvg(), this might be a bit better, the only downside is that it will have to convert a 
// integer to a float here, but i think that's probably still faster.
template <unsigned long long WINDOW_SIZE> class MovingAvg {
public:
  MovingAvg(float initialValue = 0) : m_sum(0), m_next(0) {
    for (int i = 0; i < WINDOW_SIZE; ++i) {
      m_readings[i] = initialValue;
      m_sum += initialValue;
    }
  }

  void reading(float newReading) {
    m_sum = m_sum - m_readings[m_next] + newReading;

    m_readings[m_next] = newReading;
    if (++m_next >= WINDOW_SIZE)
      m_next = 0;
  }

  float getAvg() { return m_sum / WINDOW_SIZE; };

  void reset(float initalValue = 0) {
    m_sum = initalValue * WINDOW_SIZE;
    for (int i = 0; i < WINDOW_SIZE; ++i) {
      m_readings[i] = initalValue;
    }
    m_next = 0;
  }

  float *getReadings() { return m_readings; }

private:
  float m_sum;                   // sum of the m_readings array
  int m_next;                    // index to the next reading
  float m_readings[WINDOW_SIZE]; // interval array
};
