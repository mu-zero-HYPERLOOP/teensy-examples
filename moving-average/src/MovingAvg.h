#ifndef MOVINGAVG_H_INCLUDED
#define MOVINGAVG_H_INCLUDED

template<unsigned long long WINDOW_SIZE>
class MovingAvg
{
    public:
        MovingAvg(float initialValue) : m_sum(0), m_next(0) {
            for (int i = 0; i < WINDOW_SIZE; ++i) {
                m_readings[i] = initialValue;
                m_sum += initialValue;
            }
        }
        void reading(float newReading) {
            m_sum = m_sum - m_readings[m_next] + newReading;

            m_readings[m_next] = newReading;
            if (++m_next >= WINDOW_SIZE) m_next = 0;
        }
        float getAvg() {
            return m_sum / WINDOW_SIZE;
        };
        void reset() {
            m_sum = 0;
            m_next = 0;
        }
        float* getReadings() {return m_readings;}

    private:
        float m_sum;         // sum of the m_readings array
        int m_next;         // index to the next reading
        float m_readings[WINDOW_SIZE];    // interval array
};
#endif