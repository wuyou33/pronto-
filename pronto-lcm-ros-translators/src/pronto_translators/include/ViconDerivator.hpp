#ifndef VICONDERIVATOR_HPP
#define VICONDERIVATOR_HPP

#include <Eigen/Dense>

class ViconDerivator {
public:
    ViconDerivator();
    ViconDerivator(unsigned int period);
    virtual ~ViconDerivator();
    void updateIIR(double time, const Eigen::Vector3d position, Eigen::Vector3d& velocity);
    void updateMA(double time, const Eigen::Vector3d position, Eigen::Vector3d& velocity);
    void updateRaw(double time, const Eigen::Vector3d position, Eigen::Vector3d& velocity);
private:
    Eigen::Vector3d old_velocity;
    Eigen::Vector3d old_position;
    bool first_time;
    double alpha;
    double filter_bw;
    double dt;
    double last_time;

    /* FOR THE MOVING AVERAGE */
    unsigned int period_;
    Eigen::Vector3d * window; // Holds the values to calculate the average of.

    // Logically, head is before tail
    Eigen::Vector3d * head; // Points at the oldest element we've stored.
    Eigen::Vector3d * tail; // Points at the newest element we've stored.

    Eigen::Vector3d total; // Cache the total so we don't sum everything each time.

    // Bumps the given pointer up by one.
    // Wraps to the start of the array if needed.
    void inc(Eigen::Vector3d * & p) {
        if (++p >= window + period_) {
            p = window;
        }
    }

    // Returns how many numbers we have stored.
    ptrdiff_t size() const {
        if (head == NULL)
            return 0;
        if (head == tail)
            return period_;
        return (period_ + tail - head) % period_;
    }
};

#endif // VICONDERIVATOR_HPP

