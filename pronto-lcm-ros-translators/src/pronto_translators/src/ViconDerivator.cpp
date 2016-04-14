#include <ViconDerivator.hpp>
#include <iostream>

ViconDerivator::ViconDerivator() :
    filter_bw(0.2),
    first_time(true),
    period_(5),
    window(new Eigen::Vector3d[period_]),
    head(NULL),
    tail(NULL),
    total(Eigen::Vector3d::Zero()) {

}

ViconDerivator::ViconDerivator(unsigned int period) :
    filter_bw(0.2),
    first_time(true),
    period_(period),
    window(new Eigen::Vector3d[period_]),
    head(NULL),
    tail(NULL),
    total(Eigen::Vector3d::Zero()) {

}

ViconDerivator::~ViconDerivator() {
    delete window;
}

void ViconDerivator::updateIIR(double time, const Eigen::Vector3d position, Eigen::Vector3d &velocity) {
    if(first_time) {
        last_time = time;
        old_position = position;
        std::cout << "[ Vicon Derivator ]: First time!" << std::endl;
        velocity = Eigen::Vector3d::Zero();
        first_time = false;
        return;
    }
    // Computing the velocity pose
    dt = time - last_time;
    Eigen::Vector3d linear_velocity = (position - old_position) / dt;

    // Filtering the velocity pose
    alpha = dt / (filter_bw + dt);

    velocity = old_velocity + alpha * (linear_velocity - old_velocity);

    last_time = time;
    old_velocity = velocity;
    old_position = position;

}

void ViconDerivator::updateMA(double time, const Eigen::Vector3d position, Eigen::Vector3d &velocity) {
    if(first_time) {
        last_time = time;
        old_position = position;
        std::cout << "[ Vicon Derivator ]: First time!" << std::endl;
        velocity = Eigen::Vector3d::Zero();

        head = window;
        *head = velocity;
        tail = head;
        inc(tail);
        total = velocity;

        first_time = false;
        return;
    }
    // Computing the velocity pose
    dt = time - last_time;
    Eigen::Vector3d linear_velocity = (position - old_position) / dt;

    // Were we already full?
    if (head == tail) {
        // Fix total-cache
        total -= *head;
        // Make room
        inc(head);
    }

    // Write the value in the next spot.
    *tail = linear_velocity;
    inc(tail);

    // Update our total-cache
    total += linear_velocity;

    last_time = time;
    old_position = position;

    ptrdiff_t size = this->size();
    if (size == 0) {
        velocity = Eigen::Vector3d::Zero(); // No entries => 0 average
    }
    velocity =  total / (double) size; // Cast to double for floating point arithmetic
}

void ViconDerivator::updateRaw(double time,
                               const Eigen::Vector3d position,
                               Eigen::Vector3d &velocity) {
    if(first_time) {
        last_time = time;
        old_position = position;
        std::cout << "[ Vicon Derivator ]: First time!" << std::endl;
        velocity = Eigen::Vector3d::Zero();
        first_time = false;
        return;
    }
    // Computing the velocity pose
    dt = time - last_time;
    velocity = (position - old_position) / dt;
}

