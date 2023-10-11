#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

namespace control_loop {

// Make an abstract control loop class with a run method

class ControlLoop {
   public:
    virtual void run(float speed) = 0;
    // Define a max motor_speed member constant
    static constexpr float MAX_MOTOR_SPEED = 1.0f;
};

}  // namespace control_loop

#endif  // CONTROL_LOOP_HPP