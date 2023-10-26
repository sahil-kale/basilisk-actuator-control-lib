#ifndef CONTROL_LOOP_HPP
#define CONTROL_LOOP_HPP

namespace control_loop {

// Make an abstract control loop class with a run method

class ControlLoop {
   public:
    class ControlLoopStatus {
       public:
        enum class ControlLoopBaseStatus {
            OK,
            WARNING,
            ERROR,
        };
        ControlLoopBaseStatus status = ControlLoopBaseStatus::OK;

        // Define an equal operator that checks whether a status class is equal to a ControlLoopBaseStatus
        bool operator==(ControlLoopBaseStatus status) const { return this->status == status; }

        // Define a not equal operator that checks whether a status class is not equal to a ControlLoopBaseStatus
        bool operator!=(ControlLoopBaseStatus status) const { return this->status != status; }
    };
    virtual ControlLoopStatus run(float speed) = 0;
    // Define a max motor_speed member constant
    static constexpr float MAX_MOTOR_SPEED = 1.0f;
};

}  // namespace control_loop

#endif  // CONTROL_LOOP_HPP