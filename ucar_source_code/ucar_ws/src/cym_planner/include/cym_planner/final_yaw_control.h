#ifndef CYM_PLANNER_FINAL_YAW_CONTROL_H_
#define CYM_PLANNER_FINAL_YAW_CONTROL_H_

#include <cmath>

namespace cym_planner
{

static const double kFinalYawPi = 3.14159265358979323846;

// Keeps the final-yaw error continuous when tf::getYaw() crosses its
// [-pi, pi] branch cut.
class FinalYawTracker
{
  public:
    FinalYawTracker()
        : initialized_(false), unwrapped_error_(0.0)
    {
    }

    void reset()
    {
        initialized_ = false;
        unwrapped_error_ = 0.0;
    }

    double update(double normalized_error)
    {
        if (!initialized_)
        {
            initialized_ = true;
            unwrapped_error_ = normalized_error;
            return unwrapped_error_;
        }

        // Choose the representation nearest to the previous error.  This
        // preserves the selected rotation direction when a pose update moves
        // tf::getYaw() from +pi to -pi (or the reverse).
        double candidate = normalized_error;
        while (candidate - unwrapped_error_ > kFinalYawPi)
            candidate -= 2.0 * kFinalYawPi;
        while (candidate - unwrapped_error_ < -kFinalYawPi)
            candidate += 2.0 * kFinalYawPi;
        unwrapped_error_ = candidate;
        return unwrapped_error_;
    }

  private:
    bool initialized_;
    double unwrapped_error_;
};

}  // namespace cym_planner

#endif  // CYM_PLANNER_FINAL_YAW_CONTROL_H_
