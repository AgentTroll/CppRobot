#include <ctre/phoenix/MotorControl/CAN/TalonSRX.h>
#include "WPILib.h"
#include "MyHeader.h"

namespace motor = ctre::phoenix::motorcontrol;
namespace can = motor::can;

#define POUTPUT motor::ControlMode::PercentOutput

class Robot : public IterativeRobot {
private:
    frc::Joystick leftJoy{0};
    frc::Joystick rightJoy{1};

    can::TalonSRX left;
    can::TalonSRX right;
public:
    Robot() : left{0}, right{1} {
    }

    void RobotInit() {}

    void DisabledInit() {}

    void AutonomousInit() {}

    void TeleopInit() {}

    void TestInit() {}

    void DisabledPeriodic() {}

    void AutonomousPeriodic() {}

    void TeleopPeriodic() {
        double left = leftJoy.GetRawAxis(1);
        double right = rightJoy.GetRawAxis(1);

        this->left.Set(POUTPUT, left);
        this->right.Set(POUTPUT, right);
    }

    void TestPeriodic() {}
};

START_ROBOT_CLASS(Robot)