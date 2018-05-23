#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>
#include <LiveWindow/LiveWindow.h>
#include <iostream>
#include "WPILib.h"

#define LEFT frc::XboxController::JoystickHand::kLeftHand
#define RIGHT frc::XboxController::JoystickHand::kRightHand
#define PCT_OUT motorctl::ControlMode::PercentOutput

#define REST 0
#define STOW 1
#define EXTEND 2

namespace motorctl = ctre::phoenix::motorcontrol;
namespace can = motorctl::can;

class Robot : public IterativeRobot {
    frc::XboxController controller{0};

    frc::DigitalInput armsStowed{7};
    frc::DigitalInput handlerEmpty{6};

    frc::Encoder armsEncoder{8, 9};

    can::TalonSRX arms{6};
    can::TalonSRX armRoller{5};
    can::TalonSRX launcher{8};
    can::TalonSRX handler{7};

    unsigned int ctlStatus = REST;

    unsigned int time = {0};
    unsigned int launcherRamp{0};

    can::WPI_TalonSRX left1{0};
    can::WPI_TalonSRX left2{1};
    can::WPI_TalonSRX right1{2};
    can::WPI_TalonSRX right2{3};

    double m_deadband = 0.02;
    double m_maxOutput = 1.0;
    double m_rightSideInvertMultiplier = -1.0;
public:
    void RobotInit() override {
        ctlStatus = REST;
    }

    void TeleopPeriodic() override {
        // Driving logic
        double leftSpeed = -controller.GetY(LEFT);
        double rightSpeed = controller.GetX(RIGHT);
        arcadeDrive(leftSpeed, rightSpeed);

        // Control
        if (abs(armsEncoder.Get()) >= 795) {
            arms.Set(PCT_OUT, 0);
            armRoller.Set(PCT_OUT, -1);
            handler.Set(PCT_OUT, -1);

            ctlStatus = STOW;
        } else if (controller.GetAButton()) {
            ctlStatus = EXTEND;
        }

        if (ctlStatus == STOW) {
            if (controller.GetAButton()) {
                return;
            }

            if (armsStowed.Get()) {
                armRoller.Set(PCT_OUT, 0);
                ctlStatus = REST;
            } else {
                armRoller.Set(PCT_OUT, 0);
                arms.Set(PCT_OUT, 1);
            }
        } else if (ctlStatus == EXTEND) {
            if (controller.GetBButtonPressed() || !handlerEmpty.Get()) {
                armRoller.Set(PCT_OUT, 0);
                handler.Set(PCT_OUT, 0);

                ctlStatus = STOW;
            } else {
                arms.Set(PCT_OUT, -1);
            }
        } else if (ctlStatus == REST) {
            arms.Set(PCT_OUT, 0);
        }

        // Press and hold the Y button to spit out
        if (controller.GetYButton()) {
            handler.Set(PCT_OUT, 1);
        } else if (controller.GetTriggerAxis(RIGHT) > 0) {
            handler.Set(PCT_OUT, -1);
        } else {
            handler.Set(PCT_OUT, 0);
        }

        // Every 25*20 millis (.5 sec), check left bumper
        // to see if the launcher should activate
        // Each half second the bumper is held, the
        // launcher ramps up before being set back down
        time++;
        if (time == 25) {
            time = 0;

            if (controller.GetBumper(LEFT)) {
                launcherRamp++;
            }

            switch (launcherRamp) {
                case 0:
                    launcher.Set(PCT_OUT, 0);
                    break;
                case 1:
                    launcher.Set(PCT_OUT, .4);
                    break;
                case 2:
                    launcher.Set(PCT_OUT, .6);
                    break;
                case 3:
                    launcher.Set(PCT_OUT, .8);
                    break;
                default:
                    launcherRamp = 0;
                    break;
            }
        }
    }

    void arcadeDrive(double xSpeed, double zRotation,
                     bool squaredInputs = true) {
        xSpeed = Limit(xSpeed);
        xSpeed = ApplyDeadband(xSpeed, m_deadband);

        zRotation = Limit(zRotation);
        zRotation = ApplyDeadband(zRotation, m_deadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squaredInputs) {
            xSpeed = std::copysign(xSpeed * xSpeed, xSpeed);
            zRotation = std::copysign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput =
                std::copysign(std::max(std::abs(xSpeed), std::abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
            // First quadrant, else second quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
            }
        }

        double leftSpeed = Limit(leftMotorOutput) * m_maxOutput;
        double rightSpeed = Limit(rightMotorOutput) * m_maxOutput * m_rightSideInvertMultiplier;
        left1.Set(PCT_OUT, leftSpeed);
        left2.Set(PCT_OUT, leftSpeed);
        right1.Set(PCT_OUT, rightSpeed);
        right2.Set(PCT_OUT, rightSpeed);
    }

    double Limit(double value) {
        if (value > 1.0) {
            return 1.0;
        }
        if (value < -1.0) {
            return -1.0;
        }
        return value;
    }

    double ApplyDeadband(double value, double deadband) {
        if (std::abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }
};

START_ROBOT_CLASS(Robot)
