#include <ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h>
#include "WPILib.h"

#define LEFT frc::XboxController::JoystickHand::kLeftHand
#define RIGHT frc::XboxController::JoystickHand::kRightHand
#define PCT_OUT motorctl::ControlMode::PercentOutput

namespace motorctl = ctre::phoenix::motorcontrol;
namespace can = motorctl::can;

class Robot : public IterativeRobot {
    frc::XboxController controller;

    frc::DigitalInput stowArmsLimit;
    frc::DigitalInput handlerTriggered;

    frc::Encoder armsEncoder;

    can::TalonSRX arms;
    can::TalonSRX armRoller;
    can::TalonSRX launcher;
    can::TalonSRX handler;

    frc::DifferentialDrive *drive;

    bool shouldStow = false;

    unsigned int time = 0;
    unsigned int launcherRamp = 0;

public:
    Robot() :
            controller(0),
            stowArmsLimit(7), handlerTriggered(6),
            armsEncoder(8, 9),
            arms(6), armRoller(7), launcher(8), handler(5) {
        can::WPI_TalonSRX left1(0);
        can::WPI_TalonSRX left2(1);
        can::WPI_TalonSRX right1(2);
        can::WPI_TalonSRX right2(3);

        frc::SpeedControllerGroup leftGroup(left1, left2);
        frc::SpeedControllerGroup rightGroup(right1, right2);

        drive = new frc::DifferentialDrive(leftGroup, rightGroup);
    }

    ~Robot() override {
        delete(drive);
    }

    void TeleopPeriodic() override {
        // Driving logic
        double leftSpeed = controller.GetY(LEFT);
        double rightSpeed = controller.GetX(RIGHT);
        drive->ArcadeDrive(leftSpeed, rightSpeed, false);

        // If arms have been stowed already, stop
        // Otherwise, if requested by driver, stop
        // and stow arms
        if (stowArmsLimit.Get()) {
            arms.Set(PCT_OUT, 0);
            shouldStow = false;
        } else if (controller.GetBButtonPressed()) {
            armRoller.Set(PCT_OUT, 0);
            handler.Set(PCT_OUT, 0);
            shouldStow = true;
        }

        // If arms should be stowed (either by request
        // of the driver or the handler switch, then
        // stow arms
        if (shouldStow) {
            arms.Set(PCT_OUT, -1);
        }

        // If arms are within 775-815 ticks, stop opening
        // and start the rollers/handlers
        // Otherwise, check to see if the driver wants to
        // open the arms
        int armDistance = abs(armsEncoder.Get());
        if (armDistance >= 775 && armDistance <= 815) {
            arms.Set(PCT_OUT, 0);
            armRoller.Set(PCT_OUT, 1);
            handler.Set(PCT_OUT, -1);
        } else if (controller.GetAButtonPressed()) {
            arms.Set(PCT_OUT, 1);
        }

        // If the boulder triggers the handler, then stop
        // the roller and handler, and stow the arms
        if (handlerTriggered.Get()) {
            armRoller.Set(PCT_OUT, 0);
            handler.Set(PCT_OUT, 0);
            shouldStow = true;
        }

        // Press and hold the Y button to spit out
        if (controller.GetYButtonPressed()) {
            handler.Set(PCT_OUT, 1);
        } else {
            handler.Set(PCT_OUT, 0);
        }

        // Press trigger to shoot boulder
        if (controller.GetTriggerAxis(RIGHT) > 0) {
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

            if (controller.GetBumperPressed(LEFT)) {
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
};

START_ROBOT_CLASS(Robot)