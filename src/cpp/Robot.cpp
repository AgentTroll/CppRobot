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
    frc::XboxController controller;

    frc::DigitalInput armsStowed;
    frc::DigitalInput handlerEmpty;

    frc::Encoder armsEncoder;

    can::TalonSRX arms;
    can::TalonSRX armRoller;
    can::TalonSRX launcher;
    can::TalonSRX handler;

    frc::DifferentialDrive *drive;

    unsigned int ctlStatus = REST;

    unsigned int time = 0;
    unsigned int launcherRamp = 0;

public:
    Robot() :
            controller(0),
            armsStowed(7), handlerEmpty(6),
            armsEncoder(8, 9),
            arms(6), armRoller(5), launcher(8), handler(7) {
        // FIXME: Perhaps not the best solution to the crashing issue
        LiveWindow *liveWindow = frc::LiveWindow::GetInstance();
        liveWindow->SetEnabled(false);
        liveWindow->DisableAllTelemetry();

        can::WPI_TalonSRX left1(0);
        can::WPI_TalonSRX left2(1);
        can::WPI_TalonSRX right1(2);
        can::WPI_TalonSRX right2(3);

        frc::SpeedControllerGroup leftGroup(left1, left2);
        frc::SpeedControllerGroup rightGroup(right1, right2);

        drive = new frc::DifferentialDrive(leftGroup, rightGroup);
        // FIXME: Another hack that should get figured out
        drive->SetSafetyEnabled(false);
    }

    ~Robot() override {
        delete (drive);
    }

    void TeleopPeriodic() override {
        // Driving logic
        // double leftSpeed = -controller.GetY(LEFT);
        // double rightSpeed = controller.GetX(RIGHT);
        // drive->ArcadeDrive(leftSpeed, rightSpeed, false);

        // Control
        if (abs(armsEncoder.Get()) >= 795) {
            arms.Set(PCT_OUT, 0);
            armRoller.Set(PCT_OUT, -1);
            handler.Set(PCT_OUT, 1);

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
};

START_ROBOT_CLASS(Robot)