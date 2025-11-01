package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    private Chassis chassis;
    private Launcher launcher;
    private DriveStation driveStation;
    private Limelight limelight;
    private boolean isDemoBot = false;
    private Intake intake;
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
        chassis = new Chassis(hardwareMap, initialPose);
        driveStation = new DriveStation(driverController, operatorController);
        if(!isDemoBot) {
            launcher = new Launcher(hardwareMap);
            intake = new Intake(hardwareMap);
        }
        limelight = new Limelight(hardwareMap);
    }
    public void updateTeleOp(Telemetry telemetry) {
        double desiredStrafe;
        double desiredForward;
        double desiredRotation;

        //get fresh inputs first
        driveStation.update();

        desiredForward = driveStation.forward;
        desiredStrafe = driveStation.strafe;
        desiredRotation = driveStation.rotation;

        if(isDemoBot) {
            desiredForward = -1.0*desiredForward;
            desiredRotation = -1.0*desiredRotation;
        }

        chassis.update(
                desiredForward,
                desiredStrafe,
                desiredRotation,
                true,
                driveStation.isGyroReset,
                driveStation.isTargetOriented,
                driveStation.isRedAlliance
        );

        if(!isDemoBot) {
            //Basic intaking control
            if (driveStation.isIntaking) {
                intake.intakePower = 1.0;
                intake.transportPower = 1.0;
            } else {
                intake.intakePower = 0.0;
                intake.transportPower = 0.0;
            }
            //Basic launching control for now
            if (driveStation.isLaunching) {
                if (driveStation.launchingTargetTime < driveStation.darylsTimer.seconds()) {
                    intake.transportPower = 1.0;
                }
            }
        }

        //distance get and send to launcher
        if(driveStation.isLaunching) {
            launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
        } else {
            launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
        }

        if(!isDemoBot) {
            launcher.update();
            intake.update();
        }
        limelight.update();

        chassis.log(telemetry);
        if(!isDemoBot) {
            launcher.log(telemetry);
            intake.log(telemetry);
        }
        limelight.log(telemetry);

        telemetry.update();
    }
    public MecanumDrive getMecanumDrive() {
        return chassis.getMecanumDrive();
    }

}
