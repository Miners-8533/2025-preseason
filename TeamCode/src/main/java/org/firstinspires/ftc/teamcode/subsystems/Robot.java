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
    private Intake intake;
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
        chassis = new Chassis(hardwareMap, initialPose);
        driveStation = new DriveStation(driverController, operatorController);
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
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

        chassis.update(
                desiredForward,
                desiredStrafe,
                desiredRotation,
                true,
                driveStation.isGyroReset,
                driveStation.isTargetOriented,
                driveStation.isRedAlliance
        );

        //Basic intaking control
        if (driveStation.isIntaking) {
            intake.intakePower = 1.0;
            intake.transportPower = 0.25;
        } else if(driveStation.intakeBackoutTargetTime > driveStation.darylsTimer.seconds()) {
            intake.intakePower = -0.0;
            intake.transportPower = -0.5;
        } else {
            intake.intakePower = 0.0;
            intake.transportPower = 0.0;
        }

        if(driveStation.isOuttaking) {
            intake.intakePower = -1.0;
            intake.transportPower = -1.0;
        }

        //Basic launching control for now
        if (driveStation.isLaunching) {
            if (launcher.isVelocityGood()) {
                intake.transportPower = 1.0;
                intake.transportPower = 1.0;
            } else {
                //Hold feed until we are up to flywheel speed
                intake.transportPower = 0.0;
            }
        }

        //only spin up if target locked for now
        //TODO need to consider manual aiming case
        launcher.isSetForSpin = !driveStation.isTargetOriented;

        //TODO need to set distance of Launcher for angle/speed targets

        //distance get and send to launcher
        if(driveStation.isTargetOriented) {
            launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
        } else {
            launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
        }

        //spin intake on shoot
        //target for first needs to be 100 for first shot
        //intake cannot be running during target lock
        //need time delay for lock to close

        launcher.update();
        intake.update();
        limelight.update();

        chassis.log(telemetry);
        launcher.log(telemetry);
        intake.log(telemetry);
        limelight.log(telemetry);

        telemetry.update();
    }
    public MecanumDrive getMecanumDrive() {
        return chassis.getMecanumDrive();
    }

}
