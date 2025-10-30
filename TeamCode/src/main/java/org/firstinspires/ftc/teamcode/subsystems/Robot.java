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
    //private Limelight limelight;
    private Intake intake;
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
        chassis = new Chassis(hardwareMap, initialPose);
        driveStation = new DriveStation(driverController, operatorController);
        launcher = new Launcher(hardwareMap);
        //limelight = new Limelight(hardwareMap);
        intake = new Intake(hardwareMap);
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

        //distance get and send to launcher
        if(driveStation.isStopRelease) {
            launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
        } else {
            launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
        }
        intake.intakePower = driveStation.intake;
        intake.transportPower = driveStation.transport;

        launcher.update();
        //limelight.update();
        intake.update();

        chassis.log(telemetry);
        launcher.log(telemetry);
        //limelight.log(telemetry);
        intake.log(telemetry);

        telemetry.update();
    }
    public MecanumDrive getMecanumDrive() {
        return chassis.getMecanumDrive();
    }

}
