package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private Chassis chassis;
    private DriveStation driveStation;
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
        chassis = new Chassis(hardwareMap, initialPose);
        driveStation = new DriveStation(driverController, operatorController);
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
                false,
                driveStation.isGyroReset,
                driveStation.isTargetOriented,
                true
        );

        chassis.log(telemetry);

        telemetry.update();
    }

}
