package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
public class Chassis {
    private MecanumDrive drive;
    private FeedForwardController headingController;
    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kD = 0.001;
    public static double kS = 0.05;
    private double gyroOffset;
    private double commandedHeaading;
    private double targetHeading;

    public Chassis(HardwareMap hardwareMap, Pose2d initialPose) {
        drive = new MecanumDrive(hardwareMap, initialPose);
        gyroOffset = 0;
        headingController = new FeedForwardController(new PIDFCoefficients(kP, kI, kD, 0.0));
    }

    public void update(
            double forward,
            double strafe,
            double rotation,
            boolean isFieldOrientedControl,
            boolean isGyroReset,
            boolean isTargetOrientedControl,
            boolean isRedAlliance
    ) {
        drive.updatePoseEstimate();
        Vector2d commanded_translation;
        double heading = drive.localizer.getPose().heading.toDouble();
        if (isGyroReset){
            gyroOffset = heading;
        }
        heading = heading - gyroOffset;

        Vector2d BLUE_TARGET = new Vector2d(-64,-64);
        Vector2d RED_TARGET = new Vector2d(-64,64);

        //Controller for target heading lock
        Vector2d robotPose = drive.localizer.getPose().position;
        Vector2d targetPos;
        if (isRedAlliance) {
            targetPos = robotPose.minus(RED_TARGET);
        } else {
            targetPos = robotPose.minus(BLUE_TARGET);
        }
        headingController.targetValue = (int) (Math.atan2(targetPos.y, targetPos.x) * 1000);
        if(isTargetOrientedControl) {
            rotation = headingController.update((int) (heading * 1000));
            headingController.updateCoef(new PIDFCoefficients(kP, kI, kD, 0.0),kS);
        }

        if (isFieldOrientedControl) {
            commanded_translation = rotate(forward, strafe, heading);
        } else {
            commanded_translation = new Vector2d(forward, strafe);
        }

        commandedHeaading = heading;

        //Chassis drive is run independent of robot state
        drive.setDrivePowers(
                new PoseVelocity2d(
                        commanded_translation,
                        rotation
                )
        );
    }

    public void log(Telemetry tele) {
        tele.addData("Pinpoint X: ",drive.localizer.getPose().position.x);
        tele.addData("Pinpoint Y: ",drive.localizer.getPose().position.y);
        tele.addData("Pinpoint H: ",drive.localizer.getPose().heading.log());
        tele.addData("Target heading: ", headingController.targetValue);
        tele.addData("Heading effort: ", commandedHeaading);
    }

    public void setPose(){
        PoseStorage.poseStorage = drive.localizer.getPose();
    }

    private Vector2d rotate(double x, double y, double theta) {
        return new Vector2d(x * Math.cos(theta) + y * Math.sin(theta),
                -x * Math.sin(theta) + y * Math.cos(theta));
    }
    public MecanumDrive getMecanumDrive() {
        return drive;
    }
}
