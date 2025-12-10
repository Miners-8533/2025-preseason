package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="Red Far Elim", group="Competition")
public class AutonRedFarElim extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isFlipped = true;
        PoseStorage.isRedAllicance = true;

        Pose2d initialPose          = mirrorPose(new Pose2d(64.5,-17.5, Math.toRadians(180 )), isFlipped);
        Pose2d pickLoadingZoneStart = mirrorPose(new Pose2d(56.0,-60.0, Math.toRadians(-45 )), isFlipped);
        Pose2d pickLoadingZoneEnd   = mirrorPose(new Pose2d(65.0,-65.0, Math.toRadians(-90 )), isFlipped);
        Pose2d farScorePose         = mirrorPose(new Pose2d(57.0,-12.0, Math.toRadians(204.5)), isFlipped);
        Pose2d pickBaseGroup        = mirrorPose(new Pose2d(35.0,-26.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickBaseGroupEnd     = mirrorPose(new Pose2d(38.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupStart   = mirrorPose(new Pose2d(12.0,-25.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupEnd     = mirrorPose(new Pose2d(12.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d gate                 = mirrorPose(new Pose2d( 3.5,-56.0, Math.toRadians( 90 )), isFlipped);
        Pose2d park                 = mirrorPose(new Pose2d(57.0,-27.0, Math.toRadians(-90)), isFlipped);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder initToScore = drive.actionBuilder(initialPose)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder scoreToLoadZone = drive.actionBuilder(farScorePose)
                //.turnTo(Math.toRadians(290), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .splineToLinearHeading(pickLoadingZoneStart, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .splineToLinearHeading(pickLoadingZoneEnd, mirrorTangent(Math.toRadians(180), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder loadZoneToScore = drive.actionBuilder(pickLoadingZoneEnd)
                .setReversed(true)
                .splineToSplineHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-20,50));

        TrajectoryActionBuilder blindPick = drive.actionBuilder(farScorePose)
                .turnTo(mirrorTangent(Math.toRadians(290), isFlipped), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .splineToLinearHeading(pickLoadingZoneEnd, mirrorTangent(Math.toRadians(180), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder scoreToBase = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickBaseGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped), null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder baseToScore = drive.actionBuilder(pickBaseGroupEnd)
                .setReversed(true)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder scoreToPark = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(park, mirrorTangent(Math.toRadians(-90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
                robot.setLaunchFar(),
                robot.readyLaunch(),
                initToScore.build(),
                robot.launchLong(),
                robot.runIntake(),
                scoreToLoadZone.build(),
                robot.readyLaunch(),
                loadZoneToScore.build(),
                robot.launchLong(),
                robot.runIntake(),
                scoreToBase.build(),
                robot.readyLaunch(),
                baseToScore.build(),
                robot.launchLong(),
                robot.runIntake(),
                blindPick.build(),
                new ParallelAction(
                        loadZoneToScore.build(),
                        new SequentialAction(
                                robot.setNoIntake(),
                                new SleepAction(.2),
                                robot.setIntake(),
                                new SleepAction(.5),
                                robot.readyLaunch())),
                robot.launchLong(),
                robot.runIntake(),
                scoreToPark.build()
        )));
    }

    public Pose2d mirrorPose(Pose2d in, boolean isFlipped) {
        Pose2d tempPose = in;
        if(isFlipped) {
            tempPose = new Pose2d(
                    new Vector2d(
                        in.position.x,
                        in.position.y * -1.0
                    ),
                    -1.0*in.heading.toDouble()
            );
        }
        return tempPose;
    }

    public double mirrorTangent(double in, boolean isFlipped) {
        if(isFlipped) {
            return in*-1.0;
        } else {
            return in;
        }
    }

}
