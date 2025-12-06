package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Disabled
@Autonomous(name="Blue Far Elim", group="Competition")
public class AutonBlueFarElim extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isFlipped = false;
        PoseStorage.isRedAllicance = false;

        Pose2d initialPose          = mirrorPose(new Pose2d(64.5,-17.5, Math.toRadians(180 )), isFlipped);
        Pose2d pickLoadingZoneStart = mirrorPose(new Pose2d(56.0,-60.0, Math.toRadians(-45 )), isFlipped);
        Pose2d pickLoadingZoneEnd   = mirrorPose(new Pose2d(65.0,-65.0, Math.toRadians(-90 )), isFlipped);
        Pose2d farScorePose         = mirrorPose(new Pose2d(57.0,-12.0, Math.toRadians(203.5 )), isFlipped);
        Pose2d pickBaseGroup        = mirrorPose(new Pose2d(35.0,-26.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickBaseGroupEnd     = mirrorPose(new Pose2d(38.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupStart   = mirrorPose(new Pose2d( 12.0,-25.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupEnd     = mirrorPose(new Pose2d( 12.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d gate                 = mirrorPose(new Pose2d(  3.5,-56.0, Math.toRadians( 90 )), isFlipped);
        Pose2d park                 = mirrorPose(new Pose2d(36.0,-12.0, Math.toRadians(-135)), isFlipped);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(farScorePose)
                .turnTo(Math.toRadians(290), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                //.splineToLinearHeading(pickLoadingZoneStart, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .splineToLinearHeading(pickLoadingZoneEnd, mirrorTangent(Math.toRadians(180), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickLoadingZoneEnd)
                .setReversed(true)
                .splineToSplineHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-20,20));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickBaseGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped), null, new ProfileAccelConstraint(-20,30));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickBaseGroupEnd)
                .setReversed(true)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickGateGroupStart, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .splineToLinearHeading(pickGateGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .turnTo(mirrorTangent(Math.toRadians(45), isFlipped), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI));

        TrajectoryActionBuilder tab7 = drive.actionBuilder(pickGateGroupEnd)
                .setReversed(true)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab8 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickLoadingZoneEnd, mirrorTangent(Math.toRadians(-90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
            robot.setLaunchFar(),
            robot.readyLaunch(),
            tab1.build(),
            robot.launchLong(),
            robot.runIntake(),
            tab2.build(),
            robot.readyLaunch(),
            tab3.build(),
            robot.launchLong(),
            robot.runIntake(),
            tab4.build(),
            robot.readyLaunch(),
            tab5.build(),
            //new SleepAction(2.0),
            robot.launchLong(),
            robot.runIntake(),
            tab6.build()
//            robot.readyLaunch(),
//            tab7.build(),
//            robot.launch(),
//            robot.runIntake(),
//            tab8.build()
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
