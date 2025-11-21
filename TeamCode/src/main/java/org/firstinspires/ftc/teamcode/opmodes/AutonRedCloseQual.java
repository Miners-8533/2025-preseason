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

@Autonomous(name="Red Close", group="Competition")
public class AutonRedCloseQual extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isFlipped = true;
        PoseStorage.isRedAllicance = true;

        Pose2d initialPose          = mirrorPose(new Pose2d(-55.0,-46.0, Math.toRadians(-135)), isFlipped);
        Pose2d pickLoadingZoneStart = mirrorPose(new Pose2d(54.0,-60.0, Math.toRadians(-45 )), isFlipped);
        Pose2d pickLoadingZoneEnd   = mirrorPose(new Pose2d(64.0,-60.0, Math.toRadians(-90 )), isFlipped);
        Pose2d closeScorePose       = mirrorPose(new Pose2d(-21.0,-21.0, Math.toRadians(-134.5)), isFlipped);
        Pose2d pickBaseGroup        = mirrorPose(new Pose2d(39.0,-26.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickBaseGroupEnd     = mirrorPose(new Pose2d(39.0,-49.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupStart   = mirrorPose(new Pose2d( 10.0,-25.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupEnd     = mirrorPose(new Pose2d( 10.0,-49.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickRampGroupStart   = mirrorPose(new Pose2d(-11.0,-28.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickRampGroupEnd     = mirrorPose(new Pose2d(-11.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d gate                 = mirrorPose(new Pose2d(  -3.0,-60.0, Math.toRadians( 90 )), isFlipped);
        Pose2d gatePark             = mirrorPose(new Pose2d(  3.5,-40.0, Math.toRadians( 90 )), isFlipped);
        Pose2d park                 = mirrorPose(new Pose2d(36.0,-12.0, Math.toRadians(-135)), isFlipped);
        Pose2d juggle               = mirrorPose(new Pose2d(-52.0,-42.0, Math.toRadians(-130)), isFlipped);
        Pose2d limelight            = mirrorPose(new Pose2d(-44.0,-39.0, Math.toRadians( 135)), isFlipped);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(45), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(closeScorePose)
                .turnTo(mirrorTangent(Math.toRadians(290), isFlipped), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                //.splineToLinearHeading(pickRampGroupStart, mirrorTangent(Math.toRadians(45), isFlipped),null, new ProfileAccelConstraint(-40,50))
                .splineToLinearHeading(pickRampGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-40,50));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickRampGroupEnd)
                .turnTo(mirrorTangent(Math.toRadians(135), isFlipped), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .setReversed(true)
                .splineToLinearHeading(gate, mirrorTangent(Math.toRadians(270), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(gate)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(180), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(closeScorePose)
                .splineToLinearHeading(pickGateGroupStart, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .splineToLinearHeading(pickGateGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(pickGateGroupEnd)
                .setReversed(true)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab7 = drive.actionBuilder(closeScorePose)
                .setReversed(true)
                //.splineToLinearHeading(pickBaseGroup, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .splineToLinearHeading(pickBaseGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab8 = drive.actionBuilder(pickBaseGroupEnd)
                .setReversed(true)
                .splineToSplineHeading(closeScorePose, mirrorTangent(Math.toRadians(180), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder tab9 = drive.actionBuilder(closeScorePose)
                //.turnTo(Math.toRadians(90), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .setReversed(true)
                .splineToSplineHeading(gatePark, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-50,50));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
            robot.setLaunchClose(),
            robot.readyLaunch(),
            tab1.build(),
            robot.launch(),
            robot.runIntake(),
            tab2.build(),
            tab3.build(),
            new SleepAction(0.5),
            robot.readyLaunch(),
            tab4.build(),
            robot.launch(),
            robot.runIntake(),
            tab5.build(),
            robot.readyLaunch(),
            tab6.build(),
            robot.launch(),
            robot.runIntake(),
            tab7.build(),
            robot.readyLaunch(),
            tab8.build(),
            robot.launch(),
            robot.runIntake(),
            tab9.build()
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
