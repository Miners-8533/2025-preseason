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

@Autonomous(name="Blue Close Elim", group="Competition")
public class AutonBlueCloseElim extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isFlipped = false;
        PoseStorage.isRedAllicance = false;

        Pose2d initialPose          = mirrorPose(new Pose2d(-55.0,-46.0, Math.toRadians(-135)), isFlipped);
        Pose2d pickLoadingZoneStart = mirrorPose(new Pose2d(54.0,-60.0, Math.toRadians(-45 )), isFlipped);
        Pose2d pickLoadingZoneEnd   = mirrorPose(new Pose2d(64.0,-60.0, Math.toRadians(-90 )), isFlipped);
        Pose2d closeScorePose       = mirrorPose(new Pose2d(-21.0,-21.0, Math.toRadians(-132.5)), isFlipped);
        Pose2d pickBaseGroup        = mirrorPose(new Pose2d(39.0,-26.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickBaseGroupEnd     = mirrorPose(new Pose2d(39.0,-49.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupStart   = mirrorPose(new Pose2d( 10.0,-25.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupEnd     = mirrorPose(new Pose2d( 10.0,-49.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickRampGroupStart   = mirrorPose(new Pose2d(-11.0,-28.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickRampGroupEnd     = mirrorPose(new Pose2d(-11.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d gate                 = mirrorPose(new Pose2d(  -3.0,-60.0, Math.toRadians( 90 )), isFlipped);
        Pose2d gatePark             = mirrorPose(new Pose2d(  3.5,-40.0, Math.toRadians( 90 )), isFlipped);
        Pose2d park                 = mirrorPose(new Pose2d(36.0,-12.0, Math.toRadians(-135)), isFlipped);
        Pose2d parkClose            = mirrorPose(new Pose2d(-11.0,-31.0, Math.toRadians(-132.5)), isFlipped);
        Pose2d juggle               = mirrorPose(new Pose2d(-52.0,-42.0, Math.toRadians(-130)), isFlipped);
        Pose2d limelight            = mirrorPose(new Pose2d(-44.0,-39.0, Math.toRadians( 135)), isFlipped);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder initToScore = drive.actionBuilder(initialPose)
                .setReversed(true)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(45), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder scoreToRamp = drive.actionBuilder(closeScorePose)
                .turnTo(Math.toRadians(290), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .splineToLinearHeading(pickRampGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder rampToGate = drive.actionBuilder(pickRampGroupEnd)
                .turnTo(Math.toRadians(135), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .setReversed(true)
                .splineToLinearHeading(gate, mirrorTangent(Math.toRadians(270), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder gateToScore = drive.actionBuilder(gate)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(180), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder scoreToGatePick = drive.actionBuilder(closeScorePose)
                .setReversed(true)
                .splineToLinearHeading(pickGateGroupStart, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50))
                .splineToLinearHeading(pickGateGroupEnd, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder gatePickToGate = drive.actionBuilder(pickGateGroupEnd)
                .turnTo(Math.toRadians(45), new TurnConstraints(6*Math.PI, -2*Math.PI,2*Math.PI))
                .setReversed(true)
                .splineToLinearHeading(gate, mirrorTangent(Math.toRadians(270), isFlipped), null, new ProfileAccelConstraint(-50,50));

        TrajectoryActionBuilder scoreToPark = drive.actionBuilder(closeScorePose)
                .setReversed(true)
                .splineToConstantHeading(parkClose.position, mirrorTangent(Math.toRadians(-45), isFlipped), null, new ProfileAccelConstraint(-50,50));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
            robot.setLaunchClose(),
            robot.readyLaunch(),
            initToScore.build(),
            robot.launch(),
            robot.runIntake(),
            scoreToRamp.build(),
            rampToGate.build(),
            new SleepAction(0.1),
            robot.readyLaunch(),
            gateToScore.build(),
            robot.launch(),
            robot.runIntake(),
            scoreToGatePick.build(),
            gatePickToGate.build(),
            new SleepAction(0.1),
            robot.readyLaunch(),
            gateToScore.build(),
            robot.launch(),
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
