package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="Blue Close", group="Competition")
public class AutonBlueClose extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isFlipped = false;
        PoseStorage.isRedAllicance = false;

        Pose2d initialPose        = mirrorPose(new Pose2d(-55.0,-46.0, Math.toRadians(-135)), isFlipped);
        Pose2d pickGateGroupStart = mirrorPose(new Pose2d( 10.0,-25.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickGateGroupEnd   = mirrorPose(new Pose2d( 10.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d closeScorePose     = mirrorPose(new Pose2d(-21.0,-21.0, Math.toRadians(-135)), isFlipped);
        Pose2d pickRampGroupStart = mirrorPose(new Pose2d(-11.0,-25.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickRampGroupEnd   = mirrorPose(new Pose2d(-11.0,-48.0, Math.toRadians(-90 )), isFlipped);
        Pose2d park               = mirrorPose(new Pose2d(  0.0,-36.0, Math.toRadians( 90 )), isFlipped);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(45), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(closeScorePose)
                .splineToLinearHeading(pickRampGroupStart, mirrorTangent(Math.toRadians(-90), isFlipped),null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(pickRampGroupEnd, mirrorTangent(Math.toRadians(-90), isFlipped),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickRampGroupEnd)
                .setReversed(true)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(closeScorePose)
                .splineToLinearHeading(pickGateGroupStart, mirrorTangent(Math.toRadians(-45), isFlipped),null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(pickGateGroupEnd, mirrorTangent(Math.toRadians(-45), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickGateGroupEnd)
                .setReversed(true)
                .splineToLinearHeading(closeScorePose, mirrorTangent(Math.toRadians(135), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(closeScorePose)
                .splineToLinearHeading(park, mirrorTangent(Math.toRadians(-145), isFlipped), null, new ProfileAccelConstraint(-30,30));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
            robot.setLaunchClose(),
            new ParallelAction(
                robot.readyLaunch(),
                tab1.build()
            ),
            robot.launch(),
            robot.runIntake(),
            tab2.build(),
            robot.readyLaunch(),
            tab3.build(),
            robot.launch(),
            robot.runIntake(),
            tab4.build(),
            //robot.readyLaunch(),
            tab5.build(),
            //robot.launch(),
            tab6.build()
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
