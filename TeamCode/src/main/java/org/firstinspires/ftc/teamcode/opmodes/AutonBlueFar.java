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

@Autonomous(name="Blue Far", group="Competition")
public class AutonBlueFar extends LinearOpMode {

    @Override
    public void runOpMode() {
        boolean isFlipped = false;
        PoseStorage.isRedAllicance = false;

        Pose2d initialPose          = mirrorPose(new Pose2d(64.5,-17.5, Math.toRadians(180 )), isFlipped);
        Pose2d pickLoadingZoneStart = mirrorPose(new Pose2d(40.0,-65.0, Math.toRadians(0   )), isFlipped);
        Pose2d pickLoadingZoneEnd   = mirrorPose(new Pose2d(60.0,-65.0, Math.toRadians(0   )), isFlipped);
        Pose2d farScorePose         = mirrorPose(new Pose2d(57.0,-12.0, Math.toRadians(203 )), isFlipped);
        Pose2d pickCloseGroup       = mirrorPose(new Pose2d(35.0,-26.0, Math.toRadians(-90 )), isFlipped);
        Pose2d pickCloseGroupSweep  = mirrorPose(new Pose2d(35.0,-58.0, Math.toRadians(-90 )), isFlipped);
        Pose2d park                 = mirrorPose(new Pose2d(36.0,-12.0, Math.toRadians(-135)), isFlipped);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickCloseGroup, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(pickCloseGroupSweep, mirrorTangent(Math.toRadians(270), isFlipped),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickCloseGroupSweep)
                .setReversed(true)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(270), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickLoadingZoneStart, mirrorTangent(Math.toRadians(0), isFlipped),null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(pickLoadingZoneEnd, mirrorTangent(Math.toRadians(0), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickLoadingZoneEnd)
                .setReversed(true)
                .splineToLinearHeading(farScorePose, mirrorTangent(Math.toRadians(90), isFlipped), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(park, mirrorTangent(Math.toRadians(-135), isFlipped), null, new ProfileAccelConstraint(-30,30));

        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
            robot.setLaunchFar(),
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
            //robot.readyLaunch(), For Hydra
            tab5.build(),
            //robot.launch(), For Hydra
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
