package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@Autonomous(name="Testing", group="Competition")
public class AutonTesting extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(64.5,-17.5, Math.toRadians(180));
        Pose2d pickLoadingZoneStart = new Pose2d(40,-65, Math.toRadians(0));
        Pose2d pickLoadingZoneEnd = new Pose2d(58,-65, Math.toRadians(0));
        Pose2d farScorePose = new Pose2d(57, -12,Math.toRadians(203));
        Pose2d pickCloseGroup = new Pose2d(35,-26, Math.toRadians(-90));
        Pose2d pickCloseGroupSweep = new Pose2d(35,-58, Math.toRadians(-90));

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);
        MecanumDrive drive = robot.getMecanumDrive();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(farScorePose, Math.toRadians(90), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickCloseGroup, Math.toRadians(270),null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(pickCloseGroupSweep, Math.toRadians(270),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(pickCloseGroupSweep)
                .setReversed(true)
                //.setTangent(Math.toRadians(45))
                .splineToLinearHeading(farScorePose, Math.toRadians(270), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(farScorePose)
                .splineToLinearHeading(pickLoadingZoneStart, Math.toRadians(0),null, new ProfileAccelConstraint(-30,30))
                .splineToLinearHeading(pickLoadingZoneEnd, Math.toRadians(0), null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(pickLoadingZoneEnd)
                .setReversed(true)
                .splineToLinearHeading(farScorePose, Math.toRadians(90), null, new ProfileAccelConstraint(-30,30));

/*
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(pickLoadingZone, Math.toRadians(90),null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder driveToIntermediate = drive.actionBuilder(pickLoadingZone)
                .setReversed(true)
                .splineToLinearHeading(secondScorePose, Math.toRadians(180));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(pickLoadingZone)
                .setReversed(true)
                .setTangent(Math.toRadians(270.0))
                .splineToSplineHeading(secondScorePose, Math.toRadians(180))
                .splineToSplineHeading(firstSpikeMark, Math.toRadians(90),null, new ProfileAccelConstraint(-30,30));

        TrajectoryActionBuilder tab3  = drive.actionBuilder(firstSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180));

        TrajectoryActionBuilder tab4 = drive.actionBuilder(scoreHighBasket)
                .setTangent(0)
                .splineToLinearHeading(secondSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-15,50));

        TrajectoryActionBuilder tab5 = drive.actionBuilder(secondSpikeMark)
                .splineToLinearHeading(scoreHighBasket, Math.toRadians(180),null, new ProfileAccelConstraint(-40,40));

        TrajectoryActionBuilder tab6 = drive.actionBuilder(scoreHighBasket)
                .setReversed(true)
                .splineToLinearHeading(parkNearSubmersible, Math.toRadians(0),null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder tab7 = drive.actionBuilder(scoreHighBasket)
                .setTangent(0)
                .splineToLinearHeading(thirdSpikeMark,Math.toRadians(180),null, new ProfileAccelConstraint(-15,40));
*/
        // Wait for the game to start (driver presses START)
        waitForStart();

        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
            new ParallelAction(
                robot.readyLaunch(),
                tab1.build()
            ),
            //may need velocity guard here
            robot.launch(),
            robot.runIntake(),
            tab2.build(),
            robot.readyLaunch(),
            tab3.build(),
            robot.launch(),
            robot.runIntake(),
            tab4.build(),
            robot.readyLaunch(),
            tab5.build(),
            robot.launch()
            /*,
            robot.scoreThree()*/

//                new ParallelAction(
//                        robot.autonStart(),
//                        tab1.build()
//                ),
//                robot.scoreSpecimen(),
//                robot.goToReadyPose(),
//                tab2.build(),
//                new ParallelAction(
//                        new SleepAction(1.0),
//                        robot.floorAcquire()
//                ),
//                robot.floorAcquireReach(),
//                new ParallelAction(
//                        tab3.build(),
//                        robot.prepareScoreHighBasket()
//                ),
//                robot.highBasketReach(),
//                new ParallelAction(
//                        new SleepAction(0.5),
//                        robot.scoreHighBasket()
//                ),
//                robot.prepareScoreHighBasket(),
//                new ParallelAction(
//                        robot.goToReadyPose(),
//                        tab4.build()
//                ),
//                new ParallelAction(
//                        new SleepAction(1.0),
//                        robot.floorAcquire()
//                ),
//                robot.floorAcquireReach(),
//                new ParallelAction(
//                        robot.prepareScoreHighBasket(),
//                        tab5.build()
//                ),
//                robot.highBasketReach(),
//                new ParallelAction(
//                        new SleepAction(0.5),
//                        robot.scoreHighBasket()
//                ),
//                robot.prepareScoreHighBasket(),
//                new ParallelAction(
//                        robot.goToReadyPose(),
//                        tab7.build()
//                ),
//                new ParallelAction(
//                        new SleepAction(1.0),
//                        robot.floorAcquire()
//                ),
//                robot.floorAcquireReach(),
//                new SleepAction(0.5),
//                new ParallelAction(
//                        robot.prepareScoreHighBasket(),
//                        tab5.build()
//                ),
//                robot.highBasketReach(),
//                new ParallelAction(
//                        new SleepAction(0.5),
//                        robot.scoreHighBasket()
//                ),
//                robot.prepareScoreHighBasket(),
//                new ParallelAction(
//                        robot.goToReadyPose(),
//                        tab6.build()
//                ),
//                robot.goToReadyPose()
        )));
    }

}
