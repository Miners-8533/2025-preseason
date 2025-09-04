package org.firstinspires.ftc.teamcode.opmodes;

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

        Pose2d initialPose = new Pose2d(-16,-62, Math.toRadians(90));
        Pose2d scoreChamber = new Pose2d(-8,-28, Math.toRadians(90));
        Pose2d intermediatePose = new Pose2d(-21, -37,Math.toRadians(120));
        Pose2d scoreHighBasket = new Pose2d(-56, -51, Math.toRadians(225));
        Pose2d firstSpikeMark = new Pose2d(-35.5, -22.5, Math.toRadians(180));
        Pose2d secondSpikeMark = new Pose2d(-44.5, -22.5, Math.toRadians(180));
        Pose2d thirdSpikeMark = new Pose2d(-52.5, -23, Math.toRadians(175));
        Pose2d parkNearSubmersible = new Pose2d(-15, -6.5, Math.toRadians(0));

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        Robot robot = new Robot(hardwareMap,gamepad1,gamepad2, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .splineToLinearHeading(scoreChamber, Math.toRadians(90),null, new ProfileAccelConstraint(-30,50));

        TrajectoryActionBuilder driveToIntermediate = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .splineToLinearHeading(intermediatePose, Math.toRadians(180));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(scoreChamber)
                .setReversed(true)
                .setTangent(Math.toRadians(270.0))
                .splineToSplineHeading(intermediatePose, Math.toRadians(180))
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

        // Wait for the game to start (driver presses START)
        waitForStart();

//        Actions.runBlocking(new ParallelAction(robot.autonUpdate(), new SequentialAction(
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
//        )));
    }

}
