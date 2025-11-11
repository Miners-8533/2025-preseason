package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
@Disabled
@TeleOp(name="TeleOp - Testing", group="Testing")
public class TestingTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, PoseStorage.poseStorage);

        Launcher launcher = new Launcher(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            launcher.testingPower();
            launcher.log(telemetry);
            telemetry.update();
            //robot.updateTeleOp(telemetry);
        }
    }
}
