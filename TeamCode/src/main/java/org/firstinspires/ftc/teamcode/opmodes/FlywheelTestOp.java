package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TeleOp - Flywheel Testing", group="Test")
public class FlywheelTestOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Flywheel flywheel = new Flywheel(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            flywheel.update();

            if(gamepad1.a) {
                flywheel.set_Power(gamepad1.left_stick_y);
            }

            flywheel.log(telemetry);

            telemetry.update();
        }
    }
}
