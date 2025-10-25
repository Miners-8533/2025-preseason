package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;

@TeleOp(name="TeleOp - Flywheel Testing", group="Test")
public class FlywheelTestOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Launcher flywheel = new Launcher(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            flywheel.update();

            flywheel.log(telemetry);

            telemetry.update();
        }
    }
}
