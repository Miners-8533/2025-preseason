package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SubSystemConfigs;

@Disabled
@TeleOp(name="TeleOp - Testing", group="Testing")
public class TestingTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, PoseStorage.poseStorage);

        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Limelight limelight = new Limelight(hardwareMap);

        waitForStart();
        intake.intakePower = SubSystemConfigs.INTAKE_RUN;
        launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
        intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
        launcher.autoBallCount = 0;
        launcher.isSetForSpin = true;
        while (opModeIsActive()) {
//            if(launcher.isVelocityGoodAuton() && gamepad1.a) {
//                intake.transportPower = SubSystemConfigs.TRANSPORT_INTAKE;
//            } else {
//                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
//            }
            launcher.update();
            intake.update();
            limelight.update();
            launcher.log(telemetry);
            intake.log(telemetry);
            limelight.log(telemetry);
            telemetry.update();
            //robot.updateTeleOp(telemetry);
        }
    }
}
