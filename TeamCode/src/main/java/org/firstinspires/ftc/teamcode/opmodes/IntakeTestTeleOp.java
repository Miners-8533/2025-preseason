package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

@TeleOp(name="TeleOp - IntakeTest", group="Test")
public class IntakeTestTeleOp extends LinearOpMode {

    private DcMotorEx leftFront, leftBack, rightBack, rightFront;
    private DcMotorEx intake;
    private boolean intake_toogle = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: reverse motor directions if needed
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            float forwardVelocity = -gamepad1.left_stick_y;
            float strafeVelocity = gamepad1.left_stick_x;
            float rotationVelocity = gamepad1.right_stick_x;

            leftFront.setPower(forwardVelocity + strafeVelocity + rotationVelocity);
            leftBack.setPower(forwardVelocity - strafeVelocity + rotationVelocity);
            rightBack.setPower(forwardVelocity - strafeVelocity - rotationVelocity);
            rightFront.setPower(forwardVelocity + strafeVelocity - rotationVelocity);

            if(gamepad1.aWasPressed()) {
                intake_toogle = !intake_toogle;
            }

            if(intake_toogle) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0.0);
            }

        }
    }
}
