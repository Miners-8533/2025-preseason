package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Flywheel {
    //private MotorWithEncoderAndVelocityController fly_motor;
    private DcMotorEx fly_motor;
    private DcMotorEx fly_follow;
    public static double targetVelocity = 0.0;
    public static double kP = 10.0;
    public static double kD = 5.0;
    public static double kF = 12.0;
    public Flywheel(HardwareMap hardwareMap) {
        fly_motor = hardwareMap.get(DcMotorEx.class, "fly_motor_enc");
        fly_follow = hardwareMap.get(DcMotorEx.class, "fly_follow");

        fly_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        fly_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly_motor.setMotorEnable();
        fly_motor.setVelocityPIDFCoefficients(kP, 0.0, kD, kF);
        fly_motor.setVelocity(0.0);

        fly_follow.setDirection(DcMotorSimple.Direction.FORWARD);
        fly_follow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly_follow.setMotorEnable();
        fly_follow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly_follow.setPower(0.0);
    }

    public void update() {
        fly_motor.setVelocityPIDFCoefficients(kP, 0.0, kD, kF);
        fly_motor.setVelocity(targetVelocity);

        fly_follow.setPower(fly_motor.getPower());
    }

    public void log(Telemetry tele) {
        tele.addData("Fly motor encoder ticks per second",  fly_motor.getVelocity());
        tele.addData("Fly motor encoder position",          fly_motor.getCurrentPosition());
        tele.addData("Fly motor current (A)",               fly_motor.getCurrent(CurrentUnit.AMPS));
        tele.addData("Fly motor power (+/-%FS)",            fly_motor.getPower());
        tele.addData("Follow motor current (A)",            fly_follow.getCurrent(CurrentUnit.AMPS));
        tele.addData("Follow motor power (+/-%FS)",         fly_follow.getPower());
    }

}
