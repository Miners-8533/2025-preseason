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
    public static double targetVelocity = 0.0;
    public static double kP = 1.0;
    public static double kD = 1.0;
    public static double kF = 2.8;
    public Flywheel(HardwareMap hardwareMap) {
        fly_motor = hardwareMap.get(DcMotorEx.class, "fly_motor");
        fly_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        fly_motor.setMotorEnable();
        fly_motor.setVelocityPIDFCoefficients(kP, 0.0, kD, kF);
        fly_motor.setVelocity(0.0);
    }

    public void update() {
        fly_motor.setVelocityPIDFCoefficients(kP, 0.0, kD, kF);
        fly_motor.setVelocity(targetVelocity);
    }

    public void set_Power(double req_power) {
        fly_motor.setPower(req_power);
    }

    public void log(Telemetry tele) {
        tele.addData("Fly motor encoder ticks per second",  fly_motor.getVelocity());
        tele.addData("Fly motor encoder position",          fly_motor.getCurrentPosition());
        tele.addData("Fly motor current (A)",               fly_motor.getCurrent(CurrentUnit.AMPS));
        tele.addData("Fly motor power (+/-%FS)",            fly_motor.getPower());
    }

}
