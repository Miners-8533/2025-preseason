package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private DcMotorEx intake;
    private DcMotorEx transport;//needs encoder connected

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transport = hardwareMap.get(DcMotorEx.class, "transport");

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0.0);
        intake.setMotorEnable();

        transport.setDirection(DcMotorSimple.Direction.FORWARD);
        transport.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transport.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transport.setPower(0.0);
        transport.setMotorEnable();
    }

    public void update(double intakePower, double transportPower) {
        intake.setPower(intakePower);
        transport.setPower(transportPower);
    }
    public void log(Telemetry tele) {
        tele.addData("Intake current (A)",               intake.getCurrent(CurrentUnit.AMPS));
        tele.addData("Intake power (+/-%FS)",            intake.getPower());
        tele.addData("Transport current (A)",            transport.getCurrent(CurrentUnit.AMPS));
        tele.addData("Transport power (+/-%FS)",         transport.getPower());
        tele.addData("Transport position (ticks)",         transport.getCurrentPosition());
    }
}
