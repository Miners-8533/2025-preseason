package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class MotorWithEncoderAndVelocityController {
    private int tolerance;
    private DcMotorEx motor;
    private FeedForwardController ffc;
    private ElapsedTime timer = new ElapsedTime();
    private final String name;
    private boolean isFlipped;
    public MotorWithEncoderAndVelocityController(HardwareMap hardwareMap, Config config) {
        name = config.deviceName;
        motor = hardwareMap.get(DcMotorEx.class, config.deviceName);
        ffc = new FeedForwardController(config.coefficients);
        ffc.kStiction = config.kStiction;
        motor.setDirection(config.direction);
        motor.setMotorEnable();
        tolerance = config.tolerance;
        isFlipped = config.isFlipped;
    }
    public void update() {
        int vel = (int) motor.getVelocity();
        double motorPower = ffc.update(vel);

        if (isFlipped){
            motor.setPower(-motorPower);
        } else {
            motor.setPower(motorPower);
        }
    }
    public void setTarget(int targetVelocity) {
        ffc.targetValue = targetVelocity;
    }
    public boolean isDone() {
        int error = ((int) motor.getVelocity()) - ffc.targetValue;
        boolean temp = Math.abs(error) < tolerance;
//        if (temp){
//            motor.setPower(-ffc.coefficients.f);
//        }
        return temp;
    }
    public void log(Telemetry tele) {
        tele.addData(name + " current encoder ticks",   motor.getCurrentPosition());
        tele.addData(name + " motor current (A)",       motor.getCurrent(CurrentUnit.AMPS));
        tele.addData(name + " current target position", ffc.targetValue);
        tele.addData(name + " atTarget?",               isDone());
        tele.addData(name + " motor power (+/-%FS)",    motor.getPower());
    }

    public void autonLog(TelemetryPacket packet) {
        packet.put(name + " current encoder ticks per second",   motor.getVelocity());
        packet.put(name + " current target velocity", ffc.targetValue);
        packet.put(name + " motor power (+/-%FS)",    motor.getPower());
    }
    public static class Config {
        public String deviceName;
        public PIDFCoefficients coefficients;
        public double kStiction;
        public DcMotorSimple.Direction direction;
        public boolean isFlipped;
        public int tolerance;
        public Config(String deviceName,
                      PIDFCoefficients coefficients,
                      double kStiction,
                      DcMotorSimple.Direction direction, int tolerance, boolean isFlipped) {
            this.deviceName = deviceName;
            this.coefficients = coefficients;
            this.kStiction = kStiction;
            this.direction = direction;
            this.tolerance = tolerance;
            this.isFlipped = isFlipped;
        }
    }
}
