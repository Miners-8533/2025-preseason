package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Config
public class Launcher {
    private DcMotorEx fly_motor;
    private DcMotorEx fly_follow;
    public static double targetVelocity = 740.0;//0.0;
    private Servo hood;
    private Servo stop;
    public static double hoodTarget = SubSystemConfigs.HOOD_MAX;
    public double stopTarget = SubSystemConfigs.STOP_LOCK;
    public boolean isFirstLaunch = false;
    public boolean isSetForSpin = false;
    private final double[][] launchMap = {
            //Distance (in) , effort (ticks/second), angle (servo position [0,1.0]
            {0.0,   460.0, 0.66},//first point needs min values
            {25.0,  460.0, 0.66},
            {35.0,  500.0, 0.62},
            {60.0,  560.0, 0.61},
            {87.0,  640.0, 0.59},
            {105.0, 700.0, 0.57},
            {120.0, 720.0, 0.565},
            {130.0, 740.0, 0.565},
            {150.0, 820.0, 0.56},
            {200.0, 820.0, 0.56}//last point needs max values
    };
    public Launcher(HardwareMap hardwareMap) {
        fly_motor = hardwareMap.get(DcMotorEx.class, "fly_motor_enc");
        fly_follow = hardwareMap.get(DcMotorEx.class, "fly_follow");
        hood = hardwareMap.get(Servo.class, "hood");
        stop = hardwareMap.get(Servo.class, "stop");

        fly_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        fly_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly_motor.setMotorEnable();
        fly_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fly_motor.setVelocity(0.0);

        fly_follow.setDirection(DcMotorSimple.Direction.REVERSE);
        fly_follow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fly_follow.setMotorEnable();
        fly_follow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fly_follow.setPower(0.0);
    }

    public void setDistance(double distance) {
        //find nearest points
        int i = 1;
        while(i < (launchMap.length-1) && (distance >= launchMap[i][0])) {
            i++;
        }

        // Perform linear interpolation
        double x0 = launchMap[i-1][0];
        double x1 = launchMap[i][0];
        double prop = ((distance - x0) / (x1 - x0));

        if(!Double.isNaN(prop)) {
            double y0 = launchMap[i - 1][1];
            double y1 = launchMap[i][1];
            targetVelocity = y0 + (y1 - y0) * prop;

            y0 = launchMap[i - 1][2];
            y1 = launchMap[i][2];
            hoodTarget = y0 + (y1 - y0) * prop;
        } else {
            //handle case where x1 = x0 (NaN)
            targetVelocity = launchMap[i][1];
            hoodTarget = launchMap[i][2];
        }
    }

    void autonSet(double hood, double vel) {
        targetVelocity = vel;
        hoodTarget = hood;
    }

    public boolean isVelocityGood() {
        double velocity = fly_motor.getVelocity();
        double threshold = targetVelocity;
        if(isFirstLaunch) {
            threshold *= 1.0;
        } else {
            threshold *= 0.9;
        }
        boolean isThresholdMet = velocity > threshold;
        if(isThresholdMet && isFirstLaunch) {
            isFirstLaunch = false;
        }
        return isThresholdMet;
    }
    public boolean isVelocityGoodAuton() {
        double velocity = fly_motor.getVelocity();
        double threshold = targetVelocity;
        if(isFirstLaunch) {
            threshold *= 1.0;
        } else {
            threshold *= 0.95;
        }
        boolean isThresholdMet = velocity > threshold;
        if(isThresholdMet && isFirstLaunch) {
            isFirstLaunch = false;
        }
        return isThresholdMet;
    }

    public void update() {
            if ((fly_motor.getVelocity() > targetVelocity) || (targetVelocity == 0.0) || !isSetForSpin) {
                fly_motor.setPower(0.0);
                fly_follow.setPower(0.0);
            } else {
                fly_motor.setPower(1.0);
                fly_follow.setPower(1.0);
            }

        hood.setPosition(hoodTarget);
        stop.setPosition(stopTarget);
    }
    public void log(Telemetry tele) {
        tele.addData("Fly motor encoder ticks per second",  fly_motor.getVelocity());
        tele.addData("Fly motor current (mA)",               fly_motor.getCurrent(CurrentUnit.MILLIAMPS));
        tele.addData("Follow motor current (mA)",            fly_follow.getCurrent(CurrentUnit.MILLIAMPS));
        tele.addData("Fly motor power (+/-%FS)",            fly_motor.getPower());
        tele.addData("Follow motor power (+/-%FS)",         fly_follow.getPower());
        tele.addData("Target Velocity: ", targetVelocity);
        tele.addData("Hood target: ", hoodTarget);
    }

    public void autonLog(TelemetryPacket packet) {
        packet.put("Velocity Measured: ", fly_motor.getVelocity());
        packet.put("Velocity Target: ", targetVelocity);
        packet.put("isVelocityGood", isVelocityGood());
    }

}
