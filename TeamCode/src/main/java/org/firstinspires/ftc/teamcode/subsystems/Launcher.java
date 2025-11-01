package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Launcher {
    private DcMotorEx fly_motor;
    private DcMotorEx fly_follow;
    public static double targetVelocity = 0.0;
    public static double kP = 0.2;
    public static double kD = 0;
    public static double kF = 0.0;
    private Servo hood;
    private Servo stop;
    public static double hoodTarget = SubSystemConfigs.HOOD_MAX;
    public static double stopTarget = SubSystemConfigs.STOP_LOCK;
    public static double kP_I = 0.0001;
    public static double kD_I = 0.0;
    public static double kF_I = 0.0;
    private FeedForwardController currentFollower;
    private FeedForwardController velocityPID;
    private final double[][] launchMap = {
            //Distance (in) , effort (ticks/second), angle (servo position [0,1.0]
            {0.0, 0.0, 0.0},//first point needs min values
            {0.0, 0.0, 0.0},
            {0.0, 740.0, 0.57}//last point needs max values
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

        currentFollower = new FeedForwardController(new PIDCoefficients(kP_I, 0.0, kD_I));
        currentFollower.feedForwardFunc = (input)->{return kF_I*input;};

        velocityPID = new FeedForwardController(new PIDCoefficients(kP,0.0,kD));
        velocityPID.feedForwardFunc = (input)->{return kF*input;};
    }

    public void setDistance(double distance) {
        //find nearest points
        int i = 1;
        while((i < launchMap.length) && (distance < launchMap[i][0])) {
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

    public void update() {
        velocityPID.coefficients = new PIDCoefficients(kP, 0.0, kD); //TODO for config REMOVE
        velocityPID.feedForwardFunc = (input)->{return kF*input;}; //TODO for config REMOVE
        currentFollower.coefficients = new PIDCoefficients(kP_I, 0.0, kD_I); //TODO for config REMOVE
        currentFollower.feedForwardFunc = (input)->{return kF_I*input;}; //TODO for config REMOVE

        velocityPID.targetValue = targetVelocity;
        double newFlyMotorPower = velocityPID.update(fly_motor.getVelocity());
        if(newFlyMotorPower < 0.0 ) {newFlyMotorPower = 0.0;}
        fly_motor.setPower(newFlyMotorPower);

        currentFollower.targetValue = fly_motor.getCurrent(CurrentUnit.MILLIAMPS);
        double newFollowPower = currentFollower.update(fly_follow.getCurrent(CurrentUnit.MILLIAMPS));
        if(newFollowPower < 0.0 ) {newFollowPower = 0.0;}
        fly_follow.setPower(newFollowPower);

        hood.setPosition(hoodTarget);
        stop.setPosition(stopTarget);
    }
    public void log(Telemetry tele) {
        tele.addData("Fly motor encoder ticks per second",  fly_motor.getVelocity());
        tele.addData("Fly motor current (mA)",               fly_motor.getCurrent(CurrentUnit.MILLIAMPS));
        tele.addData("Follow motor current (mA)",            fly_follow.getCurrent(CurrentUnit.MILLIAMPS));
        tele.addData("Fly motor power (+/-%FS)",            fly_motor.getPower());
        tele.addData("Follow motor power (+/-%FS)",         fly_follow.getPower());
    }

}
