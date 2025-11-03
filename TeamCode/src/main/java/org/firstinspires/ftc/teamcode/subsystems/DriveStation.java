package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveStation {
    private Gamepad driver;
    private Gamepad operator;

    public double forward;
    public double strafe;
    public double rotation;
    public boolean lastAcquire;
    public boolean isGyroReset;
    public boolean isTargetOriented = false;
    public boolean isRedAlliance;
    public boolean isStopRelease = false;
    public boolean isIntaking = false;
    public boolean isLaunching = false;
    public ElapsedTime darylsTimer;
    public double launchingTargetTime = Double.MAX_VALUE;
    public double intakeBackoutTargetTime = 0.0;
    public double transportBackoutTargetTime = 0.0;
    public boolean isOuttaking = false;
    public DriveStation(Gamepad driverController, Gamepad operatorController) {
        driver = driverController;
        operator = operatorController;
        darylsTimer = new ElapsedTime();
        lastAcquire = false;
    }

    public void update() {
        //driver move
        forward = -driver.left_stick_y;
        strafe = -driver.left_stick_x;
        rotation = -driver.right_stick_x / 2.0;
        isGyroReset = driver.back;

        if(driver.yWasPressed()) {
            isTargetOriented = !isTargetOriented;
        }

        if(driver.xWasPressed()) {
            isRedAlliance = !isRedAlliance;
        }

        if(driver.bWasPressed()) {
            isIntaking = !isIntaking;
            if(!isIntaking) {
                intakeBackoutTargetTime = darylsTimer.seconds() + SubSystemConfigs.INTAKE_BACKOUT_TIME;
                transportBackoutTargetTime = intakeBackoutTargetTime + SubSystemConfigs.TRANSPORT_BACKOUT_TIME;
            }
        }

        isOuttaking = driver.dpad_down;

        isLaunching = driver.right_bumper;
        if(driver.rightBumperWasPressed()) {
            launchingTargetTime = darylsTimer.seconds() + SubSystemConfigs.TRANSPORT_DELAY;
        }


//        //driver score
//        isScoreSpecimen = driver.right_bumper;
//        isOutakeSample = driver.right_bumper;
//
//        //driver reset
//        isReady = driver.y;
//
//        //operator climb
//        isClimb = operator.dpad_up;
//        isClimbPrep = operator.dpad_right || operator.dpad_left;
//        isClimbReset = operator.dpad_down;

        //operator acquire

//        lastAcquire = isAquireSample;
//        isAquireSample = operator.right_bumper;
//        isTargetSample = operator.right_trigger > 0.5;
//        if(isAquireSample != lastAcquire) {
//            acquireTimer.reset();
//        }
//
//        //operator bumper override
//        isBumperDown = operator.left_trigger > 0.5;
//
//        isWingDown = operator.left_bumper;
//
//        //operator prepare for high basket
//        isScoreBasket = operator.y;
//
//        //operator scrubs
//        reachScrub = operator.right_stick_y;
//        liftScrub = operator.left_stick_y;
//
//        //operator conditional drive
//        if(isTargetSample) {
//            forward = forward - operator.left_stick_y;
//            forward = Math.max(Math.min(1.0, forward), -1.0);
//            strafe = strafe - operator.left_stick_x;
//            strafe = Math.max(Math.min(1.0, strafe), -1.0);
//            //rotation = rotation - operator.right_stick_x / 3.0;
//            //rotation = Math.max(Math.min(1.0, rotation), -1.0);
//        }
    }
}
