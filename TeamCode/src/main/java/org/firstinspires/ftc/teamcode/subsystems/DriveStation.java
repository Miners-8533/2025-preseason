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
    public boolean isFieldOriented = true;
    public boolean isTargetLocked = false;
    public boolean isRedAlliance;
    public boolean isIntaking = false;
    public boolean isLaunching = false;
    public boolean isLaunchingPressed = false;
    public ElapsedTime darylsTimer;
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

        //TODO do we leave this in or force move to robot oriented control?
        isGyroReset = driver.back;
        //Manual override for loss of gyro
        if(driver.startWasPressed()) {isFieldOriented = !isFieldOriented;}

        //TODO for debug should be sourced from auton
        if(driver.xWasPressed()) {
            isRedAlliance = !isRedAlliance;
        }

        //Target lock
        isTargetLocked = operator.a;

        //Outake
        isOuttaking = operator.dpad_down;

        //Launch
        isLaunching = operator.right_bumper;

    }
}
