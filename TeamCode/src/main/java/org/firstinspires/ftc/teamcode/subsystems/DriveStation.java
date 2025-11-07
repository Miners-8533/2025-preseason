package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DriveStation {
    private Gamepad driver;
    private Gamepad operator;
    public double forward;
    public double strafe;
    public double rotation;
    public boolean isFieldOriented = true;
    public boolean isTargetLocked = false;
    public boolean isLaunching = false;
    public boolean isOuttaking = false;
    public double targetLockScrub = 0.0;
    public DriveStation(Gamepad driverController, Gamepad operatorController) {
        driver = driverController;
        operator = operatorController;
    }

    public void update() {
        //driver move
        forward = -driver.left_stick_y;
        strafe = -driver.left_stick_x;
        rotation = -driver.right_stick_x / 2.0;

        //Manual override for loss of gyro
        if(driver.startWasPressed()) {isFieldOriented = !isFieldOriented;}

        if(driver.dpadUpWasPressed())    {targetLockScrub = 0.0;           }//Reset scrub
        double dirFix = (PoseStorage.isRedAllicance) ? 1.0 : -1.0;
        if(driver.dpadRightWasPressed()) {targetLockScrub += Math.PI/180.0 * dirFix;}
        if(driver.dpadLeftWasPressed())  {targetLockScrub -= Math.PI/180.0 * dirFix;}

//        //TODO for debug should be sourced from auton
//        if(driver.xWasPressed()) {
//            isRedAlliance = !isRedAlliance;
//        }

        //Target lock
        isTargetLocked = operator.a;

        //Outake
        isOuttaking = operator.dpad_down;

        //Launch
        isLaunching = operator.right_bumper;

    }
}
