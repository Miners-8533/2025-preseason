package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    private Chassis chassis;
    private Launcher launcher;
    private DriveStation driveStation;
    private Limelight limelight;
    private Intake intake;
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
        chassis = new Chassis(hardwareMap, initialPose);
        driveStation = new DriveStation(driverController, operatorController);
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }
    public void updateTeleOp(Telemetry telemetry) {
        //get fresh inputs first
        driveStation.update();

        double desiredForward = driveStation.forward;
        double desiredStrafe = driveStation.strafe;
        double desiredRotation = driveStation.rotation;

        //Basic intaking control
        if (driveStation.isIntaking) {
            intake.intakePower = SubSystemConfigs.INTAKE_MAX;
            //TODO may need a stop lockout time delay before moving transport
            intake.transportPower = SubSystemConfigs.TRANSPORT_INTAKE;
        } else if(driveStation.transportBackoutTargetTime > driveStation.darylsTimer.seconds()) {
            intake.intakePower = -0.0;
            intake.transportPower = SubSystemConfigs.TRANSPORT_BACKOUT;
        } else {
            intake.intakePower = SubSystemConfigs.INTAKE_STOP;
            intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
        }

        if(driveStation.isOuttaking) {
            intake.intakePower = SubSystemConfigs.INTAKE_OUTAKE;
            intake.transportPower = SubSystemConfigs.TRANSPORT_OUTAKE;
        }

        //Launch control
        //Target lock
        if(driveStation.isTargetLocked) {
            launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
            intake.intakePower = 0.0;
            intake.transportPower = 0.0;
        } else {
            launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
        }

        //only spin up if target locked for now
        //TODO need to consider manual aiming case
        launcher.isSetForSpin = driveStation.isTargetLocked;

        //TODO need to set distance of Launcher for angle/speed targets
        //launcher.setDistance(chassis.targetDist);

        if(driveStation.isLaunchingPressed) {
            launcher.isFirstLaunch = true;
        }

        if (driveStation.isLaunching) {
            if (launcher.isVelocityGood()) {
                intake.transportPower = SubSystemConfigs.TRANSPORT_MAX;
                //run intake while shooting to make sure artifacts move through transport
                intake.intakePower = SubSystemConfigs.INTAKE_MAX;
            } else {
                //Hold feed until we are up to flywheel speed
                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
            }
        }

        //All updates grouped together (except driveStation)
        launcher.update();
        intake.update();
        limelight.update();
        chassis.update(
                desiredForward,
                desiredStrafe,
                desiredRotation,
                true,
                driveStation.isGyroReset,
                driveStation.isTargetLocked,
                driveStation.isRedAlliance
        );

        //All logs grouped together
        chassis.log(telemetry);
        launcher.log(telemetry);
        intake.log(telemetry);
        limelight.log(telemetry);

        //Must come after logs and is best if last thing in TeleOp function
        telemetry.update();
    }
    public MecanumDrive getMecanumDrive() {
        return chassis.getMecanumDrive();
    }
    public Action autonUpdate(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.update();
                intake.update();
                limelight.update();
                chassis.setPose();
                return true; //never stop
            }
        };
    }
    public Action readyLaunch(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
                launcher.isSetForSpin = true;
                launcher.setDistance(0.0);//TODO get dist
                intake.intakePower = SubSystemConfigs.INTAKE_STOP;
                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                return false;
            }
        };
    }
    public Action launch(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
                launcher.isSetForSpin = true;
                launcher.setDistance(0.0);//TODO get dist
                intake.intakePower = SubSystemConfigs.INTAKE_MAX;
                intake.transportPower = SubSystemConfigs.TRANSPORT_MAX;
                return false;
            }
        };
    }
    public Action runIntake(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
                launcher.isSetForSpin = false;
                //launcher.setDistance(0.0);//no need to change target velocity or hood
                intake.intakePower = SubSystemConfigs.INTAKE_MAX;
                intake.transportPower = SubSystemConfigs.INTAKE_MAX;
                return false;
            }
        };
    }
}
