package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Robot {
    private Chassis chassis;
    private Launcher launcher;
    private DriveStation driveStation;
    private Limelight limelight;
    private Intake intake;
    private boolean isRedAlliance;
    private RobotState robotState;
    private ElapsedTime time;
    private enum RobotState {
        READY,
        INTAKE_PREP,
        INTAKING,
        INTAKE_FINISH,
        TARGET_LOCK_PREP,
        TARGET_LOCK,
        LAUNCHING
    }
    public Robot(HardwareMap hardwareMap, Gamepad driverController, Gamepad operatorController, Pose2d initialPose) {
        chassis = new Chassis(hardwareMap, initialPose);
        driveStation = new DriveStation(driverController, operatorController);
        launcher = new Launcher(hardwareMap);
        intake = new Intake(hardwareMap);
        limelight = new Limelight(hardwareMap);
        this.isRedAlliance = false;
        robotState = RobotState.INTAKE_PREP;
        time = new ElapsedTime();
    }
    public void updateTeleOp(Telemetry telemetry) {
        //get fresh inputs first
        driveStation.update();

        double desiredForward = driveStation.forward;
        double desiredStrafe = driveStation.strafe;
        double desiredRotation = driveStation.rotation;

        /* Desired controls are to always intake unless target locking (with a) and launch with
         * right bumper. When intaking starts the intake can run immediately and flywheel stop, but
         * the transport needs to wait for the gate to close. On entering target lock the transport
         * needs to backout slightly. When launching from the target lock state the transport has to
         * wait for the gate to open. When launching the intake should always run and the transport
         * depending on flywheel velocity. Outake should override intake and transport behavior.
         */

        switch(robotState) {//every state needs to set intake and transport b/c override
            case INTAKE_PREP:
                intake.intakePower = SubSystemConfigs.INTAKE_RUN;
                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
                robotState = RobotState.INTAKING;
                launcher.isSetForSpin = false;
                time.reset();
                break;
            case INTAKING:
                intake.intakePower = SubSystemConfigs.INTAKE_RUN;
                if(time.seconds() < SubSystemConfigs.STOP_DELAY) {
                    intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                } else {
                    intake.transportPower = SubSystemConfigs.TRANSPORT_INTAKE;
                }
                if(driveStation.isTargetLocked) {
                    robotState = RobotState.INTAKE_FINISH;
                    time.reset();
                }
                break;
            case INTAKE_FINISH:
                intake.intakePower = SubSystemConfigs.INTAKE_RUN;
                intake.transportPower = SubSystemConfigs.TRANSPORT_BACKOUT;
                if(time.seconds() >= SubSystemConfigs.TRANSPORT_BACKOUT_TIME) {
                    robotState = RobotState.TARGET_LOCK_PREP;
                    time.reset();
                }
                break;
            case TARGET_LOCK_PREP:
                intake.intakePower = SubSystemConfigs.INTAKE_STOP;
                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
                launcher.isSetForSpin = true;
                launcher.isFirstLaunch = true;
                if(time.seconds() >= SubSystemConfigs.STOP_DELAY) {
                    robotState = RobotState.TARGET_LOCK;
                }
                break;
            case TARGET_LOCK:
                if (driveStation.isLaunching) {
                    //run intake while shooting to make sure artifacts move through transport
                    intake.intakePower = SubSystemConfigs.INTAKE_MAX;
                    if (launcher.isVelocityGood()) {
                        intake.transportPower = SubSystemConfigs.TRANSPORT_MAX;
                    } else {
                        //Hold feed until we are up to flywheel speed
                        intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                    }
                } else {
                    intake.intakePower = SubSystemConfigs.INTAKE_STOP;
                    intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                }
                if(!driveStation.isTargetLocked) {
                    robotState = RobotState.INTAKE_PREP;
                }
                break;
            default: //For safety
                intake.intakePower = SubSystemConfigs.INTAKE_STOP;
                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
                launcher.isSetForSpin = false;
                break;
        }

        // State independent
        if(driveStation.isFieldOriented) {
            launcher.setDistance(chassis.targetDist);
        } else {
            launcher.setDistance(70.0);// on line near point of triangle
        }

        // State independent overrides
        if(driveStation.isOuttaking) {
            intake.intakePower = SubSystemConfigs.INTAKE_OUTAKE;
            intake.transportPower = SubSystemConfigs.TRANSPORT_OUTAKE;
        }

        //All updates grouped together (except driveStation)
        launcher.update();
        intake.update();
        limelight.update();
        chassis.update(
                desiredForward,
                desiredStrafe,
                desiredRotation,
                driveStation.targetLockScrub,
                driveStation.isFieldOriented,
                driveStation.isTargetLocked,
                PoseStorage.isRedAllicance
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
                launcher.autonSet(0.57, 760.0);
                intake.intakePower = SubSystemConfigs.INTAKE_STOP;
                intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                return false;
            }
        };
    }
    public Action launch(){
        return new Action(){
            private boolean firstRun = true;
            private ElapsedTime timeOut = new ElapsedTime();
            private double target = Double.MAX_VALUE;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(firstRun) {
                    firstRun = false;
                    launcher.isFirstLaunch = true;
                    launcher.autoBallCount = 0;
                    launcher.stopTarget = SubSystemConfigs.STOP_OPEN;
                    launcher.isSetForSpin = true;
                    launcher.autonSet(0.57, 760.0);
                    intake.intakePower = SubSystemConfigs.INTAKE_MAX;
                    target = timeOut.seconds() + 2.25;
                }
                launcher.autonLog(packet);
                if(launcher.isVelocityGoodAuton()) {
                    intake.transportPower = SubSystemConfigs.TRANSPORT_MAX;
                } else {
                    intake.transportPower = SubSystemConfigs.TRANSPORT_STOP;
                }
                boolean notDone = (timeOut.seconds() < target) && (launcher.autoBallCount < 3);
                return notDone;
            }
        };
    }
    public Action runIntake(){
        return new Action(){
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.stopTarget = SubSystemConfigs.STOP_LOCK;
                launcher.isSetForSpin = false;
                launcher.autonSet(0.58, 0.0);
                intake.intakePower = SubSystemConfigs.INTAKE_MAX;
                intake.transportPower = SubSystemConfigs.INTAKE_MAX;
                return false;
            }
        };
    }
    public Action targetLock(){//must not be run at same time as a trajectory action
        return new Action(){
            private ElapsedTime timeOut = new ElapsedTime();
            private boolean firstRun = true;
            private MecanumDrive drive = chassis.getMecanumDrive();
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if(firstRun) {
                    firstRun = false;
                    timeOut.reset();
                }

                double currentHeading = drive.localizer.getPose().heading.toDouble();
                double rotation = chassis.calcTargetLock(isRedAlliance, currentHeading);
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0,0),//No translation
                                rotation
                        )
                );
                double headingError = chassis.targetHeading - currentHeading;
                return ((timeOut.seconds() < SubSystemConfigs.TARGET_LOCK_TIMEOUT) ||
                        (headingError < Math.toRadians(SubSystemConfigs.HEADING_THRESHOLD)));
            }
        };
    }
}
