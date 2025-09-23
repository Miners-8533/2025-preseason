package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Flywheel {
    private MotorWithEncoderAndVelocityController fly_motor;
    public Flywheel(HardwareMap hardwareMap) {
        MotorWithEncoderAndVelocityController.Config config = new MotorWithEncoderAndVelocityController.Config(
                "fly_motor",
                new PIDFCoefficients(1.0, 1.0, 0.0, 0.0),
                0.0,
                DcMotorSimple.Direction.FORWARD,
                100,
                false
        );
        fly_motor = new MotorWithEncoderAndVelocityController(hardwareMap, config);
    }

    public void update() {
        fly_motor.update();
    }

    public void log(Telemetry tele) {
        fly_motor.log(tele);
    }

}
