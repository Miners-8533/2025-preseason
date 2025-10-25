package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Limelight {
    private Limelight3A limelight;
    private GoBildaPinpointDriver imu;
    private LLResult result;
    private Pose3D botpose;
    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // Asks the LL for data 100 times / second
        limelight.pipelineSwitch(0); // Resets the pipeline to 0 in case it was last on something else
        limelight.start();

        imu = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void update() {
        limelight.updateRobotOrientation(imu.getHeading(AngleUnit.DEGREES));
        result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            botpose = result.getBotpose_MT2();
        }

    }

    public void log(Telemetry tele) {
        tele.addData("tx", result.getTx());
        tele.addData("ty", result.getTy());
        if (result != null && result.isValid()) {
            tele.addData("Botpose", botpose.toString());
        }
    }
}
