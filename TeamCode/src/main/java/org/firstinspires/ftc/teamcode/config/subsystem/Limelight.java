package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Limelight {
    public Limelight3A limelight;
    HardwareMap hwMap;
    Telemetry telemetry;
    public Limelight(HardwareMap hardwareMap, Telemetry telem){
        telemetry=telem;
        hwMap=hardwareMap;
        limelight = hwMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)


    }
    public void setPollRate(int Hz){
        limelight.setPollRateHz(Hz);
    }


    public void startLL() {
        limelight.start();
    }
    public void setPipeline(int pipeline){
        limelight.pipelineSwitch(pipeline);
    }

    public LLResult get_results(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }
        return result;
    }
}
