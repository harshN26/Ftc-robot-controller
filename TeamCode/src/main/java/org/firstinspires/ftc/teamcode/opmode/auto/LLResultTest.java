package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.Limelight;
@Autonomous(name="LLDetectionTest")

public class LLResultTest extends LinearOpMode {
    Limelight ll;
    LLResult llResult;
    public void runOpMode(){
        Limelight ll=new Limelight(hardwareMap, telemetry);
        ll.startLL();
        ll.setPipeline(2);
        waitForStart();
        while(opModeIsActive()){
            llResult=ll.get_results();
            telemetry.update();
        }

    }
}
