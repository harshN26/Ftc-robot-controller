package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.SpecArm;


@Config
@Autonomous(name = "SpecArmTest")
public class SpecArmTest extends LinearOpMode {

    private SpecArm specArm;

    public static int target=0;
    public static double p=0,i=0,d=0,f=0;

    @Override
    public void runOpMode() {
        // Initialize the SpecArm subsystem
        specArm = new SpecArm(hardwareMap,telemetry);

        // Reset encoder to establish zero position
        specArm.init_();


        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        // Initial PIDF tuning (optional, adjust via Dashboard)


        // Move arm to 50 units with 50% power


        // Monitor until position is reached or timeout



        while (opModeIsActive()) {
            specArm.loop_();
            specArm.set_target(target);
            telemetry.update();
        }

        // Stop the motor


        telemetry.addLine("Test complete.");
        telemetry.update();
    }
}
