package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.ArmSlide;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
@Autonomous(name = "ArmResetBasket")
public class GoToStartBasket extends LinearOpMode {

    Arm arm;
    Slide slide;

    ArmSlide armSlide;
    boolean flag = true;
    @Override
    public void runOpMode() throws InterruptedException {

        // Set arm to run to position mode without resetting encoder
        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        armSlide = new ArmSlide(arm, slide);
        waitForStart();



        double[] positions = new double[2];
        positions = armSlide.returnPos(1, 42);


        // Set the target position for the arm (position 0)



        // Adjust power as necessary

        int armPosition = arm.sendPosition();
        // Wait until the arm reaches the target position
        while (opModeIsActive()) {

            if  (flag) {
                arm.resetArm();
                arm.setPosition(200, 0.25);
                flag = false;
            }

            if (arm.sendPosition() > 200) {
                arm.resetArm();
                arm.setPowerArm(0);
            }
            telemetry.addData("Arm Position", armPosition);
            telemetry.addData("Arm Target", positions[0]);
            telemetry.addData("Slide Target", positions[1]);
            telemetry.update();
        }

        // Once the arm reaches the target, stop it
    }
}
