package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This teleop showcases movement and control of arm, slide, and other motors using PedroPathing.
 * The robot is controlled with the gamepad, where the left and right joysticks control robot movement.
 *
 * @version 2.0, 11/28/2024
 */

@TeleOp(name = "method")

public class GamepadSpec extends LinearOpMode {

    private Follower follower;
    private boolean isArmDoing = false;

    double clawPosition = 1;
    double armPosition = 0.5;
    double wristPosition = 0.5;
    double draggerPosition = (0.76);

    private final Pose observationPose = new Pose(5, 30, Math.toRadians(180));
    private final Pose parkPose = new Pose(0, 57, Math.toRadians(180));

    private final Pose bucketPose = new Pose(6.5, 131.5, Math.toRadians(315));


    private Path goPark, placeSpecimen2;


    Claw claw;
    Arm arm;
    Slide slide;

    // Variables to store current motor power
    double currentForwardPower = 0;
    double currentStrafePower = 0;
    double currentTurnPower = 0;

    // Acceleration rate
    double accelerationStep = 0.1;  // Adjust this value for faster or slower acceleration

    // Slow mode factor
    double slowModeFactor = 0.25;  // This controls how slow the robot moves when the right trigger is pressed

    @Override
    public void runOpMode() throws InterruptedException {



        follower = new Follower(hardwareMap);
        follower.setStartingPose(parkPose);

        waitForStart();

        if (isStopRequested()) return;
        follower.startTeleopDrive();

        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        claw = new Claw(hardwareMap);

        placeSpecimen2 = new Path(new BezierLine(new Point(5.5, 26, Point.CARTESIAN), new Point(28.5, 60, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(2);


        while (opModeIsActive()) {
            double targetForwardPower = -gamepad1.left_stick_y;
            double targetStrafePower = -gamepad1.left_stick_x;
            double targetTurnPower = -gamepad1.right_stick_x;

            // Check if right trigger is pressed to activate slow mode
            if (gamepad1.right_trigger > 0) {
                targetForwardPower *= slowModeFactor;
                targetStrafePower *= slowModeFactor;
                targetTurnPower *= slowModeFactor;
            }

            // Apply smooth power change
//            currentForwardPower = smoothPowerChange(currentForwardPower, targetForwardPower);
//            currentStrafePower = smoothPowerChange(currentStrafePower, targetStrafePower);
//            currentTurnPower = smoothPowerChange(currentTurnPower, targetTurnPower);

            follower.setTeleOpMovementVectors(
                    targetForwardPower * 0.8,
                    targetStrafePower * 0.8,
                    targetTurnPower,
                    false
            );
            follower.update();

            goPark = new Path(new BezierCurve(
                    new Point(follower.getPose()),
                    new Point(bucketPose))
            );
            goPark.setLinearHeadingInterpolation(follower.getPose().getHeading(), bucketPose.getHeading());


            // THIS IS THE ALREADY WORKING CODE BELOW
            if (gamepad2.right_stick_y != 0.0) {
                if (arm.sendPosition() > -2750 || gamepad2.right_stick_y > 0.0)
                    arm.setPowerArm(gamepad2.right_stick_y);
                else arm.setPowerArm(0);


            } else {
                arm.setPowerArm(0);

            }

            // SLIDE WORKING CODE BELOW
            if (gamepad2.left_stick_y != 0.0) {
                if (slide.sendPosition() > -1800 || gamepad2.left_stick_y > 0.0 || arm.sendPosition() < -1500){
                    slide.setPowerSlide(gamepad2.left_stick_y);
                } else {
                    slide.setPowerSlide(0);
                }
            } else {
                slide.setPowerSlide(0.0001);
            }

            if (slide.sendPosition() < -1750 && arm.sendPosition() > -1500) {
                slide.setPosition(-1700, 1.0);
            } else {
                slide.setModeEncoder();
            }

            if (gamepad2.y) {
                clawPosition = 0.25;
            }
            if (gamepad2.x) {
                clawPosition = 1.0;
            }

            if (gamepad2.b) {
                slide.resetSlide();
            }

            if (gamepad2.right_bumper) {
                wristPosition += 0.01;
            }
            if (gamepad2.left_bumper) {
                wristPosition -= 0.01;
            }

            if (gamepad2.left_trigger > 0) {
                draggerPosition += 0.0015;
            }
            if (gamepad2.right_trigger > 0) {
                draggerPosition -= 0.0015;
            }
            if (armPosition > 0.6845) {
                armPosition = 0.6825;
            }
            if (armPosition < 0.573) {
                armPosition = 0.572;
            }
            if (wristPosition > 1.0) {
                wristPosition = 1.0;
            }

            if (wristPosition < 0.797) {
                wristPosition = 0.799;          }

            if (gamepad1.left_bumper) {
                setLoadPosClaw();
            }

            if (gamepad1.left_trigger != 0) {
                setPopulatePosClaw();
            }

            if (gamepad1.right_bumper) {
                setScorePosClaw();
            }


//            if (gamepad2.right_bumper) {
//                wristPosition -= 0.015;
//            }

//            if (slide.sendPosition() > 150) {
//                slide.setPosition(0, 1.0);
//            }

            if (gamepad1.y) {
                follower.breakFollowing();
                follower.startTeleopDrive();
            }



            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
//            claw.setArmPosition(armPosition);
            claw.setDraggerPosition(draggerPosition);

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Arm Motor Position", arm.sendPosition());
            telemetry.addData("Slide Motor Position", slide.sendPosition());
            telemetry.addData("Claw Position", clawPosition);
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("dagger Position", draggerPosition);
            telemetry.update();
        }
    }

    public void setLoadPosClaw() {
        clawPosition = (0.25);
        wristPosition = (0.85);
        armPosition = (0.6825);

    }

    public void setScorePosClaw() {
        clawPosition=(1.0);
        wristPosition=(0.856);
        armPosition=(0.572);
    }

    public void setPopulatePosClaw() {
        clawPosition = (0.25);
        wristPosition = (0.853);
        armPosition = (0.635);
    }
}
