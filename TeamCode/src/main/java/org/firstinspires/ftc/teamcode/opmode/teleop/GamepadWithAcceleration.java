package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This teleop showcases movement and control of arm, slide, and other motors using PedroPathing.
 * The robot is controlled with the gamepad, where the left and right joysticks control robot movement.
 *
 * @version 2.0, 11/28/2024
 */
@Disabled()
@TeleOp(name = "GamepadWithAcceleration")
public class GamepadWithAcceleration extends LinearOpMode {

    private Follower follower;
    private boolean isArmDoing = false;

    double clawPosition = 0;
    double armPosition = 0.5;
    double wristPosition = 0.5;
    boolean running = false;

    private final Pose observationPose = new Pose(5, 30, Math.toRadians(180));
    private final Pose parkPose = new Pose (52, 100, Math.toRadians(270));

    private final Pose bucketPose = new Pose(6.5, 131.5, Math.toRadians(315));

    private Path goPark;


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

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParams);

        follower = new Follower(hardwareMap);
        follower.setStartingPose(parkPose);

        waitForStart();

        if (isStopRequested()) return;
        follower.startTeleopDrive();

        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        claw = new Claw(hardwareMap);


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
                if (gamepad2.a) {
                    arm.setPowerArm(gamepad2.right_stick_y / 3);
                } else {
                    arm.setPowerArm(gamepad2.right_stick_y);
                }
            } else {
                if (!running) {arm.setPowerArm(0);}
            }


//            int armTargetPosition = arm.sendPosition() + (int) (gamepad2.right_stick_y * 200);  // Adjust the factor (100) to control speed/precision
//            double armSpeed = (gamepad2.right_trigger > 0) ? 0.33 : 1.0;  // Slow down if the right trigger is pressed
//
//            arm.setPosition(armTargetPosition, armSpeed);  // Move arm to the target position with speed



//            int slideTargetPosition = slide.sendPosition() + (int) (gamepad2.left_stick_y * 200);  // Adjust the factor (100) to control speed/precision
//            double slideSpeed = (gamepad2.left_trigger > 0) ? 0.33 : 1.0;  // Slow down if the left trigger is pressed
//
//            slide.setPosition(slideTargetPosition, slideSpeed);  // Move slide to the target position with speed


            // SLIDE WORKING CODE BELOW
            if (gamepad2.left_stick_y != 0.0) {
                if (gamepad2.b) {
                    slide.setPowerSlide(gamepad2.left_stick_y / 3);
                } else {
                    slide.setPowerSlide(gamepad2.left_stick_y);
                }
            } else {
                if (!running) {slide.setPowerSlide(0); }
            }

            if (gamepad2.y) {
                clawPosition = 0.7;
            }
            if (gamepad2.x) {
                clawPosition = 1.0;
            }

            if (gamepad2.left_bumper) {
                wristPosition += 0.01;
            }
            if (gamepad2.right_bumper) {
                wristPosition -= 0.01;
            }

            if (gamepad2.left_trigger > 0) {
                armPosition += 0.003;
            }
            if (gamepad2.right_trigger > 0) {
                armPosition -= 0.003;
            }

            if (gamepad1.left_bumper) {
                setLoadPosClaw();
            }

            if (gamepad1.right_bumper) {
                setScorePosClaw();
            }

            if (slide.sendPosition() > 150) {
                slide.setPosition(0, 1.0);
            }

            if (gamepad1.x) {
                running = true;
//                if (follower.getPose().getX() > 30 && follower.getPose().getY() < 120)
                follower.followPath(goPark);
                arm.setPosition(-3300, 1.0);
            }

            if (running && arm.sendPosition() < -2000 && follower.getPose().getX() < 30) {
            }

            if (running && arm.sendPosition() < -3200 && slide.sendPosition() < -2950) {running = false;}

            if (slide.sendPosition() < -2000 && running) {
                setScorePosClaw();
            }

            if (gamepad2.a) {
                running = false;
            }

            if (gamepad1.y) {
                follower.breakFollowing();
                follower.startTeleopDrive();
            }

            if (running & !follower.isBusy()) {
                follower.startTeleopDrive();
            }

            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
            claw.setArmPosition(armPosition);

            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Arm Motor Position", arm.sendPosition());
            telemetry.addData("Slide Motor Position", slide.sendPosition());
            telemetry.addData("Claw Position", clawPosition);
            telemetry.addData("Wrist Position", wristPosition);
            telemetry.addData("Arm Position", armPosition);
            telemetry.update();
        }
    }

    private double smoothPowerChange(double current, double target) {
        if (current < target) {
            current += accelerationStep;
            if (current > target) {
                current = target;
            }
        } else if (current > target) {
            current -= accelerationStep;
            if (current < target) {
                current = target;
            }
        }
        return current;
    }

    public void setLoadPosClaw() {
        clawPosition = (0.5);
        wristPosition = (0.15);
        armPosition = (0.56); // was 0.4

    }

    public void setScorePosClaw() {
        clawPosition=(1.0);
        wristPosition=(0.8); // was 0.8
        armPosition=(0.505);
    }
}
