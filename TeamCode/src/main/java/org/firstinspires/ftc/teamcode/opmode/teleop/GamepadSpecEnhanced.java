package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_CLOSE;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.headingPIDFCoefficients;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "GamepadSpecEnhanced", group = "TeleOp")
public class GamepadSpecEnhanced extends LinearOpMode {

    private Follower follower;
    private PIDFController headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
    private Claw claw;
    private Arm arm;
    private Slide slide;

    // Teleop state: 0 = manual; 9, 10, 11 = autonomous segments; 1 = slide command active
    private int teleopPathState = 0;
    private int altPathState = 0;
    private Timer pathTimer;
    private Timer differenceTimer;
    private int counter = 0;
    private double yPlace = 62.5;

    // Autonomous paths for sequence
    private Path pickMore, placeSpecimen2, moveLeft;

    // Starting poses for teleop (if used)
    private final Pose parkPose = new Pose(0, 57, Math.toRadians(180));
    private final Pose bucketPose = new Pose(6.5, 131.5, Math.toRadians(315));

    // Drivetrain slow mode factor
    double slowModeFactor = 0.25;

    // Servo position variables
    double clawPosition = 1.0;
    double armPosition = 0.5;
    double wristPosition = 0.5;
    double draggerPosition = DRAGGER_CLOSE;
    boolean isYActive = false;

    int phase = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize IMU for field-centric (if needed)

        // Initialize follower and set starting pose
        follower = new Follower(hardwareMap);
        follower.setStartingPose(parkPose);

        // Initialize subsystems
        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        claw = new Claw(hardwareMap);
        // Set initial servo positions
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572); // CHANGE
        double headingCorrection;

        // Initialize timer and counters
        pathTimer = new Timer();
        differenceTimer = new Timer();
        counter = 0;
        yPlace = 65;
        // Build default autonomous paths
        buildPaths();

        waitForStart();
        if (isStopRequested()) return;
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            // --- Manual Drivetrain Control ---
            double targetForward = -gamepad1.left_stick_y;
            double targetStrafe = -gamepad1.left_stick_x;
            double targetTurn = -gamepad1.right_stick_x;
            if (gamepad1.right_trigger > 0) {
                targetForward *= slowModeFactor;
                targetStrafe *= slowModeFactor;
                targetTurn *= slowModeFactor;
            }
            // Allow teleop movement in both manual state (0) and slide command active state (1)
            if (teleopPathState == 0 || teleopPathState == 1 || teleopPathState == 9) {
                if (isYActive) {
                    double targetHeading = Math.toRadians(180);

                    double headingError = targetHeading - follower.getPose().getHeading();
                    headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

                    if (Math.abs(headingError) < Math.toRadians(0.5)) {
                        headingCorrection = 0;
                    } else {
                        headingPIDF.updateError(headingError);
                        headingCorrection = headingPIDF.runPIDF();
                    }



                        follower.setTeleOpMovementVectors(targetForward * 1,
                                targetStrafe * 1,
                                headingCorrection,
                                false);

                } else {
                    follower.setTeleOpMovementVectors(targetForward * 0.8,
                            targetStrafe * 0.8,
                            targetTurn,
                            false);
                }


            }
            follower.update();

            // --- Autonomous Sequence Triggering ---
            if (teleopPathState == 0 && gamepad2.a) {
                setPathState(9);
            }
            if (teleopPathState != 0 && gamepad1.b) {
                follower.breakFollowing();
                follower.startTeleopDrive();
                teleopPathState = 0;
            }

//            if ((teleopPathState == 0 && gamepad1.y) || teleopPathState == 1) {
//                follower.startTeleopDrive();
//            }

            // When gamepad1.x is pressed, execute the slide command and set state 14
            if (gamepad1.x) {
                setPathState(14);
            }

            if (gamepad1.y && differenceTimer.getElapsedTimeSeconds() > 0.5) {
                follower.setPose(new Pose(5.75, 3, follower.getPose().getHeading()));
                isYActive = !isYActive;
                differenceTimer.resetTimer();
            }

            if (Math.abs(gamepad2.left_stick_x) > 0 || Math.abs(gamepad2.left_stick_y) > 0) {
                setPathState(0);
            }

            // --- Autonomous Sequence State Machine ---
            switch (teleopPathState) {


                case 9:
                    // Case 11: After 0.5 sec, adjust yPlace, set arm, and build a new placeSpecimen2 path.


                    arm.setPosition(-2600, 0.75);

                    if (arm.sendPosition() < -1500) {
                        slide.setPosition(-250, 0.5);
                        if (slide.sendPosition() < -200 && arm.sendPosition() < -2550) {
                            setPathState(0); // End autonomous sequence, return to manual.
                        }
                    }
                    setScorePosClaw();
                    break;
            case 1:
                // Remain in this state until the slide reaches near the target.
                if (slide.sendPosition() < -1550) {
                    setPathState(0);
                }
                break;
            default:
                // Manual mode (teleopPathState == 0)
                break;
        }


            // --- Mechanism Controls (Arm, Slide, Claw) ---
            // Allow manual control when in state 0 (fully manual) or state 1 (slide command active)
            if (teleopPathState == 0 || teleopPathState == 1) {
                if (teleopPathState == 0) {
                    if (gamepad2.right_stick_y != 0.0) {
                        if (arm.sendPosition() > -2750 || gamepad2.right_stick_y > 0.0)
                            arm.setPowerArm(gamepad2.right_stick_y);
                        else
                            arm.setPowerArm(0);
                    } else {
                        arm.setPowerArm(0);
                    }
                }

                if (teleopPathState == 0) {
                    if (gamepad2.left_stick_y != 0.0) {
                        if (slide.sendPosition() > -1800 || gamepad2.left_stick_y > 0.0 || arm.sendPosition() < -1500)
                            slide.setPowerSlide(gamepad2.left_stick_y);
                        else
                            slide.setPowerSlide(0);
                    } else {
                        slide.setPowerSlide(0.001);
                    }
                }

                if (slide.sendPosition() < -1750 && arm.sendPosition() > -1500) {
                    slide.setPosition(-1700, 1.0);
                } else {
                    slide.setModeEncoder();
                }
            }

            // Claw and servo controls (always active)
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
                wristPosition += 0.0075;
            }
            if (gamepad2.left_bumper) {
                wristPosition -= 0.0075;
            }
            if (gamepad2.left_trigger > 0) {
                armPosition += 0.0045;
            }
            if (gamepad2.right_trigger > 0) {
                armPosition -= 0.0045;
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
                wristPosition = 0.799;
            }

            if (gamepad1.left_bumper) {
                setLoadPosClaw();
            }
            if (gamepad1.left_trigger != 0) {
                setPopulatePosClaw();
            }
            if (gamepad1.right_bumper) {
                setScorePosClaw();
            }
            if (gamepad2.a) {
                slide.setPosition(-100, 1.0);
            }

            // Update claw servo positions
            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
            claw.setArmPosition(armPosition);
            claw.setDraggerPosition(draggerPosition);

            // Telemetry
            Pose p = follower.getPose();
            telemetry.addData("Teleop State", teleopPathState);
            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(p.getHeading()));
            telemetry.addData("Arm Pos", arm.sendPosition());
            telemetry.addData("Slide Pos", slide.sendPosition());
            telemetry.addData("Servo Arm Pos", armPosition);

            telemetry.update();
        }
    }

    // Builds default autonomous paths.
    private void buildPaths() {
        // Define initial pickMore path
        pickMore = new Path(new BezierLine(
                new Point(34.0, 65.0, Point.CARTESIAN),
                new Point(5.5, 26, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
        pickMore.setZeroPowerAccelerationMultiplier(1.25);

        // Define initial placeSpecimen2 path using current yPlace value
        placeSpecimen2 = new Path(new BezierLine(
                new Point(5.5, 26, Point.CARTESIAN),
                new Point(34, yPlace, Point.CARTESIAN)
        ));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
    }

    public void setPathState(int pState) {
        teleopPathState = pState;
        pathTimer.resetTimer();
    }

    public void setAltState(int pState) {
        altPathState = pState;
    }

    public void setLoadPosClaw() {
        clawPosition = 0.25;
        wristPosition = 0.85;
        armPosition = 0.681; // CHANGE
    }

    public void setScorePosClaw() {
        clawPosition = 1.0;
        wristPosition = 0.856;
        armPosition = 0.572; // CHANGE
    }

    public void setPopulatePosClaw() {
        clawPosition = 0.25;
        wristPosition = 0.853;
        armPosition = 0.627; // CHANGE
    }

}
