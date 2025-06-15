package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="BasketAutoComp")
@Disabled()
public class BasketAutoComp extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 112, Math.toRadians(270));
    private final Pose bucketPose = new Pose(5.5, 124.5, Math.toRadians(315));
    private final Pose parkPose = new Pose (52, 100, Math.toRadians(270));

    private final Pose sample1Pose = new Pose(26, 117, Math.toRadians(0));

    private final Pose sample2Pose = new Pose(26, 128, Math.toRadians(0));
    private final Pose sample3Pose = new Pose(27, 128, Math.toRadians(45)); // 128

    private Path path, goToSample1, basketFromSample1, goToSample2, basketFromSample2, goToSample3, basketFromSample3, goPark;
    private Claw claw;
    private Slide slide;
    private Arm arm;

    public void buildPaths() {
        // Build the hangSpecimen1 PathChain
        path = new Path(new BezierLine(
                new Point(startPose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(bucketPose)));
        path.setLinearHeadingInterpolation(startPose.getHeading(), bucketPose.getHeading());
        path.setPathEndVelocityConstraint(0);
        path.setZeroPowerAccelerationMultiplier(1.5);

        goToSample1 = new Path(new BezierLine(
                new Point(bucketPose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(sample1Pose)));
        goToSample1.setLinearHeadingInterpolation(bucketPose.getHeading(), sample1Pose.getHeading());
        goToSample1.setZeroPowerAccelerationMultiplier(1);

        basketFromSample1 = new Path(new BezierLine(
                new Point(sample1Pose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(bucketPose)));
        basketFromSample1.setLinearHeadingInterpolation(sample1Pose.getHeading(), bucketPose.getHeading());
        basketFromSample1.setPathEndVelocityConstraint(0);
        basketFromSample1.setZeroPowerAccelerationMultiplier(1.5);

        goToSample2 = new Path(new BezierLine(
                new Point(bucketPose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(sample2Pose)));
        goToSample2.setLinearHeadingInterpolation(bucketPose.getHeading(), sample1Pose.getHeading());
        goToSample2.setZeroPowerAccelerationMultiplier(1);
        // Build the goToSamples PathChain

        basketFromSample2 = new Path(new BezierLine(
                new Point(sample2Pose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(bucketPose)));
        basketFromSample2.setLinearHeadingInterpolation(sample1Pose.getHeading(), bucketPose.getHeading());
        basketFromSample2.setPathEndVelocityConstraint(0);
        basketFromSample2.setZeroPowerAccelerationMultiplier(1.5);

        goToSample3 = new Path(new BezierLine(
                new Point(bucketPose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(sample3Pose)));
        goToSample3.setLinearHeadingInterpolation(bucketPose.getHeading(), sample3Pose.getHeading());
        goToSample3.setZeroPowerAccelerationMultiplier(1);

        basketFromSample3 = new Path(new BezierLine(
                new Point(sample2Pose),
//                new Point(44.000, 119.000, Point.CARTESIAN),
                new Point(bucketPose)));
        basketFromSample3.setLinearHeadingInterpolation(sample3Pose.getHeading(), bucketPose.getHeading());
        basketFromSample3.setPathEndVelocityConstraint(0);
        basketFromSample3.setZeroPowerAccelerationMultiplier(1.5);

        goPark = new Path(new BezierCurve(
                new Point(bucketPose),
                new Point(62.000, 114.000, Point.CARTESIAN),
                new Point(parkPose))
        );
        goPark.setLinearHeadingInterpolation(bucketPose.getHeading(), parkPose.getHeading());
        goPark.setPathEndVelocityConstraint(0);
        goPark.setZeroPowerAccelerationMultiplier(1.5);
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Follow the hangSpecimen1 PathChain
                follower.followPath(path, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                arm.setPosition(-2550, 0.7);
                slide.setPosition(-3200, 1.0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && arm.sendPosition() < -2500) {
                    claw.setArmPosition(0.505);
                    claw.setClawPosition(0.7);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.617); // 0.6
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(800, 1.0);
                    slide.setPosition(-200, 1.0);
                    if (arm.sendPosition() > -2000 && slide.sendPosition() > -1500) {
                        follower.followPath(goToSample1);
                        pickupBasket(0);
                        setPathState(4);
                    }
                }
                break;
            case 4:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.2) {
                    arm.setPosition(1050, 1.0);
                    if (arm.sendPosition() > 1000) {
                        claw.clawSetPosition(1.0);
                        setPathState(5);

                    }
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(basketFromSample1, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                    initBasket();
                    arm.setPosition(-2550, 0.7);
                    slide.setPosition(-3200, 1.0);

                    setPathState(6);
                }
                break;
            case 6:
                if(!follower.isBusy() && arm.sendPosition() < -2500) {
                    claw.setArmPosition(0.505);
                    claw.setClawPosition(0.7);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.617);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(800, 1.0);
                    slide.setPosition(-200, 1.0);
                    if (arm.sendPosition() > -2000 && slide.sendPosition() > -1500) {
                        follower.followPath(goToSample2);
                        pickupBasket(0);
                        setPathState(9);
                    }
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    arm.setPosition(1050, 1.0);
                    if (arm.sendPosition() > 1000) {
                        claw.clawSetPosition(1.0);
                        if (claw.getClawPosition() > 0.9) {
                            setPathState(10);
                        }
                    }
                }
                break;
            case 10:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(basketFromSample2, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                    initBasket();
                    arm.setPosition(-2550, 0.7);
                    slide.setPosition(-3200, 1.0);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy() && arm.sendPosition() < -2500) {
                    claw.setArmPosition(0.505);
                    claw.setClawPosition(0.7);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.617);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(800, 1.0);
                    slide.setPosition(-850, 1.0);
                    if (arm.sendPosition() > -2000 && slide.sendPosition() > -1500) {
                        follower.followPath(goToSample3);
                        pickupBasket(-0.3);
                        if (claw.getWristPosition() < 0.3) {
                            setPathState(14);
                        }
                    }
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    arm.setPosition(1050, 1.0);
                    if (arm.sendPosition() > 1000) {
                        claw.clawSetPosition(1.0);
                        if (claw.getClawPosition() > 0.9) {
                            setPathState(15);
                        }
                    }
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(basketFromSample3, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                    initBasket();
                    arm.setPosition(-2550, 1.0);
                    slide.setPosition(-3200, 1.0);
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy() && arm.sendPosition() < -2500) {
                    claw.setArmPosition(0.505); // 0.475
                    claw.setClawPosition(0.7);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.617);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(800, 1.0);
                    slide.setPosition(0, 1.0);
                    if (arm.sendPosition() > -2000 && slide.sendPosition() > -1000) {
                        follower.followPath(goPark);
                        arm.setPosition(-400, 1.0);
                        pickupBasket(0);
                        setPathState(19);
                    }
                }
                break;
            case 19:
                if (!follower.isBusy()) {
                    slide.setPosition(-800, 1.0);
                    arm.setPosition(-500, 1.0);
                }

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        claw = new Claw(hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        arm = new Arm("armMotor", hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        initBasket();
        claw.setClawPosition(1.0);

        buildPaths();

        waitForStart(); // ^^^^^^^^^^^^^^ on INIT

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            follower.getDashboardPoseTracker();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("claw", claw.getClawPosition());
            telemetry.addData("arm", claw.getArmPosition());
            telemetry.addData("wrist", claw.getWristPosition());
            telemetry.update();
        }
    }

    public void initBasket() {
        claw.setWristPosition(0.5);
        claw.setArmPosition(0.505);
    }

    public void pickupBasket(double wristOffset) {
        claw.setClawPosition(0.5);
        claw.setWristPosition(0.5  +wristOffset);
        claw.setArmPosition(0.617);
    }

}
