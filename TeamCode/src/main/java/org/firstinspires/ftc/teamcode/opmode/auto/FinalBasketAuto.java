package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name="FinalBasketAuto")
public class FinalBasketAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(0, 112, Math.toRadians(270));
    private final Pose bucketPose = new Pose(4, 127, Math.toRadians(315));
    private final Pose bucketPose1 = new Pose(3, 129, Math.toRadians(315));
    private final Pose parkPose = new Pose (52, 99, Math.toRadians(270));

    private final Pose sample1Pose = new Pose(25, 121.5, Math.toRadians(0));

    private final Pose sample2Pose = new Pose(24, 132, Math.toRadians(0));
    private final Pose sample3Pose = new Pose(25, 130, Math.toRadians(60)); // 128

    private Path path, goToSample1, basketFromSample1, goToSample2, basketFromSample2, goToSample3, basketFromSample3, goPark;
    private Claw claw;
    private Slide slide;
    private Arm arm;

    private int ARM_POSITION_EXTEND = 2850;
    private int SLIDE_POSITION_EXTEND = 3000;

    private int ARM_POSITION_DOWN = 500;

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
                new Point(62.000, 112.000, Point.CARTESIAN),
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
                follower.followPath(path, false);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                arm.setPosition(-ARM_POSITION_EXTEND, 0.7);

                setPathState(2);
                break;
//            case 1:
//                if(!follower.isBusy()&&arm.sendPosition()<-500){
//                    Path runnable=new Path(new BezierCurve(new Point(bucketPose),new Point(bucketPose1)));
//                    runnable.setConstantHeadingInterpolation(315);
//                    follower.followPath(runnable);
//                    setPathState(2);
//                }
//                break;
            case 2:
                if (arm.sendPosition() < -100) {
                    slide.setPosition(-SLIDE_POSITION_EXTEND, 1.0);
                    setPathState(3);
                }
                break;

            case 3:
                if(!follower.isBusy() && slide.sendPosition() < -SLIDE_POSITION_EXTEND) {
                    arm.setPosition(-ARM_POSITION_EXTEND, 1.0);
                    claw.setClawPosition(0.25);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.6825);
                    // 0.6
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(-400, 1.0);


                    if ( slide.sendPosition() > -500) {
                        arm.setPosition(ARM_POSITION_DOWN, 1.0);
                        follower.followPath(goToSample1);
                        pickupBasket(0);
                        setPathState(6);
                    }
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5 && arm.sendPosition() < 550) {

                    claw.clawSetPosition(1.0);
                    setPathState(7);


                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {

                    follower.followPath(basketFromSample1, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                    initBasket();
                    arm.setPosition(-ARM_POSITION_EXTEND, 0.8);

                    setPathState(8);
                }
                break;
            case 8:
                if (arm.sendPosition() < 250 || !follower.isBusy()) {
                    slide.setPosition(-SLIDE_POSITION_EXTEND, 1.0);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && slide.sendPosition() < -2600 && arm.sendPosition() < -2350) {
                    arm.setPosition(-ARM_POSITION_EXTEND, 1.0);
                    claw.setClawPosition(0.25); // 0.6
                    setPathState(10);
                }
                break;
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.6825);
                    setPathState(11);
                }
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(-400, 1.0);


                    if ( slide.sendPosition() > -500) {
                        arm.setPosition(ARM_POSITION_DOWN, 1.0);
                        follower.followPath(goToSample2);
                        pickupBasket(0);
                        setPathState(12);
                    }
                }
                break;
            case 12:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {

                    claw.clawSetPosition(1.0);
                    setPathState(13);


                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 0.7) {
                    follower.followPath(basketFromSample2, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                    initBasket();
                    arm.setPosition(-ARM_POSITION_EXTEND, 0.7);

                    setPathState(14);
                }
                break;
            case 14:
                if (arm.sendPosition() < 150 || !follower.isBusy()) {
                    slide.setPosition(-SLIDE_POSITION_EXTEND, 1.0);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && slide.sendPosition() < -2600 && arm.sendPosition() < -2350) {
                    arm.setPosition(-ARM_POSITION_EXTEND, 1.0);
                    claw.setClawPosition(0.25);

                    // 0.6
                    setPathState(16);
                }
                break;
            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.6825);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(-800, 1.0);
                    claw.setWristPosition(0.881);


                    if ( slide.sendPosition() > -900) {
                        arm.setPosition(775, 1.0);
                        follower.followPath(goToSample3);
                        pickupBasket(0.031);
                        setPathState(18);
                    }
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && arm.sendPosition() > 450) {
                    follower.breakFollowing();
                    claw.clawSetPosition(1.0);
                    setPathState(19);


                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(basketFromSample3, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                    initBasket();
                    arm.setPosition(-ARM_POSITION_EXTEND, 0.6);

                    setPathState(20);
                }
                break;
            case 20:
                if (arm.sendPosition() < 250 || !follower.isBusy()) {
                    slide.setPosition(-SLIDE_POSITION_EXTEND, 1.0);
                    setPathState(21);
                }
                break;
            case 21:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && slide.sendPosition() < -SLIDE_POSITION_EXTEND && arm.sendPosition() < -2350) {
                    arm.setPosition(-ARM_POSITION_EXTEND, 1.0);
                    claw.setClawPosition(0.25); // 0.6
                    setPathState(22);
                }
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    claw.setArmPosition(0.6825);
                    setPathState(23);
                }
                break;
            case 23:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(-600, 1.0);
                    claw.setWristPosition(0.881);


                    if ( slide.sendPosition() > -700) {
                        arm.setPosition(-1000, 1.0);
                        follower.followPath(goPark);
                        initBasket();
                        setPathState(-1);
                    }
                }
                break;
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
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);
    }

    public void pickupBasket(double wristOffset) {
        claw.setClawPosition(0.25);
        claw.setWristPosition(0.85 + wristOffset);
        claw.setArmPosition(0.6825);
    }

}
