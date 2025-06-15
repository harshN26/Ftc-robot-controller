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


@Disabled()
@Autonomous(name="TestAuto")
public class TestAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private final Pose observationPose = new Pose(5, 30, Math.toRadians(180));


    private final Pose startPose = new Pose(0, 50, Math.toRadians(180));
    private final Pose chamberPose = new Pose(28.5, 60, Math.toRadians(180));

    private PathChain hangSpecimen1, goToSamples,  takeSample3;
    private Path pickSpecimen, placeSpecimen, pickMore, placeSpecimen2;
    private Claw claw;
    private Slide slide;
    private Arm arm;
    private int yPlace = 65;

    public void buildPaths() {
        // Build the hangSpecimen1 PathChain
        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose))) // First path
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading()) // Heading interpolation
                .build();


        // Build the goToSamples PathChain
        goToSamples = follower.pathBuilder()
                .addPath(new BezierLine(new Point(chamberPose), new Point(20, 60, Point.CARTESIAN))) // First path
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(20.000, 60.000, Point.CARTESIAN),
                        new Point(8, 9.5, Point.CARTESIAN),
                        new Point(48, 47.000, Point.CARTESIAN),
                        new Point(52.000, 23.500, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .addPath(new BezierCurve( // First path - straight line
                        new Point(52, 23.5, Point.CARTESIAN),
                        new Point(59.000, 13, Point.CARTESIAN),
                        new Point(12, 20, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .addPath(new BezierCurve( // First path - straight line
                        new Point(12, 20, Point.CARTESIAN),
                        new Point(40, 38.000, Point.CARTESIAN),
                        new Point(44.000, 17.000, Point.CARTESIAN)
                )) // I WENT UP TO HERE ^^^
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .addPath(new BezierCurve( // First path - straight line
                        new Point(44.000, 17.000, Point.CARTESIAN),
                        new Point(65.000, 2.000, Point.CARTESIAN),
                        new Point(10.000, 10, Point.CARTESIAN)
                ))

                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                // third specimen
                .addPath(new BezierCurve( // First path - straight line
                        new Point(10.000, 10.000, Point.CARTESIAN),
                        new Point(30.000, 17.000, Point.CARTESIAN),
                        new Point(69, 7.8, Point.CARTESIAN),
                        new Point(45, 2, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))

//                .addPath(new BezierLine(
//                        new Point(51, 12, Point.CARTESIAN),
//                        new Point(51, 4, Point.CARTESIAN)
//                ))// Heading interpolation
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setPathEndVelocityConstraint(0)
                .addPath(new BezierLine(
                        new Point(45, 2, Point.CARTESIAN),
                        new Point(6.5, 0, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(0.1)
//                .addPath(new BezierLine(
//                        new Point(14, 3.75, Point.CARTESIAN),
//                        new Point(30, 30, Point.CARTESIAN)
//                ))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .setPathEndVelocityConstraint(0)
                .build();

        takeSample3 = follower.pathBuilder()

                .build();

        // Build the takeSampleOne PathChain


        pickSpecimen = new Path(new BezierLine(new Point(30, 30, Point.CARTESIAN), new Point(observationPose)));
        pickSpecimen.setPathEndVelocityConstraint(0);

        placeSpecimen2 = new Path(new BezierLine(new Point(observationPose), new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));

        placeSpecimen = new Path(new BezierLine(new Point(observationPose), new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(180));

        pickMore = new Path( new BezierCurve(
                new Point(34.000, 65.000, Point.CARTESIAN),
                new Point(13.000, 63.250, Point.CARTESIAN),
                new Point(34.000, 32.500, Point.CARTESIAN),
                new Point(5.500, 25, Point.CARTESIAN)
        ));
        pickMore.setPathEndVelocityConstraint(0);
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));

    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Follow the hangSpecimen1 PathChain
                follower.followPath(hangSpecimen1, true);  // holdEnd is true to allow corrections
//                arm.setPosition(-3750, 1.0); // this is the old arm position
                arm.setPosition(-2900, 1.0);
                slide.setPosition(-700, 1.0);
                setPathState(1);
                break;
            case 1:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(0, 1.0);
//                    claw.setArmPosition(0.85);
                    if (slide.sendPosition() > -75) {
                        claw.setClawPosition(0.3);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                if (!follower.isBusy()) {// Follow the goToSamples PathChain
                    arm.setPosition(275, 1.0);
                    claw.setClawPosition(0.3);
                    claw.setWristPosition(0.8);
                    claw.setArmPosition(0.68);
                    follower.followPath(goToSamples, false);
                    setPathState(4);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(pickSpecimen, false);
                    setPathState(4); // NOT RUNNING
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    setPathState(5);
                    break;
                }
            case 5:
                if(claw.getClawPosition() > 0.8 && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(placeSpecimen, true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(-2900, 0.5);
                    slide.setPosition(-700, 1.0);
                    claw.setClawPosition(1.0);
                    claw.setWristPosition(0.2);
                    claw.setArmPosition(1.0);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) { setPathState(7); } break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(0, 1.0);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(275, 1.0);
                    claw.setClawPosition(0.3);
                    claw.setWristPosition(0.8);
                    claw.setArmPosition(0.68);

                    follower.followPath(pickMore, false);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    setPathState(10);
                }
                break;
            case 10:
                if(claw.getClawPosition() > 0.8 && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    yPlace += 2;
                    follower.followPath(placeSpecimen, true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(-2900, 0.5);
                    slide.setPosition(-700, 1.0);

                    claw.setClawPosition(1.0);
                    claw.setWristPosition(0.2);
                    claw.setArmPosition(1.0);

                    setPathState(11);
                }
                break;
            case 11:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 0.3) { setPathState(12); } break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(0, 1.0);
                    setPathState(-1);
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
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.8);
        claw.setArmPosition(1.0);

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
            telemetry.update();
        }
    }
}
