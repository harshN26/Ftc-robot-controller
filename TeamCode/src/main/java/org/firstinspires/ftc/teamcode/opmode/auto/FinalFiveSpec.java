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

@Autonomous(name="FinalFiveSpec")
@Disabled
public class FinalFiveSpec extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(180));
    private final Pose startPose = new Pose(0, 57, Math.toRadians(180));
    private final Pose chamberPose = new Pose(29, 60.5, Math.toRadians(180));
    private Claw claw;
    private Slide slide;
    private Arm arm;
    private int counter = 0;
    private double yPlace = 58.5;
    private PathChain hangSpecimen1, goToSamples;
    private Path goBack, pickSpecimen, placeSpecimen, placeSpecimen2, pickMore;

    public void buildPaths() {
        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose))) // First path
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())// Heading interpolation
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        goBack = new Path(
                new BezierLine(
                        new Point(follower.getPose())
                        ,new Point(19, 59, Point.CARTESIAN)));
        goBack.setConstantHeadingInterpolation(chamberPose.getHeading());
        goBack.setZeroPowerAccelerationMultiplier(0.9);
//        goBack.setZeroPowerAccelerationMultiplier(1.25);

        pickSpecimen = new Path(new BezierLine(new Point(follower.getPose()), new Point(observationPose)));
        pickSpecimen.setPathEndVelocityConstraint(0);

        placeSpecimen = new Path(new BezierCurve(new Point(5.75, 3, Point.CARTESIAN),
                new Point(20, 10, Point.CARTESIAN),
                new Point(0, 35, Point.CARTESIAN),
                new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen.setPathEndVelocityConstraint(5);
        placeSpecimen.setZeroPowerAccelerationMultiplier(5);

        placeSpecimen2 = new Path(new BezierLine(new Point(5.5, 26, Point.CARTESIAN), new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
//        placeSpecimen2.setZeroPowerAccelerationMultiplier(1.25);

        pickMore = new Path( new BezierLine(
                new Point(34.000, 65.000, Point.CARTESIAN),
//                new Point(13.000, 63.250, Point.CARTESIAN),
//                new Point(34.000, 32.500, Point.CARTESIAN),
                new Point(5.5, 26, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
        pickMore.setZeroPowerAccelerationMultiplier(2);
//        pickMore.setZeroPowerAccelerationMultiplier(1.25);

        goToSamples = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(19.000, 59.000, Point.CARTESIAN),
                        new Point(25, 17, Point.CARTESIAN),
                        new Point(40, 39, Point.CARTESIAN),
                        new Point(46, 22.5, Point.CARTESIAN)
                ))
                .setZeroPowerAccelerationMultiplier(5)
                .setPathEndVelocityConstraint(5)
                .setConstantHeadingInterpolation(Math.toRadians(180)) // Heading interpolation
                .addPath(new BezierCurve( // First path - straight line
                        new Point(46, 22.5, Point.CARTESIAN),
                        new Point(25, 18, Point.CARTESIAN),
                        new Point(17.5, 18, Point.CARTESIAN)
                ))
                .setZeroPowerAccelerationMultiplier(5)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(17.5, 18, Point.CARTESIAN),
                        new Point(44, 23, Point.CARTESIAN),
                        new Point(47.5, 10, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)
                .setZeroPowerAccelerationMultiplier(5)

                .addPath(new BezierLine( // First path - Bezier curve
                        new Point(47.5, 10, Point.CARTESIAN),
                        new Point(17.5, 10, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setPathEndVelocityConstraint(5)
                .setZeroPowerAccelerationMultiplier(5)

                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(17.5, 10, Point.CARTESIAN),
                        new Point(26, 18, Point.CARTESIAN),
                        new Point(37, 4.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.5)


                .addPath(new BezierLine( // First path - Bezier curve
                        new Point(37, 4.5, Point.CARTESIAN),
                        new Point(5.75, 2.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(3.5)
                // third specimen
                .build();




        // First path.
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Logic for this case without paths
                follower.followPath(hangSpecimen1, true);
                follower.setMaxPower(1.0);
                arm.setPosition(-2600, 1.0);
                slide.setPosition(-100, 1.0);
                setPathState(1); // Set pathState to 1 (you can modify this based on desired flow)
                break;
            case 1:
                if (!follower.isBusy() && arm.sendPosition() < -2500) {
                    slide.setPosition(-850, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(2);
                } else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy() && arm.sendPosition() < -2500) {
                    follower.breakFollowing();
                    slide.setPosition(-850, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(2);
                }
                break;
            case 2:
                if (slide.sendPosition() < -800) {
                    follower.setMaxPower(1.0);
                    claw.setClawPosition(0.5);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.1) {
//                    setClawLoad();
                    slide.setPosition(0, 1.0);

                    follower.followPath(goBack);
                    setPathState(4);
                }
                break;
            case 4:

                if (!follower.isBusy()) {
                    arm.setPosition(250, 1.0);

                    follower.setMaxPower(1);
                    follower.followPath(goToSamples);
                    setPathState(5);
                };
                break;
            case 5:
                if (follower.getPose().getY() < 6) {
                    slide.resetSlide();
                    setClawLoad();
                    setPathState(6);

                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    setPathState(7);
                }
                else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                    follower.breakFollowing();
                    claw.setClawPosition(1.0);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 0.1) {

//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {

                    arm.setPosition(-2550, 0.75);
                    if (!follower.isBusy()) {
                        follower.followPath(placeSpecimen, true);
                    }
                    setClawPut();
                    if (follower.getPose().getX() > 10) {
                        slide.setPosition(-300, 0.5);
                        setPathState(8);


                    }
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    slide.setPosition(-900, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(18);
                } else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                    follower.breakFollowing();
                    slide.setPosition(-900, 1.0);
                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() > 0.1 && counter < 3 && slide.sendPosition() < -850) {
                    arm.setPosition(250, 1);
                    slide.setPosition(0, 1);

                    setClawLoad();
                    counter++;
                    follower.followPath(pickMore);
                    setPathState(10);
                }
                break;
            case 12:

                if (!follower.isBusy()) {
                    slide.setPosition(-850, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(9);
                } else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                    follower.breakFollowing();
                    slide.setPosition(-850, 1.0);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 0.25 && counter < 4 && slide.sendPosition() < -800) {
                    arm.setPosition(250, 1);
                    slide.setPosition(0, 1);

                    setClawLoad();
                    counter++;
                    follower.followPath(pickMore);
                    setPathState(10);
                }
                break;
            case 10:

                if (!follower.isBusy()) {
                    slide.resetSlide();
                    claw.setClawPosition(1.0);
                    setPathState(11);
                }
                else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                    slide.resetSlide();
                    follower.breakFollowing();
                    claw.setClawPosition(1.0);
                    setPathState(11);
                }
                break;
            case 11:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {

                        if (counter == 1) {
                            yPlace = yPlace - 1;
                        } else {
                            yPlace = yPlace - 1;
                        }
                    }

//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(-2550, 0.75);
                    placeSpecimen2 = new Path(new BezierLine(new Point(4.75, 25, Point.CARTESIAN), new Point(28.5, yPlace, Point.CARTESIAN)));
                    placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
                    placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
                    if (!follower.isBusy()) {
                        follower.followPath(placeSpecimen2, true);
                    }
                    if (follower.getPose().getX() > 10) {
                        slide.setPosition(-200, 0.5);
                        if (slide.sendPosition() < -150) {
                            setPathState(8);
                        }


                    }

                    setClawPut();

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
//        arm.resetArm();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);

        buildPaths(); // No paths to build

        waitForStart(); // ^^^^^^^^^^^^^^ on INIT

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
//            follower.getDashboardPoseTracker();
            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void setClawLoad() {
        claw.setClawPosition(0.25);
        claw.setWristPosition(0.853);
        claw.setArmPosition(0.635);
    }

    public void setClawPut() {
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572); // 0.475

    }


}
