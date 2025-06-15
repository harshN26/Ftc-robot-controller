package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_CLOSE;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_OPEN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="StatesFiveSpec")
public class HybridFiveSpec extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(180));
    private final Pose startPose = new Pose(0, 57, Math.toRadians(180));
    private final Pose chamberPose = new Pose(29, 65, Math.toRadians(180)); // was 65
    private Claw claw;
    private Slide slide;
    private Arm arm;
    private int counter = 0;
    private double yPlace = 64;
    // These paths remain unchanged
    private PathChain hangSpecimen1, goToSample3, transferSample3;
    private Path goBack, pickSpecimen, placeSpecimen, placeSpecimen2, pickMore;
    // New individual paths for the original goToSamples composite
    private Path goToSamplesPart1, goToSamplesPart2, goToSamplesPart3, goToSamplesPart4;

    public void buildPaths() {

        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(3)
                .build();

        goBack = new Path(
                new BezierLine(
                        new Point(follower.getPose()),
                        new Point(19, 59, Point.CARTESIAN)));
        goBack.setConstantHeadingInterpolation(chamberPose.getHeading());
        goBack.setZeroPowerAccelerationMultiplier(0.5);

        pickSpecimen = new Path(new BezierLine(new Point(follower.getPose()), new Point(observationPose)));
        pickSpecimen.setPathEndVelocityConstraint(0);
        pickSpecimen.setZeroPowerAccelerationMultiplier(0.75);

        placeSpecimen = new Path(new BezierCurve(new Point(5.75, 3, Point.CARTESIAN),
                new Point(20, 10, Point.CARTESIAN),
                new Point(0, 35, Point.CARTESIAN),
                new Point(29.5, yPlace, Point.CARTESIAN)));
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen.setPathEndVelocityConstraint(0);
        placeSpecimen.setZeroPowerAccelerationMultiplier(6);

        placeSpecimen2 = new Path(new BezierLine(new Point(5.5, 26, Point.CARTESIAN),
                new Point(29.5, yPlace, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(6);

        pickMore = new Path(new BezierLine(
                new Point(34.000, 65.000, Point.CARTESIAN),
                new Point(5.5, 26, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
        pickMore.setZeroPowerAccelerationMultiplier(1.5);

        // Break down the original goToSamples path into four separate segments:
        goToSamplesPart1 = new Path(new BezierCurve(
                new Point(19.000, 59.000, Point.CARTESIAN),
                new Point(25, 12, Point.CARTESIAN),
                new Point(40, 39, Point.CARTESIAN),
                new Point(46, 20.5, Point.CARTESIAN)
        ));
        goToSamplesPart1.setConstantHeadingInterpolation(Math.toRadians(180));


        goToSamplesPart2 = new Path(new BezierCurve(
                new Point(46, 20.5, Point.CARTESIAN),
                new Point(25, 18, Point.CARTESIAN),
                new Point(12, 18, Point.CARTESIAN)
        ));
        goToSamplesPart2.setConstantHeadingInterpolation(Math.toRadians(180));

        goToSamplesPart3 = new Path(new BezierCurve(
                new Point(12, 18, Point.CARTESIAN),
                new Point(44, 23, Point.CARTESIAN),
                new Point(47.5, 10, Point.CARTESIAN)
        ));
        goToSamplesPart3.setConstantHeadingInterpolation(Math.toRadians(180));

        goToSamplesPart4 = new Path(new BezierLine(
                new Point(47.5, 10, Point.CARTESIAN),
                new Point(10, 10, Point.CARTESIAN)
        ));
        goToSamplesPart4.setConstantHeadingInterpolation(Math.toRadians(180));
        goToSamplesPart4.setZeroPowerAccelerationMultiplier(7);

        // goToSample3 remains as before
        goToSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(10, 10, Point.CARTESIAN),
                        new Point(15, 24, Point.CARTESIAN),
                        new Point(37.5, 11.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        transferSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(37.5, 11.5, Point.CARTESIAN),
                        new Point(20, 15.5, Point.CARTESIAN),
                        new Point(3.5, 26, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.75)
                .build();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(hangSpecimen1, true);
                arm.setPosition(-2600, 1.0);
                slide.setPosition(-100, 1.0);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && arm.sendPosition() < -2500) {
                    slide.setPosition(-850, 1.0);
                    setPathState(2);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                        follower.isBusy() && arm.sendPosition() < -2500) {
                    follower.breakFollowing();
                    slide.setPosition(-850, 1.0);
                    setPathState(2);
                }
                break;
            case 2:
                if (slide.sendPosition() < -800) {
                    claw.setClawPosition(0.25);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.2) {
                    slide.setPosition(0, 1.0);
                    arm.setPosition(150, 1.0);
                    while (slide.sendPosition() < -400) {

                    }
                    follower.followPath(goBack);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {

                    // Start with the first segment of goToSamples
                    follower.followPath(goToSamplesPart1);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(goToSamplesPart2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(goToSamplesPart3);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(goToSamplesPart4);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()) {
//                    claw.setDraggerPosition(0.73);
                    follower.followPath(goToSample3);
                    setPathState(11);
                }
                break;
            case 11:
                if ((follower.getPose().getX() > 36.5 && follower.getPose().getX() < 38.5) ||
                        (follower.getPose().getY() > 10.5 && follower.getPose().getY() < 12.5) &&
                                (follower.getPose().getHeading() > 265 && follower.getPose().getHeading() < 275)) {
                    claw.setDraggerPosition(DRAGGER_OPEN);
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(transferSample3);
                    setClawLoad();
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getPose().getY() > 22) {
                    claw.setDraggerPosition(DRAGGER_CLOSE);

                    setPathState(14);
                }
                break;
            case 14:
                if (!follower.isBusy()) {
                    slide.resetSlide();
                    claw.setClawPosition(1.0);
                    setPathState(15);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                        follower.isBusy()) {
                    slide.resetSlide();
                    follower.breakFollowing();
                    claw.setClawPosition(1.0);
                    setPathState(15);
                }
                break;
            case 15:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    if (!follower.isBusy()) {
                        yPlace = yPlace - 1;
                    }
                    arm.setPosition(-2550, 0.75);
                    placeSpecimen2 = new Path(new BezierLine(new Point(4.75, 25, Point.CARTESIAN),
                            new Point(28.5, yPlace, Point.CARTESIAN)));
                    placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
                    placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
                    if (!follower.isBusy()) {
                        follower.followPath(placeSpecimen2, true);
                    }
                    if (follower.getPose().getX() > 10) {
                        slide.setPosition(-200, 0.5);
                        if (slide.sendPosition() < -150) {
                            setPathState(16);
                        }
                    }
                    setClawPut();
                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    slide.setPosition(-900, 1.0);
                    setPathState(17);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                        follower.isBusy()) {
                    follower.breakFollowing();
                    slide.setPosition(-900, 1.0);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.1 && counter < 4 && slide.sendPosition() < -850) {
                    arm.setPosition(150, 1);
                    slide.setPosition(0, 1);
                    setClawLoad();
                    counter++;
                    follower.followPath(pickMore);
                    setPathState(14);
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
        claw.setDraggerPosition(DRAGGER_CLOSE);
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);
        claw.setArmPosition(0.572);

        buildPaths();

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
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
        claw.setArmPosition(0.627);
    }

    public void setClawPut() {
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);
    }
}
