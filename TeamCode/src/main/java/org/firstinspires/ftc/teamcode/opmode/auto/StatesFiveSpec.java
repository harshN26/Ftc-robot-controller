package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_CLOSE;

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

@Autonomous(name="fsdjfsldk")
@Disabled
public class StatesFiveSpec extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(180));
    private final Pose startPose = new Pose(0, 57, Math.toRadians(180));
    private final Pose chamberPose = new Pose(29, 61.5, Math.toRadians(180));
    private Claw claw;
    private Slide slide;
    private Arm arm;
    private int counter = 0;
    private double yPlace = 60;
    private PathChain hangSpecimen1, goToSamples, transferSample1, goToSample2, transferSample2, goToSample3, transferSample3;
    private Path goBack, pickSpecimen, placeSpecimen, placeSpecimen2, pickMore;

    public void buildPaths() {
        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose))) // First path
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())// Heading interpolation
                .setZeroPowerAccelerationMultiplier(4)
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
                        new Point(23, 30, Point.CARTESIAN),
                        new Point(28.5, 33.3, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(325))


                .build();
        transferSample1 = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(28.5, 33.3, Point.CARTESIAN),
                        new Point(10, 20, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(325), Math.toRadians(270))

                .build();

        goToSample2 = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(10, 20, Point.CARTESIAN),
                        new Point(30.5, 23, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(320))
                .build();

        transferSample2 = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(30.5, 23, Point.CARTESIAN),
                        new Point(10, 23, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(320), Math.toRadians(270))

                .build();

        goToSample3 = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(10, 23, Point.CARTESIAN),
                        new Point(45, 15.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        transferSample3 = follower.pathBuilder()
                .addPath(new BezierCurve( // First path - Bezier curve
                        new Point(45, 15.5, Point.CARTESIAN),
                        new Point(20, 15.5, Point.CARTESIAN),
                        new Point(3.5, 26, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(0.8)
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
                    arm.setPosition(-300, 1.0);

                    follower.setMaxPower(1);
                    follower.followPath(goToSamples);
                    setPathState(5);
                };
                break;
            case 5:
                if (!follower.isBusy()) {
                    claw.setDraggerPosition(0.695);
                    setPathState(6);
                };
                break;

            case 6:
                if( pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(transferSample1);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    claw.setDraggerPosition(0.76);
                    follower.followPath(goToSample2);
                    setPathState(8);
                }
                break;
            case 8:
                if((follower.getPose().getX() > 29.5 && follower.getPose().getX() < 31.5) || (follower.getPose().getY() > 22 && follower.getPose().getY() < 24) && (follower.getPose().getHeading() > 315 && follower.getPose().getHeading() < 325)) {
                    claw.setDraggerPosition(0.695);
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(transferSample2);
                    setPathState(10);
                }
                break;
            case 10:
                if(!follower.isBusy()) {
                    claw.setDraggerPosition(0.76);
                    follower.followPath(goToSample3);
                    setPathState(11);
                }
                break;
            case 11:
                if((follower.getPose().getX() > 44 && follower.getPose().getX() < 46) || (follower.getPose().getY() > 13.5 && follower.getPose().getY() < 15.5) && (follower.getPose().getHeading() > 265 && follower.getPose().getHeading() < 275)) {
                    claw.setDraggerPosition(0.695);
                    setPathState(12);
                }
                break;
            case 12:
                if(pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(transferSample3);
                    setPathState(13);
                }
                break;
            case 13:
                if (follower.getPose().getY() > 18) {
                    claw.setDraggerPosition(0.76);
                    setClawLoad();
                    setPathState(14);
                }
                break;
            case 14:

                if (!follower.isBusy()) {
                    slide.resetSlide();
                    claw.setClawPosition(1.0);
                    setPathState(15);
                }
                else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                    slide.resetSlide();
                    follower.breakFollowing();
                    claw.setClawPosition(1.0);
                    setPathState(15);
                }
                break;
            case 15:
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
                            setPathState(16);
                        }


                    }

                    setClawPut();

                }
                break;
            case 16:
                if (!follower.isBusy()) {
                    slide.setPosition(-900, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(17);
                } else if (follower.getVelocity().getMagnitude() < 1.0 && follower.getCurrentPath().getClosestPointTValue() > 0.8 && follower.isBusy()) {
                    follower.breakFollowing();
                    slide.setPosition(-900, 1.0);
                    setPathState(17);
                }
                break;
            case 17:
                if (pathTimer.getElapsedTimeSeconds() > 0.1 && counter < 4 && slide.sendPosition() < -850) {
                    arm.setPosition(-300, 1);
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
//        arm.resetArm();
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572); // CHANGE
        slide.resetSlide();

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
        claw.setArmPosition(0.627); // CHANGE
    }

    public void setClawPut() {
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572); // CHANGE

    }


}
