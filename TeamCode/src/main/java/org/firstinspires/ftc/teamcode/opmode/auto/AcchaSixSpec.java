package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_CLOSE;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_OPEN;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.config.subsystem.SpecArm;
import org.firstinspires.ftc.teamcode.config.subsystem.SpecArmClaw;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="Michiana 5 spec",group="!")
public class AcchaSixSpec extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(0));
    private final Pose startPose = new Pose(0, 57, Math.toRadians(0));
    private final Pose chamberPose = new Pose(30.8, 63, Math.toRadians(0)); // was 65
    private Claw claw;


    private SpecArmClaw specArmClaw;
    private Slide slide;
    private SpecArm specArm;
    private Arm arm;
    private int counter = 0;
    private double yPlace = 65;
    // These paths remain unchanged
    private PathChain hangSpecimen1, goToSample3, transferSample3;
    private Path goBack, pickSpecimen, placeSpecimen, placeSpecimen2, pickMore;
    // New individual paths for the original goToSamples composite
    private Path goToSamplesPart1, goToSamplesPart2, goToSamplesPart3, goToSamplesPart4;

    public void buildPaths() {

        hangSpecimen1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(chamberPose.getX(),(chamberPose.getY()+3))))
                .setLinearHeadingInterpolation(startPose.getHeading(), chamberPose.getHeading())
                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndVelocityConstraint(28)
                .setPathEndTValueConstraint(0.95)
                .setPathEndHeadingConstraint(10)
                .build();

        goBack = new Path(
                new BezierLine(
                        new Point(follower.getPose()),
                        new Point(19, 59, Point.CARTESIAN)));
        goBack.setConstantHeadingInterpolation(chamberPose.getHeading());
        goBack.setZeroPowerAccelerationMultiplier(0.5);
        goBack.setPathEndHeadingConstraint(10.0);

        pickSpecimen = new Path(new BezierLine(new Point(follower.getPose()), new Point(observationPose)));
        pickSpecimen.setZeroPowerAccelerationMultiplier(6);
        pickSpecimen.setPathEndVelocityConstraint(28);
        pickSpecimen.setPathEndTValueConstraint(0.95);

        placeSpecimen = new Path(new BezierCurve(new Point(-2, 3, Point.CARTESIAN),
//                new Point(20, 10, Point.CARTESIAN),
//                new Point(0, 35, Point.CARTESIAN),
//                new Point(5,yPlace + 10,Point.CARTESIAN),
                new Point(29, yPlace, Point.CARTESIAN)));
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(0));
        placeSpecimen.setZeroPowerAccelerationMultiplier(7);
        placeSpecimen.setPathEndVelocityConstraint(28);
        placeSpecimen.setPathEndTValueConstraint(0.95);



        placeSpecimen2 = new Path(new BezierCurve(new Point(5.5, 26, Point.CARTESIAN),
                new Point(5, yPlace + 10, Point.CARTESIAN),
                new Point(25, yPlace, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(7);
        placeSpecimen2.setPathEndVelocityConstraint(28);
        placeSpecimen2.setPathEndTValueConstraint(0.95);

        pickMore = new Path(new BezierLine(
                new Point(30, 65, Point.CARTESIAN),
                new Point(-4, 20, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(0));
        pickMore.setZeroPowerAccelerationMultiplier(2.5);
        pickMore.setPathEndVelocityConstraint(28);
        pickMore.setPathEndTValueConstraint(0.95);

        // Break down the original goToSamples path into four separate segments:
        goToSamplesPart1 = new Path(new BezierCurve(
                new Point(19.000, 59.000, Point.CARTESIAN),
                new Point(20, 12, Point.CARTESIAN),
                new Point(40, 39, Point.CARTESIAN),
                new Point(48, 20.5, Point.CARTESIAN)
        ));
        goToSamplesPart1.setConstantHeadingInterpolation(Math.toRadians(0));
        goToSamplesPart1.setPathEndHeadingConstraint(10.0);


        goToSamplesPart2 = new Path(new BezierCurve(
                new Point(48, 20.5, Point.CARTESIAN),
                new Point(25, 18, Point.CARTESIAN),
                new Point(12, 18, Point.CARTESIAN)
        ));
        goToSamplesPart2.setConstantHeadingInterpolation(Math.toRadians(0));
        goToSamplesPart2.setPathEndHeadingConstraint(10.0);
        goToSamplesPart2.setZeroPowerAccelerationMultiplier(2.0);


        goToSamplesPart3 = new Path(new BezierCurve(
                new Point(12, 18, Point.CARTESIAN),
                new Point(44, 26, Point.CARTESIAN),
                new Point(50.5, 10, Point.CARTESIAN)
        ));
        goToSamplesPart3.setConstantHeadingInterpolation(Math.toRadians(0));
        goToSamplesPart3.setPathEndHeadingConstraint(10.0);


        goToSamplesPart4 = new Path(new BezierLine(
                new Point(49.5, 10, Point.CARTESIAN),
                new Point(10, 10, Point.CARTESIAN)
        ));
        goToSamplesPart4.setConstantHeadingInterpolation(Math.toRadians(0));
        goToSamplesPart4.setPathEndHeadingConstraint(10.0);
        goToSamplesPart4.setZeroPowerAccelerationMultiplier(2.0);


        // goToSample3 remains as before
        goToSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(10, 10, Point.CARTESIAN),
                        new Point(15, 24, Point.CARTESIAN),
                        new Point(48.5, 6.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(7)
                .setPathEndVelocityConstraint(28)
                .setPathEndHeadingConstraint(10)

                .build();


        transferSample3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(48.5, 5.5, Point.CARTESIAN),
//                        new Point(20, 15.5, Point.CARTESIAN),
                        new Point(-2, 5.5, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setZeroPowerAccelerationMultiplier(2.5)
                .setPathEndVelocityConstraint(28)
                .setPathEndHeadingConstraint(0.95)

                .build();
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(hangSpecimen1, true);
//                specArm.set_target(0);
                specArm.set_target(-330);
                arm.setPosition(-700, 1.0);
//                specArmClaw.setWristPosition(0.6);
                setPathState(1);
            case 1:
                if (follower.getPose().getX() > 22.5) {
                    specArm.set_target(-490);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    specArmClaw.setClawPosition(0.5);
                    setPathState(3);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.5 &&
                        follower.isBusy()) {
                    follower.breakFollowing();
                    specArmClaw.setClawPosition(0.5);

                    setPathState(3);
                }
                break;
            case 3:

                setPathState(4);

                break;
            case 4:
//                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(goBack);
                    setPathState(5);
//                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    specArm.set_target(100);


                    // Start with the first segment of goToSamples
                    follower.followPath(goToSamplesPart1);
                    setPathState(6);


                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(goToSamplesPart2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(goToSamplesPart3);
                    setPathState(8);
                }
                break;
            case 8:
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
                if (!follower.isBusy()) {

                    setPathState(12);
                }
                break;
            case 12:

                follower.followPath(transferSample3);

                setPathState(13);
                break;
            case 13:


                setPathState(14);

                break;
            case 14:
                if (!follower.isBusy()) {

                    setPathState(15);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                        follower.isBusy()) {

                    follower.breakFollowing();
                    setPathState(15);
                }
                break;
            case 15:
                specArmClaw.setClawPosition(1);

                if(pathTimer.getElapsedTimeSeconds() > 0.3) {
                    specArm.set_target(-330);
                    if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                        specArmClaw.setWristPosition(1);
                        follower.followPath(placeSpecimen);
                        setPathState(16);
                    }

                }
                break;
            case 16:
                if (follower.getPose().getX() > 22.5) {
                    specArm.set_target(-490);
                    setPathState(17);
                }
                break;
            case 17:
                if (!follower.isBusy()) {
                    specArmClaw.setClawPosition(0);
                    setPathState(18);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.3 &&
                        follower.isBusy()) {
                    follower.breakFollowing();
                    specArmClaw.setClawPosition(0);

                    setPathState(18);
                }
                break;
            case 18:
                if (pathTimer.getElapsedTimeSeconds() < 0.2) {
                    follower.followPath(pickMore);
                }
                if (follower.getPose().getX() < 20) {
                    specArmClaw.setWristPosition(0.34);
                    specArm.set_target(100);
                    setPathState(19);
                }

                break;
            case 19:
                if (!follower.isBusy()) {
                    setPathState(20);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                        follower.isBusy()) {
                    setPathState(20);
                }
                break;
            case 20:
                specArmClaw.setClawPosition(1);

                if(pathTimer.getElapsedTimeSeconds() > 0.4) {
                    specArm.set_target(-330);
                    if (pathTimer.getElapsedTimeSeconds() > 0.4) {
                        yPlace = yPlace - 1.5;
                        placeSpecimen2 = new Path(new BezierCurve(new Point(-4, 25, Point.CARTESIAN),
//                                new Point(5, yPlace + 10, Point.CARTESIAN),
                                new Point(29, yPlace, Point.CARTESIAN)));
                        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(0));
                        placeSpecimen2.setZeroPowerAccelerationMultiplier(7);
                        placeSpecimen2.setPathEndHeadingConstraint(10);
                        follower.followPath(placeSpecimen2);
                        specArmClaw.setWristPosition(1);
                        setPathState(16);
                    }

                }
                break;
//            case 21:
//                if (follower.getPose().getX() > 22.5) {
//                    specArm.set_target(-500);
//                    setPathState(16);
//                }
//                break;
            case 22:
                if (!follower.isBusy()) {
                    setPathState(17);
                } else if (follower.getVelocity().getMagnitude() < 1.0 &&
                        follower.getCurrentPath().getClosestPointTValue() > 0.8 &&
                        follower.isBusy()) {
                    follower.breakFollowing();
                    setPathState(17); // set to 17
                }
                break;
            case 170:
                if (pathTimer.getElapsedTimeSeconds() > 0.1 && counter < 4) {

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
        specArmClaw = new SpecArmClaw(hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        arm = new Arm("armMotor", hardwareMap);
        claw.setDraggerPosition(DRAGGER_CLOSE);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        specArm = new SpecArm(hardwareMap,telemetry);
        specArm.init_();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        specArmClaw.setClawPosition(1);
        specArmClaw.setWristPosition(0.34); // 0.34

        claw.setClawPosition(0.9);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);

        buildPaths();

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            specArm.loop_();
            follower.update();

            autonomousPathUpdate();

            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.addData("velocity", follower.getPose().getVector().getMagnitude());
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
