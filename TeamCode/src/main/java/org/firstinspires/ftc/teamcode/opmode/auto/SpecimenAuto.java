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
@Autonomous(name="SpecimenAuto")
public class SpecimenAuto extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(180));
    private final Pose startPose = new Pose(0, 50, Math.toRadians(180));
    private final Pose chamberPose = new Pose(30, 60, Math.toRadians(180));
    private Claw claw;
    private Slide slide;
    private Arm arm;
    private int counter = 0;
    private int yPlace = 65;
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
                        ,new Point(20, 60, Point.CARTESIAN)));
        goBack.setConstantHeadingInterpolation(chamberPose.getHeading());
        goBack.setZeroPowerAccelerationMultiplier(2);

        pickSpecimen = new Path(new BezierLine(new Point(follower.getPose()), new Point(observationPose)));
        pickSpecimen.setPathEndVelocityConstraint(0);

        placeSpecimen = new Path(new BezierLine(new Point(5.75, 1, Point.CARTESIAN), new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(180));

        placeSpecimen2 = new Path(new BezierLine(new Point(4.75, 25, Point.CARTESIAN), new Point(28.5, yPlace, Point.CARTESIAN)));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));

        pickMore = new Path( new BezierCurve(
                new Point(34.000, 65.000, Point.CARTESIAN),
                new Point(13.000, 63.250, Point.CARTESIAN),
                new Point(34.000, 32.500, Point.CARTESIAN),
                new Point(4.75, 25, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
        pickMore.setZeroPowerAccelerationMultiplier(1.5);

        goToSamples = follower.pathBuilder()
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
                ))
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
                        new Point(45, 2.5, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(
                        new Point(45, 2.5, Point.CARTESIAN),
                        new Point(5.75, 1, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setZeroPowerAccelerationMultiplier(1.5)
                .build();

        // First path
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                // Logic for this case without paths
                follower.followPath(hangSpecimen1, true);

                arm.setPosition(-2550, 1.0);
                slide.setPosition(-350, 1.0);
                setPathState(1); // Set pathState to 1 (you can modify this based on desired flow)
                break;
            case 1:
                if (!follower.isBusy()) {
                    slide.setPosition(-1200, 1.0);
//                    claw.setArmPosition(0.7);
                    setPathState(2);
                }
                break;
            case 2:
                if (slide.sendPosition() < -1050) {
                    claw.setClawPosition(0.5);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    slide.setPosition(0, 1.0);
                    follower.followPath(goBack, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    arm.setPosition(275, 1.0);
                    setClawLoad();
                    follower.followPath(goToSamples, false);
                    setPathState(5);
                };
                break;
            case 5:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    setPathState(6);

                }
                break;
            case 6:
                if(claw.getClawPosition() > 0.8 && pathTimer.getElapsedTimeSeconds() > 0.5) {
                    follower.followPath(placeSpecimen, true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(-2550, 0.5);
                    slide.setPosition(-350, 1.0);
                    setClawPut();
                    setPathState(7);
                }
                break;
            case 7:
            case 11:
                if (!follower.isBusy()) {
                    slide.setPosition(-1200, 1.0);
    //                    claw.setArmPosition(0.7);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 0.5 && counter < 2) {
                    arm.setPosition(275, 1.0);
                    slide.setPosition(0, 1.0);
                    setClawLoad();
                    counter++;
                    follower.followPath(pickMore);
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
                    follower.followPath(placeSpecimen2, true);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    arm.setPosition(-2550, 0.5);
                    slide.setPosition(-350, 1.0);

                    setClawPut();

                    setPathState(11);
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
        claw.setWristPosition(0.5);
        claw.setArmPosition(0.505);

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
        claw.setClawPosition(0.3);
        claw.setWristPosition(0.5);
        claw.setArmPosition(0.57); // 0.536
    }

    public void setClawPut() {
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.5);
        claw.setArmPosition(0.505); // 0.475
    }


}
