package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Disabled()
@Autonomous(name="PickDrop")
public class PickDrop extends LinearOpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(30, 30, Math.toRadians(180));
    private final Pose observationPose = new Pose(12, 30, Math.toRadians(180));

    private Claw claw;
    private Slide slide;
    private Arm arm; //-700
    private Path pickSpecimen, placeSpecimen, pickMore;

    public void buildPaths() {
        pickSpecimen = new Path(new BezierLine(new Point(startPose), new Point(observationPose)));
        pickSpecimen.setPathEndVelocityConstraint(0);

        placeSpecimen = new Path(new BezierLine(new Point(observationPose), new Point(35.5, 65, Point.CARTESIAN)));
        placeSpecimen.setPathEndVelocityConstraint(0);
        placeSpecimen.setConstantHeadingInterpolation(Math.toRadians(180));

        pickMore = new Path(new BezierLine(new Point(34, 65, Point.CARTESIAN), new Point(observationPose)));
        pickMore.setPathEndVelocityConstraint(0);
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:
                follower.followPath(pickSpecimen, false);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    claw.setClawPosition(1.0);
                    follower.followPath(placeSpecimen, false);
//                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        arm.setPosition(-2950, 1.0);
                        slide.setPosition(-1000, 1.0);
                        claw.setClawPosition(1.0);
                        claw.setWristPosition(0.15);
                        claw.setArmPosition(0.9);
                        setPathState(2);


                }
                break;
            case 2:
                if(!follower.isBusy()) {
                    slide.setPosition(0, 1.0);
                    if (slide.sendPosition() > -200) {
                        claw.setClawPosition(0);
                        setPathState(3);
                    }
                }
                break;
            case 3:
                if(!follower.isBusy()) {
                    arm.setPosition(0, 1.0);
                    claw.setClawPosition(0.3);
                    claw.setWristPosition(0.8);
                    claw.setArmPosition(0.5);
                    follower.followPath(pickMore);
                    setPathState(1);
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
        claw.setClawPosition(0.3);
        claw.setWristPosition(0.8);
        claw.setArmPosition(0.5);

        buildPaths();

        waitForStart();

        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            follower.getDashboardPoseTracker();
            autonomousPathUpdate();

            telemetry.addData("arm position", arm.sendPosition());
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }
}
