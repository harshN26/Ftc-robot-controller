package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_CLOSE;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_OPEN;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.ENC_PER_INCH;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Claw;
import org.firstinspires.ftc.teamcode.config.subsystem.Limelight;
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

@TeleOp(name="sUBlOAD")
public class SubPick extends LinearOpMode {

    Limelight ll;
    double cameraFOVPeriToRobot=21.5;
    LLResult result;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private final Pose observationPose = new Pose(5.75, 30, Math.toRadians(0));
    private final Pose startPose = new Pose(28, 60, Math.toRadians(0));
    private final Pose chamberPose = new Pose(28, 60, Math.toRadians(0)); // was 65
    private Claw claw;


    private SpecArmClaw specArmClaw;
    private Slide slide;
    private SpecArm specArm;
    private Arm arm;

    private int counter = 0;
    public double ta=0;
    private double yPlace = 62;
    // These paths remain unchanged
    private PathChain hangSpecimen1, goToSample3, transferSample3, goToMethod;
    private Path goBack, pickSpecimen, placeSpecimen, placeSpecimen2, pickMore;
    // New individual paths for the original goToSamples composite
    private Path goToSamplesPart1, goToSamplesPart2, goToSamplesPart3, goToSamplesPart4 ;

    double distanceFromLimelightToGoalInches;
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    double horzError=0;
    double distError=0;

    public void buildPaths() {

    }

    public void autonomousPathUpdate() {
        switch(pathState) {
            case 0:

        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void runOpMode() {
        ll=new Limelight(hardwareMap,telemetry);
        ll.setPipeline(2);
        ll.startLL();
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
        specArmClaw.setWristPosition(0.34);

        claw.setClawPosition(0.9);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.6);

        buildPaths();
        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);

        while(opModeInInit()){
            result=ll.get_results();

            horzError=result.getTx(); // y-posRobot
            distError=result.getTy();//x-posRobot
            ta=result.getTa();
            if(ta<1.0)
                ta=0.853;
            else
                ta=1.0;



            // distance from the target to the floor

            //calculate distance
            distanceFromLimelightToGoalInches = distError+21+cameraFOVPeriToRobot;
            telemetry.addLine("Slide Ticks Needed: "+(int)(-1 * distanceFromLimelightToGoalInches*ENC_PER_INCH));
            telemetry.addLine("ta: "+ta);
            telemetry.update();
        }

        opmodeTimer.resetTimer();



        while (opModeIsActive()) {
            arm.setPosition(500, 1.0);
            slide.setPosition((int)(-1*distanceFromLimelightToGoalInches*ENC_PER_INCH),1.0);
            claw.setWristPosition(ta);
            claw.setArmPosition(0.8);
            claw.setClawPosition(0.7);
            specArm.loop_();
            if (!follower.isBusy()) {
                goToMethod = follower.pathBuilder()
                        .addPath(new BezierLine(new Point(startPose), new Point(startPose.getX(), startPose.getY()+7))) // First path
                        .setLinearHeadingInterpolation(startPose.getHeading(), startPose.getHeading())// Heading interpolation
                        .setZeroPowerAccelerationMultiplier(1)
                        .build();
                follower.followPath(goToMethod);
            }
            if(!follower.isBusy()&&slide.getMotor().getCurrentPosition()>(int)(-1*distanceFromLimelightToGoalInches*ENC_PER_INCH)-200){
                arm.setPosition(700,1.0);
            }

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
}
