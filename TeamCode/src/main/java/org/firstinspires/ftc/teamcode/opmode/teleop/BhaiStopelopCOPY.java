package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.DRAGGER_CLOSE;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.ENC_PER_INCH;

import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@TeleOp(name = "MichianaTeleOp SPEC COPY", group = "TeleOp")
public class BhaiStopelopCOPY extends LinearOpMode {

    Limelight ll;
    double cameraFOVPeriToRobot=21.5;
    LLResult result;
    public double ta=0;
    boolean game2RightBumperLast=false;
    boolean game2RightBumperCurr=false;
    boolean game2LeftBumperLast=false;
    boolean game2LeftBumperCurr=false;
    double distanceFromLimelightToGoalInches;
    double limelightMountAngleDegrees = 0.0;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 20.0;

    double horzError=0;
    double distError=0;


    private Follower follower;

    private int constant = 0;
    private PIDFController headingPIDF = new PIDFController(FollowerConstants.headingPIDFCoefficients);
    private Claw claw;
    private Arm arm;
    private Slide slide;
    private SpecArmClaw specArmClaw;
    private SpecArm specArm;
    private double prevClawPosition = -1.0; // Track main claw position changes

    boolean trouble=false;

    private double specArmClawPos = (1.0);
    private double specArmWristPos = (0.34);

    boolean subpick=false;

    // Previous position tracking variables
    private double prevSpecArmClawPos = -1.0;  // Initialize to invalid value to force first update
    private double prevSpecArmWristPos = -1.0; // Initialize to invalid value to force first update

    private int teleopPathState = 0;
    private int altPathState = 0;
    private Timer pathTimer;
    private Timer differenceTimer;
    private int counter = 0;
    private double yPlace = 62.5;

    private Path pickMore, placeSpecimen2, moveLeft;

    private final Pose parkPose = new Pose(0, 57, Math.toRadians(0));
    private final Pose observationPose = new Pose(6, 30, Math.toRadians(0));
    private Pose chamberPose = new Pose(28, 70, Math.toRadians(0));

    double slowModeFactor = 0.25;
    double clawPosition = 1.0;
    int target = 80;
    int phase = 0;
    double armPosition = 0.5;
    double wristPosition = 0.5;
    double draggerPosition = DRAGGER_CLOSE;
    boolean isYActive = false;

    // Button binding variables
    private boolean gamepad1_a_bind = true;
    private boolean gamepad1_b_bind = true;
    private boolean gamepad1_x_bind = true;
    private boolean gamepad1_y_bind = true;

    // Button cooldown timers
    private Timer gamepad1_a_timer = new Timer();
    private Timer gamepad1_b_timer = new Timer();

    private boolean running = false;
    private Timer gamepad1_x_timer = new Timer();
    private Timer gamepad1_y_timer = new Timer();
    private final double BUTTON_COOLDOWN = 0.4; // 0.1 seconds

    public enum AUTOPLACEPOS{LEFT,CENTER,RIGHT};
    AUTOPLACEPOS autoPlacePos= AUTOPLACEPOS.RIGHT;

    public enum SPECARMSTATE{WALL_PICKUP,WALL_CLOSE,ARMSCOREPOS,TROUBLE_SCORING,SPEC_PLACED, ARM_SCORE_TO_TROUBLE_SCORING, WALL_PICKUP_TO_WALL_CLOSE, FINALLYDONE, SPEC_PLACED_OPEN}

    SPECARMSTATE specState= SPECARMSTATE.WALL_CLOSE;

    public enum SPECARMCONTROL{MANUAL,AUTO};
    SPECARMCONTROL specArmControl= SPECARMCONTROL.MANUAL;

    public enum CLAWPICKSTATE{CLAW_OPEN,CLAW_CLOSE}
    CLAWPICKSTATE clawState= CLAWPICKSTATE.CLAW_CLOSE;

    private PathChain goToMethod;

    boolean gamepad2XLast=false;
    boolean gamepad2XCurr=false;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(observationPose);


        ll=new Limelight(hardwareMap,telemetry);
        ll.setPipeline(2);
        ll.startLL();

        arm = new Arm("armMotor", hardwareMap);
        slide = new Slide("slideMotor", hardwareMap);
        claw = new Claw(hardwareMap);

        claw.setClawPosition(1.0);
        claw.setWristPosition(0.856);
        claw.setArmPosition(0.572);
        specArmClaw = new SpecArmClaw(hardwareMap);

        double headingCorrection;

        pathTimer = new Timer();
        differenceTimer = new Timer();
        counter = 0;
        yPlace = 65;

        buildPaths();

        specArm = new SpecArm(hardwareMap, telemetry);
        specArm.init_();

        // Set initial spec arm positions
        specArmClaw.setClawPosition(specArmClawPos);
        specArmClaw.setWristPosition(specArmWristPos);

        // Update previous position tracking
        prevSpecArmClawPos = specArmClawPos;
        prevSpecArmWristPos = specArmWristPos;

        // Initialize button timers
        gamepad1_a_timer.resetTimer();
        gamepad1_b_timer.resetTimer();
        gamepad1_x_timer.resetTimer();
        gamepad1_y_timer.resetTimer();

        waitForStart();
        if (isStopRequested()) return;
        follower.startTeleopDrive();

        while (opModeIsActive()) {
            double targetForward = -gamepad1.left_stick_y;
            double targetStrafe = -gamepad1.left_stick_x;
            double targetTurn = -gamepad1.right_stick_x;



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

            gamepad2XCurr=gamepad2.x;

            if (gamepad1.right_trigger > 0) {
                targetForward *= slowModeFactor;
                targetStrafe *= slowModeFactor;
                targetTurn *= slowModeFactor;
            }

            if (teleopPathState == 0 || teleopPathState == 1 || teleopPathState == 9) {
                if (isYActive) {
                    double targetHeading = Math.toRadians(180);
                    double headingError = targetHeading - follower.getPose().getHeading();
                    headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

                    if (Math.abs(headingError) < Math.toRadians(0.5)) {
                        headingCorrection = 0;
                    } else {
                        headingPIDF.updateError(headingError);
                        headingCorrection = headingPIDF.runPIDF();
                    }

                    follower.setTeleOpMovementVectors(targetForward * 1, targetStrafe * 1, headingCorrection, false);
                } else {
                    follower.setTeleOpMovementVectors(targetForward * 0.8, targetStrafe * 0.8, targetTurn, false);
                }
            }

            follower.update();

            // Only update servo positions if they have changed
            updateSpecArmServos();


            switch(specArmControl) {
                case MANUAL:
                    switch (specState) {
                        case WALL_PICKUP:
                            target = 80 + constant;
                            specArmWristPos = 0.34;;
                            specArmClawPos = 0.7;
                            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                                specState = SPECARMSTATE.WALL_CLOSE;
                                gamepad1_b_timer.resetTimer();
                            }
                            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                                specState = SPECARMSTATE.SPEC_PLACED;
                                gamepad1_a_timer.resetTimer();
                            }
                            if (gamepad1.y && canExecuteButton("gamepad1_y")) {
                                chamberPose=new Pose(follower.getPose().getX(),follower.getPose().getY()-7,Math.toRadians(follower.getPose().getHeading()));
                                gamepad1_y_timer.resetTimer();
                            }
                            break;
                        case WALL_CLOSE:
                            target = 80 + constant;
                            specArmWristPos = 0.34;;
                            specArmClawPos = 1.0;
                            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                                specState = SPECARMSTATE.ARMSCOREPOS;
                                gamepad1_b_timer.resetTimer();
                            }
                            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                                specState = SPECARMSTATE.WALL_PICKUP;
                                gamepad1_a_timer.resetTimer();
                            }
                            break;
                        case ARMSCOREPOS:
                            target = -390 + constant;
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;
                            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                                specState = SPECARMSTATE.SPEC_PLACED;
                                gamepad1_b_timer.resetTimer();
                            }
                            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                                specState = SPECARMSTATE.WALL_CLOSE;
                                gamepad1_a_timer.resetTimer();
                            }
                            if (gamepad1.y && canExecuteButton("gamepad1_y")) {
                                specState = SPECARMSTATE.TROUBLE_SCORING;
                                trouble = true;
                                gamepad1_y_timer.resetTimer();
                            }
                            break;
                        case TROUBLE_SCORING:
                            if (trouble) {
                                target = -100 + constant;
                            }
                            if (specArm.getPos() >= (-150 + constant)) {
                                specState = SPECARMSTATE.SPEC_PLACED;
                                trouble = false;
                            }
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;
                            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                                specState = SPECARMSTATE.SPEC_PLACED_OPEN;
                                gamepad1_b_timer.resetTimer();
                            }
                            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                                specState = SPECARMSTATE.ARMSCOREPOS;
                                gamepad1_a_timer.resetTimer();
                            }
                            break;
                        case SPEC_PLACED:
                            target = -500 + constant;
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;
                            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                                specState = SPECARMSTATE.SPEC_PLACED_OPEN;
                                gamepad1_b_timer.resetTimer();
                            }
                            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                                specState = SPECARMSTATE.ARMSCOREPOS;
                                gamepad1_a_timer.resetTimer();
                            }
                            break;
                        case SPEC_PLACED_OPEN:
                            target = -500 + constant;
                            specArmWristPos = 1.0;
                            specArmClawPos = 0.7;
                            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                                specState = SPECARMSTATE.WALL_PICKUP;
                                gamepad1_b_timer.resetTimer();
                            }
                            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                                specState = SPECARMSTATE.SPEC_PLACED;
                                gamepad1_a_timer.resetTimer();
                            }
                            break;
                        case WALL_PICKUP_TO_WALL_CLOSE:
                            specState= SPECARMSTATE.WALL_CLOSE;
                            break;
                        case ARM_SCORE_TO_TROUBLE_SCORING:
                            specState= SPECARMSTATE.ARMSCOREPOS;
                            break;
                        case FINALLYDONE:
                            specState= SPECARMSTATE.WALL_PICKUP;
                            break;


                    }
//                    if (gamepad1.x && canExecuteButton("gamepad1_x")) {
////                        follower.setPose(observationPose);
////                        specArmControl=SPECARMCONTROL.AUTO;
////                        specState=SPECARMSTATE.WALL_CLOSE;
////                        gamepad1_x_timer.resetTimer();
////                        pathTimer.resetTimer();
//
//                    }
                    break;
                case AUTO:
                    switch(specState) {
                        case WALL_PICKUP:

                            target = 80 + constant;
                            specArmWristPos = 0.34;
                            specArmClawPos = 0.7;

                            Path path1=new Path(
                                    new BezierCurve(
                                            new Point(follower.getPose().getX(), follower.getPose().getY()),
                                            new Point(observationPose.getX()+5, observationPose.getY()),
                                            new Point(observationPose.getX(), observationPose.getY()))
                            );
                            path1.setConstantHeadingInterpolation(Math.toRadians(0));
                            path1.setZeroPowerAccelerationMultiplier(0.3);

                            if (!follower.isBusy()&&follower.getPose()!=observationPose) {
                                follower.followPath(path1);
                                specState= SPECARMSTATE.WALL_PICKUP_TO_WALL_CLOSE;
                            }

                            break;
                        case WALL_PICKUP_TO_WALL_CLOSE:
                            target = 80 + constant;
                            specArmWristPos = 0.34;
                            specArmClawPos = 0.7;
                            if (!follower.isBusy()) {
                                specArmControl= SPECARMCONTROL.MANUAL;
                                specState= SPECARMSTATE.WALL_PICKUP;
                                gamepad1_x_timer.resetTimer();
                                follower.startTeleopDrive();
                                pathTimer.resetTimer();
                            }
                            break;
                        case WALL_CLOSE:
                            if (pathTimer.getElapsedTime() < 500){
                                target = 80 + constant;
                                specArmWristPos =0.34;;
                                specArmClawPos = 1.0;
                            }else{
                                specState= SPECARMSTATE.ARMSCOREPOS;
                                pathTimer.resetTimer();
                            }
                            break;
                        case ARMSCOREPOS:
                            target = -330 + constant;
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;
                            Path path2= new Path(
                                new BezierCurve(
                                        new Point(follower.getPose().getX(), follower.getPose().getY()),
                                        new Point(chamberPose.getX()-10,chamberPose.getY()),
                                        new Point(chamberPose.getX(), chamberPose.getY()))

                            );
                            path2.setConstantHeadingInterpolation(Math.toRadians(0));
                            if (!follower.isBusy()) {
                                follower.followPath(path2);
                                specState = SPECARMSTATE.ARM_SCORE_TO_TROUBLE_SCORING;
                            }
                            break;
                        case ARM_SCORE_TO_TROUBLE_SCORING:
                            if (!follower.isBusy()) {
                                specState = SPECARMSTATE.TROUBLE_SCORING;
                                trouble=true;
                                pathTimer.resetTimer();
                            }
                            else if (follower.getVelocity().getMagnitude() < 1.0 &&
                                    follower.getCurrentPath().getClosestPointTValue() > 0.5 &&
                                    follower.isBusy()) {
                                follower.breakFollowing();
                                specState = SPECARMSTATE.TROUBLE_SCORING;
                                trouble=true;
                                pathTimer.resetTimer();
                            }
                            break;
                        case TROUBLE_SCORING:
                            if (trouble) {
                                target = -100 + constant;
                            }
                            if (specArm.getPos() >= -150) {
                                specState = SPECARMSTATE.SPEC_PLACED;
                                pathTimer.resetTimer();
                                trouble = false;
                            }
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;


                            break;
                        case SPEC_PLACED:
                            target = -500 + constant;
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;
                            if(pathTimer.getElapsedTime()>200){
                                specState= SPECARMSTATE.SPEC_PLACED_OPEN;
                                pathTimer.resetTimer();
                            }


                            break;
                        case SPEC_PLACED_OPEN:
                            target = -500 + constant;
                            specArmWristPos = 1.0;
                            specArmClawPos = 1.0;
                            Path runnable=new Path(
                                new BezierLine(
                                        new Point(follower.getPose().getX(), follower.getPose().getY()),
                                        new Point(chamberPose.getX()+2, chamberPose.getY()))
                            );
                            runnable.setConstantHeadingInterpolation(Math.toRadians(0));
                            switch(autoPlacePos) {
                                case LEFT:
                                    runnable=new Path(
                                            new BezierLine(
                                                    new Point(follower.getPose().getX(), follower.getPose().getY()),
                                                    new Point(chamberPose.getX()+2, chamberPose.getY()+3))
                                    );
                                    runnable.setConstantHeadingInterpolation(Math.toRadians(0));
                                    break;
                                case RIGHT:
                                    runnable=new Path(
                                            new BezierLine(
                                                    new Point(follower.getPose().getX(), follower.getPose().getY()),
                                                    new Point(chamberPose.getX()+2, chamberPose.getY()-3))
                                    );
                                    runnable.setConstantHeadingInterpolation(Math.toRadians(0));

                                    break;
                                case CENTER:
                                    specArmClawPos = 0.7;
                                    break;

                            }
                            if (!follower.isBusy()) {
                                runnable.setZeroPowerAccelerationMultiplier(0.5);
                                follower.followPath(runnable);
                                specState= SPECARMSTATE.FINALLYDONE;
                                pathTimer.resetTimer();
                            }

                            break;
                        case FINALLYDONE:
                            if (!follower.isBusy()) {
                                if(pathTimer.getElapsedTime()<500){
                                    target = -500 + constant;
                                    specArmWristPos = 1.0;
                                    specArmClawPos = 0.7;;
                                }else {
                                    specState = SPECARMSTATE.WALL_PICKUP;

                                    pathTimer.resetTimer();
                                }
                            }

                            break;



                    }
                    if (gamepad1.a && canExecuteButton("gamepad1_a")) {
                        autoPlacePos= AUTOPLACEPOS.LEFT;
                        gamepad1_a_timer.resetTimer();
                    }
                    if (gamepad1.b && canExecuteButton("gamepad1_b")) {
                        autoPlacePos= AUTOPLACEPOS.RIGHT;
                        gamepad1_b_timer.resetTimer();
                    }
                    if (gamepad1.y && canExecuteButton("gamepad1_y")) {
                        autoPlacePos= AUTOPLACEPOS.CENTER;
                        gamepad1_y_timer.resetTimer();
                    }


//                    if (gamepad1.x && canExecuteButton("gamepad1_x")) {
//                        specArmControl=SPECARMCONTROL.MANUAL;
//                        gamepad1_x_timer.resetTimer();
//                        pathTimer.resetTimer();
//                        follower.startTeleopDrive();
//                    }
                    break;
            }
//            if (gamepad1.b && canExecuteButton("gamepad1_b")) {
//                executeButtonAction("gamepad1_b");
//            }
//            if (gamepad1.a && canExecuteButton("gamepad1_a")) {
//                executeButtonAction("gamepad1_a");
//            }
//            if (gamepad1.y && canExecuteButton("gamepad1_y")) {
//                executeButtonAction("gamepad1_y");
//            }
//            if (gamepad1.x && canExecuteButton("gamepad1_x")) {
//                executeButtonAction("gamepad1_x");
//            }


            if (gamepad1.x) {
                constant += 5;
            } if (gamepad1.y && specState != SPECARMSTATE.TROUBLE_SCORING) {
                constant -= 5;
            }
            if (Math.abs(gamepad2.left_stick_x) > 0 || Math.abs(gamepad2.left_stick_y) > 0) {
                setPathState(0);
            }

            switch (teleopPathState) {
                case 9:
                    arm.setPosition(-2600, 0.75);
                    if (arm.sendPosition() < -1500) {
                        slide.setPosition(-250, 0.5);
                        if (slide.sendPosition() < -200 && arm.sendPosition() < -2550) {
                            setPathState(0);
                        }
                    }
                    setScorePosClaw();
                    break;

                case 1:
                    if (slide.sendPosition() < -1550) {
                        setPathState(0);
                    }
                    break;

                default:
                    break;
            }



                if (teleopPathState == 0 || teleopPathState == 1) {
                    if (teleopPathState == 0) {
                        if (gamepad2.right_stick_y != 0.0) {
//                        if (arm.sendPosition() > -2750 || gamepad2.right_stick_y > 0.0)
//                        if (gamepad2.right_stick_y > 0.0)
                            arm.setModeEncoder();
                            arm.setPowerArm(gamepad2.right_stick_y);
//                        else
//                            arm.setPowerArm(0);
                        } else {
                            arm.setPowerArm(0);
                        }
                    }

                    if (teleopPathState == 0 && !running) {
                        if (gamepad2.left_stick_y != 0.0) {
//                        if (slide.sendPosition() > -1800 || gamepad2.left_stick_y > 0.0 || arm.sendPosition() < -1500)
//                        if (gamepad2.left_stick_y > 0.0)
                            slide.setModeEncoder();
                            slide.setPowerSlide(gamepad2.left_stick_y);
//                        else
//                            slide.setPowerSlide(0);
                        } else {
                            slide.setPowerSlide(0);
                        }
                    }

//                if (slide.sendPosition() < -1750 && arm.sendPosition() > -1500) {
//                    slide.setPosition(-1700, 1.0);
//                } else {
//                    slide.setModeEncoder();
//                }
                }





            if (gamepad2XCurr&&!gamepad2XLast&&clawPosition==1.0){prevClawPosition = clawPosition; clawPosition = 0.25;}
            else if (gamepad2XCurr&&!gamepad2XLast&&clawPosition==0.25) {prevClawPosition = clawPosition; clawPosition = 1.0;}

            if (gamepad2.b) slide.resetSlide();
            if (gamepad2.a) {
                slide.setPosition(-500, 1.0);
                running = true;

            };


            if (gamepad2.left_stick_y != 0) {
                slide.setModeEncoder();
                running = false;
            }

            game2LeftBumperCurr=gamepad2.left_bumper;
            game2RightBumperCurr=gamepad2.right_bumper;

            if (game2RightBumperCurr&&!game2RightBumperLast) wristPosition += 0.03;
            if (game2LeftBumperCurr&&!game2LeftBumperLast) wristPosition -= 0.03;

            game2LeftBumperLast=game2LeftBumperCurr;
            game2RightBumperLast=game2RightBumperCurr;

            if(wristPosition<=0){
                wristPosition=0.0;
            }
            else if(wristPosition>=1.0){
                wristPosition=1.0;
            }

            if (gamepad2.left_trigger > 0) armPosition += 0.0045;
            if (gamepad2.right_trigger > 0) armPosition -= 0.0045;

            if (armPosition > 0.6845) armPosition = 0.6825;
            if (armPosition < 0.573) armPosition = 0.572;
            if (wristPosition > 1.0) wristPosition = 1.0;
            if (wristPosition < 0.797) wristPosition = 0.799;

            if (gamepad1.left_bumper) setLoadPosClaw();
            if (gamepad1.left_trigger != 0) setPopulatePosClaw();
            if (gamepad1.right_bumper) setScorePosClaw();
            if (gamepad2.a) slide.setPosition(-100, 1.0);

            if (prevClawPosition != clawPosition) {
                claw.setClawPosition(clawPosition);
            }
            claw.setClawPosition(clawPosition);
            claw.setWristPosition(wristPosition);
            claw.setArmPosition(armPosition);
            claw.setDraggerPosition(draggerPosition);
            specArm.loop_();
            specArm.set_target(target);

            Pose p = follower.getPose();
            telemetry.addData("Teleop State", teleopPathState);
            telemetry.addData("X", p.getX());
            telemetry.addData("Y", p.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(p.getHeading()));
            telemetry.addData("Arm Pos", arm.sendPosition());
            telemetry.addData("Slide Pos", slide.sendPosition());
            telemetry.addData("Servo Arm Pos", armPosition);
            telemetry.update();
            gamepad2XLast=gamepad2XCurr;
        }
    }

    /**
     * Updates spec arm servo positions only when they have changed
     */
    private void updateSpecArmServos() {
        // Check if claw position has changed
        if (specArmClawPos != prevSpecArmClawPos) {
            specArmClaw.setClawPosition(specArmClawPos);
            prevSpecArmClawPos = specArmClawPos;
        }

        // Check if wrist position has changed
        if (specArmWristPos != prevSpecArmWristPos) {
            specArmClaw.setWristPosition(specArmWristPos);
            prevSpecArmWristPos = specArmWristPos;
        }
    }

    private void buildPaths() {
        pickMore = new Path(new BezierLine(
                new Point(34.0, 65.0, Point.CARTESIAN),
                new Point(5.5, 26, Point.CARTESIAN)
        ));
        pickMore.setConstantHeadingInterpolation(Math.toRadians(180));
        pickMore.setZeroPowerAccelerationMultiplier(1.25);

        placeSpecimen2 = new Path(new BezierLine(
                new Point(5.5, 26, Point.CARTESIAN),
                new Point(34, yPlace, Point.CARTESIAN)
        ));
        placeSpecimen2.setConstantHeadingInterpolation(Math.toRadians(180));
        placeSpecimen2.setZeroPowerAccelerationMultiplier(2);
    }

    public void setPathState(int pState) {
        teleopPathState = pState;
        pathTimer.resetTimer();
    }

    public void setAltState(int pState) {
        altPathState = pState;
    }

    public void setLoadPosClaw() {
        clawPosition = 0.25;
        wristPosition = 0.85;
        armPosition = 0.681;
    }

    public void setScorePosClaw() {
        clawPosition = 1.0;
        wristPosition = 0.856;
        armPosition = 0.572;
    }

    public void setPopulatePosClaw() {
        clawPosition = 0.25;
        wristPosition = 0.853;
        armPosition = 0.627;
    }

    // Button binding functions
    public void bindGamepad1A(boolean enable) {
        gamepad1_a_bind = enable;
    }

    public void bindGamepad1B(boolean enable) {
        gamepad1_b_bind = enable;
    }

    public void bindGamepad1X(boolean enable) {
        gamepad1_x_bind = enable;
    }

    public void bindGamepad1Y(boolean enable) {
        gamepad1_y_bind = enable;
    }

    private boolean canExecuteButton(String button) {
        switch (button) {
            case "gamepad1_a":
                return gamepad1_a_bind && gamepad1_a_timer.getElapsedTimeSeconds() >= BUTTON_COOLDOWN;
            case "gamepad1_b":
                return gamepad1_b_bind && gamepad1_b_timer.getElapsedTimeSeconds() >= BUTTON_COOLDOWN;
            case "gamepad1_x":
                return gamepad1_x_bind && gamepad1_x_timer.getElapsedTimeSeconds() >= BUTTON_COOLDOWN;
            case "gamepad1_y":
                return gamepad1_y_bind && gamepad1_y_timer.getElapsedTimeSeconds() >= BUTTON_COOLDOWN;
        }
        return false;
    }

    private boolean executeButtonAction(String button) {
        switch (button) {
            case "gamepad1_x":
                    if (specArmClawPos == 0.5) {
                        target = 80 + constant;
                    } else {
                        specArmClawPos = 0.5;
                        specArmWristPos = 0.34;

                    }


                gamepad1_x_timer.resetTimer();
                return true;

            case "gamepad1_b":
                if (specArmWristPos == 0.34) {
                    specArmWristPos = (1.0);
                } else {
                    specArmWristPos = (0.34);
                }
                gamepad1_b_timer.resetTimer();
                return true;

            case "gamepad1_a":
                if (phase == 0) {
                    target = 80 + constant;
                    phase = 1;
                } else if (phase == 1) {
                    target = -600 + constant;
                    phase = 0;
                }
                gamepad1_x_timer.resetTimer();
                return true;

            case "gamepad1_y":
                if (follower.getPose().getX() < 10) {
                    target = 80 + constant;
                    if (specArmClawPos == 0.5) {
                        specArmWristPos = 1;
                        specArmClawPos = 1.0;
                    } else {
                        specArmClawPos = 0.7;
                        specArmWristPos = 0.34;;
                    }
                } else {
                    target = -600 + constant;
                }
                gamepad1_y_timer.resetTimer();
                return true;
        }
        return false;
    }
}