package org.firstinspires.ftc.teamcode.opmode.auto;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.subsystem.Arm;
import org.firstinspires.ftc.teamcode.config.subsystem.Slide;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

@Autonomous(name="huskytestmethod")

public class HuskyLensTestAuto extends LinearOpMode {

    private Follower follower;
    private static final double VERTICAL_FOV_RADIANS = 0.87;   // ~49.6° vertical
    private static final double HORIZONTAL_FOV_RADIANS = 1.10;
    private static final double CAMERA_PITCH_RADIANS = Math.toRadians(45);  // Camera tilt downward (from horizontal)

    private Timer pathTimer;
    private HuskyLens huskyLens;
    private double groundDistance = 0;
    private double lateralOffset = 0;
    private Slide slide;
    private Arm arm;
    private int pathState;
    private static final double CAMERA_HEIGHT_INCHES = 8.5;  // Camera height in inches
    private static final int IMAGE_WIDTH = 320;
    private static final int IMAGE_HEIGHT = 240;
    private static final double ALIGN_TOLERANCE = 1.0;

    public void buildPaths() {}

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                HuskyLens.Block[] blocks = huskyLens.blocks(3);
                boolean blueFound = false;



                for (HuskyLens.Block block : blocks) {
                    if (block != null) {
                        // Compute distance
                        double[] distances = computeDistance(block.x, block.y, block.height);
                        groundDistance = distances[0];
                        lateralOffset = distances[2];  // Using lateral offset for alignment

                        // Output computed values to telemetry

                            telemetry.addData("Block", block.toString());
                            telemetry.addData("Ground Distance", "%.2f inches", groundDistance);
                            telemetry.addData("Lateral Offset", "%.2f inches", lateralOffset);


                        // Check if blue object is within alignment tolerance

                            generateLateralAlignmentPath(lateralOffset);
                            setPathState(1);


                        blueFound = true;
                        break;
                    }
                }

                if (!blueFound) {
                    telemetry.addData("Status", "Searching for blue object...");
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    arm.setPosition(550, 1.0);
                    if (arm.sendPosition() > 525) {
                        slide.setPosition((int) (groundDistance * -103.5), 1.0);
                        setPathState(-1);
                    }

                }
                break;

            case -1:
                break;
        }
    }

    @Override
    public void runOpMode() {
        slide = new Slide("slideMotor", hardwareMap);
        arm = new Arm("armMotor", hardwareMap);
        follower = new Follower(hardwareMap);
        Pose startPose = new Pose(72, 72, Math.toRadians(0));
        follower.setStartingPose(startPose);
        pathTimer = new Timer();

        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        buildPaths();
        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            follower.getDashboardPoseTracker();
            autonomousPathUpdate();

            telemetry.addData("lateral distance", lateralOffset);
            telemetry.addData("ground distance", groundDistance);
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());

            telemetry.update();
        }
    }

    private double[] computeDistance(int objectX, int objectY, int objectHeightPx) {
        // Determine the bottom y-coordinate of the detected object.
        // (Assuming block.y is the center of the block.)
        double bottomY = objectY + (objectHeightPx / 2.0);

        // Compute the vertical angle offset from the center of the image.
        // (A positive value indicates a point lower in the image.)
        double pixelAngle = (bottomY - (IMAGE_HEIGHT / 2.0)) * (VERTICAL_FOV_RADIANS / IMAGE_HEIGHT);

        // The effective ray angle from the horizontal is the camera’s pitch plus the pixel offset.
        double rayAngle = CAMERA_PITCH_RADIANS + pixelAngle;

        // Compute ground distance using the relation: tan(rayAngle) = CAMERA_HEIGHT / groundDistance.
        double groundDistance = CAMERA_HEIGHT_INCHES / Math.tan(rayAngle);

        // Compute the line-of-sight distance using the Pythagorean theorem:
        // (lineOfSightDistance)^2 = (groundDistance)^2 + (CAMERA_HEIGHT)^2
        double lineOfSightDistance = Math.sqrt((groundDistance * groundDistance) + (CAMERA_HEIGHT_INCHES * CAMERA_HEIGHT_INCHES));

        // Compute lateral offset.
        // First, determine the horizontal deviation in pixels from the image center.
        double pixelOffsetX = objectX - (IMAGE_WIDTH / 2.0);
        // At the computed ground distance, determine half the width of the ground’s view using the horizontal FOV.
        double halfWidthGround = groundDistance * Math.tan(HORIZONTAL_FOV_RADIANS / 2.0);
        // Scale the pixel offset relative to the half-width of the image.
        double lateralOffset = (pixelOffsetX / (IMAGE_WIDTH / 2.0)) * halfWidthGround;

        return new double[]{groundDistance, lineOfSightDistance, lateralOffset};
    }

    private void generateLateralAlignmentPath(double lateralOffset) {
        Pose currentPose = follower.getPose();
        double heading = currentPose.getHeading();
        double dy = lateralOffset;

        if (dy < 0) {
            dy += 6;
        } else {
            dy -= 6;
        }

        Pose targetPose = new Pose(currentPose.getX(), currentPose.getY() - dy, heading);
        Path lateralPath = new Path(new BezierLine(new Point(currentPose), new Point(targetPose)));
        lateralPath.setConstantHeadingInterpolation(heading);

        follower.followPath(lateralPath, true);
    }
}
