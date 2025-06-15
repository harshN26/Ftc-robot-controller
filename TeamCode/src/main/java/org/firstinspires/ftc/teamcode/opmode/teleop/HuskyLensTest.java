package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "HuskyLensTest", group = "TeleOp")
public class HuskyLensTest extends LinearOpMode {

    // Camera and object configuration – update these after calibrating your setup
    private static final double CAMERA_HEIGHT_INCHES = 8.5;             // Camera height above the ground
    private static final double CAMERA_PITCH_RADIANS = Math.toRadians(45);  // Camera tilt downward (from horizontal)
    private static final double OBJECT_REAL_HEIGHT_INCHES = 1.5;          // Real-world height of the object (if used for size-based methods)
    private static final int IMAGE_WIDTH = 320;                         // HuskyLens image width in pixels
    private static final int IMAGE_HEIGHT = 240;                        // HuskyLens image height in pixels

    // Updated Camera Field-of-View (derived from typical values for Gobilda HuskyLens)
    private static final double VERTICAL_FOV_RADIANS = 0.87;   // ~49.6° vertical
    private static final double HORIZONTAL_FOV_RADIANS = 1.10;   // ~63.2° horizontal

    private final int READ_PERIOD = 1;  // Seconds between reads
    private HuskyLens huskyLens;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the HuskyLens and select an algorithm
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();  // Allow immediate first read

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            // Get up to 3 detected blocks from HuskyLens
            HuskyLens.Block[] blocks = huskyLens.blocks(3);

            // Process each detected block (if any)
            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block block = blocks[i];

                // Compute distances for the detected object using our revised algorithm
                double[] distances = computeDistance(block.x, block.y, block.height);

                // ----- Telemetry Output -----
                telemetry.addData("Block " + i, block.toString());
                telemetry.addData("Ground Distance", "%.2f inches", distances[0]);
                telemetry.addData("Line-of-Sight", "%.2f inches", distances[1]);
                telemetry.addData("Lateral Offset", "%.2f inches", distances[2]);
            }
            telemetry.update();
        }
    }

    /**
     * Computes the ground distance, line-of-sight distance, and lateral offset for an object.
     * This method assumes the object’s bottom edge is touching the ground.
     * <p>
     * The algorithm uses:
     *  1. The vertical pixel position of the object’s bottom (objectY + objectHeightPx/2)
     *     to compute the vertical angle relative to the camera’s center.
     *  2. Adding that angle to the camera’s tilt (CAMERA_PITCH_RADIANS) to find the ray angle from horizontal.
     *  3. Using the camera height and ray angle to compute the ground distance.
     *  4. Using basic trigonometry for the line-of-sight distance and the horizontal FOV to compute lateral offset.
     *
     * @param objectX        The x-coordinate of the detected object (assumed to be its center) in pixels.
     * @param objectY        The y-coordinate of the detected object (assumed to be its center) in pixels.
     * @param objectHeightPx The height of the detected object in pixels.
     * @return A double array where:
     *         [0] = ground distance (inches),
     *         [1] = line-of-sight distance (inches),
     *         [2] = lateral offset (inches).
     */
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
}