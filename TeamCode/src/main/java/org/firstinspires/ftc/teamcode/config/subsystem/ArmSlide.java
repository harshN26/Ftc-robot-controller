package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.teamcode.config.RobotConstants.ARM_HEIGHT;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.ENC_PER_DEC;
import static org.firstinspires.ftc.teamcode.config.RobotConstants.ENC_PER_INCH;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSlide {
    private Arm arm;
    private Slide slide;

    public ArmSlide(Arm arm, Slide slide) {
        this.arm = arm;
        this.slide = slide;
    }

    public double[] returnPos(double x, double y) {
        double[] positions = new double[2];
        double theta = Math.atan2((y - ARM_HEIGHT), x);
        double s = Math.abs(y - ARM_HEIGHT) / Math.sin(theta);

        positions[0] = theta * ENC_PER_DEC;
        positions[1] = -(s * ENC_PER_INCH);

        return positions;
    }
}
