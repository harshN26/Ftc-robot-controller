package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SpecArmClaw {
    private Servo claw;
    private Servo wrist;

    public SpecArmClaw(HardwareMap hardwareMap) {
        this.wrist = hardwareMap.get(Servo.class, "specArmClaw");
        this.claw = hardwareMap.get(Servo.class, "specArmWrist");
    }

    public void openClaw() {
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(1.0);
    }

    public void clawSetPosition(double pos) {
        claw.setPosition(pos);
    }

    public void wristSetPosition(double pos) {
        wrist.setPosition(pos);
    }

    public Servo getClaw() {
        return claw;
    }

    public void setClaw(Servo claw) {
        this.claw = claw;
    }

    public Servo getWrist() {
        return wrist;
    }

    public void setWrist(Servo wrist) {
        this.wrist = wrist;
    }

    public double getClawPosition() {
        return claw.getPosition();
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }

    public double getWristPosition() {
        return wrist.getPosition();
    }

    public void setWristPosition(double position) {
        wrist.setPosition(position);
    }




}
