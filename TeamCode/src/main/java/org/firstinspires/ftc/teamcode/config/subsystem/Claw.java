package org.firstinspires.ftc.teamcode.config.subsystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private Servo claw;
    private Servo wrist;
    private Servo arm;
    private Servo dragger;

    public Claw(HardwareMap hardwareMap) {
        this.claw = hardwareMap.get(Servo.class, "clawServo");
        this.wrist = hardwareMap.get(Servo.class, "wristServo");
        this.arm = hardwareMap.get(Servo.class, "armServo");
        this.dragger = hardwareMap.get(Servo.class, "dragger");
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

    public void armSetPosition(double pos) {
        arm.setPosition(pos);
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

    public Servo getArm() {
        return arm;
    }

    public void setArm(Servo arm) {
        this.arm = arm;
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

    public double getArmPosition() {
        return arm.getPosition();
    }

    public void setArmPosition(double position) {
        arm.setPosition(position);
    }

    public double getDraggerPosition() {
        return dragger.getPosition();
    }

    public void setDraggerPosition(double position) {
        dragger.setPosition(position);
    }

    public void initBasket() {
        setClawPosition(1.0);
        setWristPosition(0.5);
        setArmPosition(0.505);
    }

    public void initSpecAuto() {
        setClawPosition(1.0);
        setWristPosition(0.8);
        setArmPosition(1.0);
    }

    public void pickupBasket(double wristOffset) {
        setClawPosition(0.5);
        setWristPosition(0.5  +wristOffset);
        setArmPosition(0.617);
    }


}
