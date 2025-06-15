package org.firstinspires.ftc.teamcode.config.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Slide {
    private DcMotorEx slide;
    private String slideName;

    public Slide(String slideName, HardwareMap hardwareMap) {
        this.slide = hardwareMap.get(DcMotorEx.class, slideName);
        slide.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public int sendPosition() {
        return slide.getCurrentPosition();
    }
    public void setPosition(int pos, double speed) {
        slide.setTargetPosition(pos);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide.setPower(speed);
    }

    public void resetSlide() {
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setPowerSlide(double power) {
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slide.setPower(power);
    }

    public void setModeEncoder(){
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setModeNoEncoder(){
        slide.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public DcMotor getMotor() {
        return slide;
    }



}