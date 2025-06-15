package org.firstinspires.ftc.teamcode.config.subsystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config

public class SpecArm {
    public int arm_start_pos=150;//1930
    private PIDController arm_controller;
    public static double arm_p=0.015, arm_i=0.015, arm_d=0.001;
    public static double arm_f=-0.9;

    public static int arm_target;
    public static double max = 0.5;

    private final double ticks_in_degrees=(double)(350)/90.0;

    private DcMotorEx arm_motor;
    Telemetry telemetry;
    HardwareMap hwMap;

    private VoltageSensor voltageSensor;

    public SpecArm(HardwareMap hardwareMap,Telemetry telem){
        telemetry=telem;
        hwMap=hardwareMap;
        voltageSensor=hwMap.voltageSensor.iterator().next();
    }

    public void init_(){
        arm_controller=new PIDController(arm_p, arm_i, arm_d);


        arm_motor=hwMap.get(DcMotorEx.class, "specArm");

        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arm_motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void loop_(){
        arm_controller.setPID(arm_p,arm_i,arm_d);
        int armPos=arm_motor.getCurrentPosition()+arm_start_pos;
        double arm_pid=arm_controller.calculate(armPos,arm_target);
        double arm_ff=Math.cos(Math.toRadians(arm_target/ticks_in_degrees))*arm_f;
        double arm_power=(arm_pid+arm_ff)*max;
        arm_motor.setPower(arm_power*(13.3/voltageSensor.getVoltage())); //use nominal voltage to adjust arm power



        telemetry.addData("arm position", armPos);
        telemetry.addData("arm target", arm_target);
        telemetry.addData("arm power", arm_power);

        telemetry.update();
    }

    public void setMax(double max) {
        this.max = max;
    }

    public double getMax() {
        return max;
    }

    public int getPos() {
        return arm_motor.getCurrentPosition()+arm_start_pos;
    }




    public void set_target(int target){
        arm_target=target;
    }
}
