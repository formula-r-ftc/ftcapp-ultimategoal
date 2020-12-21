package org.firstinspires.ftc.teamcode;


import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class autonomous extends OpMode {

    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;

    ElapsedTime t1 = new ElapsedTime();

    double encoderSpeed(double targetPosition, double maxSpeed){
        double avgEncodePos = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() +LBMotor.getCurrentPosition())/4;
        double distance = targetPosition - avgEncodePos;
        telemetry.addData("distance",distance);
        double power = Range.clip(distance/500,-0.5, 0.5); //y=mx+b
        return power;
    }

    public void rampUp(double distance, double heading, double time, double maxSpeed){
        double accelerationSlope = maxSpeed * time;
        double power = accelerationSlope * t1.seconds();
        if(Math.abs(power) < encoderSpeed(distance, maxSpeed)){
            setTurnPower(turn(heading), power);
        }else{
            
        }
    }

    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LFMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        LBMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "initialized");

        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

    }
}
