package org.firstinspires.ftc.teamcode;


import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
@Disabled
public class autonomous extends OpMode {

    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;

    ElapsedTime t1 = new ElapsedTime();

    double EncoderSpeed(double targetPosition, double maxSpeed){
        double avgEncoderPos =  (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() +RBMotor.getCurrentPosition()+ LBMotor.getCurrentPosition())/4;
        double distance = targetPosition - avgEncoderPos;
        double power = Range.clip(distance/500, -maxSpeed, maxSpeed);
        return power;
    }

    double getHeading(){
       return angles.firstAngle;
    }

    public void setTurnPower(double turnPower, double power){
        RFMotor.setPower(-turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(-turnPower - power);
        LBMotor.setPower(turnPower - power);
    }

    double turn(double targetAngle){
        getHeading();
        double angle = targetAngle - getHeading();
        telemetry.addData("Angle", angle);
        double power = Range.clip(angle/45, -0.3, 0.3);
        return power;
    }

    public void rampUp(double distance, double heading, double time, double MaxSpeed){
        double AccelerationSlope = MaxSpeed/t1.seconds();
        double power = AccelerationSlope * t1.seconds();
        if (Math.abs(power ) < EncoderSpeed(distance, MaxSpeed)){ // if acceleration is less than speed
            setTurnPower(turn(heading), power); // then set motor power to turn towards heading and accelerate until max speed
        } else{
            setTurnPower(turn(heading), EncoderSpeed(distance, MaxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
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

        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        rampUp(540.6, 0, 0.5, 0.5);
    }
}
