package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class FormulaRTeleOp extends OpMode {
    // initializing and variables
    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Shooter;

//    ElapsedTime t1 = new ElapsedTime();
//    ElapsedTime t2 = new ElapsedTime();
//    ElapsedTime t3 = new ElapsedTime();
//    ElapsedTime t4 = new ElapsedTime();

    public void setTurnPower(double turnPower, double power){
        RFMotor.setPower(turnPower - power);
        LFMotor.setPower(-turnPower - power);
        RBMotor.setPower(turnPower - power);
        LBMotor.setPower(-turnPower - power);
    }

public class ledLight {
    if RFMotor.setpower() == 0;
        make it red;
    }
