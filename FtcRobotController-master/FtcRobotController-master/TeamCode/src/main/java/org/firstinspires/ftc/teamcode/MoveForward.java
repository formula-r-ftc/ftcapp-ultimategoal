package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous
public class MoveForward extends OpMode {

    BNO055IMU imu;
    Orientation angles;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;


    double one = 537.6;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    //  this gives you the distance and speed of encoders
        double encoderSpeed(double targetPosition, double maxSpeed){
            double AverageEncoderPosition = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() +RBMotor.getCurrentPosition()+ LBMotor.getCurrentPosition())/4;
            double distance = targetPosition - AverageEncoderPosition;
            //telemetry.addData("Encoder Speed distance",distance);
            double speed = Range.clip(distance/500, -maxSpeed, maxSpeed); // clip the speed
            return speed;
        }

        public void setTurnPower(double turnPower, double power){
            RFMotor.setPower(turnPower - power);
            LFMotor.setPower(-turnPower - power);
            RBMotor.setPower(turnPower - power);
            LBMotor.setPower(-turnPower - power);
        }

        double getHeading(){
            angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            this.imu.getPosition();
            return angles.firstAngle;
        }


        double turn(double targetAngle) {
                getHeading();
                double turnAngle = targetAngle - getHeading();
               // telemetry.addData("turnAngle", turnAngle);
                double power = Range.clip(turnAngle / 50, -0.3, 0.3);
            return power;
        }
        double AvgEncPos = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() +RBMotor.getCurrentPosition()+ LBMotor.getCurrentPosition())/4;
        public void rampUp(double distance, double heading, double time, double maxSpeed, double busyTime) {
            double AccelerationSlope = maxSpeed / time;
            double power = t1.seconds() * AccelerationSlope;
            if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
                setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
            } else {
                // if(AvgEncPos < ){
                if (runtime.seconds() < busyTime) {
                    telemetry.addData("motor is: ", "busy");
                    setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
                } else {
                    telemetry.addData("motor is: ", "not busy");
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                    setTurnPower(0,0);
                }
            }
        }

        boolean tripLoopDone = false;
        boolean EncoderPower;
        boolean tripLoop(){
            double AverageEncPower = (RFMotor.getPower() + LFMotor.getPower() + RBMotor.getPower() + LBMotor.getPower())/4;

            if (AverageEncPower == 0){
                EncoderPower = false;
            } else{
                EncoderPower = true;
            }

            if (!tripLoopDone && EncoderPower){
                tripLoopDone = true;
            }

            if (tripLoopDone && !EncoderPower){
                return  true;
            }
            else {
                return false;
            }
        }


    public final void idle() {
        // Otherwise, yield back our thread scheduling quantum and give other threads at
        // our priority level a chance to run
        Thread.yield();
    }
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }


    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("status", "initialized");

        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

        //gyro stuff
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();
    }
    @Override
    public void start() {
        t1.reset();
        runtime.reset();
    }
    boolean trip1 = false;
    boolean trip2 = false;
    boolean trip3 = false;
    boolean trip4 = false;
    boolean trip5 = false;
    boolean trip6 = false;
    boolean trip7 = false;
    boolean trip8 = false;
    @Override
    public void loop(){
        if (!trip1) {
        rampUp(2*one, 0, 0.5, 0.5, 5);
        trip1 = tripLoop();
        }else if(trip1 && !trip2) {
        rampUp(0,90, 0.5, 0.2, 10);
        trip2 = tripLoop();
        } else if (trip2 && !trip3){
            rampUp(2*one, 90, 0.5, 0.3, 15);
            trip3 = tripLoop();
        } else if(trip3 && !trip4){
            rampUp(-one, 0,0.5,0.3,20);
            trip4 = tripLoop();
        }


        telemetry.addData("Runtime: ", runtime.seconds());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("LFMotor encoder:", LFMotor.getCurrentPosition());
        telemetry.addData("RBMotor encoder:", RBMotor.getCurrentPosition());
        telemetry.addData("LBMotor encoder:", LBMotor.getCurrentPosition());
        telemetry.update();
    }
}
