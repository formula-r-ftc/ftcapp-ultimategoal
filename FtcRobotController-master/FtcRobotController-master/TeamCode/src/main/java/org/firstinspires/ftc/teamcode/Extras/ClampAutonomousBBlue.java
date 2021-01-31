package org.firstinspires.ftc.teamcode;
import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous

public class ClampAutonomousBBlue extends OpMode{

    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    //private CRServo servoArm;
    private ColorSensor sensorColor;
    private Servo clamperL;
    private Servo clamperR;

    private ElapsedTime t1 = new ElapsedTime();
    private ElapsedTime t2 = new ElapsedTime();
    private ElapsedTime t3 = new ElapsedTime();
    private ElapsedTime t4 = new ElapsedTime();
    private ElapsedTime t5 = new ElapsedTime();
    static final double one = 746.66666666;
    static final double onepointfive =1.5*746.66666666;
    static final double two = 2 * 746.66666666;
    static final double three = 3 * 746.66666666;
    static final double four = 4 * 746.66666666;
    static final double righthalf = -373.33333333;
    static final double FORWARD_SPEED = 0.2;
    static final double FORWARD_SPEED_FAST = -0.5;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;

    boolean trip1 = false;
    boolean trip2 = false;
    boolean trip3 = false;
    boolean trip4 = false;
    boolean trip5 = false;
    boolean trip6 = false;
    boolean trip7 = false;

    float hsvValues[] = {0F, 0F, 0F};
    final float values[] = hsvValues;
    final double SCALE_FACTOR = 255;

    private void moveToPosition(double targetPosition) {
        double difference = targetPosition - RFMotor.getCurrentPosition();
        telemetry.addData("difference", difference);
        setTankPower(-difference / 500);
    }

    private void moveOne(DcMotor a, double targetPosition) {
        a.getCurrentPosition();
        double difference = targetPosition - a.getCurrentPosition();
        telemetry.addData("difference", difference);
        double power = Range.clip(-difference / 250, -0.5, 0.5);
        a.setPower(power);
    }

    private double turn(double targetAngle) {

        double current = getHeading();
        double difference = targetAngle - current;
        telemetry.addData("difference in turn is", difference);
        double power = Range.clip(difference / 45, -0.5, 0.5);
        return power;

    }

    private double encoderSpeed(double targetEncoderValue, double maxSpeed) {
        double avgEncPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double difference = targetEncoderValue - avgEncPosition;
        telemetry.addData("difference", difference);
        double power = Range.clip(difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

    private double encoderSpeedSide(double targetPosition, double maxSpeed) {
        double avgEncPosition = (-(LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;
        double difference = targetPosition - avgEncPosition;
        telemetry.addData("difference", difference);
        double power = Range.clip(difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

    private void setTurnPower(double turnPower, double power) {
        RFMotor.setPower(-turnPower - power);
        LFMotor.setPower(turnPower - power);
        RBMotor.setPower(-turnPower - power);
        LBMotor.setPower(turnPower - power);
        telemetry.addData("turn power", turnPower);
    }

    private void driveSideways(double turnPower, double encoderSpeedSide) {
        RFMotor.setPower(-turnPower - encoderSpeedSide);
        LFMotor.setPower(turnPower + encoderSpeedSide);
        RBMotor.setPower(-turnPower + encoderSpeedSide);
        LBMotor.setPower(turnPower - encoderSpeedSide);

    }

    private void move(double targetPosition) {
        moveOne(RFMotor, targetPosition);
        moveOne(LFMotor, targetPosition);
        moveOne(RBMotor, targetPosition);
        moveOne(LBMotor, targetPosition);
    }


    private void setTankPower(double power) {
        telemetry.addData("power", power);
        RFMotor.setPower(FORWARD_SPEED);
        LFMotor.setPower(FORWARD_SPEED);
        RBMotor.setPower(FORWARD_SPEED);
        LBMotor.setPower(FORWARD_SPEED);
    }

    double lastAngle = 0;
    double dAngle = 0;
    private double getHeading() {
        //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //this.imu.getPosition();
       /* double difference = angles.firstAngle - lastAngle;
        if (difference > 180) {
            dAngle = dAngle - 360;
        }
        else if (difference < -180) {
            dAngle = dAngle = 360;
        }
         double lastAngle = angles.firstAngle;*/
        return angles.firstAngle;
    }

    private void rampUp(double length, double heading, double time, double maxSpeed) {
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(length, maxSpeed))) {
            setTurnPower(turn(heading), power);
        } else {
            setTurnPower(turn(heading), encoderSpeed(length, maxSpeed));

        }

    }

    private void rampUpSide(double length, double heading, double time, double maxSpeed) {
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeedSide(length, maxSpeed))) {
            driveSideways(turn(heading), power);
        } else {
            driveSideways(turn(heading), encoderSpeedSide(length, maxSpeed));

        }

    }

    boolean tripLoopSidewaysDone = false;
    private boolean tripLoopSideways(double length){
        double avgEncPosition = ((-LFMotor.getCurrentPosition() - LFPreviousValue) - (RBMotor.getCurrentPosition() - RBPreviousValue) + (RFMotor.getCurrentPosition() - RFPreviousValue) + (LBMotor.getCurrentPosition() - LBPreviousValue)) / 4;

        if (!tripLoopSidewaysDone && Math.abs(length - avgEncPosition) < 100) {
            tripLoopSidewaysDone = true;
            t2.reset();
        }
        if (tripLoopSidewaysDone && t2.seconds() > 2) {
            tripLoopSidewaysDone = false;
            telemetry.addData ("IN TLSW", "SHOULD NOT SEE THIS");
            telemetry.update();
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            return true;
        }
        else {
            return false;
        }
    }

    boolean tripLoopDone = false;
    private boolean tripLoop(double length) {
        double avgEncPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition()  - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;

        if (!tripLoopDone && Math.abs(length - avgEncPosition) < 100) {
            tripLoopDone = true;
            t2.reset();
        }
        if (tripLoopDone && t2.seconds() > 1) {
            tripLoopDone = false;
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            return true;
        }
        else {
            return false;
        }
    }

    boolean tripLoopTurnDone = false;
    private boolean tripLoopTurn(double turn) {

        if (!tripLoopTurnDone && Math.abs(turn - getHeading()) < 20) {
            tripLoopTurnDone = true;
            t2.reset();
        }
        if (tripLoopTurnDone && t2.seconds() > 1) {
            tripLoopTurnDone = false;
            return true;
        }
        else {
            return false;
        }
    }


    private void moveForwardAndBackwards(double length, double heading, double time, double maxSpeed) {
        t1.reset();
        t2.reset();

        boolean done = false;
        while (true) {
            rampUp(length, heading, time, maxSpeed);
            double avgEncPosition = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
            if (!done && Math.abs(length - avgEncPosition) < 100) {
                done = true;
                t2.reset();
            }
            if (done == true && t2.seconds() > 1) {
                break;
            }
        }
    }

    private void rotation(double length, double heading, double time, double maxSpeed) {
        t1.reset();
        t2.reset();

        boolean done = false;
        while (true) {
            setTurnPower(turn(heading), encoderSpeed(length, maxSpeed));
            telemetry.update();
            if (!done && Math.abs(heading - Math.abs(getHeading())) <= 10) {
                done = true;
                t2.reset();
            }
            if (done == true && t2.seconds() > 1) {
                break;
            }
        }
    }
    private void moveSideways(double length, double heading, double time, double maxSpeed) {
        t1.reset();
        t2.reset();

        boolean done = false;
        while (true) {
            rampUpSide(length, heading, time, maxSpeed);
            double avgEncPosition = (-LFMotor.getCurrentPosition() - RBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
            if (!done && Math.abs(length - avgEncPosition) < 100) {
                done = true;
                t2.reset();
            }
            if (done == true && t2.seconds() > 1) {
                break;
            }
        }
    }

    private void colorSense() {
        Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                (int) (sensorColor.green() * SCALE_FACTOR),
                (int) (sensorColor.blue() * SCALE_FACTOR),
                hsvValues);

        // send the info back to driver station using telemetry function.
        telemetry.addData("Alpha", sensorColor.alpha());
        telemetry.addData("Red  ", sensorColor.red());
        telemetry.addData("Green", sensorColor.green());
        telemetry.addData("Blue ", sensorColor.blue());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.update();
    }
    boolean sensedBlue = false;
    boolean ends = false;
    private void forwardUntilBlue(){
        if (hsvValues[0] < 195.0 || hsvValues[0] > 225.0){
            colorSense();
            setTurnPower(turn(0.0), 0.25);
        }
        else{
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            sensedBlue = true;
            ends = true;
        }
    }

    private void moveBackwardSlow() {
        RFMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        LFMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        RBMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        LBMotor.setPower(0.5*-FORWARD_SPEED_FAST);
    }
    boolean reset3 = false;
    /*boolean servoPosDown = false;
    private void moveArmDown(){
        if (!reset3){
            t3.reset();
            reset3 = true;
        }
        if (t3.seconds() < 1.0){
            servoArm.setPower(FORWARD_SPEED_FAST);
        }
        else {
            servoArm.setPower(0);
            servoPosDown = true;
            reset3 = false;
        }
    }*/
    boolean clampDown = false;
    /*private void clamp(){
        if (!reset3){
            t3.reset();
            reset3 = true;
        }
        if (t3.seconds() < 1.0){
            clamperL.setPower(0.5);
            clamperR.setPower(-0.5);
            ;
        }
        else {
            clampDown = true;
            reset3 = false;
        }
    }*/
    boolean clampUp = false;
   /* private void release(){
        if (!reset3){
            t3.reset();
            reset3 = true;
        }
        if (t3.seconds() < 1.0){
            clamperL.setPosition(0);
            clamperR.setPosition(0);
            ;
        }
        else {
            clampUp = true;
            reset3 = false;
        }
    }*/

    /*boolean servoPosUp = false;
    private void moveArmUp(){
        if (!reset3){
            t3.reset();
            reset3 = true;
        }
        if (t3.seconds() < 1.0){
            servoArm.setPower(-FORWARD_SPEED_FAST);
        }
        else {
            servoArm.setPower(0);
            servoPosUp = true;
            reset3 = false;
        }


    }*/
    boolean resetA4 = true;
    private void moveClampsDown(){
        t4.reset();

        while (t4.seconds() < 0.9){
            clamperL.setPosition(0.5);
            clamperR.setPosition(0.0);
            resetA4 = false;
        }
        if (!resetA4)
            clampDown = true;
    }

    boolean resetA5 = true;
    private void moveClampsUp(){
        t5.reset();

        while (t5.seconds() < 0.5){
            clamperL.setPosition(0.0);
            clamperR.setPosition(0.5);
            resetA5 = false;
        }
        if (!resetA5)
            clampUp = true;
    }
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // msStuckDetectInit=1000;
        // msStuckDetectStop=1000;
        //msStuckDetectInitLoop=5000;
        //msStuckDetectStart=1000;
        msStuckDetectLoop=10000;


        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        //servoArm = hardwareMap.get(CRServo.class, "servoArm");
        sensorColor = hardwareMap.get(ColorSensor.class, "Color");
        clamperL = hardwareMap.get(Servo.class, "clamperR");
        clamperR = hardwareMap.get(Servo.class, "clamperL");

        imu = hardwareMap.get(BNO055IMU.class, "emu");

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");
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
        parameters.loggingEnabled=false;

        imu.initialize(parameters);


    }
    @Override
    public void init_loop() {
        telemetry.addData("LF Distance", LFMotor.getCurrentPosition());
        telemetry.addData("RF Distance", RFMotor.getCurrentPosition());
        telemetry.addData("LB Distance", LBMotor.getCurrentPosition());
        telemetry.addData("RB Distance", RBMotor.getCurrentPosition());

        moveOne(RFMotor, 0);
        moveOne(LFMotor, 0);
        moveOne(RBMotor, 0);
        moveOne(LBMotor, 0);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", angles.firstAngle);
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.update();
    }

    @Override
    public void start() {
        t1.reset();
        t2.reset();
        t3.reset();
    }

    @Override
    public void loop() {
        if(!trip1){
            rampUpSide(-one*3.1, 0.0, 0.5, 0.5);
            trip1 = tripLoopSideways(-one*3.1);
        }
        else if(!trip2){
            rampUp(-one*0.875, 0.0, 0.5, 0.25);
            trip2 = tripLoop(-one*0.875);
        }
        else if (!clampDown){
            moveClampsDown();
        }
        else if (!trip3){
            rampUp(one*2.25, 0.0, 0.5,0.5);
            trip3 = tripLoop(one*2.25);
        }
        else if (!trip4){
            rampUp(-0.0, 90.0, 0.5, -0.5);
            trip4 = tripLoopTurn(90.0);
        }
        else if (!trip5){
            rampUp(-one*0.0625, 90.0, 0.5, 0.5);
            trip5 = tripLoop(-one*0.0625);
        }
        else if (!clampUp){
            moveClampsUp();
        }
        else if (!trip6){
            rampUp(one*2.85, 90.0, 0.5, 0.75);
            trip6 = tripLoop(one*2.85);
        }
        /*else if (!trip7){
            rampUpSide(-one*0.125, 0.0, 0.5, 0.5);
            trip6 = tripLoopSideways(-one*0.125);
        }
        else {
            rampUp(0.0, 90.0, 0.5, 0.5);
        }*/
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading: ", getHeading());
        telemetry.addData("cool", angles.firstAngle);
        telemetry.addData("Timer 3", t3.seconds());
        telemetry.addData("Roll: ", angles.secondAngle);
        telemetry.addData("Pitch: ", angles.thirdAngle);
        telemetry.addData("dangle", dAngle);
        telemetry.addData("QWR", reset3);
        telemetry.addData("Trip 4",trip4);
        telemetry.addData("Timer 2", t2.seconds());
        telemetry.addData("Ends", ends);
        telemetry.update();
        getHeading();
    }

}

