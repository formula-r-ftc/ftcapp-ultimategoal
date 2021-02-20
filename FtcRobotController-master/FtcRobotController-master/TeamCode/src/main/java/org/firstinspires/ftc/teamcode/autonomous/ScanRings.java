package org.firstinspires.ftc.teamcode.autonomous;//package org.firstinspires.ftc.teamcode.autonomous;
//package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.autonomous.autoMovements;

@Autonomous
public class ScanRings<tfod> extends OpMode {

    autoMovements run = new autoMovements();

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ASIwwWv/////AAABmR/+9d4sSkVEshzTIOkfUgAWTcQCqWQ3NeZFwrYj+HewIITQOcdzK95pLGiq3w+muSW12YMucPY4gr+LXUWae13of2pAVIwC03KapsTkznFaL5vJQvBSmir72Q0XFzO975UhES7phEj54qmV0HANvVXc9SVvzljLiSJvJt/6eDUEyqco/rUOnneZhEarLqZch8ma+TNUbWnNO4HnNu+E31xQVjR1ADGmSpln14EFvrLD22aWyGRFufLDPxMNZ0+HYMQg2rmyDK1HFxDnk6qpvtCYTjIXcLpUPXaDF5if3wIO3mDOaTk0OwdnBav9N1/bmwmYdEzjhRnTb7A8UCAnAUSxlAYIIH3WABg2FvfhQsRJ";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

    BNO055IMU imu;
    Orientation angles;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;
    DcMotor slides;
    private Servo WobbleArmR;
    private Servo WobbleArmL;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;


    double one = 537.6;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    double AverageEncoderPosition;

    //  this gives you the distance and speed of motors
    double encoderSpeed(double targetPosition, double maxSpeed){
        AverageEncoderPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
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

    public void rampUp(double distance, double heading, double time, double maxSpeed, double busyTime) {
        double AvgEncPos = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        } else {
            if (!(Math.abs(distance - AvgEncPos) < 80)) {
                telemetry.addData("motor is: ", "busy");
                setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
            } else {
                telemetry.addData("motor is: ", "not busy");
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                setTurnPower(0, 0);

            }
        }
    }



    public void rampUpTurn (double distance, double heading, double time, double maxSpeed, double busyTime) {
        double AvgEncPos = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        } else {
            if (!(Math.abs(heading - getHeading()) < 7)) {
                telemetry.addData("motor is: ", "busy");
                setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
            } else {
                telemetry.addData("motor is: ", "not busy");
                RFMotor.setPower(0);
                LFMotor.setPower(0);
                RBMotor.setPower(0);
                LBMotor.setPower(0);
                setTurnPower(0, 0);

            }
        }
    }

    //Start of tensorflow program -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    boolean Single = false;
    boolean Quad = false;
    boolean None = false;

    public void scan(){

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    telemetry.addData("TFOD","No Items Detected");
                    telemetry.addData("Target Zone", "A");
                    None = true;
                } else {
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions) {
                        telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                        telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                recognition.getLeft(), recognition.getTop());
                        telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                recognition.getRight(), recognition.getBottom());
                        if (recognition.getLabel().equals("Single")) {
                            telemetry.addData("Target Zone", "B");
                             Single = true;
                             Quad = false;
                             None = false;
                        } else if (recognition.getLabel().equals("Quad")) {
                            telemetry.addData("Target Zone", "C");
                             Quad = true;
                             Single = false;
                             None = false;
                        } else {
                            telemetry.addData("Target Zone", "None");
                            None = true;
                            Quad = false;
                            Single = false;
                        }
                    }
                }


            }

        }
        if (tfod != null) {
            tfod.shutdown();
        }

    }


    public void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

// THIS IS WHERE THE PROGRAM STARTS---------------------------------------------------------------------------------------------------------------------------------------------------------------------/
boolean trip1 = false;
    boolean trip2 = false;
    boolean trip3 = false;
    boolean trip4 = false;
    boolean trip5 = false;
    boolean trip6 = false;
    boolean trip7 = false;
    boolean trip8 = false;

    public void moveTargetZoneA(){
        if(None == true){
            if (!trip1) {
                rampUp(2.15 * one, -45, 0.5, 0.5, 5);
                trip1 = tripLoop();
                telemetry.addData("trip", "1");
            }
            else if(trip1 && !trip2) {
                rampUpTurn(0 * one,0, 0.5, 0.4, 5);
                trip2 = tripLoop();
                telemetry.addData("trip", "2");
            }
            else if (trip2 && !trip3){
                rampUp(3.15 * one, 0, 0.5, 0.5, 5);
                trip3 = tripLoop();
                telemetry.addData("trip", "3");
            }
            else if (trip3 && !trip4){
                rampUpTurn(0,-90, 0.5, 0.2,5);
                WobbleArmL.setPosition(0.35);
                WobbleArmR.setPosition(0.1);
                trip4 = tripLoop();
                telemetry.addData("trip", "4");
            }
//            else if (trip4 && !trip5){
//                rampUp(0,-110,0,0,0);
//                trip5 = tripLoop();
//                telemetry.addData("trip", "5");
//            }
            
        }
    }

    public void moveTargetZoneB(){
        if (Single == true){
            if (!trip1) {
                rampUp(2.9 * one, 15, 0.5, 0.5, 5);
                trip1 = tripLoop();
                telemetry.addData("trip", "1");
            } else if (trip1 && !trip2){
                rampUpTurn(0, -10, 0.5, 0.4, 5);
                trip2 = tripLoop();
                telemetry.addData("trip", "2");
            } else if (trip2 && !trip3){
                rampUp(3.5 * one, -15, 0.5, 0.6, 5);
                trip3 = tripLoop();
                telemetry.addData("trip", "3");
                WobbleArmL.setPosition(0);
                WobbleArmR.setPosition(0.5);
            }  else if (trip3 && !trip4){
                rampUp(1* -one, 0,0.5,0.2,5);
                trip4 = tripLoop();
                telemetry.addData("trip", "4");
            }

        }
    }

    public void moveTargetZoneC(){
        if(Quad == true){
            if (!trip1) {
                rampUp(2.2 * one, -45, 0.5, 0.5, 5);
                trip1 = tripLoop();
                telemetry.addData("trip", "1");
            }
            else if(trip1 && !trip2) {
                rampUpTurn(0,0, 0.5, 0.4, 5);
                trip2 = tripLoop();
                telemetry.addData("trip", "2");


            }
            else if (trip2 && !trip3){
                rampUp(6.55 * one, 0, 0.5, 0.7, 5);
                trip3 = tripLoop();
                telemetry.addData("trip", "3");
                WobbleArmL.setPosition(0.1);
                WobbleArmR.setPosition(0.1   );

            }  else if (trip3 && !trip4){
                WobbleArmL.setPosition(0);
                WobbleArmR.setPosition(0.5);
                rampUp(2.25* -one, 0,0.5,0.5,5);
                trip4 = tripLoop();
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

            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            return  true;
        }
        else {
            telemetry.addData("tripLoop return:", "FALSE" );
            return false;

        }
    }



    public void initHardware(){
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        WobbleArmL = hardwareMap.get(Servo.class, "WobbleArmL");
        WobbleArmR = hardwareMap.get(Servo.class, "WobbleArmR");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        slides = hardwareMap.get(DcMotor.class, "slides");

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
       // slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void slideSpeed(double targetPosition, double maxSpeed){
        double EncoderPosition = slides.getCurrentPosition();
        double distance = targetPosition - EncoderPosition;
        //telemetry.addData("Encoder Speed distance",distance);
        double speed = Range.clip(distance/100, -maxSpeed, maxSpeed); // clip the speed
        if (!(Math.abs(distance - EncoderPosition) < 80)){
            slides.setPower(speed);
        }
    }

    boolean closed = false;
    boolean drop = false;
    public void holdwobble(){
        WobbleArmL.setPosition(0.49);
        WobbleArmR.setPosition(0.31);
        if (WobbleArmR.getPosition() == 0.31 && WobbleArmL.getPosition() == 0.49){
            closed = true;
        }
        if (drop){
            WobbleArmL.setPosition(0);
            WobbleArmR.setPosition(0.5);
        }
    }

    @Override
    public void init() {
        initHardware();
        initVuforia();
        initTfod();

       }



    @Override
    public void init_loop() {
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }


        telemetry.addData("WobbleArmR ", WobbleArmR.getPosition());
        telemetry.addData("WobbleArmL ", WobbleArmL.getPosition());
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


        //linear slides
        double initPosition = slides.getCurrentPosition();
        telemetry.addData("slide encoders;", initPosition);

    }

    @Override
    public void start() {
        t1.reset();
    }


    @Override
    public void stop() {
        if (tfod != null){
            tfod.shutdown();
        }
    }

boolean up = false;
    @Override
    public void loop(){
        if (!closed){
           holdwobble();
        }else {
            scan();
            moveTargetZoneA();
            moveTargetZoneB();
            moveTargetZoneC();
        }
        telemetry.addData("none: ", None);
        telemetry.addData("quad: ", Quad);
        telemetry.addData("single: ", Single);
        telemetry.addData("Slides Encoders: ",  slides.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.update();
    }
}
