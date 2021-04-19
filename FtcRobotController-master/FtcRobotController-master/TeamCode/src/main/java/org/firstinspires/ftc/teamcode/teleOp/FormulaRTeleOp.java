 package org.firstinspires.ftc.teamcode.teleOp;

import android.text.util.Rfc822Token;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class FormulaRTeleOp extends OpMode {

    //initializing and variables
    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Shooter;
    private Servo Pusher;
    private DcMotor slides;
//    private DcMotor slides2;
    private DcMotor intake;
    private Servo WobbleArmR;
    private Servo WobbleArmL;
    private Servo RStack;
    private Servo LStack;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime t2 = new ElapsedTime();
    ElapsedTime t3 = new ElapsedTime();
    ElapsedTime t4 = new ElapsedTime();

    boolean Outake = false;

    int booleanIncrementer = 0;

    int buttonInc = 0;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();

    //DriveTrain method
    public void moveDriveTrain(){
        if(gamepad1.left_bumper){
            LFMotor.setPower(0.6*(gamepad1.right_stick_y));
            LBMotor.setPower(0.6*(gamepad1.right_stick_y));
            RFMotor.setPower(0.6*(-gamepad1.left_stick_y));
            RBMotor.setPower(0.6*(-gamepad1.left_stick_y));

            telemetry.addData("joystick", gamepad1.right_stick_y);
            telemetry.addData("joystick", gamepad1.left_stick_y);

        }else {
            LFMotor.setPower(gamepad1.right_stick_y);
            LBMotor.setPower(gamepad1.right_stick_y);
            RFMotor.setPower(-gamepad1.left_stick_y);
            RBMotor.setPower(-gamepad1.left_stick_y);

            telemetry.addData("joystick", gamepad1.right_stick_y);
            telemetry.addData("joystick", gamepad1.left_stick_y);
        }
    }

    //Push 3 times method
    public void push(){
        boolean g1rb = gamepad1.right_bumper;
        boolean toggleReady = true;
        double servoArmPos = Pusher.getPosition();

        if(g1rb == false){
            toggleReady = true;
        }
        if(g1rb && toggleReady){
            toggleReady = false;
            if (servoArmPos == 0.0 && gamepad1.right_bumper){
                for (int i = 0; i<3; i++){
                    moveDriveTrain();
                    telemetry.addData("Counter is", "push " + i);
//                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);

                }
                targetPosition = initPosition;
                targetPosition2 = initPosition2;
            }
        }
    }

    //Push 1 time method
    public void push1(){
        boolean g1rb = gamepad1.a;
        boolean toggleReady = true;
        double servoArmPos = Pusher.getPosition();


        if(g1rb == false){
            toggleReady = true;
        }
        if(g1rb && toggleReady){
            toggleReady = false;
            if (servoArmPos == 0.0 && gamepad1.a){
                for (int i = 0; i<1; i++){
                    moveDriveTrain();
                    telemetry.addData("Counter is", "push " + i);
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    buttonInc++;
                    telemetry.addData("button pressed:", buttonInc);
//                    telemetry.update();
                }
            }

                if (buttonInc == 3) {
                    targetPosition = initPosition;
                    targetPosition2 = initPosition2;
                    buttonInc = 0;
                }
        }
    }

    //Shooter method
    public void shoot() {

        if((gamepad1.right_trigger) > 0.3 && t1.seconds() > 0.3 && Shooter.getPower() == 0){
            Shooter.setPower(1);
        }else if ((gamepad1.y)){
            Shooter.setPower(0);
        }

    }

//    Intake method
    public void Intake(){
        if ((gamepad1.left_trigger) > 0.5 && t1.seconds() > 0.5) {
            intake.setPower(1);
        } else if((gamepad1.right_stick_button)) {
            intake.setPower(-1);
        } else if (gamepad1.b) {
            intake.setPower(0);
        }

    }


    //Wobble arm method
    public void WobbleArm() {
        if (gamepad2.right_bumper) {
            WobbleArmL.setPosition(0.1);
            WobbleArmR.setPosition(0.38  );
        } else{
            WobbleArmL.setPosition(0.45);
            WobbleArmR.setPosition(0.04);

        }
        telemetry.addData("WobbleArmLPosition", WobbleArmL);
        telemetry.addData("WobbleArmRPosition", WobbleArmR);
    }

    public void StackArm() {
        if (gamepad2.left_bumper) {
            RStack.setPosition(0.48);
            LStack.setPosition(0);
        } else {
            RStack.setPosition(0);
            LStack.setPosition(0.48);

        }
    }

    //Linear Slide methods ------------------------------------------------------------------------

    double linearSlideInitPos = 0;

    private double linearSlideEncSpeed(double targetPosition, double maxSpeed){
        double distance = targetPosition + linearSlideInitPos - slides.getCurrentPosition();
        telemetry.addData("LS distance", distance);
        double power = Range.clip(-distance / 500, -maxSpeed, maxSpeed);
        return power;
    }
//    private double linearSlide2EncSpeed(double targetPosition, double maxSpeed){
//        double distance = targetPosition + linearSlideInitPos - slides2.getCurrentPosition();
//        telemetry.addData("LS 2 distance", distance);
//        double power = Range.clip(distance / 500, -maxSpeed, maxSpeed);
//        return power;
//    }

    public void slidePos(double position, double time, double maxSpeed){
        double Acceleration = maxSpeed/time;
        double power = t1.seconds() * Acceleration;
        if (Math.abs(power) < linearSlideEncSpeed(position, maxSpeed)){
            slides.setPower(linearSlideEncSpeed(position, maxSpeed));
        } else {
            slides.setPower(linearSlideEncSpeed(position, maxSpeed));
        }
    }


    public void linearSlidesStick(){
        slides.setPower(gamepad2.left_stick_y);

//        if(gamepad2.left_stick_y > 0.0 || gamepad2.left_stick_y < 0.0){
//            targetPosition = slides.getCurrentPosition();
//        }

    }
    double initPosition;
    double initPosition2;
    double targetPosition;
    double targetPosition2;
    public void slideButtons( ){
         initPosition = 0;
         initPosition2 = 0;

        if(ifPressed(gamepad2.a)) {
            targetPosition = 700 + initPosition;
            targetPosition2 = 700 + initPosition2;

        } else if (ifPressed(gamepad2.b) ) {
            targetPosition = initPosition;
            targetPosition2 = initPosition2;
        }
        else if (ifPressed(gamepad2.x)){
            targetPosition = 2200;
        }


        slides.setPower(linearSlideEncSpeed(targetPosition, 1));
//        slides2.setPower(linearSlide2EncSpeed(targetPosition2, 1));
        booleanIncrementer = 0;
    }

    // toggle switch methods
    private boolean ifPressed(boolean button){
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer){
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (button != buttonWas && button == true){
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer = booleanIncrementer+1;
        return output;

    }

    private boolean ifPressedFloat(double button){
        boolean output = false;
        boolean buttonBoolean = false;
        if (button >= 0.25){            buttonBoolean = true;
        }
        if (booleanArray.size() == booleanIncrementer){
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (buttonBoolean != buttonWas && buttonBoolean == true){
            output = true;
        }
        booleanArray.set(booleanIncrementer, buttonBoolean);

        booleanIncrementer = booleanIncrementer+1;
        return output;

    }

    //sleep methods
    public final void idle() {
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
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        slides = hardwareMap.get(DcMotor.class, "slides");
//        slides2 = hardwareMap.get(DcMotor.class, "slides2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        intake = hardwareMap.get(DcMotor.class, "intake");
        WobbleArmL = hardwareMap.get(Servo.class, "WobbleArmL");
        WobbleArmR = hardwareMap.get(Servo.class, "WobbleArmR");
        linearSlideInitPos = slides.getCurrentPosition();
        RStack = hardwareMap.get(Servo.class, "RStack");
        LStack = hardwareMap.get(Servo.class, "LStack");

    }

    @Override
    public void init_loop() {

        telemetry.addData("a: ", ifPressed(gamepad1.a));
        telemetry.addData("WobbleArmLPosition", WobbleArmL.getPosition());
        telemetry.addData("WobbleArmRPosition", WobbleArmR.getPosition());
        telemetry.addData("RStackPosition", LStack.getPosition());
        telemetry.addData("LStackPosition", RStack.getPosition());
        telemetry.addData("slides Pos: ", slides.getCurrentPosition());



//        telemetry.addData("slides2 Pos: ", slides2.getCurrentPosition());
        telemetry.update();



        //slides2.setPower(linearSlide2EncSpeed(initPosition2, 0.75));



    }

    @Override
    public void start(){
        t2.reset();

    }

    @Override
    public void loop() {
        moveDriveTrain();
        push();
        shoot();
        Intake();
        WobbleArm();
        push1();
//        slideButtons();
        StackArm();
        linearSlidesStick();
                telemetry.addData("servoArmPosL", WobbleArmL.getPosition());
        telemetry.addData("servoArmPosR", WobbleArmR.getPosition());
        telemetry.addData("intakePower", intake.getPower());
        telemetry.addData("gamepad1.a: ", ifPressed(gamepad1.a));
        telemetry.addData("slides  Position", slides.getCurrentPosition());
        telemetry.addData("slides Pos: ", slides.getCurrentPosition());
        telemetry.addData("WobbleArmLPosition", WobbleArmL.getPosition());
        telemetry.addData("WobbleArmRPosition", WobbleArmR.getPosition());
//        telemetry.addData("slides2 Pos: ", slides2.getCurrentPosition());
        telemetry.addData("buttonInc", buttonInc);
        telemetry.update();

    }

}