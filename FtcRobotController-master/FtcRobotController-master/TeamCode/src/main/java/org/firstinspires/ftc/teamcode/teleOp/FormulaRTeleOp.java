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
    private DcMotor intake;
    private Servo WobbleArmR;
    private Servo WobbleArmL;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime t2 = new ElapsedTime();
    ElapsedTime t3 = new ElapsedTime();
    ElapsedTime t4 = new ElapsedTime();

    boolean Outake = false;

    int booleanIncrementer = 0;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();


    //First driver controls start
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

//    pusher method;
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
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);

                }
            }
        }
    }

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
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);

                }
            }
        }
    }

    //First Driver controls done

    //Second Driver controls start
    //Shooter method
    public void shoot() {

        if((gamepad1.right_trigger) > 0.3 && t1.seconds() > 0.3 && Shooter.getPower() == 0){
            Shooter.setPower(1);
        }else if ((gamepad1.right_trigger) > 0.3 && t1.seconds() > 0.3){
            Shooter.setPower(0);
        }

    }

    //Wobble arm method
    public void WobbleArm() {
        if (gamepad2.right_bumper) {
            WobbleArmL.setPosition(0.35);
            WobbleArmR.setPosition(0.1);
        } else{
            WobbleArmL.setPosition(0);
            WobbleArmR.setPosition(0.5);
        }
        telemetry.addData("WobbleArmLPosition", WobbleArmL);
        telemetry.addData("WobbleArmRPosition", WobbleArmR);
    }

// linear slides stuff------------------------------------------------------------------------------
    double linearSlideInitPos = 0;

    private double linearSlideEncSpeed(double targetPosition, double maxSpeed){
        double distance = targetPosition + linearSlideInitPos - slides.getCurrentPosition();
        telemetry.addData("LS distance", distance);
        double power = Range.clip(-distance / 500, -maxSpeed, maxSpeed);
        return power;
    }

    public void slidePos(double position, double time, double maxSpeed){
        double Acceleration = maxSpeed/time;
        double power = t1.seconds() * Acceleration;
        if (Math.abs(power) < linearSlideEncSpeed(position, maxSpeed)){
            slides.setPower(linearSlideEncSpeed(position, maxSpeed));
        } else {
            slides.setPower(linearSlideEncSpeed(position, maxSpeed));
        }
    }

    public void moveSlides(){
        boolean g2a = gamepad2.a;
        boolean g2b = gamepad2.b;
        boolean toggleReady1 = true;
        boolean toggleReady2 = true;


        if(g2a == false){
            toggleReady1 = true;
        }
        if (g2b == false) {
            toggleReady2 = true;
        }

        if (gamepad2.a && toggleReady1) {
            toggleReady1 = false;
            if (gamepad2.a && !(slides.getCurrentPosition()==1300)) {
                slidePos(1300, 0.5, 0.75);
            }
        }
        if (gamepad2.b){
            slidePos(100, 0.5, 0.75);
        }
    }

    //intake method
    public void Intake(){
        if((gamepad1.right_stick_button)){
            intake.setPower(-1);
        }else if (gamepad1.a) {
            intake.setPower(0);
        }
        if ((gamepad1.left_trigger) > 0.5 && t1.seconds() > 0.5){
            intake.setPower(1);
        }
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        intake = hardwareMap.get(DcMotor.class, "intake");
        WobbleArmL = hardwareMap.get(Servo.class, "WobbleArmL");
        WobbleArmR = hardwareMap.get(Servo.class, "WobbleArmR");

        slides = hardwareMap.get(DcMotor.class, "slides");

        linearSlideInitPos = slides.getCurrentPosition();
//        slidePos(200, 0.5, 0.75);
//
    }

    @Override
    public void init_loop() {



        telemetry.addData("a: ", ifPressed(gamepad1.a));
        telemetry.addData("WobbleArmLPosition", WobbleArmL.getPosition());
        telemetry.addData("WobbleArmRPosition", WobbleArmR.getPosition());
        telemetry.update();

        telemetry.addData("slides Pos: ", slides.getCurrentPosition());
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
        moveSlides();
        telemetry.addData("servoArmPosL", WobbleArmL.getPosition());
        telemetry.addData("servoArmPosR", WobbleArmR.getPosition());
        telemetry.addData("intakePower", intake.getPower());
        telemetry.addData("gamepad1.a: ", ifPressed(gamepad1.a));
        telemetry.addData("slides  Position", slides.getCurrentPosition());
        telemetry.addData("slides Pos: ", slides.getCurrentPosition());
        telemetry.addData("WobbleArmLPosition", WobbleArmL.getPosition());
        telemetry.addData("WobbleArmRPosition", WobbleArmR.getPosition());
        telemetry.update();

   }


}