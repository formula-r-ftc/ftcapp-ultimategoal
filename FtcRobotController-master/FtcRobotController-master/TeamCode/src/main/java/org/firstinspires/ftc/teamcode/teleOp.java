package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.lang.annotation.Target;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class teleOp extends OpMode {
// initializing and variables
    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Shooter;
    private Servo Pusher;
    private DcMotor slides;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime t2 = new ElapsedTime();
    ElapsedTime t3 = new ElapsedTime();
    ElapsedTime t4 = new ElapsedTime();

    boolean flyWeel = false;

    int booleanIncrementer = 0;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();

    //drivetrian method
    public void moveDriveTrain(){
        if(gamepad1.left_bumper){
            LFMotor.setPower(0.6*(gamepad1.right_stick_y));
            LBMotor.setPower(0.6*(gamepad1.right_stick_y));
            RFMotor.setPower(0.6*(-gamepad1.left_stick_y));
            RBMotor.setPower(0.6*(-gamepad1.left_stick_y));
        }else {
            LFMotor.setPower(gamepad1.right_stick_y);
            LBMotor.setPower(gamepad1.right_stick_y);
            RFMotor.setPower(-gamepad1.left_stick_y);
            RBMotor.setPower(-gamepad1.left_stick_y);
        }
    }

    // shooter method
    public void shoot() {
        double g1rt = gamepad1.right_trigger;

        boolean g1rtPressed = ifPressedFloat(g1rt);

        if(g1rtPressed && Shooter.getPower() == 0.0){
            Shooter.setPower(-1);
        }else if (gamepad1.right_trigger > 0.1 && t1.seconds() > 0.1){
            Shooter.setPower(0);
        }

        booleanIncrementer = 0;
    }
//sleep methods
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


    // pusher method
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
                for (int i=0; i<3; i++){
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.27);
                    sleep(330);
                    Pusher.setPosition(0.0);
                    sleep(400);

                }
            }
        }
    }



    public void moveLinearSlide(){
        slides.setPower(gamepad1.left_trigger);


    }

    //Linear Slide Methods
    double initPosition = -60;
    double linearSlideInitPos = 0;
    private double linearSlideEncSpeed(double targetPosition, double maxSpeed){
        double difference = targetPosition + linearSlideInitPos - slides.getCurrentPosition();
        telemetry.addData("LS Difference", difference);
        double power = Range.clip(-difference / 200, -maxSpeed, maxSpeed);
        return power;
    }

    double targetPosition;
    public void slideButtons(){
        double a = -300 + initPosition, b = -1100 + initPosition, y = -2000 + initPosition, x = -2900 + initPosition;

        boolean g1a = gamepad1.a;
        boolean g1b = gamepad1.b;
        boolean g1y = gamepad1.y;
        boolean g1x = gamepad1.x;

        boolean g1aPressed = ifPressed(g1a);
        boolean g1bPressed = ifPressed(g1b);
        boolean g1yPressed = ifPressed(g1y);
        boolean g1xPressed = ifPressed(g1x);

        if(ifPressed(gamepad1.a)){
            targetPosition = -300 + initPosition;

        } else if (ifPressed(gamepad1.b) ){
            targetPosition = -1100 + initPosition;

        } else if (ifPressed(gamepad1.y)){
            targetPosition = -2000 + initPosition;
        } else if (ifPressed(gamepad1.x)){
            targetPosition = -2900 + initPosition;
        }
            booleanIncrementer = 0;
            slides.setPower(linearSlideEncSpeed(targetPosition, 0.75));

            telemetry.addData("a: ", a);
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
        if (button >= 0.1){
            buttonBoolean = true;
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



    }

    @Override
    public void init_loop() {
        targetPosition = initPosition;
        slides.setPower(linearSlideEncSpeed(targetPosition, 0.75));

        telemetry.addData("a: ", ifPressed(gamepad1.a));
        telemetry.addData("initPos: ", initPosition);
        telemetry.addData("targetPosition: ", targetPosition);
        telemetry.addData("slides Positiion", slides.getCurrentPosition());
        telemetry.update();
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
        slideButtons();
        //moveLinearSlide();

        //LinearSlides movements



        telemetry.addData("gampad1.a: ", ifPressed(gamepad1.a));
        telemetry.addData("initPos: ", initPosition);
        telemetry.addData("targetPosition: ", targetPosition);
        telemetry.addData("slides Positiion", slides.getCurrentPosition());
        telemetry.update();

    }


}