package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@TeleOp
public class MecanumDrive extends OpMode {

    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Intake;
    private Servo Lindexer;
    private CRServo LindexerP;
    private Servo PivotL;
    private Servo PivotR;
    private DcMotor ShooterR;
    private DcMotor ShooterL;

    private ElapsedTime t1 = new ElapsedTime();

    int booleanIncrementer = 0;
    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();

    public void moveDriveTrain() {
        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;

        vertical = -gamepad1.left_stick_y;
        horizontal = -gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            RFMotor.setPower(0.25 * (pivot + (vertical + horizontal)));
            RBMotor.setPower(0.25 * (pivot + (vertical - horizontal)));
            LFMotor.setPower(0.25 * (-pivot + (vertical - horizontal)));
            LBMotor.setPower(0.25 * (-pivot + (vertical + horizontal)));
        } else {
            RFMotor.setPower(pivot + (vertical + horizontal));
            RBMotor.setPower(pivot + (vertical - horizontal));
            LFMotor.setPower(-pivot + (vertical - horizontal));
            LBMotor.setPower(-pivot + (vertical + horizontal));
        }
    }

    // pivots the lindexer
    public void tilt() {
        boolean g1b = gamepad1.b;
        boolean toggleReady = true;
        double PivotLPosition = PivotL.getPosition();
        double PivotRPosition = PivotR.getPosition();

        if (g1b == false) {
            toggleReady = true;
        }
//
//        if (g1b && toggleReady) {
//            toggleReady = false;
//            if (PivotRPosition == 0   && gamepad1.b) {
////                PivotL.setPosition(0);
//                PivotR.setPosition(0.18);
//            } else if (PivotRPosition == 0.18 && gamepad1.b) {
////                PivotL.setPosition(0.18);
//                PivotR.setPosition(0);
//            }
//        }

        if (gamepad1.b) {
            PivotL.setPosition(0.022);
            PivotR.setPosition(0.17);
        } else {
            PivotL.setPosition(0.18);
            PivotR.setPosition(0);
        }

    }

    // Shoots

    public void Pusher() {

        boolean g1rb = gamepad1.right_bumper;
        boolean toggleReady = true;
        double servoArmPos = Lindexer.getPosition();

        if(g1rb == false){
            toggleReady = true;
        }
        if(g1rb && toggleReady){
            toggleReady = false;
            if (servoArmPos == 0.0 && gamepad1.right_bumper){
                for (int i = 0; i<1; i++){
                    moveDriveTrain();
                    telemetry.addData("Counter is", "push " + i);
                    Lindexer.setPosition(0.2);
                    sleep(300);
                    Lindexer.setPosition(0.75);
                    sleep(330);

                }
            }
        }

        boolean g1x = gamepad1.x;
        boolean toggleReady2 = true;
        int slideIncrementer = 0;
        telemetry.addData("slideIncrementer:", slideIncrementer);

        if(g1x == false){
            toggleReady2 = true;
        }
        if(g1x && toggleReady2 && slideIncrementer == 0) {
            slideIncrementer++;
        }
        if(g1x && toggleReady2 && slideIncrementer == 1) {
            slideIncrementer++;
        }
        if(g1x && toggleReady2 && slideIncrementer == 2) {
            slideIncrementer++;
        }
        if(g1x && toggleReady2 && slideIncrementer == 3) {
            slideIncrementer++;
        }


        if (slideIncrementer == 1){
            Lindexer.setPosition(0.46);
        }
        else if(slideIncrementer == 2){
            Lindexer.setPosition(0.35);
        } else if(slideIncrementer == 3){
            Lindexer.setPosition(0.2);
        }
        else if(slideIncrementer == 4){
            Lindexer.setPosition(0.75);
            slideIncrementer = 0;
        }

//        if (gamepad1.right_bumper) {
//            Lindexer.setPosition(0.2);
//        } else {
//            Lindexer.setPosition(0.75);
//        }
    }

    public void shoot(){
        if((gamepad1.right_trigger) > 0.3 && t1.seconds() > 0.3 && ShooterR.getPower() == 0){
            ShooterR.setPower(1);
        } else if ((gamepad1.left_stick_button)){
            ShooterR.setPower(0);
        }

        if((gamepad1.right_trigger) > 0.3 && t1.seconds() > 0.3 && ShooterL.getPower() == 0){
            ShooterL.setPower(1);
        } else if ((gamepad1.left_stick_button)){
            ShooterL.setPower(0);
        }
    }

    public void Intake() {
        if ((gamepad1.left_trigger) > 0.5 && t1.seconds() > 0.5) {
            Intake.setPower(-1);
        } else if (gamepad1.right_stick_button) {
            Intake.setPower(1);
        } else if (gamepad1.y) {
            Intake.setPower(0);
        }
    }

    public final void idle () {
        Thread.yield();
    }
    public final void sleep ( long milliseconds) {
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
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        PivotL = hardwareMap.get(Servo.class, "PivotL");
        PivotR = hardwareMap.get(Servo.class, "PivotR");
        Lindexer = hardwareMap.get(Servo.class, "Lindexer");
        ShooterR = hardwareMap.get(DcMotor.class, "ShooterR");
        ShooterL = hardwareMap.get(DcMotor.class, "ShooterL");
//        LindexerP = hardwareMap.get(CRServo.class, "LindexerP");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        moveDriveTrain();
        Pusher();
        tilt();
        Intake();
        shoot();

        telemetry.addData("Left Joystick: ", gamepad1.left_stick_y);
        telemetry.addData("right Joystick: ", gamepad1.right_stick_y);
        telemetry.addData("LindexerPosition: ", Lindexer.getPosition());
        telemetry.addData("PivotR: ", PivotR.getPosition());
        telemetry.addData("PivotL: ", PivotL.getPosition());
        telemetry.update();
    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }
        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);
        booleanIncrementer = booleanIncrementer + 1;
        return output;

    }
}