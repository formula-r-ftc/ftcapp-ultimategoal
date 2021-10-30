package org.firstinspires.ftc.teamcode.FreightFrenzyLeage0;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class FreightFrenzyTeleOp extends OpMode {
    private DcMotor RFMotor;
    private DcMotor LFMotor;
    private DcMotor RBMotor;
    private DcMotor LBMotor;
    private DcMotor Arm;

    private Servo Clamp;

    ElapsedTime t1 = new ElapsedTime();

    public void moveDriveTrain() {
        double vertical = 0;
        double horizontal = 0;
        double pivot = 0;
        vertical = gamepad1.left_stick_y;
        horizontal = gamepad1.left_stick_x;
        pivot = gamepad1.right_stick_x;



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


    public void Clamp(){
        if (gamepad2.b) {
            Clamp.setPosition(0.1);
        }
        else{
            Clamp.setPosition(0.5);
        }
    }

    double linearSlideInitPos = 0;

    private double ArmMovement(double targetPosition, double maxSpeed){
        double distance = targetPosition + linearSlideInitPos - Arm.getCurrentPosition();
        telemetry.addData("LS distance", distance);
        double power = Range.clip(distance / 500, -maxSpeed, maxSpeed);
        return power;
    }

    public void ArmPos(double position, double time, double maxSpeed){
        double Acceleration = maxSpeed/time;
        double power = t1.seconds() * Acceleration;
        if (Math.abs(power) < ArmMovement(position, maxSpeed)){
            Arm.setPower(ArmMovement(position, maxSpeed));
        } else {
            Arm.setPower(ArmMovement(position, maxSpeed));
        }
    }

    public void pickUpMineral(){
        if (gamepad1.a){
            ArmPos(0,0.5,0.75);
        }else {
            ArmPos(700, 0.5, 0.75);
            telemetry.addData("Arm pos: ", Arm.getCurrentPosition());
        }
    }

    public void moveArm(){
        Arm.setPower(0.4*gamepad2.left_stick_y);
//        if (gamepad2.left_stick_y > 0.5) {
//            ArmPos(0,0.5,0.75);
//        }else if(gamepad2.left_stick_y < -0.5){
//            ArmPos(700, 0.5, 0.75);
//        }
//        telemetry.addData("Arm pos: ", Arm.getCurrentPosition());
    }

    @Override
    public void init() {
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        Clamp = hardwareMap.get(Servo.class, "Clamp");
        Arm = hardwareMap.get(DcMotor.class, "Arm");

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void init_loop() {
        telemetry.addData("Arm Pos: ", Arm.getCurrentPosition());
        telemetry.addData("clamp position:", Clamp.getPosition());
        telemetry.update();
    }

    @Override
    public void loop() {
        moveDriveTrain();
        Clamp();
//        pickUpMineral();
        moveArm();

        telemetry.addData("Left Joystick: ", gamepad1.left_stick_y);
        telemetry.addData("right Joystick: ", gamepad1.right_stick_y);
        telemetry.addData("clamp position:", Clamp.getPosition());
        telemetry.update();
    }
}
