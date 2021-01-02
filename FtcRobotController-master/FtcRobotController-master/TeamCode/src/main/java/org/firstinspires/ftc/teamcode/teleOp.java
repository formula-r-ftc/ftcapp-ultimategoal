package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class teleOp extends OpMode {

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

    boolean flyWeel = false;

    ArrayList<Boolean> booleanArray = new ArrayList<Boolean>();

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

    public void shoot() {
        if (gamepad1.right_trigger > 0.3 && t1.seconds() > 0.3)
            if (!flyWeel) {
                Shooter.setPower(-1);
                flyWeel = true;
            } else if (flyWeel && gamepad1.right_trigger > 0.3 && t3.seconds() > 0.3){
                Shooter.setPower(0);
                flyWeel = false;
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
    double getHeading(){
        angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        return angles.firstAngle;
    }

   /* private boolean ifPressed(boolean button){
        boolean output = false;
        int booleanIncrementer = 0;
        if (booleanArray.size() == booleanIncrementer){
            booleanArray.add(false);
        }
        boolean buttonWas = booleanArray.get(booleanIncrementer);
        if (button != buttonWas && button == true){
            output = true;
        }
        booleanArray.set(booleanIncrementer,button);
        booleanIncrementer = booleanIncrementer +1;
        return output;
    }
    private boolean ifPressedFloat(double button){
        boolean output = false;
        boolean buttonBoolean = false;
        int booleanIncrementer = 0;

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
        booleanArray.set(booleanIncrementer,button);
        booleanIncrementer = booleanIncrementer +1;
        return output;
    }*/


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

        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.addData("slides Positiion", slides.getCurrentPosition());
        telemetry.update();

        slides.setPower(0);

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
        moveLinearSlide();

    }

}