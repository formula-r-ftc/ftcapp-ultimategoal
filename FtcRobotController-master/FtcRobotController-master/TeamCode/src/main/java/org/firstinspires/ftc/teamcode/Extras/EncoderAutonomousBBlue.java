package org.firstinspires.ftc.teamcode.Extras;
import android.graphics.Color;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class EncoderAutonomousBBlue extends OpMode {
    BNO055IMU imu;
    Orientation angles;
    private DcMotor RFMotor;
    private DcMotor RBMotor;
    private DcMotor LFMotor;
    private DcMotor LBMotor;
    private CRServo servoArm;
    private RevColorSensorV3 sensorColor;

    private ElapsedTime t1 = new ElapsedTime();
    private ElapsedTime t2 = new ElapsedTime();
    private ElapsedTime t3 = new ElapsedTime();
    static final double one = 746.66666666;
    static final double two = 2 * 746.66666666;
    static final double three = 3 * 746.66666666;
    static final double four = 4 * 746.66666666;
    static final double FORWARD_SPEED = 0.2;
    static final double FORWARD_SPEED_FAST = -0.5;

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
        getHeading();
        double difference = Math.abs(targetAngle - getHeading());
        telemetry.addData("difference in turn is", difference);
        double power = Range.clip(difference / 45, -0.5, 0.5);
        return power;

    }

    private double encoderSpeed(double targetEncoderValue, double maxSpeed) {
        double avgEncPosition = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
        double difference = targetEncoderValue - avgEncPosition;
        telemetry.addData("difference", difference);
        double power = Range.clip(difference / 500, -maxSpeed, maxSpeed);
        return power;
    }

    private double encoderSpeedSide(double targetPosition, double maxSpeed) {
        double avgEncPosition = (-LFMotor.getCurrentPosition() - RBMotor.getCurrentPosition() + RFMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
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

    private double getHeading() {
        //angles=imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //this.imu.getPosition();
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

    private void moveFowardAndBackwards(double length, double heading, double time, double maxSpeed) {
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

    private void moveBackwardSlow() {
        RFMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        LFMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        RBMotor.setPower(0.5*-FORWARD_SPEED_FAST);
        LBMotor.setPower(0.5*-FORWARD_SPEED_FAST);
    }
    private void moveArmDown(){
        t3.reset();
        while (t3.seconds() < 0.5){
            servoArm.setPower(FORWARD_SPEED_FAST);
        }
    }

    private void moveArmUp(){
        t3.reset();
        while (t3.seconds() < 3.0){
            servoArm.setPower(-FORWARD_SPEED_FAST);
        }
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        // msStuckDetectInit=1000;
        // msStuckDetectStop=1000;
        //msStuckDetectInitLoop=5000;
        //msStuckDetectStart=1000;
        msStuckDetectLoop=31000;


        LFMotor = hardwareMap.get(DcMotor.class, "LFMotor");
        RFMotor = hardwareMap.get(DcMotor.class, "RFMotor");
        LBMotor = hardwareMap.get(DcMotor.class, "LBMotor");
        RBMotor = hardwareMap.get(DcMotor.class, "RBMotor");
        servoArm = hardwareMap.get(CRServo.class, "servoArm");
        sensorColor = hardwareMap.get(RevColorSensorV3.class, "Color");

        LFMotor.setDirection(DcMotor.Direction.FORWARD);
        RFMotor.setDirection(DcMotor.Direction.REVERSE);
        LBMotor.setDirection(DcMotor.Direction.FORWARD);
        RBMotor.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData("Status", "Initialized");

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
    }

    @Override
    public void loop() {

        //move forward 40 inches
        moveFowardAndBackwards(one*3.25, 0.0, 0.5, 0.5);
        //slide right 27 inches
        moveSideways(-one*2.5, 0.0, 0.5, 0.5);
        //move servo down
        moveArmDown();
        //slide left 27 inches
        moveSideways(one, 0.0, 0.5, 0.5);
        //move back 33 inches
        //moveFowardAndBackwards(-one*2.54, 0.0, 0.5, 0.5);
        //slide right 19 inches
        //moveSideways(-one*1.46, 0.0, 0.5, 0.5);
        //move forward 6 inches
        //moveFowardAndBackwards(one*0.46, 0.0, 0.5, 0.5);
        //slide left 19 inches
        //moveSideways(one*2.54, 0.0, 0.5, 0.5);
        //move servo up
        moveArmUp();
        //move back until blue line is sensed
        while (hsvValues[0] < 200.0 || hsvValues[0] > 220.0 ){
            colorSense();
            moveBackwardSlow();
        }
    }
}
