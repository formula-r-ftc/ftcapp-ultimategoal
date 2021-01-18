
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@Autonomous

public class ScanAutonomous<tfod> extends OpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ASIwwWv/////AAABmR/+9d4sSkVEshzTIOkfUgAWTcQCqWQ3NeZFwrYj+HewIITQOcdzK95pLGiq3w+muSW12YMucPY4gr+LXUWae13of2pAVIwC03KapsTkznFaL5vJQvBSmir72Q0XFzO975UhES7phEj54qmV0HANvVXc9SVvzljLiSJvJt/6eDUEyqco/rUOnneZhEarLqZch8ma+TNUbWnNO4HnNu+E31xQVjR1ADGmSpln14EFvrLD22aWyGRFufLDPxMNZ0+HYMQg2rmyDK1HFxDnk6qpvtCYTjIXcLpUPXaDF5if3wIO3mDOaTk0OwdnBav9N1/bmwmYdEzjhRnTb7A8UCAnAUSxlAYIIH3WABg2FvfhQsRJ";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    BNO055IMU imu;
    Orientation angles;
    DcMotor RFMotor;
    DcMotor LFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;

    double one = 537.6;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime t2 = new ElapsedTime();

    //  this gives you the distance and speed of encoders
    double encoderSpeed(double targetPosition, double maxSpeed){
        double AverageEncoderPosition = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() +RBMotor.getCurrentPosition()+ LBMotor.getCurrentPosition())/4;
        double distance = targetPosition - AverageEncoderPosition;
        telemetry.addData("Encoder Speed distance",distance);
        double power = Range.clip(distance/500, -maxSpeed, maxSpeed); // y = m*x + b
        return power;
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
        telemetry.addData("turnAngle", turnAngle);
        double power = Range.clip(turnAngle / 50, -0.3, 0.3);
        return power;
    }

    public void rampUp(double distance, double heading, double time, double maxSpeed){
        double AccelerationSlope = maxSpeed/time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        }else{
            setTurnPower(turn(heading), encoderSpeed(distance, maxSpeed));// otherwise keep motor power to heading and stop at the target Encoder Position
        }
    }
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

    public void scan(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    telemetry.addData("TFOD","No Items Detected");
                    telemetry.addData("Target Zone", "A");
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
                            rampUp(537.6, 0.0,0.5, 0.2);
                        } else if (recognition.getLabel().equals("Quad")) {
                            telemetry.addData("Target Zone", "C");
                        } else {
                            telemetry.addData("Target Zone", "UNKNOWN");
                        }
                    }
                }

                telemetry.update();
            }

        }
        if (tfod != null) {
            tfod.shutdown();
        }

    }


    @Override
    public void init() {

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

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







    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @Override
    public void init_loop() {
        scan();
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
        t2.reset();
    }

    @Override
    public void loop(){
        rampUp(one,0, 0.5, 0.3);

        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.update();
    }
}