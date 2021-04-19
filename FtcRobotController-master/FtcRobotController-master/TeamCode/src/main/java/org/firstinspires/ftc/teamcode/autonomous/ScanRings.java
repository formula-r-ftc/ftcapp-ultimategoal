

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

@Autonomous
public class ScanRings<tfod> extends OpMode {

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";

    private static final String VUFORIA_KEY =
            "ASIwwWv/////AAABmR/+9d4sSkVEshzTIOkfUgAWTcQCqWQ3NeZFwrYj+HewIITQOcdzK95pLGiq3w+muSW12YMucPY4gr+LXUWae13of2pAVIwC03KapsTkznFaL5vJQvBSmir72Q0XFzO975UhES7phEj54qmV0HANvVXc9SVvzljLiSJvJt/6eDUEyqco/rUOnneZhEarLqZch8ma+TNUbWnNO4HnNu+E31xQVjR1ADGmSpln14EFvrLD22aWyGRFufLDPxMNZ0+HYMQg2rmyDK1HFxDnk6qpvtCYTjIXcLpUPXaDF5if3wIO3mDOaTk0OwdnBav9N1/bmwmYdEzjhRnTb7A8UCAnAUSxlAYIIH3WABg2FvfhQsRJ";

    private VuforiaLocalizer vuforia;
    public TFObjectDetector tfod;

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
    private DcMotor slides2;
    private DcMotor intake;
    private Servo WobbleArmR;
    private Servo WobbleArmL;
    private Servo RStack;
    private Servo LStack;

    double RFPreviousValue = 0;
    double RBPreviousValue = 0;
    double LFPreviousValue = 0;
    double LBPreviousValue = 0;


    double one = 537.6;

    ElapsedTime t1 = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();

    double AverageEncoderPosition;

    //  this gives you the distance and speed of motors
    double encoderSpeed(double targetPosition, double maxSpeed) {
        AverageEncoderPosition = (RFMotor.getCurrentPosition() - RFPreviousValue + LFMotor.getCurrentPosition() - LFPreviousValue + RBMotor.getCurrentPosition() - RBPreviousValue + LBMotor.getCurrentPosition() - LBPreviousValue) / 4;
        double distance = targetPosition - AverageEncoderPosition;
        //telemetry.addData("Encoder Speed distance",distance);
        double speed = Range.clip(distance / 500, -maxSpeed, maxSpeed); // clip the speed
        return speed;
    }

    public void setTurnPower(double turnPower, double power) {
        RFMotor.setPower(turnPower - power);
        LFMotor.setPower(-turnPower - power);
        RBMotor.setPower(turnPower - power);
        LBMotor.setPower(-turnPower - power);
    }

    double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
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


    public void rampUpTurn(double distance, double heading, double time, double maxSpeed, double busyTime) {
        double AvgEncPos = (RFMotor.getCurrentPosition() + LFMotor.getCurrentPosition() + RBMotor.getCurrentPosition() + LBMotor.getCurrentPosition()) / 4;
        double AccelerationSlope = maxSpeed / time;
        double power = t1.seconds() * AccelerationSlope;
        if (Math.abs(power) < Math.abs(encoderSpeed(distance, maxSpeed))) { // if acceleration is less than speed
            setTurnPower(turn(heading), power);  //then set motor power to turn towards heading and accelerate until max speed
        } else {
            if (!(Math.abs(heading - getHeading()) < 15)) {
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

    public void scan() {

        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    telemetry.addData("TFOD", "No Items Detected");
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

    double linearSlideInitPos = 0;

    private double linearSlideEncSpeed(double targetPosition, double maxSpeed) {
        double distance = targetPosition + linearSlideInitPos - slides.getCurrentPosition();
        telemetry.addData("LS distance", distance);
        double power = Range.clip(-distance / 500, -maxSpeed, maxSpeed);
        return power;
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

    boolean tripLoopDone = false;
    boolean EncoderPower;
    boolean tripLoop () {
        double AverageEncPower = (RFMotor.getPower() + LFMotor.getPower() + RBMotor.getPower() + LBMotor.getPower()) / 4;

        if (AverageEncPower == 0) {
            EncoderPower = false;
        } else {
            EncoderPower = true;
        }

        if (!tripLoopDone && EncoderPower) {
            tripLoopDone = true;
        }

        if (tripLoopDone && !EncoderPower) {
            RFPreviousValue = RFMotor.getCurrentPosition();
            RBPreviousValue = RBMotor.getCurrentPosition();
            LFPreviousValue = RFMotor.getCurrentPosition();
            LBPreviousValue = RFMotor.getCurrentPosition();
            return true;
        } else {
            telemetry.addData("tripLoop return:", "FALSE");
            return false;

        }
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
    boolean trip9 = false;
    boolean trip10 = false;
    boolean trip11 = false;
    boolean trip12 = false;
    boolean trip13 = false;
    boolean trip14 = false;
    boolean trip15 = false;
    boolean trip16 = false;
    boolean trip17 = false;
    boolean trip18 = false;
    boolean trip19 = false;
    boolean trip20 = false;
    boolean trip21 = false;
    boolean trip22 = false;
    boolean trip23 = false;
    boolean trip24 = false;
    boolean trip25 = false;
    boolean trip26 = false;
    boolean trip27 = false;
    boolean trip28 = false;

    public void moveTargetZoneA() {
        if (None == true) {
            //turn and move in front of target zone A
            if (!trip1) {
                rampUp(3 * one, -45, 0.5, 0.5, 5);
                trip1 = tripLoop();
                telemetry.addData("trip", "1");
            }
            // straighten the robot so its facing target zone A
            else if (trip1 && !trip2) {
                rampUpTurn(0 * one, 0, 0.5, 0.4, 5);
                trip2 = tripLoop();
                telemetry.addData("trip", "2");
            }
            // move into and drop the wobble inside target Zone A
            else if (trip2 && !trip3) {
                rampUp(2.5 * one, 0, 0.5, 0.6, 5);
                trip3 = tripLoop();
                telemetry.addData("trip", "3");
                WobbleArmL.setPosition(0.13);
                WobbleArmR.setPosition(0.35);
            }
            //move back
            else if (trip3 && !trip4) {
                WobbleArmL.setPosition(0.45);
                WobbleArmR.setPosition(0.04);
                rampUp(-0.65 * one, 0, 0.5, 0.5, 5);
                trip4 = tripLoop();
                telemetry.addData("trip", "4");
            }
            // turn towards powershot 2
            else if (trip4 && !trip5) {
                rampUpTurn(0, 34, 0.5, 0.5, 5);
                Shooter.setPower(0.8);
                trip5 = tripLoop();
                telemetry.addData("trip", "5");
            }
            //shoot
            else if (trip5 && !trip6) {
                boolean x = false;
                sleep(850);
                for (int i = 0; i < 1; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip6 = tripLoop();
                    telemetry.addData("trip", "6");
                }
            }// turn towards powershot 3
            else if (trip6 && !trip7) {
                rampUpTurn(0.46 * one, 47, 0.5, 0.5, 5);
                trip7 = tripLoop();
                telemetry.addData("trip", "5");
            }

            //shoot
            else if (trip7 && !trip8) {
                boolean x = false;
                for (int i = 0; i < 1; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip8 = tripLoop();
                    telemetry.addData("trip", "8");
                }
            }
            // turn towards powershot 1
            else if (trip8 && !trip9) {
                rampUpTurn(0.4 * one, 19, 0.5, 0.5, 5);
                ;
                trip9 = tripLoop();
                telemetry.addData("trip", "9");
            }
            //shoot
            else if (trip9 && !trip10) {
                boolean x = false;
                for (int i = 0; i < 1; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                Shooter.setPower(0);
                if (x == true) {
                    trip10 = tripLoop();
                    telemetry.addData("trip", "10");
                }
            }
            // turn towards second wobble
            else if (trip10 && !trip11) {
                rampUpTurn(0.25 * one, 166, 0.5, 0.5, 5);
                trip11 = tripLoop();
                telemetry.addData("trip", "11");
            }
            // move back
            else if (trip11 && !trip12) {
                rampUp(5 * one, 166, 0.5, 0.65, 5);
                trip12 = tripLoop();
                telemetry.addData("trip", "12");
            }
            // pick up
            else if (trip12 && !trip13) {
                WobbleArmL.setPosition(0.13);
                WobbleArmR.setPosition(0.35);
                sleep(300);
                trip13 = true;
                telemetry.addData("trip", "13");
            }
            // turn towards target zone A
            else if (trip13 && !trip14) {
                rampUpTurn(2 * one, -8, 0.5, 0.45, 5);
                trip14 = tripLoop();
                telemetry.addData("trip", "14");
            }
            // deliver second wobble
            else if (trip14 && !trip15) {
                rampUp(3.275 * one, -8, 0.5, 0.7, 5);
                trip15 = tripLoop();
                telemetry.addData("trip", "15");
            }
            //park
            else if (trip15 && !trip16) {
                WobbleArmL.setPosition(0.45);
                WobbleArmR.setPosition(0.04);
                rampUp(-one * 1.2, -8, 0.5, 0.65, 5);
                trip16 = tripLoop();
                telemetry.addData("trip", "16");
            }
            else if (trip16 && !trip17) {
                rampUpTurn(0, 45, 0.5, 0.65, 5);
                trip17 = tripLoop();
                telemetry.addData("trip", "17");
            }
            else if (trip17 && !trip18) {
                rampUp(one * 1.6, 45, 0.5, 0.65, 5);
                trip18 = tripLoop();
                telemetry.addData("trip", "18");
            }
        }
    }

    public void moveTargetZoneB() {
        if (Single == true) {
            //turn and move past the rings
            if (!trip1) {
                rampUp(2.9 * one, 15, 0.7, 0.5, 5);
                trip1 = tripLoop();
                telemetry.addData("trip", "1");
            }
            // turn to face target zone B
            else if (trip1 && !trip2) {
                rampUpTurn(0, -10, 0.5, 0.4, 5);
                trip2 = tripLoop();
                telemetry.addData("trip", "2");
            }
            //move the wobble into target zone
            else if (trip2 && !trip3) {
                rampUp(3.7 * one, -15, 0.9, 0.6, 5);
                trip3 = tripLoop();
                telemetry.addData("trip", "3");
                WobbleArmL.setPosition(0.45);
                WobbleArmR.setPosition(0.04);
            }

            // move in front of powershots
            else if (trip3 && !trip4) {
                rampUpTurn(0, -45, 0.5, 0.6, 5);
                trip4 = tripLoop();
                telemetry.addData("trip", "4");
            }
            else if (trip4 && !trip5) {
                rampUp(3 * -one, -45, 0.5, 0.6, 5);
                trip5 = tripLoop();
                Shooter.setPower(0.9);
                telemetry.addData("trip", "5");
            }
            // turn towards powershot1
            else if (trip5 && !trip6) {
                rampUpTurn(0, -2, 0.5, 0.1, 5);
                trip6 = tripLoop();
                telemetry.addData("trip", "6");
            }
            //shoot
            else if (trip6 && !trip7) {
                boolean x = false;
                for (int i = 0; i < 1; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip7 = true;
                    telemetry.addData("trip", "7");
                }
            }
            //turn towards powershot2
            else if (trip7 && !trip8) {
                rampUpTurn(0, 8, 0.5, 0.1, 5);
                trip8 = tripLoop();
                telemetry.addData("trip", "8");
            }
            //shoot
            else if (trip8 && !trip9) {
                boolean x = false;
                for (int i = 0; i < 1; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip9 = true;
                    telemetry.addData("trip", "9");WobbleArmL.setPosition(0.45);
            WobbleArmR.setPosition(0.04);
                }
            }
            //turn towards powershot3
            else if (trip9 && !trip10) {
                rampUpTurn(0, 10, 0.5, 0.1, 5);
                trip10 = tripLoop();
                telemetry.addData("trip", "10");
            }
            //shoot
            else if (trip10 && !trip11) {
//                rampUpTurn(50, 0, 0.5, 0.3, 5);
                boolean x = false;
                for (int i = 0; i < 1; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip11 = true;
                    telemetry.addData("trip", "11");
                }
            }
            // turn towards ring
            else if (trip11 && !trip12) {
                Shooter.setPower(0);
                rampUpTurn(0, 45, 0.5, 0.2, 5);
                intake.setPower(1);
                trip12 = tripLoop();
                telemetry.addData("trip", "12");
            }
            //move back and collect ring
            else if (trip12 && !trip13) {
                rampUp(-2.2 * one, 45, 0.5, 0.5, 5);
                trip13 = tripLoop();
                telemetry.addData("trip", "13");
            }
            // turn towards high goal
            else if (trip13 && !trip14) {
                Shooter.setPower(0.74);
                rampUpTurn(0, -1.5, 0.5, 0.3, 5);
                Shooter.setPower(1.0);
                trip14 = tripLoop();
                telemetry.addData("trip", "14");
            } else if (trip14 && !trip15) {
                        rampUp(-0.3 * one, -1.5, 0.5, 0.2, 5);
                trip15 = tripLoop();
                telemetry.addData("trip", "15");
            } else if (trip15 && !trip16) {
                rampUp(-0.5 * one, -1.5, 0.5, 0.2, 5);
                trip16 = tripLoop();
                telemetry.addData("trip", "16");
            }
            //shoot
            else if (trip16 && !trip17) {
                //rampUp(50, 0, 0.5, 0.3, 5);
                boolean x = false;
                for (int i = 0; i < 4; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    intake.setPower(0);
                    trip17 = true;
                    telemetry.addData("trip", "17");
                }
            }
            // turn towards wobble
            else if (trip17 && !trip18) {
                Shooter.setPower(0.0);
                rampUpTurn(0, -160, 0.5, 0.5, 5);
                trip18 = tripLoop();
                telemetry.addData("trip", "18");
            }
            // pick up wobble
            else if (trip18 && !trip19) {
                Shooter.setPower(0.0);
                rampUp(200, -158.5, 0.5, 0.5, 5);
                trip19 = tripLoop();
                telemetry.addData("trip", "19");
            }
            else if (trip19 && !trip20) {
                WobbleArmL.setPosition(0.13);
                WobbleArmR.setPosition(0.35);
                rampUpTurn(-1.5 * one, 10, 0.5, 0.5, 5);
                trip20 = tripLoop();
                telemetry.addData("trip", "20");
            }
            //deliver
            else if (trip20 && !trip21) {
                WobbleArmL.setPosition(0.45);
                WobbleArmR.setPosition(0.04);
                rampUp(5.7 * one, 10, 0.5, 0.7, 5);
                trip21 = tripLoop();
                telemetry.addData("trip", "21");
            }
            //park
            else if (trip21 && !trip22) {
                rampUp(-0.5 * one, 10, 0.5, 0.7, 5);
                trip22 = tripLoop();
                telemetry.addData("trip", "22");
            }
        }
    }

    public void moveTargetZoneC() {
        if (Quad == true) {
            // move in between the wobble and rings
            if (!trip1) {
                rampUp(1.55 * one, 0, 0.5, 0.5, 5);
                trip1 = tripLoop();
                telemetry.addData("trip", "1");
            }
            // turn towards the wall
            else if (trip1 && !trip2) {
                rampUpTurn(0, -60, 0.5, 0.4, 5);
                trip2 = tripLoop();
                telemetry.addData("trip", "2");
            }
            // move forward in front of target zone c
            else if (trip2 && !trip3) {
                rampUp(one, -60, 0.5, 0.7, 5);
                trip3 = tripLoop();
                telemetry.addData("trip", "3");
            }
            // turn towards target zone C
            else if (trip3 && !trip4) {
                rampUpTurn(0 * one, 0, 0.5, 0.4, 5);
                trip4 = tripLoop();
                telemetry.addData("trip", "4");
            }
//            //move and drop wobble
            else if (trip4 && !trip5) {
                rampUp(5.3 * one, 0, 0.5, 0.7, 5);
                WobbleArmL.setPosition(0.45);
                WobbleArmR.setPosition(0.04);
                trip5 = tripLoop();
                telemetry.addData("trip", "5");
            }
            //turn towards shooting position
            else if (trip5 && !trip6) {
                rampUpTurn(0 * one, -30, 0.5, 0.3, 5);
                trip6 = tripLoop();
                telemetry.addData("trip", "6");
            }
            //move backward
            else if (trip6 && !trip7) {
                rampUp(-3.5 * one, -30, 0.5, 0.7, 5);
                Shooter.setPower(1);
                trip7 = tripLoop();
                telemetry.addData("trip", "7");
            }
            //Strighten towards Goal
            else if (trip7 && !trip8) {
                RStack.setPosition(0.5);
                LStack.setPosition(0);
                rampUpTurn(0, 0, 0, 0.5, 0);
                trip8 = tripLoop();
                telemetry.addData("trip", "8");
            }
            //Move Back
            else if (trip8 && !trip9) {
                rampUp(-1.97 * one, 0, 0, 0.5, 0);
                trip9 = tripLoop();
                telemetry.addData("trip", "9");
            }
            // shoots 3 rings
            else if (trip9 && !trip10) {
                boolean x = false;
                for (int i = 0; i < 4; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    RStack.setPosition(0);
                    LStack.setPosition(0.48);
                    trip10 = tripLoop();
                    telemetry.addData("trip", "10");
                }
            }
//            move back to intake
            else if (trip10 && !trip11) {
                intake.setPower(1);
                rampUp(-1.97 * one, 0, 0, 0.4, 0);
                trip11 = tripLoop();
                telemetry.addData("trip", "11");
            }
            //move up to shoot
            else if (trip11 && !trip12) {
                rampUp(.91 * one, 0, 0.5, 0.5, 0);
                trip12 = tripLoop();
                telemetry.addData("trip", "12");
            }
            // shoots 3 rings
            else if (trip12 && !trip13) {
                boolean x = false;
                for (int i = 0; i < 5; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip13 = tripLoop();
                    telemetry.addData("trip", "13");
                }
            }
//            intake 1 ring
            else if (trip13 && !trip14) {
                RStack.setPosition(0);
                LStack.setPosition(0.48);
                rampUp(-2 * one, 0, 0.5, 0.4, 0);
                trip14 = tripLoop();
                telemetry.addData("trip", "14");
            }
            //move to shoot
            else if (trip14 && !trip15) {
                rampUp(2 * one, 0, 0.5, 0.5, 0);
                trip15 = tripLoop();
                telemetry.addData("trip", "15");
            }
            // shoots rings
            else if (trip15 && !trip16) {
                boolean x = false;
                for (int i = 0; i < 5; i++) {
                    telemetry.addData("Counter is", "push " + i);
                    telemetry.update();
                    Pusher.setPosition(0.3);
                    sleep(300);
                    Pusher.setPosition(0.0);
                    sleep(330);
                    x = true;
                }
                if (x == true) {
                    trip16 = tripLoop();
                    telemetry.addData("trip", "16");
                }
            }
            else if (trip16 && !trip17) {
                Shooter.setPower(0);
                rampUpTurn(0 * one, -160, 0.5, 0.5, 0);
                trip17 = tripLoop();
                telemetry.addData("trip", "17");
            }
            else if (trip17 && !trip18) {
                rampUp(0.94 * one, -160, 0.5, 0.5, 0);
                trip18 = tripLoop();
                telemetry.addData("trip", "18");
            }
            else if (trip18 && !trip19) {
                WobbleArmL.setPosition(0.13);
                WobbleArmR.setPosition(0.35);
                intake.setPower(0);
                rampUpTurn(-2 * one, 0, 0.5, 0.5, 0);
                trip19 = tripLoop();
                telemetry.addData("trip", "19");
            }
            else if (trip19 && !trip20) {
                rampUp(6.5 * one, 0, 0.5, 1.0, 0);
                trip20 = tripLoop();
                telemetry.addData("trip", "20");
            }
            else if (trip20 && !trip21) {
                rampUp(-0.8 * one, 0, 0.5, 1.0, 0);
                trip21 = tripLoop();
                telemetry.addData("trip", "21");
            }
        }
    }


    public void initHardware () {
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
        Shooter = hardwareMap.get(DcMotor.class, "Shooter");
        Pusher = hardwareMap.get(Servo.class, "Pusher");
        intake = hardwareMap.get(DcMotor.class, "intake");
        RStack = hardwareMap.get(Servo.class, "RStack");
        LStack = hardwareMap.get(Servo.class, "LStack");
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
//       slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Gyro stuff
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    public void slideSpeed ( double targetPosition, double maxSpeed){
        double EncoderPosition = slides.getCurrentPosition();
        double distance = targetPosition - EncoderPosition;
        //telemetry.addData("Encoder Speed distance",distance);
        double speed = Range.clip(distance / 100, -maxSpeed, maxSpeed); // clip the speed
        if (!(Math.abs(distance - EncoderPosition) < 80)) {
            slides.setPower(speed);
        }
    }

    boolean closed = false;
    boolean drop = false;
    public void holdwobble () {
        WobbleArmL.setPosition(0.28);
        WobbleArmR.setPosition(0.495);
        if (WobbleArmR.getPosition() == 0.495 && WobbleArmL.getPosition() == 0.28) {
            closed = true;
        }
        if (drop) {
            WobbleArmL.setPosition(0.45);
            WobbleArmR.setPosition(0.04);
        }
    }

    @Override
    public void init () {
        initHardware();
        initVuforia();
        initTfod();
    }


    @Override
    public void init_loop () {
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0 / 9.0);
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
    public void start () {
        t1.reset();
    }

    @Override
    public void stop () {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    boolean up = false;
    @Override
    public void loop () {
        if (!closed) {
            holdwobble();
        } else {
            scan();
            moveTargetZoneA();
            moveTargetZoneB();
            moveTargetZoneC();
        }
        telemetry.addData("none: ", None);
        telemetry.addData("quad: ", Quad);
        telemetry.addData("single: ", Single);
        telemetry.addData("heading: ", getHeading());
        telemetry.addData("ShooterPower:", Shooter.getPower());
        telemetry.addData("Slides Encoders: ", slides.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.addData("RFMotor encoder:", RFMotor.getCurrentPosition());
        telemetry.update();
    }

    //sleep methods
    public final void idle () {
        Thread.yield();
    }
    public final void sleep ( long milliseconds){
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}


