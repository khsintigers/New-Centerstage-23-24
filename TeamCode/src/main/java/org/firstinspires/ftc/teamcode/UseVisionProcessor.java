package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.StagePropVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;


@Autonomous()
public class UseVisionProcessor extends LinearOpMode{

    /* Declare OpMode members. */
    private DcMotor left_front   = null;
    private DcMotor left_rear = null;
    private DcMotor right_front  = null;
    private DcMotor right_rear  = null;
    static final double     COUNTS_PER_MOTOR_REV    = 2150.8/4 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.9 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.1;
    static final double     TURN_SPEED              = 0.5;


    private ElapsedTime     runtime = new ElapsedTime();

    private StagePropVisionProcessor drawRectangleProcessor;
    private VisionPortal visionPortal;
    ColorSensor sensorColor;
    DistanceSensor sensorDistance;
    // hsvValues is an array that will hold the hue, saturation, and value information.
    private float hsvValues[] = {0F, 0F, 0F};

    // values is a reference to the hsvValues array.
    private final float values[] = hsvValues;

    // sometimes it helps to multiply the raw RGB values with a scale factor
    // to amplify/attentuate the measured values.
    private final double SCALE_FACTOR = 255;


    public void initVision() {
        drawRectangleProcessor = new StagePropVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "MyEye"), drawRectangleProcessor);
    }

    public void runOpMode() {
        initVision();
        // Initialize the drive system variables.
        left_rear  = hardwareMap.get(DcMotor.class, "RearLeft");
        right_rear = hardwareMap.get(DcMotor.class, "RearRight");
        left_front  = hardwareMap.get(DcMotor.class, "FrontLeft");
        right_front = hardwareMap.get(DcMotor.class, "FrontRight");
        sensorColor = hardwareMap.get(ColorSensor.class, "Sensor Color");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "Sensor Color");



        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_rear.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.FORWARD);
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);

        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                left_rear.getCurrentPosition(),
                right_rear.getCurrentPosition(),
                left_front.getCurrentPosition(),
                right_front.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(DRIVE_SPEED,  36,  36, 5.0, 0);  // S1: Forward 47 Inches with 5 Sec timeout
            encoderDrive(DRIVE_SPEED, 18, 18, 5.0, 2);
//            encoderDrive(DRIVE_SPEED, 24, 24, 5.0, 3);
//            encoderDrive(DRIVE_SPEED,  24,  24, 5.0, 1);


//        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

            telemetry.addData("Path", "Complete");
            telemetry.update();
        }

        sleep(1000);  // pause to display final telemetry message.
    }
//    @Override
//    public void init_loop() {
//    }

//    @Override
//    public void start() {
//        //visionPortal.stopStreaming();
//    }

    /*
    Add back in later, figure out how to thread loops
    add into the encoderDrive Loop

    @Override
    public void loop() {
        telemetry.addData("Identified", drawRectangleProcessor.getSelection());
        telemetry.addData( "Left Avg: ",drawRectangleProcessor.getLeftAvg());
        telemetry.addData( "Middle Avg: ",drawRectangleProcessor.getMiddleAvg());
        telemetry.addData( "Right Avg: ",drawRectangleProcessor.getRightAvg());
        telemetry.update();

    }
    */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS, int FBLR) {
        int newLeftTargetR;
        int newRightTargetR;
        int newLeftTargetF;
        int newRightTargetF;
        double offsetLRPerc = 1.154;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            if (FBLR == 0) {
                newLeftTargetR = left_rear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTargetR = right_rear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newLeftTargetF = left_front.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTargetF = right_front.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            } else if (FBLR == 1) {
                newLeftTargetR = left_rear.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                newRightTargetR = right_rear.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
                newLeftTargetF = left_front.getCurrentPosition() + (int)(-leftInches * COUNTS_PER_INCH);
                newRightTargetF = right_front.getCurrentPosition() + (int)(-rightInches * COUNTS_PER_INCH);
            } else if (FBLR == 2) {
                newLeftTargetR = left_rear.getCurrentPosition() + (int) (leftInches * offsetLRPerc * COUNTS_PER_INCH);
                newRightTargetR = right_rear.getCurrentPosition() + (int) (-rightInches * offsetLRPerc * COUNTS_PER_INCH);
                newLeftTargetF = left_front.getCurrentPosition() + (int) (-leftInches * offsetLRPerc * COUNTS_PER_INCH);
                newRightTargetF = right_front.getCurrentPosition() + (int) (rightInches * offsetLRPerc * COUNTS_PER_INCH);
            } else if (FBLR == 3) {
                newLeftTargetR = left_rear.getCurrentPosition() + (int) (-leftInches *offsetLRPerc * COUNTS_PER_INCH);
                newRightTargetR = right_rear.getCurrentPosition() + (int) (rightInches *offsetLRPerc* COUNTS_PER_INCH);
                newLeftTargetF = left_front.getCurrentPosition() + (int) (leftInches *offsetLRPerc* COUNTS_PER_INCH);
                newRightTargetF = right_front.getCurrentPosition() + (int) (-rightInches *offsetLRPerc* COUNTS_PER_INCH);
            } else {
                newLeftTargetR = left_rear.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTargetR = right_rear.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
                newLeftTargetF = left_front.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
                newRightTargetF = right_front.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            }


            left_rear.setTargetPosition(newLeftTargetR);
            right_rear.setTargetPosition(newRightTargetR);
            left_front.setTargetPosition(newLeftTargetF);
            right_front.setTargetPosition(newRightTargetF);

            // Turn On RUN_TO_POSITION
            left_rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            left_rear.setPower(Math.abs(speed));
            right_rear.setPower(Math.abs(speed));
            left_front.setPower(Math.abs(speed));
            right_front.setPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            int redCutOff = 90;
            int blueCutOff = 120;
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (left_rear.isBusy() && right_rear.isBusy() && left_front.isBusy() && right_front.isBusy())) {
                Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                        (int) (sensorColor.green() * SCALE_FACTOR),
                        (int) (sensorColor.blue() * SCALE_FACTOR),
                        hsvValues);
                telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                telemetry.addData("Alpha", sensorColor.alpha());
                telemetry.addData("Red  ", sensorColor.red());
                telemetry.addData("Green", sensorColor.green());
                telemetry.addData("Blue ", sensorColor.blue());
                telemetry.addData("Hue", hsvValues[0]);

                if ((sensorColor.red() >= redCutOff) || (sensorColor.blue() >= blueCutOff)){
                    telemetry.addData("Color Found!", "STOP!");
                    left_rear.setPower(0);
                    right_rear.setPower(0);
                    left_front.setPower(0);
                    right_front.setPower(0);
                }

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d:", newLeftTargetR,  newRightTargetR, newLeftTargetF, newRightTargetF);
                telemetry.addData("Currently at",  " at  %7d :%7d :%7d :%7d:",
                        left_rear.getCurrentPosition(), right_rear.getCurrentPosition(), left_front.getCurrentPosition(), right_front.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            left_rear.setPower(0);
            right_rear.setPower(0);
            left_front.setPower(0);
            right_front.setPower(0);

            // Turn off RUN_TO_POSITION
            left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }


    }
}
