package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;


@Autonomous()
public class CenterStage_Auto_goBuilda_Gyroscope extends LinearOpMode{

    /* Declare OpMode members. */
    BNO055IMU imu; //gyroscope
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    private DcMotor left_front   = null;
    private DcMotor left_rear = null;
    private DcMotor right_front  = null;
    private DcMotor right_rear  = null;
    private DcMotor swing_motor = null;
    private DcMotor left_lift  = null;
    private DcMotor right_lift  = null;
    public DcMotor extend= null;
    public Servo pixel_claw = null;
    public Servo pixel_sleeve = null;
    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // 1440 = tetrix motor, 537.7 = goBuilda
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.9 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
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
    private boolean isBlue = true;


    public void initVision() {
        drawRectangleProcessor = new StagePropVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "MyEye"), drawRectangleProcessor);

    }
    StagePropVisionProcessor obj = new StagePropVisionProcessor();

    public StagePropVisionProcessor getDrawRectangleProcessor() {
        return drawRectangleProcessor;
    }

    public void runOpMode() {

        // Initialize the drive system variables.
        left_rear  = hardwareMap.get(DcMotor.class, "RearLeft");
        right_rear = hardwareMap.get(DcMotor.class, "RearRight");
        left_front  = hardwareMap.get(DcMotor.class, "FrontLeft");
        right_front = hardwareMap.get(DcMotor.class, "FrontRight");
        sensorColor = hardwareMap.get(ColorSensor.class, "Sensor Color");
        swing_motor  = hardwareMap.get(DcMotor.class, "SwingMotor");
        left_lift  = hardwareMap.get(DcMotor.class, "LeftLift");
        right_lift  = hardwareMap.get(DcMotor.class, "RightLift");
        pixel_claw= hardwareMap.get(Servo.class, "PixelClaw");
        pixel_sleeve= hardwareMap.get(Servo.class, "PixelSleeve");

        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "Sensor Color");



        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_rear.setDirection(DcMotorSimple.Direction.FORWARD);
        right_rear.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);

        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_lift.setDirection(DcMotorSimple.Direction.REVERSE);

        initVision();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d",
                left_rear.getCurrentPosition(),
                right_rear.getCurrentPosition(),
                left_front.getCurrentPosition(),
                right_front.getCurrentPosition());
        sleep(3000);
        telemetry.addData("Identified", drawRectangleProcessor.getSelection());
        telemetry.addData( "Left Avg: ",drawRectangleProcessor.getLeftAvg());
        telemetry.addData( "Middle Avg: ",drawRectangleProcessor.getMiddleAvg());
        telemetry.addData( "Right Avg: ",drawRectangleProcessor.getRightAvg());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if(opModeIsActive()) {
            // Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            double leftAvg = drawRectangleProcessor.getLeftAvg();
            double middleAvg = drawRectangleProcessor.getMiddleAvg();
            double rightAvg = drawRectangleProcessor.getRightAvg();
            int moveTo; // right = 1, middle = 2, left = 3
            if(Math.max(rightAvg, middleAvg) == rightAvg) {
                if(Math.max(rightAvg, leftAvg) == rightAvg) {
                    moveTo = 1;
                } else {
                    moveTo = 3;
                }
            } else {
                if(Math.max(middleAvg, leftAvg) == middleAvg) {
                    moveTo = 2;
                } else {
                    moveTo = 3;
                }
            }
            encoderDrive(DRIVE_SPEED,  32,  32, 5,0, false); // move forward
            if(moveTo == 1) {
                encoderDrive(DRIVE_SPEED -.2, 18, 18, 5, 3, true); // move right
            } else if(moveTo == 2) {
                encoderDrive(DRIVE_SPEED-.2,  36,  36, 5, 0, true); // move forward
            } else if (moveTo == 3) {
                encoderDrive(DRIVE_SPEED -.2,  36,  36, 5, 2, true); // move left
            } else {
                encoderDrive(DRIVE_SPEED-.2,  36,  36, 5, 0, true); // move forward
            }
            encoderDrive(DRIVE_SPEED, 4, 4, 5, 1, false);
            telemetry.addData("Success?:", moveTo);
            pixel_claw.setPosition(0.8);
            pixel_sleeve.setPosition(0.0);
            encoderDrive(DRIVE_SPEED, 4, 4, 5, 1, false);
            sleep(1000);

            if(isBlue) {
                encoderDrive(DRIVE_SPEED, 12, 12, 5, 2, false);
                encoderDrive(DRIVE_SPEED, 8, 8, 5, 1, false);
                if(moveTo == 3) {
                    encoderDrive(DRIVE_SPEED * 2, 32, 32, 5, 2, false);
//                    encoderDrive(DRIVE_SPEED * 2, 18, 18, 5, 2, false);
                } else {
                    encoderDrive(DRIVE_SPEED, 36, 36, 5, 2, false);
                }
            } else {
                encoderDrive(DRIVE_SPEED, 12, 12, 5, 3, false);
                encoderDrive(DRIVE_SPEED, 8, 8, 5, 1, false);
                if(moveTo == 1) {
                    encoderDrive(DRIVE_SPEED * 2, 32, 32, 5, 3, false);//was 36 before
//                    encoderDrive(DRIVE_SPEED * 2, 18, 18, 5, 3, false);
                } else {
                    encoderDrive(DRIVE_SPEED, 36, 36, 5, 3, false);
                }

            }
            encoderDrive(DRIVE_SPEED, 18, -18, 5, 0, true);
            encoderDrive(DRIVE_SPEED - .2,-24,-24,5,3,true);
            encoderDrive(DRIVE_SPEED - .2,-3,-3,5,1,true);
//            encoderDrive(DRIVE_SPEED,  36,  36, 5.0, 0);  // S1: Forward 47 Inches with 5 Sec timeout
//            encoderDrive(DRIVE_SPEED, 18, 18, 5.0, 2);
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
                             double timeoutS, int FBLR, boolean color) {
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
                    (wheelsInMotion())) {
                if(color) {
                    Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                            (int) (sensorColor.green() * SCALE_FACTOR),
                            (int) (sensorColor.blue() * SCALE_FACTOR),
                            hsvValues);
                    // Displays Data
                    telemetry.addData("Distance (cm)",
                            String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));
                    telemetry.addData("Red  ", sensorColor.red());
                    telemetry.addData("Blue ", sensorColor.blue());
                    telemetry.addData("Hue", hsvValues[0]);

                    if (sensorColor.red() >= redCutOff){
                        telemetry.addData("Red Found!", "STOP!");
                        stopAllMotion();
                        isBlue = false;
                        sleep(500);


                    } else if (sensorColor.blue() >= blueCutOff) {
                        telemetry.addData("Blue Found!", "STOP!");
                        stopAllMotion();
                        isBlue = true;
                        sleep(500);
                    }
                }


                // Display encoder information for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d:", newLeftTargetR,  newRightTargetR, newLeftTargetF, newRightTargetF);
                telemetry.addData("Currently at",  " at  %7d :%7d :%7d :%7d:",
                        left_rear.getCurrentPosition(), right_rear.getCurrentPosition(), left_front.getCurrentPosition(), right_front.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            stopAllMotion();

            // Turn off RUN_TO_POSITION
            left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }


    }

    public void stopAllMotion() {
        left_rear.setPower(0);
        right_rear.setPower(0);
        left_front.setPower(0);
        right_front.setPower(0);
    }
    public boolean wheelsInMotion() {
        return left_rear.isBusy() && right_rear.isBusy() && left_front.isBusy() && right_front.isBusy();
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    // Get current cumulative angle rotation from last reset.
    // @return Angle in degrees. + = left, - = right.
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;
        lastAngles = angles;

        return globalAngle;
    }

    // See if we are moving in a straight line and if not return a power correction value.
    // @return Power adjustment, + is adjust left - is adjust right.
    private double checkDirection() {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    public void turnAngle(double angle, int tolerance, double motorPower) {
        //tolerance: how much angle can be off by
        //motorPower: how fast to turn
        int direction = 1; //+ is clockwise turning

        resetAngle();
        if (angle > 0) {
            direction = -1;
        }
        while ((Math.abs(angle) - Math.abs(getAngle()) > tolerance)) {
            left_front.setPower(motorPower * direction);
            right_front.setPower(-1 * motorPower * direction);
            left_rear.setPower(motorPower * direction);
            right_rear.setPower(-1 * motorPower * direction);

            telemetry.addData("Wanted Angle", angle);
            telemetry.addData("Curr Angle", getAngle());
            telemetry.update();
        }
    }
}
