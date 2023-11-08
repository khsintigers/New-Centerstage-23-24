/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the basics of AprilTag recognition and pose estimation,
 * including Java Builder structures for specifying Vision parameters.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 */
@TeleOp(name = "Concept: AprilTagTest", group = "Concept")
//@Disabled
//Motors :D vvvvvvvv
public class AprilTagTest extends LinearOpMode {

    private DigitalChannel red1LED;
    private DigitalChannel green1LED;
    private DigitalChannel red2LED;
    private DigitalChannel green2LED;
    private DigitalChannel red3LED;
    private DigitalChannel green3LED;
    private DigitalChannel red4LED;
    private DigitalChannel green4LED;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_rear = null;
    private DcMotor right_rear = null;
    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor lifter_motor = null;
//    private DcMotor lifter_motor2 = null;
    private DcMotor sixth_motor = null;
    public DcMotor extend = null;
    public Servo claw_right = null;
    public Servo claw_left = null;
    public Servo claw_height = null;
    public double servoPos = 0.3;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the vision portal.
     */
    private VisionPortal visionPortal;
    private int id_found;
    private double aprilx;
    private double aprily;
    private double aprilz;
    private int numberWebcamRan = 0;

    @Override
    public void runOpMode() {
/*
        left_rear = hardwareMap.get(DcMotor.class, "RearLeft");
        right_rear = hardwareMap.get(DcMotor.class, "RearRight");
        left_front = hardwareMap.get(DcMotor.class, "FrontLeft");
        right_front = hardwareMap.get(DcMotor.class, "FrontRight");
        lifter_motor = hardwareMap.get(DcMotor.class, "LifterMotor");
        claw_right = hardwareMap.get(Servo.class, "RightServo");
        claw_left = hardwareMap.get(Servo.class, "LeftServo");
        claw_height = hardwareMap.get(Servo.class, "ClawHeight");

        red1LED = hardwareMap.get(DigitalChannel.class, "Red1");
        green1LED = hardwareMap.get(DigitalChannel.class, "Green1");
        red2LED = hardwareMap.get(DigitalChannel.class, "Red2");
        green2LED = hardwareMap.get(DigitalChannel.class, "Green2");
        //  touchLeft = hardwareMap.touchSensor.get("TouchLeft");
        //  touchRight = hardwareMap.touchSensor.get("TouchRight");
*/
        //setup gyroscope
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;



/*

        //initialize motors
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setDirection(DcMotorSimple.Direction.REVERSE);
//        lifter_motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lifter_motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_rear.setDirection(DcMotorSimple.Direction.REVERSE);

        // 12/3/23 change LED mode from input to output
        red1LED.setMode(DigitalChannel.Mode.OUTPUT);
        green1LED.setMode(DigitalChannel.Mode.OUTPUT);
        red2LED.setMode(DigitalChannel.Mode.OUTPUT);
        green2LED.setMode(DigitalChannel.Mode.OUTPUT);
*/

        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        waitForStart();

        int current_id_sought = 0;
        if (opModeIsActive()) {
            while (opModeIsActive()) {
            //    mechanum();
                telemetryAprilTag();
//                if(gamepad1.a) {
//                    moveFoward(1100, 0.5);
//                }
                if (gamepad1.x) {
                    move_correct_tag(current_id_sought);
                }
                if (gamepad1.a) {
                    current_id_sought = 1;
                } else if (gamepad1.b) {
                    current_id_sought = 2;
                } else if (gamepad1.y) {
                    current_id_sought = 3;
                }
                // Push telemetry to the Driver Station.
                telemetry.addLine(String.format("Sought tag is %s", current_id_sought));
                telemetry.update();

                // Save CPU resources; can resume streaming when needed.
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming();
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming();
                }

                // Share the CPU.
                sleep(20);
            }
        }

        // Save more CPU resources when camera is no longer needed.
        visionPortal.close();

    }   // end method runOpMode()

    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "MyEye"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()


    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {
        numberWebcamRan++;
        telemetry.addData("Webcam #", numberWebcamRan);
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                id_found = detection.id;
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
                aprilx = detection.ftcPose.x;
                aprily = detection.ftcPose.y;
                aprilz = detection.ftcPose.z;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
        if (Math.abs(aprilx) > .3) {
            if (aprilx > 0) {
                telemetry.addData("Move Right", "");
                sleep(1300);
                telemetry.addData("Move Foward", "");
                sleep(2200);
            } else {
                telemetry.addData("Move Left", "");
                sleep(1300);
                telemetry.addData("Move Foward", "");
                sleep(2200);
            }

        }   // end method telemetryAprilTag()
        //private void moveForward(int time, double speed) {
        //        left_front.setPower(speed);
        //        right_front.setPower(speed);
        //        left_rear.setPower(speed);
        //        right_rear.setPower(speed);
        //        sleep(time);
        //        left_front.setPower(0);
        //        right_front.setPower(0);
        //        left_rear.setPower(0);
        //        right_rear.setPower(0);
        //    }


    }

    private void move_correct_tag(int correctTag) {
        double speed = 0.05;
        if (id_found < correctTag) {
            telemetry.addData("Move Right", "");
        //    startRight(speed);

            while(id_found < correctTag) {
                telemetryAprilTag();
                telemetryAprilTag();
                telemetryAprilTag();
                telemetryAprilTag();
             //   moveLeftTime(1300,speed);
                sleep(10);
            }
            end();

        } else if (id_found > correctTag) {
            telemetry.addData("Move Left", "");
        //    startLeft(speed);
            telemetryAprilTag();
            telemetryAprilTag();
            telemetryAprilTag();

            while(id_found < correctTag) {
                telemetryAprilTag();
                sleep(10);
            }
            end();
        } else {
            telemetry.addData("Correct Tag Found!", "");

        }

    }

    public void mechanum(){
        final double maxPower = 1.0;   //0.5

        double joy1Y = gamepad1.left_stick_x;
        joy1Y = Math.abs(joy1Y) > 0.15 ? joy1Y*3/4: 0;
        double joy1X = gamepad1.left_stick_y;
        joy1X = Math.abs(joy1X) > 0.15 ? joy1X*3/4: 0;
        double joy2X = gamepad1.right_stick_x;
        joy2X = Math.abs(joy2X) > 0.15 ? joy2X*3/4: 0;

        left_front.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        right_front.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        left_rear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        right_rear.setPower(Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));


        //telemetry.addData("LF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X - joy1X)));
        //telemetry.addData("RF: ",Math.max(-maxPower, Math.min(maxPower, joy1Y + joy2X + joy1X)));
        //telemetry.addData("LR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X + joy1X)));
        //telemetry.addData("RR: ",Math.max(-maxPower, Math.min(maxPower, joy1Y - joy2X - joy1X)));
        //telemetry.update();
    }
    private void startRight(double speed) {
        left_front.setPower(speed);
        left_rear.setPower(-speed);
        right_front.setPower(-speed);
        right_rear.setPower(speed);
    }
    private void startLeft(double speed) {
        left_front.setPower(-speed);
        left_rear.setPower(speed);
        right_front.setPower(speed);
        right_rear.setPower(-speed);
    }
    private void end(){
        left_front.setPower(0);
        left_rear.setPower(0);
        right_front.setPower(0);
        right_rear.setPower(0);

        }
    private void moveLeftTime(int time, double speed) {
        left_front.setPower(-speed);
        right_front.setPower(speed);
        left_rear.setPower(speed);
        right_rear.setPower(-speed);
        sleep(time);
    }
    }





    //end class

