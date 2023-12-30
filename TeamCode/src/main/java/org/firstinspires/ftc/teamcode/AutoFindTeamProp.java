//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.tfod.TfodProcessor;
//
//import java.util.List;
//
//
//@Autonomous(name="AutoFindTeamProp", group="FindProp")
//@Disabled
//
//public class AutoFindTeamProp extends LinearOpMode {
//
//    //map to config file
//    private ElapsedTime runtime = new ElapsedTime();
//    private DcMotor left_rear = null;
//    private DcMotor right_rear = null;
//    private DcMotor left_front = null;
//    private DcMotor right_front = null;
//    private DigitalChannel red1LED;
//    private DigitalChannel green1LED;
//    private DigitalChannel red2LED;
//    private DigitalChannel green2LED;
//    // TouchSensor touchLeft, touchRight;
//
//   // BNO055IMU imu; //gyroscope
//  //  Orientation lastAngles = new Orientation();
//    double globalAngle, correction;
//
//    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
//
//    /**
//     * The variable to store our instance of the TensorFlow Object Detection processor.
//     */
//    private TfodProcessor tfod;
//
//    /**
//     * The variable to store our instance of the vision portal.
//     */
//    private VisionPortal visionPortal;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        /** Wait for the game to begin */
//        telemetry.addData(">", "Press Play to start op mode");
//
//
//        // telemetry.addData("Status", "Initialized");
//        // telemetry.update();
//        // sleep(500);
//        left_rear = hardwareMap.get(DcMotor.class, "RearLeft");
//        right_rear = hardwareMap.get(DcMotor.class, "RearRight");
//        left_front = hardwareMap.get(DcMotor.class, "FrontLeft");
//        right_front = hardwareMap.get(DcMotor.class, "FrontRight");
//      // red1LED = hardwareMap.get(DigitalChannel.class, "Red1");
//       // green1LED = hardwareMap.get(DigitalChannel.class, "Green1");
//       // red2LED = hardwareMap.get(DigitalChannel.class, "Red2");
//       // green2LED = hardwareMap.get(DigitalChannel.class, "Green2");
//        //  touchLeft = hardwareMap.touchSensor.get("TouchLeft");
//        //  touchRight = hardwareMap.touchSensor.get("TouchRight");
//
//        //setup gyroscope
//        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//       // parameters.mode = BNO055IMU.SensorMode.IMU;
//       // parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//      //  parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//       // parameters.loggingEnabled = false;
//
//        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port on a
//        // Core Device Interface Module, configured to be a sensor of type "AREV Expansion Hub IMU",
//        // and named "imu".
//       // imu = hardwareMap.get(BNO055IMU.class, "imu");
//       // imu.initialize(parameters);
//        // make sure the imu gyro is calibrated before continuing.
//        //check out file SensorREVColorDistance for more info
//        //  colorSensor = hardwareMap.colorSensor.get("ColorSensor");
//        // bottomSensor = hardwareMap.colorSensor.get("BottomSensor");
//        // get a reference to the distance sensor that shares the same name.
//        // distanceSensor = hardwareMap.get(DistanceSensor.class, "ColorSensor");
//        // IMPORTANT-In robot config use odd numbered ports for each digital device
//
//      //  while (!isStopRequested() && !imu.isGyroCalibrated()) {
//      //     sleep(50);
//     //       idle();
//      //  }
//
//        //initialize motors
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        left_rear.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // 12/3/23 change LED mode from input to output
//      //  red1LED.setMode(DigitalChannel.Mode.OUTPUT);
//      //  green1LED.setMode(DigitalChannel.Mode.OUTPUT);
//      //  red2LED.setMode(DigitalChannel.Mode.OUTPUT);
//      //  green2LED.setMode(DigitalChannel.Mode.OUTPUT);
//
//        initTfod();
//
//        // Wait for the DS start button to be touched.
//        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
//        telemetry.addData(">", "Touch Play to start OpMode");
//        telemetry.update();
//        waitForStart();
//        runtime.reset();
//
//        if (opModeIsActive()) {
//
//           // green1LED.setState(true); //off
//          //  red1LED.setState(true); //off
//          //  green2LED.setState(true); //off
//          //  red2LED.setState(true); //off
//
//            while (opModeIsActive()) {
//
//                telemetry.update();
//
//
//
//                // Share the CPU.
//                sleep(20);
//            }
//        }
//
//
//
//
//
//
//
//    }
//
//    /**
//     * Initialize the TensorFlow Object Detection processor.
//     */
//    private void initTfod() {
//
//        // Create the TensorFlow processor by using a builder.
//        tfod = new TfodProcessor.Builder()
//
//                // Use setModelAssetName() if the TF Model is built in as an asset.
//                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
//                //.setModelAssetName(TFOD_MODEL_ASSET)
//                //.setModelFileName(TFOD_MODEL_FILE)
//
//                //.setModelLabels(LABELS)
//                //.setIsModelTensorFlow2(true)
//                //.setIsModelQuantized(true)
//                //.setModelInputSize(300)
//                //.setModelAspectRatio(16.0 / 9.0)
//
//                .build();
//
//        // Create the vision portal by using a builder.
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//
//        // Set the camera (webcam vs. built-in RC phone camera).
//        if (USE_WEBCAM) {
//            builder.setCamera(hardwareMap.get(WebcamName.class, "MyEye"));
//        } else {
//            builder.setCamera(BuiltinCameraDirection.BACK);
//        }
//
//        // Choose a camera resolution. Not all cameras support all resolutions.
//        //builder.setCameraResolution(new Size(640, 480));
//
//        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
//        //builder.enableCameraMonitoring(true);
//
//        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
//        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//
//        // Choose whether or not LiveView stops if no processors are enabled.
//        // If set "true", monitor shows solid orange screen if no processors enabled.
//        // If set "false", monitor shows camera view without annotations.
//        //builder.setAutoStopLiveView(false);
//
//        // Set and enable the processor.
//        builder.addProcessor(tfod);
//
//        // Build the Vision Portal, using the above settings.
//        visionPortal = builder.build();
//
//        // Set confidence threshold for TFOD recognitions, at any time.
//        //tfod.setMinResultConfidence(0.75f);
//
//        // Disable or re-enable the TFOD processor at any time.
//        //visionPortal.setProcessorEnabled(tfod, true);
//
//    }   // end method initTfod()
//
//    /**
//     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
//     */
//    private void telemetryTfod() {
//
//        List<Recognition> currentRecognitions = tfod.getRecognitions();
//        telemetry.addData("# Objects Detected", currentRecognitions.size());
//
//        // Step through the list of recognitions and display info for each one.
//        for (Recognition recognition : currentRecognitions) {
//            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
//            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
//
//            telemetry.addData(""," ");
//            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//            telemetry.addData("- Position", "%.0f / %.0f", x, y);
//            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
//        }   // end for() loop
//
//    }   // end method telemetryTfod()
//
//
//
//
//}
