package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.StagePropVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;


@Autonomous()
public class UseVisionProcessor extends OpMode{

    private StagePropVisionProcessor drawRectangleProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        drawRectangleProcessor = new StagePropVisionProcessor();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "MyEye"), drawRectangleProcessor);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        //visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Identified", drawRectangleProcessor.getSelection());
        telemetry.addData( "Left Avg: ",drawRectangleProcessor.getLeftAvg());
        telemetry.addData( "Middle Avg: ",drawRectangleProcessor.getMiddleAvg());
        telemetry.addData( "Right Avg: ",drawRectangleProcessor.getRightAvg());
        telemetry.update();

    }

}
