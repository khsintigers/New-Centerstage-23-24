package org.firstinspires.ftc.teamcode;


import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

/*
Hue: 0-180 Kind of color
Saturation: 0-255 Intensity of Color
Value: 0-255 Brightness

 */
public class StagePropVisionProcessor implements VisionProcessor {
    //x,y upper left hand corner of rectangle
    public Rect rectLeft = new Rect(10,90,100,120);
    public Rect rectMiddle = new Rect(210,30,200,100);
    public Rect rectRight = new Rect(520,75,100,150);
    Selected selection = Selected.NONE;
//left was 100 and width was 100, height was 100 y 140
    //mid x 300 y 20 width 200 height 100
    //right 500 y 140 wid 100 hei 150
    double leftValue;
    double middleValue;
    double rightValue;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame,hsvMat,Imgproc.COLOR_RGB2HSV);

        double satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        double satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        double satRectRight = getAvgSaturation(hsvMat, rectRight);

        leftValue = satRectLeft;
        middleValue = satRectMiddle;
        rightValue = satRectRight;

        if ((satRectLeft > satRectMiddle) && (satRectLeft > satRectRight)) {
            return Selected.LEFT;
        } else if ((satRectMiddle > satRectLeft) && (satRectMiddle > satRectRight)) {
            return Selected.MIDDLE;
        }
        return Selected.RIGHT;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity,
                            Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.RED);

        android.graphics.Rect drawRectangleLeft   = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight  = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        selection = (Selected) userContext; //returned from processFrame
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
        }

    }

    public Selected getSelection() {
        return selection;
    }

    public enum Selected {
        NONE,
        LEFT,
        MIDDLE,
        RIGHT
    }

    public double getLeftAvg() {
        return leftValue; //getAvgSaturation(hsvMat, rectLeft);
    }

    public double getMiddleAvg() {
        return middleValue; //getAvgSaturation(hsvMat, rectMiddle);
    }

    public double getRightAvg() {
        return rightValue; //getAvgSaturation(hsvMat, rectRight);
    }




}
