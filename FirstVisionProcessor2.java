package org.firstinspires.ftc.teamcode;

// code copied from the LearnJavaForFTC PDF
// modified to allow the average saturation values to be displayed via telemetry by the opMode
// and modified to place the rectangles on the spike marks based on the robot setup.
// Also highlights the chosen mark in red when viewed via Camera Stream. 

// Note: due to webcam on left side of robot we only check the left and centre spike marks.
// We assume it's on the right side if not on the two we check.
// This also assumes the robot is set up centered on the starting tile.

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class FirstVisionProcessor2 implements VisionProcessor {
    public Rect rectLeft = new Rect(105, 270, 50, 50);
    public Rect rectMiddle = new Rect(380, 260, 50, 50);
    public Rect rectRight = new Rect(590, 270, 50, 50);
    Selected selection = Selected.NONE;

    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public double satRectLeft = 0;
    public double satRectMiddle = 0;
    public double satRectRight = 0;

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        // process for team prop location
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        satRectLeft = getAvgSaturation(hsvMat, rectLeft);
        satRectMiddle = getAvgSaturation(hsvMat, rectMiddle);
        //satRectRight = getAvgSaturation(hsvMat, rectRight);

        if ((satRectLeft < 50) && (satRectMiddle < 50)) { 
            return Selected.RIGHT;   
            // grey tiles will have values around 20, if neither region has prop, return RIGHT.
        }
        
        // The team prop has values near 200
        if ((satRectLeft > satRectMiddle)) {
            return Selected.LEFT;
        } else {
            return Selected.MIDDLE;
        }
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
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);
        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
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
}
