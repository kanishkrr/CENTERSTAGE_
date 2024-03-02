package org.firstinspires.ftc.teamcode.common.vision;

import android.graphics.Bitmap;
import android.graphics.Paint;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.concurrent.atomic.AtomicReference;

import org.openftc.easyopencv.OpenCvPipeline;

public class OpModeVisionRedClose extends OpenCvPipeline {

    public double leftValue;
    public double rightValue;
    public double middleValue;
    public static boolean DETECT_RED = true;
    public int locationFinal;
    private Paint linePaint;
    public static double MINIMUM_VALUES = 40;
    public static double MAXIMUM_VALUES = 255;
    public static double MINIMUM_BLUE_LOW_HUE = 50;
    public static double MAXIMUM_BLUE_LOW_HUE = 150;
    public static double MINIMUM_BLUE_HIGH_HUE = 100;
    public static double MAXIMUM_BLUE_HIGH_HUE = 160;
    public static double MINIMUM_RED_LOW_HUE = 0;
    public static double MAXIMUM_RED_LOW_HUE = 25;
    public static double MINIMUM_RED_HIGH_HUE = 160;
    public static double MAXIMUM_RED_HIGH_HUE = 255;

    Telemetry telemetry;
    Mat mat = new Mat();

    public enum TeamPropLocation {
        Left,
        Right,
        Middle,
        Unfound
    };
    public TeamPropLocation location;
    //public Location

    static final Rect Left_ROI = new Rect(new Point(0,60), new Point(70,160));
    static final Rect Middle_ROI = new Rect(new Point(100,60), new Point(190,160));
    static final Rect Right_ROI = new Rect(new Point(210,80), new Point(280,170));
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));

    public void CenterStageCVDetection(Telemetry t){
        telemetry = t;
    }


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar MINIMUM_BLUE = new Scalar(MINIMUM_BLUE_LOW_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_BLUE = new Scalar(MAXIMUM_BLUE_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_LOW = new Scalar(MINIMUM_RED_LOW_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_LOW = new Scalar(MAXIMUM_RED_LOW_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);
        Scalar MINIMUM_RED_HIGH = new Scalar(MINIMUM_RED_HIGH_HUE,MINIMUM_VALUES,MINIMUM_VALUES);
        Scalar MAXIMUM_RED_HIGH = new Scalar(MAXIMUM_RED_HIGH_HUE,MAXIMUM_VALUES,MAXIMUM_VALUES);

        if(!DETECT_RED) {
            Core.inRange(mat, MINIMUM_BLUE, MAXIMUM_BLUE, mat);
        }
        else {
            Mat mat1 = mat.clone();
            Mat mat2 = mat.clone();
            Core.inRange(mat1, MINIMUM_RED_LOW, MAXIMUM_RED_LOW, mat1);
            Core.inRange(mat2, MINIMUM_RED_HIGH, MAXIMUM_RED_HIGH, mat2);
            Core.bitwise_or(mat1,mat2,mat);
        }

        Mat left = mat.submat(Left_ROI);
        Mat right = mat.submat(Right_ROI);
        Mat middle = mat.submat(Middle_ROI);

        leftValue = Core.sumElems(left).val[0];
        rightValue = Core.sumElems(right).val[0];
        middleValue = Core.sumElems(middle).val[0];

        //telemetry.addData("left raw val:", leftValue);
        //telemetry.addData("right raw val:", rightValue);
        //telemetry.addData("middle raw val:", middleValue);

        left.release();
        right.release();
        middle.release();

        if (leftValue > rightValue && leftValue > middleValue) {
            location = TeamPropLocation.Left;
            //telemetry.addData("location:", "left");
        } else if (rightValue > middleValue && rightValue > leftValue) {
            location = TeamPropLocation.Right;
            //telemetry.addData("location:", "right");
        } else if(middleValue > rightValue && middleValue > leftValue){
            location = TeamPropLocation.Middle;
            //telemetry.addData("location:", "middle");
        }
        else {
            location = TeamPropLocation.Unfound;
        }

        //telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);
        Scalar pixelColor = new Scalar(255,255,255);
        Scalar propColor = new Scalar(0,0,255);

        Imgproc.rectangle(mat, Left_ROI, location == TeamPropLocation.Left? pixelColor:propColor);
        Imgproc.rectangle(mat, Right_ROI, location == TeamPropLocation.Right? pixelColor:propColor);
        Imgproc.rectangle(mat, Middle_ROI, location == TeamPropLocation.Middle? pixelColor:propColor);

        return mat;
    }

}

