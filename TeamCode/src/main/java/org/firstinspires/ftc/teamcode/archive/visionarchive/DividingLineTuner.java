package org.firstinspires.ftc.teamcode.archive.visionarchive;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.text.TextPaint;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

//class to tune the left and right dividing lines for prop detection
public class DividingLineTuner implements VisionProcessor, CameraStreamSource {
    private final DoubleSupplier minArea, left, right;
    private final Scalar upper; // lower bounds for masking
    private final Scalar lower; // upper bounds for masking
    private final TextPaint textPaint;
    private final Paint linePaint, linePaintTwo;
    private final ArrayList<MatOfPoint> contours;
    private final Mat hierarchy = new Mat();
    private final Mat sel1 = new Mat(); // these facilitate capturing through 0
    private final Mat sel2 = new Mat();
    private final AtomicReference<Bitmap> lastFrame = new AtomicReference<>(Bitmap.createBitmap(1, 1, Bitmap.Config.RGB_565));
    private double largestContourX;
    private double largestContourY;
    private double largestContourArea;
    private MatOfPoint largestContour;
    private PropPositions previousPropPosition;
    private PropPositions recordedPropPosition = PropPositions.UNFOUND;


    public DividingLineTuner(@NonNull Scalar lower, @NonNull Scalar upper, DoubleSupplier minArea, DoubleSupplier left, DoubleSupplier right) {
        this.contours = new ArrayList<>();
        this.lower = lower;
        this.upper = upper;
        this.minArea = minArea;
        this.left = left;
        this.right = right;


        textPaint = new TextPaint();
        textPaint.setColor(Color.WHITE);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setAntiAlias(true);
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.DEFAULT_BOLD);

        linePaint = new Paint();
        linePaint.setColor(Color.WHITE);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(12);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);

        linePaintTwo = new Paint();
        linePaint.setColor(Color.RED);
        linePaint.setAntiAlias(true);
        linePaint.setStrokeWidth(12);
        linePaint.setStrokeCap(Paint.Cap.ROUND);
        linePaint.setStrokeJoin(Paint.Join.ROUND);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        lastFrame.set(Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565));
    }


    public double getLargestContourX() {
        return largestContourX;
    }

    public double getLargestContourY() {
        return largestContourY;
    }

    public double getLargestContourArea() {
        return largestContourArea;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);

        if (upper.val[0] < lower.val[0]) {
            Core.inRange(frame, new Scalar(upper.val[0], lower.val[1], lower.val[2]), new Scalar(0, upper.val[1], upper.val[2]), sel1);
            Core.inRange(frame, new Scalar(0, lower.val[1], lower.val[2]), new Scalar(lower.val[0], upper.val[1], upper.val[2]), sel2);
            Core.bitwise_or(sel1, sel2, frame);
        } else {
            Core.inRange(frame, lower, upper, frame);
        }

        contours.clear();

        Imgproc.findContours(frame, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

        largestContourArea = -1;
        largestContour = null;

        double minArea = this.minArea.getAsDouble();


        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area > largestContourArea && area > minArea) {
                largestContour = contour;
                largestContourArea = area;
            }
        }

        largestContourX = largestContourY = -1;

        if (largestContour != null) {
            Moments moment = Imgproc.moments(largestContour);
            largestContourX = (moment.m10 / moment.m00);
            largestContourY = (moment.m01 / moment.m00);
        }


        PropPositions propPosition;
        if (largestContour == null) {
            propPosition = PropPositions.UNFOUND;
        } else if (largestContourX < left.getAsDouble()) {
            propPosition = PropPositions.LEFT;
        } else if (largestContourX > left.getAsDouble()) {
            propPosition = PropPositions.MIDDLE;
        } else {
            propPosition = PropPositions.RIGHT;
        }


        if (propPosition != previousPropPosition && propPosition != PropPositions.UNFOUND) {
            recordedPropPosition = propPosition;
        }

        previousPropPosition = propPosition;

        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.RGB_565);
        Utils.matToBitmap(frame, b);
        lastFrame.set(b);
        return frame;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        canvas.drawLine((float) left.getAsDouble(), 0, (float) left.getAsDouble(), onscreenHeight, linePaintTwo);
        canvas.drawLine((float) right.getAsDouble(), 0, (float) right.getAsDouble(), onscreenHeight, linePaintTwo);

        String text = String.format(Locale.ENGLISH, "%s", recordedPropPosition.toString());

        canvas.drawText(text, (float) largestContourX * scaleBmpPxToCanvasPx, (float) largestContourY * scaleBmpPxToCanvasPx, textPaint);

    }

    public PropPositions getRecordedPropPosition() {
        return recordedPropPosition;
    }
    public MatOfPoint getLargestContour() {
        return largestContour;
    }

    @Override
    protected void finalize() throws Throwable {
        close();
        super.finalize();
    }

    public void close() {
        hierarchy.release();
        sel1.release();
        sel2.release();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        continuation.dispatch(bitmapConsumer -> bitmapConsumer.accept(lastFrame.get()));
    }

    public enum PropPositions {
        LEFT,
        MIDDLE,
        RIGHT,
        UNFOUND;
    }
}