package org.firstinspires.ftc.teamcode.util.robot;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeAutomation extends OpenCvPipeline {
    private boolean isRed;
    private double area;
    private Scalar lowHSV;
    private Scalar highHSV;
    private final int threshold = 100;
    private Rect[] boundRect;
    private int highPos = 0;
    private int cx = 0;
    private int cy = 0;

    public ConeAutomation(boolean isRed) {
        this.isRed = isRed;
    }

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Set Red / Blue Threshold
        if (isRed){
            lowHSV = new Scalar(0, 155,120);
            highHSV = new Scalar(90, 255, 255);
        } else {
            lowHSV = new Scalar(31, 51,121);
            highHSV = new Scalar(128, 255, 255);
        }
        Mat thresh = new Mat();

        //b&w image where white represents cone.
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // findContours connects the edges of the thresh.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        //Get Boundary Boxes
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        //Iterate through the bounding boxes and draw the box with the most area (theoretically the cone)
        //Catch the possibility of there being no bounding boxes
        try {
            double highArea = boundRect[0].area();
            int highPos = 0;
            for (int i = 0; i != boundRect.length; i++) {
                if (highArea < boundRect[i].area()) {
                    highArea = boundRect[i].area();
                    highPos = i;
                }
            }
            Imgproc.rectangle(mat, boundRect[highPos], new Scalar(250, 250, 250), 3);
            area = boundRect[highPos].area();
            cx = boundRect[highPos].x + boundRect[highPos].width / 2;
            cy = boundRect[highPos].y + boundRect[highPos].height / 2;
        }
        catch(ArrayIndexOutOfBoundsException exception) {
            //No Cone Found
        }
        return mat;
    }
    public double getArea() {
        return area;
    }

    public double getCenterX() {
        return cx;
    }

    public double getCenterY() {
        return cy;
    }

}