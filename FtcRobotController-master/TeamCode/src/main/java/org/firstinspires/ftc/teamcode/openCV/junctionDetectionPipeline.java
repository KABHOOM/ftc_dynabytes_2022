package org.firstinspires.ftc.teamcode.openCV;

import android.provider.ContactsContract;
import org.opencv.core.MatOfPoint;
import java.util.ArrayList;
import java.util.List;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class junctionDetectionPipeline extends OpenCvPipeline {
    Mat _region = new Mat();
    int CAMERA_WIDTH = 800;
    float h_position;
    List<MatOfPoint> contoursList = new ArrayList<>();
    int numContoursFound;
    int HORIZON = (int)((100.0/320.0) * CAMERA_WIDTH);
    Rect maxRect = new Rect();
    @Override
    public Mat processFrame(Mat input) {

        //Step 1: converting image to HSV image
        Imgproc.cvtColor(input,_region, Imgproc.COLOR_RGB2HSV);
        if (_region.empty()){
            return input;
        }


        // Step 2: apply lenient filter to get black and white image of yellow objects
        Scalar lowHSV = new Scalar(20,70,80);
        Scalar highHSV = new Scalar(32,255,255);
        Mat _thresh = new Mat();
        Core.inRange(_region,lowHSV,highHSV,_thresh);

        // Step3: Apply mask
        Mat _masked = new Mat();
        Core.bitwise_and(_region,_region,_masked,_thresh);

        // Step 4: Adjusting saturation by scaling it
        Scalar average = Core.mean(_masked,_thresh);
        Mat _scaledMask = new Mat();
        _masked.convertTo(_scaledMask,-1,150/average.val[0],0);

        // Step5: Getting thresh scaled image
        Mat _scaledThresh = new Mat();
        Scalar strict_lowHSV = new Scalar(0,150,100);
        Scalar strict_highHSV = new Scalar(255,255,255);
        Core.inRange(_scaledMask,strict_lowHSV,strict_highHSV,_scaledThresh);

        // Step 6: apply final mask to color the image
        Mat _finalMasked = new Mat();
        Core.bitwise_and(_region,_region,_finalMasked,_scaledThresh);

        //Step 7: Find edges
        Mat _edges = new Mat();
        Imgproc.Canny(_finalMasked,_edges,100,200);

        //Step 8: Finding contours
        Imgproc.findContours(_scaledThresh, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        numContoursFound = contoursList.size();
        int maxWidth = 0;

        for (MatOfPoint c: contoursList){
            MatOfPoint2f copy = new MatOfPoint2f(c.toArray());
            Rect rect = Imgproc.boundingRect(copy);

            int w = rect.width;
            if (w>maxWidth){ //&& rect.y+rect.height > HORIZON){
                maxWidth = w;
                maxRect = rect;
            }
            c.release();
            copy.release();

        }
        Scalar rcolor = new Scalar(255,0,0);
        Point  p1 = new Point(maxRect.x,maxRect.y);
        Point  p2 = new Point(maxRect.x+maxRect.width,maxRect.y+maxRect.height);
        Imgproc.rectangle(_finalMasked,p1,p2,rcolor,3);

        return _finalMasked;
    }
    public Rect getRect(){

        return this.maxRect;
    }

}
