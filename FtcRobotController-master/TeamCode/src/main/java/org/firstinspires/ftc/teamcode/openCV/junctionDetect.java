package org.firstinspires.ftc.teamcode.openCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Rect;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Junction Detection")
public class junctionDetect extends DBTRobot {

    OpenCvCamera camera;
    junctionDetectionPipeline JunctionDetectionPipeline;
    Rect rect = new Rect();
    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    // Sleeve TAG IDs
    int LEFT = 3;
    int MIDDLE = 4;
    int RIGHT = 5;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        JunctionDetectionPipeline = new junctionDetectionPipeline();

        set_config();



        camera.setPipeline(JunctionDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            rect = JunctionDetectionPipeline.getRect();

            telemetry.addLine(String.format("Translation X: %d feet", rect.x));
            telemetry.addLine(String.format("Translation y: %d feet", rect.y));
            telemetry.addLine(String.format("Translation heigth: %d feet", rect.height));
            telemetry.addLine(String.format("Translation width: %d feet", rect.width));
            telemetry.update();
        }


         while (opModeIsActive()) {sleep(20);}
    }

}
