package org.firstinspires.ftc.teamcode.util.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class ConeAutomation {
    OpenCvCamera camera;
    ConeAutomationPipeline ConeAutomationPipeline;

    public ConeAutomationPipeline cameraSetup(LinearOpMode opMode) {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ConeAutomationPipeline = new ConeAutomationPipeline(true);

        camera.setPipeline(ConeAutomationPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                opMode.telemetry.addData("Camera Error:", errorCode);
                opMode.telemetry.update();
            }
        });
        return ConeAutomationPipeline;
    }
    public Point getConeCenter(ConeAutomationPipeline pipeline) {
        return pipeline.center;
    }
}