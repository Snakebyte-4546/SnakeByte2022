package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.robot.ConeAutomation;
import org.firstinspires.ftc.teamcode.util.robot.ConeAutomationPipeline;

@Autonomous(name="Vision Test")
public class VisionTest extends LinearOpMode {
    ConeAutomation ConeAutomation = new ConeAutomation();
    ConeAutomationPipeline pipeline = new ConeAutomationPipeline(true);

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive() & !isStopRequested()) {
            telemetry.addData("Center of Cone: ", ConeAutomation.getConeCenter(pipeline));
            telemetry.update();
        }
    }
}