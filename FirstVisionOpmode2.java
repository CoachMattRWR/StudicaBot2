package org.firstinspires.ftc.teamcode;

// code copied from the LearnJavaForFTC PDF
// contains extra code to handle the Pushbot pan-tilt webcam, it also reports the average saturation values
// Notes: the reported saturation of a team prop is > 200, but is about 20 for a grey tile so there is clear distinction.
//   Don't have a measurement yet for a spike mark on tile.

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous()
public class FirstVisionOpmode2 extends OpMode {
    private FirstVisionProcessor2 visionProcessor;
    private VisionPortal visionPortal;

    @Override
    public void init() {
        visionProcessor = new FirstVisionProcessor2();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 2"), visionProcessor);

    }

    @Override
    public void init_loop() {
        telemetry.addData("Init loop", visionProcessor.getSelection());
        telemetry.addData("satRectLeft", visionProcessor.satRectLeft);
        telemetry.addData("satRectMiddle", visionProcessor.satRectMiddle);
        telemetry.addData("satRectRight", visionProcessor.satRectRight);
    }

    @Override
    public void start() {
        //visionPortal.stopStreaming();
    }

    @Override
    public void loop() {
        telemetry.addData("Selection:", visionProcessor.getSelection());
        telemetry.addData("satRectLeft", visionProcessor.satRectLeft);
        telemetry.addData("satRectMiddle", visionProcessor.satRectMiddle);
        telemetry.addData("satRectRight", visionProcessor.satRectRight);
    }
}
