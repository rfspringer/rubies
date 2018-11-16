package org.firstinspires.ftc.teamcode.Lib;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.HWMaps.Robot;

import java.util.List;

public class TensorFlow {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AT2E+kb/////AAAAGf8YTkfSRkKHrsNx2Sj+Qjc53Nu5YtFH4jlzliMbmb1MAPff8lD9LEAQWgI0eOHtfIYcMZANkhE2mrZBXtrvtfoEUXb0kzcjPmNhnVQHpqM3wlvZTxsNSCk5Y8cMPAbW/jMFilmWd+iP7YhFhajlZ1+FlYsedp1voNI0qBWhBaOyvlBs3zHNeKnA8xb03W7U8cNdq7hhKFfzt1zrlbhzrj4UIw5oFoA6kXlpAPTZE6e31356A6u5FyBiFwBfQ4CoNp/isAkNhCkKrmAolfuQjmHwnM7pg0ueMbvL12e7MmqwEVCWL4qKaqMxxNe/K++LgLFtmV/gMPBCUjNH53+8E/7qBceeNYBvcatdFgWapJv1\";\n";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private Robot robot = Robot.getInstance();

    public void init() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            throw new RuntimeException("This phone is not compatible with Tensorflow");
        }
    }

    private int getGoldMineralX() {
        /* Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        int goldMineralX = -1;

        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                        goldMineralX = (int) recognition.getLeft();
                    }
                }
            }
        }
        return goldMineralX;
    }

    public void shutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = robot.sensors.hwMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
