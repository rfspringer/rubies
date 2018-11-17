/* Copyright (c) 2018 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.Lib.TensorFlow;

import java.util.List;

import static org.firstinspires.ftc.teamcode.Lib.TensorFlow.LABEL_GOLD_MINERAL;

/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "2 Concept: TensorFlow Object Detection Webcam", group = "Concept")
//@Disabled
public class WebcamTest2 extends LinearOpMode {
    private TensorFlow tensorFlow = new TensorFlow();
//    private static final String tensorFlow.tfod_MODEL_ASSET = "RoverRuckus.tflite";
//    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
//    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
//
//    /*
//     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
//     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
//     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
//     * web site at https://developer.vuforia.com/license-manager.
//     *
//     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
//     * random data. As an example, here is a example of a fragment of a valid key:
//     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
//     * Once you've obtained a license key, copy the string from the Vuforia web site
//     * and paste it in to your code on the next line, between the double quotes.
//     */
//    private static final String VUFORIA_KEY = "AVRfS7L/////AAABmaGT8EE2D0ognaN6WhR7wTsd4Zu3Bn3gJjl8WAi95O+bXTS8qogcR58wbZP+UoYB99sjHS22e4oF03SQ5f3y0j9oUyDrOw6vbqPCmductE5WDpTqj+RQIbkUX/0zAmOIsLdq0a7jWPEPAGtI5RRVD3+pFqwU8jvy16q0zvTa+zpvcQU4uYDTOtLEwhGUnStDbK8sgrNzjehUojKnMezx5ypO0C69YT+N8nChher2V+kghuea9ysf4auTD2vIhL7mw8oEZKDcJd3kf9hLX8dlukarDrVcyT+pDC92zARDSWybAU7IxHvol627lXekBv+lo+Jv9UNUvma6tSB4AR7zeBmnhMEMmXoOau7JABDkzT9m";
//    /**
//     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
//     * localization engine.
//     */
//    private VuforiaLocalizer vuforia;
//
//    /**
//     * {@link #tensorFlow.tfod} is the variable we will use to store our instance of the Tensor Flow Object
//     * Detection engine.
//     */
//    private TFObjectDetector tensorFlow.tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
//        initVuforia();
//
//        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
//            inittensorFlow.tfod();
//        } else {
//            telemetry.addData("Sorry!", "This device is not compatible with tensorFlow.tfod");
//        }
        
        tensorFlow.init(hardwareMap);

        /* Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
//            /* Activate Tensor Flow Object Detection. */
//            if (tensorFlow.tfod != null) {
//                tensorFlow.tfod.activate();
//            }
            tensorFlow.activate();

            while (opModeIsActive()) {
                if (tensorFlow.tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
//                    List<Recognition> updatedRecognitions = tensorFlow.tfod.getUpdatedRecognitions();
                    List<Recognition> updatedRecognitions = tensorFlow.tfod.getUpdatedRecognitions();
                    seeGold(updatedRecognitions);
//                    telemetry.addData("see gold", seeGold(updatedRecognitions));
//                    if (updatedRecognitions != null) {
//                      telemetry.addData("# Object Detected", updatedRecognitions.size());
////                      if (updatedRecognitions.size() == 3) {
//                        int goldMineralX = -1;
//                        for (Recognition recognition : updatedRecognitions) {
//                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
//                            goldMineralX = (int) recognition.getLeft();
//                          }
//                        }
//
//                        telemetry.addData("Gold", goldMineralX);
//
////                        if (tensorFlow.identifyGoldMineral() != null) {
//                            telemetry.addData("test", "1");
//                            telemetry.addData("test gold x out of function", identifyGoldMineralX());
////
////
////                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
////                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
////                            telemetry.addData("Gold Mineral Position", "Left");
////                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
////                            telemetry.addData("Gold Mineral Position", "Right");
////                          } else {
////                            telemetry.addData("Gold Mineral Position", "Center");
////                          }
////                        }
////                      }
//                      telemetry.update();
//                    }
                }
            }
        }
        tensorFlow.shutdown();
//        if (tensorFlow.tfod != null) {
//            tensorFlow.tfod.shutdown();
//        }
    }

    public int identifyGoldMineralX() {
        int goldMineral = 0;
        List<Recognition> updatedRecognitions = tensorFlow.tfod.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineral =(int) recognition.getLeft();
                    return goldMineral;
                }
            }
        }
        return goldMineral;
    }

    public void seeGold(List<Recognition> updatedRecognitions) {
        if (updatedRecognitions != null) {
            telemetry.addData("# Object Detected", updatedRecognitions.size());
//                      if (updatedRecognitions.size() == 3) {
            int goldMineralX = -1;
            for (Recognition recognition : updatedRecognitions) {
                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                    goldMineralX = (int) recognition.getLeft();
                }
            }

            telemetry.addData("Gold", goldMineralX);

//                        if (tensorFlow.identifyGoldMineral() != null) {
            telemetry.addData("test", "1");
            telemetry.addData("test gold x", identifyGoldMineralX());
//
//
//                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
//                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Left");
//                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
//                            telemetry.addData("Gold Mineral Position", "Right");
//                          } else {
//                            telemetry.addData("Gold Mineral Position", "Center");
//                          }
//                        }
//                      }
            telemetry.update();
//            return goldMineralX;
        }
//        return -20000000;
    }
//
//    /**
//     * Initialize the Vuforia localization engine.
//     */
//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = tensorFlow.VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        tensorFlow.vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
//    }
//
//    /**
//     * Initialize the Tensor Flow Object Detection engine.
//     */
//    private void inittensorFlow.tfod() {
//        int tensorFlow.tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//            "tensorFlow.tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tensorFlow.tfodParameters = new TFObjectDetector.Parameters(tensorFlow.tfodMonitorViewId);
//        tensorFlow.tfod = ClassFactory.getInstance().createTFObjectDetector(tensorFlow.tfodParameters, vuforia);
//        tensorFlow.tfod.loadModelFromAsset(tensorFlow.tfod_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
//    }
}
