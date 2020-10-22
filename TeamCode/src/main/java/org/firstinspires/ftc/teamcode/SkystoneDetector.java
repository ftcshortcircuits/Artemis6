/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector
{
    /*
     * Constants to use in calls to constructor.  Available to
     * callers as GenericLoad.Alliance.*
     */
    public enum Alliance {
        BLUE,
        RED
    }
    private Telemetry telemetry;
    private Alliance alliance;
    private HardwareMap hwmap;

    OpenCvCamera webcam;

    private Scalar leftMean, middleMean, rightMean;

    private int skystone;  // The calculated Skystone position, alliance aware, for get method

    /**
     * Constructor for SkystoneDetector class.
     *
     * @param hardwareMap hardwareMap from calling OpMode
     * @param tele telemetry object from calling OpMode
     * @param allianceColor SkystoneDetector.Alliance.{BLUE,RED}
     */
    SkystoneDetector(HardwareMap hardwareMap, Telemetry tele, Alliance allianceColor ) {
        alliance = allianceColor;
        hwmap = hardwareMap;
        telemetry = tele;
        /*
         * Instantiate an OpenCvCamera object for the camera we'll be using.
         * We're using a webcam: "Webcam 1", in the RC configuration.  This really should not
         * be hard-coded here, but we'll leave it for now.
         *
         * We pass it the view that we wish to use for camera monitor (on
         * the RC phone, streamed to the DS phone). If no camera monitor is desired,
         * use the alternate single-parameter constructor instead (commented out below)
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View
        //webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));

        /*
         * Open the connection to the camera device
         */
        webcam.openCameraDevice();

        /*
         * Specify the image processing pipeline we wish to invoke upon receipt
         * of a frame from the camera.
         */
        webcam.setPipeline(new SkystonePipeline());

        /*
         * Tell the webcam to start streaming images to us! Note that you must make sure
         * the resolution you specify is supported by the camera. If it is not, an exception
         * will be thrown.
         *
         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
         * supports streaming from the webcam in the uncompressed YUV image format. This means
         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
         * Streaming at 720p will limit you to up to 10FPS. However, streaming at frame rates other
         * than 30FPS is not currently supported, although this will likely be addressed in a future
         * release. TLDR: You can't stream in greater than 480p from a webcam at the moment.
         *
         * Also, we specify the rotation that the webcam is used in. This is so that the image
         * from the camera sensor can be rotated such that it is always displayed with the image upright.
         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
         * away from the user.
         */
        webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    /**
     * getStone - the important get method.  Identifies darkest sample, left, middle, or right,
     * and return 1, 2, or 3 for BLUE alliance, or 3, 2, or 1 for RED alliance, respectively.
     *
     * @return stone location, as above
     */
    int getStone(){
        int stone = 2;      // Initialized to shut up Java

        /*
         * The zeroth element of each OpenCV Scalar object should contain the mean lumen value from
         * the transformed and cropped sub-matrices, as processed by the frame pipeline.
         *
         * Don't try to be cute, here.  The "or equals" will only happen if everything returned
         * zero, so it's not worth trying to find an optimal default case.
         */
        if((leftMean.val[0]<=middleMean.val[0]) && (leftMean.val[0]<=rightMean.val[0]))
            stone = (alliance== Alliance.BLUE ? 1 : 3);  // Left position, 1 on BLUE or 3 on RED

        if((middleMean.val[0]<=leftMean.val[0]) && (middleMean.val[0]<=rightMean.val[0]))
            stone = 2;      // Alliance doesn't matter: stone is in the middle

        if((rightMean.val[0]<=middleMean.val[0]) && (rightMean.val[0]<=leftMean.val[0]))
            stone = (alliance== Alliance.BLUE ? 3 : 1);  // Right position, 3 on BLUE or 1 on RED

        return stone;
    }

    /*
     * Some possibly useful methods follow
     */

    /**
     * stopDetect method.  Shuts it all down to save processing time after the Skystone
     * location has been found and reported to the calling OpMode.
     */
    void stopDetect(){
        webcam.stopStreaming();
        webcam.closeCameraDevice();
        //TODO Should we free up memory, too?
    }

    /**
     * pauseDetect method.  Temporary pause of viewport.  We probably won't need this.
     */
    void pauseDetect(){
        webcam.pauseViewport();
    }

    /**
     * resumeDetect method.  Restart from above temporary pause.  We probably won't need this.
     */
    void resumeDetect(){
        webcam.resumeViewport();
    }


    /*
     * Image processing pipeline to be run upon receipt of each frame from the camera.
     * Note that the processFrame() method is called serially from the frame worker thread -
     * that is, a new camera frame will not come in while you're still processing a previous one.
     * In other words, the processFrame() method will never be called multiple times simultaneously.
     *
     * However, the rendering of your processed image to the viewport is done in parallel to the
     * frame worker thread. That is, the amount of time it takes to render the image to the
     * viewport does NOT impact the amount of frames per second that your pipeline can process.
     *
     * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
     * frame worker thread. This should not be a problem in the vast majority of cases. However,
     * if you're doing something weird where you do need it synchronized with your OpMode thread,
     * then you will need to account for that accordingly.
     */
    class SkystonePipeline extends OpenCvPipeline
    {
        /*
         * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
         * highly recommended to declare them here as instance variables and re-use them for
         * each invocation of processFrame(), rather than declaring them as new local variables
         * each time through processFrame(). This removes the danger of causing a memory leak
         * by forgetting to call mat.release(), and it also reduces memory pressure by not
         * constantly allocating and freeing large chunks of memory.
         */

        /*
         * CONCEPT: Instantiate a static Mat for the HSV (or grayscale) converted image, and
         * three Mats for the selected regions.
         */
        private Mat convertedInput = new Mat();
        private Mat leftSample = new Mat();
        private Mat middleSample = new Mat();
        private Mat rightSample = new Mat();

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * IMPORTANT NOTE: the input Mat that is passed in as a parameter to this method
             * will only dereference to the same image for the duration of this particular
             * invocation of this method. That is, if for some reason you'd like to save a copy
             * of this particular frame for later use, you will need to either clone it or copy
             * it to another Mat.
             */

            /*
             * Draw three rectangles to sample part of the face of the three visible stones.
             * We're using 1/8 screen granularity for rows, 1/12 screen granularity for
             * columns.  The boxes will use the center 1/4 of the screen vertically, and each box
             * be centered horizontally in each 1/3 of the screen (with an equal-sized gap between
             * boxes.)
             *
             * To be clear: the boxes are 2/12 of the screen wide, and 2/8 of the screen tall.
             *
             * Note that we're working with the original RGB color image, not transformed
             * in any way.  This is the image that is used for camera view.
             *
             * Good coding practice would require that I define the rectangles separately from
             * the drawing, and the image processing, and use the same references in both code
             * segments.  I don't yet know how to do that with the OpenCV library methods, yet.
             *
             */
            Imgproc.rectangle(                              // Leftmost of three
                    input,
                    new Point(
                            (input.cols()*1)/12,      // Left edge
                            (input.rows()*3)/8 ),     // Top edge
                    new Point(
                            (input.cols()*3)/12,      // Right edge
                            (input.rows()*5)/8 ),     // Bottom edge
                    new Scalar(255, 0, 0), 4);      // Red boundary

            Imgproc.rectangle(                              // Middle of three
                    input,
                    new Point(
                            (input.cols()*5)/12,      // Left edge
                            (input.rows()*3)/8 ),     // Top edge
                    new Point(
                            (input.cols()*7)/12,      // Right edge
                            (input.rows()*5)/8 ),     // Bottom edge
                    new Scalar(0, 255, 0), 4);      // Green boundary

            Imgproc.rectangle(                              // Rightmost of three
                    input,
                    new Point(
                            (input.cols()*9)/12,      // Left edge
                            (input.rows()*3)/8 ),     // Top edge
                    new Point(
                            (input.cols()*11)/12,     // Right edge
                            (input.rows()*5)/8 ),     // Bottom edge
                    new Scalar(0, 0, 255), 4);      // Blue boundary

            /*
             * Convert input image to yCrCb, which seems to be popular for this sort of thing.
             * Then take subregions as per box definitions above.  Lousy programming style,
             *  but it will do for now.
             */
            Imgproc.cvtColor(input, convertedInput, Imgproc.COLOR_RGB2YCrCb);

            int columnCount = (input.cols()*2)/12;  // Number of columns to copy
            int rowCount    = (input.rows()*2)/8;      // Number of rows to copy

            Rect rectLeft   = new Rect((input.cols()*1)/12, (input.rows()*3)/8, columnCount, rowCount);
            Rect rectMiddle = new Rect((input.cols()*5)/12, (input.rows()*3)/8, columnCount, rowCount);
            Rect rectRight  = new Rect((input.cols()*9)/12, (input.rows()*3)/8, columnCount, rowCount);

            convertedInput.submat(rectLeft).copyTo(leftSample);
            convertedInput.submat(rectMiddle).copyTo(middleSample);
            convertedInput.submat(rectRight).copyTo(rightSample);

            /*
             * Now, extract the first (zero index) channel, in-place.  This should be y (Lumen)
             */
            Core.extractChannel(leftSample, leftSample, 0);
            Core.extractChannel(middleSample, middleSample, 0);
            Core.extractChannel(rightSample, rightSample, 0);

            /*
             * Calculate the three means. They will be static in the class, so the getStone() method
             * will be able to read them anytime after the first frame.  Core.mean() will return an
             * OpenCV Scalar object type, which is going to require extra effort to process.
             */
            leftMean = Core.mean(leftSample);
            middleMean = Core.mean(middleSample);
            rightMean = Core.mean(rightSample);

            return input;
        }
    }
}