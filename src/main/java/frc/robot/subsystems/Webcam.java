// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Webcam extends SubsystemBase {
  Thread visionThread;
  private static Webcam frontWebcam, backWebcam;

  public static Webcam getFrontWebcam() {
    if (frontWebcam == null) frontWebcam = new Webcam(0);
    return frontWebcam;
  }

  public static Webcam getBackWebcam() {
    if (backWebcam == null) backWebcam = new Webcam(1);
    return backWebcam;
  }

  private Webcam(int port) {
    visionThread =
    new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture(port);
          // Set the resolution
          camera.setResolution(640, 480);

          // Get a CvSink. This will capture Mats from the camera
          CvSink cvSink = CameraServer.getVideo();
          // Setup a CvSource. This will send images back to the Dashboard
          CvSource outputStream = CameraServer.putVideo("Rectangle", 640, 480);

          // Mats are very memory expensive. Lets reuse this Mat.
          Mat mat = new Mat();

          // This cannot be 'true'. The program will never exit if it is. This
          // lets the robot stop this thread when restarting robot code or
          // deploying.
          while (!Thread.interrupted()) {
            // Tell the CvSink to grab a frame from the camera and put it
            // in the source mat.  If there is an error notify the output.
            if (cvSink.grabFrame(mat) == 0) {
              // Send the output the error.
              outputStream.notifyError(cvSink.getError());
              // skip the rest of the current iteration
              continue;
            }
            // Put a rectangle on the image
            // Imgproc.rectangle(
            //     mat, new Point(100, 100), new Point(400, 400), new Scalar(255, 255, 255), 5);

            Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2GRAY);
            // Give the output stream a new image to display
            outputStream.putFrame(mat);
          }
        });
    visionThread.setDaemon(true);
    visionThread.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
