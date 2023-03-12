// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.*;
import javax.sound.sampled.SourceDataLine;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;  

public class Webcam extends SubsystemBase {

  final NetworkTableInstance instance = NetworkTableInstance.getDefault();
  final NetworkTable nt_vision = instance.getTable("Vision");

  Thread visionThread;
  public Webcam() {
    visionThread =
    new Thread(
        () -> {
          // Get the UsbCamera from CameraServer
          UsbCamera camera = CameraServer.startAutomaticCapture();
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
    SmartDashboard.putNumber("Relative Pitch", getTargetRelativePitch());
    SmartDashboard.putNumber("Relative Yaw", getTargetRelativeYaw());
    SmartDashboard.putNumber("Relative Distance", getTargetRelativeDistance());
    SmartDashboard.putNumberArray("Center Coordinate", centerCoordinate());
  }

  public double getTargetRelativePitch() {
    DoubleSubscriber pitchSub = nt_vision.getDoubleTopic("Relative Pitch").subscribe(0.0);
    return pitchSub.get();
  }

  public double getTargetRelativeYaw() {
    DoubleSubscriber yawSub = nt_vision.getDoubleTopic("Relative Yaw").subscribe(0.0);
    return yawSub.get();
  }

  public double getTargetRelativeDistance() {
    DoubleSubscriber distSub = nt_vision.getDoubleTopic("Relative Distance").subscribe(0.0);
    return distSub.get();
  }

  public double[] centerCoordinate() {
    double[] holder= {};
    DoubleArraySubscriber centerSub = nt_vision.getDoubleArrayTopic("Center Coordinate").subscribe(holder);
    double[] coord = centerSub.get();
    return coord;
  }
}
