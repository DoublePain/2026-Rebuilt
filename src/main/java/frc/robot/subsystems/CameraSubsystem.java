package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSubsystem extends SubsystemBase {

  private UsbCamera camera;

  public CameraSubsystem() {
    // Start automatic capture from USB camera 0 AKA Logitech Camera
    camera = CameraServer.startAutomaticCapture(0);


    camera.setResolution(320, 240);
    camera.setFPS(20);


    camera.setBrightness(50);
    camera.setExposureAuto();
  }

  @Override
  public void periodic() {
   
  }
}