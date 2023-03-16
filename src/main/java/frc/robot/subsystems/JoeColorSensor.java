// all credits to rev robotics for making example code : 
//https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//imports for color sensor v3
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;


import com.revrobotics.ColorMatch;

public class JoeColorSensor extends SubsystemBase {
  
  public String lastdetectedColor = "";
  int objectdistance = 0;
  //raise the division factor to slow down how often this happens. 
  int divisionfactor = 2;
  int currentPass = 0;
  public JoeColorSensor()
  {
    buildColorMatches();
  }

    /**
   * Change the I2C port below to match the connection of your color sensor
   */
  private final I2C.Port i2cPort = I2C.Port.kMXP;

  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch m_colorMatcher = new ColorMatch();

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  // private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  // private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  // private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  // private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  
  private final Color kConeTarget = new Color(0.367, 0.578, 0.05);
  private final Color kConeFarTarget = new Color(0.312, 0.475, 0.207);
  private final Color kCubeTarget = new Color(0.192, 0.290, 0.517);
  private final Color kCubeFarTarget = new Color(0.247, 0.444, 0.301);
  private final Color kCubeLogoTarget = new Color(0.237, 0.47, 0.297);
  private final Color kRoomLightingTarget = new Color(0.25, .48, 0.27);
  private final Color kRoomLightingVertical = new Color(0.318, .470, 0.212);
  private final Color kRoomLightingAngle = new Color(0.287, .477, 0.235);
  private final Color kRoomLightingGround = new Color(0.297, .479, 0.224);

  @Override
    public void periodic() {
      GetColorSensorData();
  }

  public void GetColorSensorData() {
    if (currentPass < divisionfactor)
    {
      currentPass++;
      return;
    }
    else{
      currentPass = 0;
    }
      /**
       * The method GetColor() returns a normalized color value from the sensor and can be
       * useful if outputting the color to an RGB LED or similar. To
       * read the raw color, use GetRawColor().
       * 
       * The color sensor works best when within a few inches from an object in
       * well lit conditions (the built in LED is a big help here!). The farther
       * an object is the more light from the surroundings will bleed into the 
       * measurements and make it difficult to accurately determine its color.
       */
      Color detectedColor = m_colorSensor.getColor();
  
      /**
       * Run the color match algorithm on our detected color
       */
      String colorString;
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

      if (match.color == kConeTarget || match.color == kConeFarTarget) {
        lastdetectedColor = "Cone";
        colorString = "Cone";
      } else if (match.color == kCubeLogoTarget || match.color == kCubeFarTarget || match.color == kCubeTarget) {
        colorString = "Cube";
        lastdetectedColor = "Cube";
      } else if (match.color == kRoomLightingTarget || match.color == kRoomLightingAngle || match.color == kRoomLightingGround || match.color == kRoomLightingVertical) {
        colorString = "RoomLight";
        lastdetectedColor = "RoomLight";
      } else {
        colorString = "Unknown";
      }
      
      /**
       * Open Smart Dashboard or Shuffleboard to see the color detected by the 
       * sensor.
       */
      SmartDashboard.putNumber("Red", detectedColor.red);
      SmartDashboard.putNumber("Green", detectedColor.green);
      SmartDashboard.putNumber("Blue", detectedColor.blue);
      SmartDashboard.putNumber("Confidence", match.confidence);
      SmartDashboard.putString("Detected Color", colorString);

      objectdistance = m_colorSensor.getProximity();
      SmartDashboard.putNumber("ObjectDistance", objectdistance);
    }

    public void buildColorMatches() {
      //m_colorMatcher.addColorMatch(kBlueTarget);
      //m_colorMatcher.addColorMatch(kGreenTarget);
      //m_colorMatcher.addColorMatch(kRedTarget);
      //m_colorMatcher.addColorMatch(kYellowTarget);
      m_colorMatcher.addColorMatch(kConeTarget); 
      m_colorMatcher.addColorMatch(kCubeTarget); 
      m_colorMatcher.addColorMatch(kConeFarTarget); 
      m_colorMatcher.addColorMatch(kCubeFarTarget);
      m_colorMatcher.addColorMatch(kCubeLogoTarget);
      m_colorMatcher.addColorMatch(kRoomLightingTarget);   
      m_colorMatcher.addColorMatch(kRoomLightingAngle);
      m_colorMatcher.addColorMatch(kRoomLightingVertical);
      m_colorMatcher.addColorMatch(kRoomLightingGround); 
    }
      
}
