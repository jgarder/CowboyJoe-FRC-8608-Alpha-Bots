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

import javax.lang.model.util.ElementScanner14;

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
  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
  private final Color kGreenTarget = new Color(0.197, 0.561, 0.240);
  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);
  private final Color kYellowTarget = new Color(0.361, 0.524, 0.113);
  private final Color kConeTarget = new Color(0.345, 0.515, 0.147);
  private final Color kCubeTarget = new Color(0.220, 0.369, 0.412);
  private final Color kRoomLightingTarget = new Color(0.25, .48, 0.27);

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
  
      if (match.color == kBlueTarget) {
        colorString = "Blue";
      } else if (match.color == kRedTarget) {
        colorString = "Red";
      } else if (match.color == kGreenTarget) {
        colorString = "Green";
      } else if (match.color == kYellowTarget) {
        colorString = "Yellow";
      } else if (match.color == kConeTarget) {
        lastdetectedColor = "Cone";
        colorString = "Cone";
      } else if (match.color == kCubeTarget) {
        colorString = "Cube";
        lastdetectedColor = "Cube";
      } else if (match.color == kRoomLightingTarget) {
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
      m_colorMatcher.addColorMatch(kRoomLightingTarget);    
    }
      
}
