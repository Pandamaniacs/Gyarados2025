package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIONavX implements GyroIO {
  private final AHRS m_NavX;

  public GyroIONavX() {
    m_NavX = new AHRS(NavXComType.kMXP_SPI); // instantiates gyroscope
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = m_NavX.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(getYawVelocity());
  }

  public double getYaw() {
    return -m_NavX.getAngle(); // must be negative or rotation issue happens.
  }

  public double getYawVelocity() {
    return m_NavX.getRate();
  }

  @Override
  public void zero() {
    m_NavX.reset();
  }
}
