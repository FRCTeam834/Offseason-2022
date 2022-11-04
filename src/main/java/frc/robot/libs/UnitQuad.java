package frc.robot.libs;

public class UnitQuad implements UnitScaleFunction {
  public double calculate(double input) {
    return Math.pow(input, 2);
  }
}
