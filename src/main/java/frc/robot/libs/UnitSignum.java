package frc.robot.libs;

/**
 * 
 * Signum scaled to fit (1, 1)
 * 
 * Tuning:
 * https://www.desmos.com/calculator/c5a1t6pn8a
 */
public class UnitSignum {
  private final double b;
  private final double L;
  private final double n;
  private final double s;
  private final double k;
  /**
   * 
   * @param b ~ horizontal scale
   * @param L ~ vertical scale
   * @param n ~ also horizontal scale
   * @param s ~ vertical displacement
   */
  public UnitSignum(double b, double L, double n, double s) {
    this.b = b;
    this.L = L;
    this.n = n;
    this.s = s;
    this.k = -Math.log(
      (L / (1 - s) - 1) / b
    ) - n;
  }

  /** */
  public double calculate(double input) {
    return (
      L / (1 + b * Math.pow(Math.E, -k * input - n)) + s
    );
  }
}
