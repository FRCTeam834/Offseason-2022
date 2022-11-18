package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.libs.PIDGains;
import frc.robot.libs.TuneableNumber;

public class TuneablePIDGains extends PIDGains {
  private static final NetworkTable TUNEABLE_TABLE = NetworkTableInstance.getDefault().getTable("Tuneable");

  private String name;
  private TuneableNumber tuneableP = null;
  private TuneableNumber tuneableI = null;
  private TuneableNumber tuneableD = null;

  // TuneableNumber builder
  private TuneableNumber tnBuilder(String type, double defaultValue) {
    return new TuneableNumber(TUNEABLE_TABLE, name + "_" + type, defaultValue, Constants.tuningMode);
  }

  public TuneablePIDGains(String name, double kP) {
    this(kP, 0.0, 0.0);

    this.name = name;
    tuneableP = tnBuilder("P_GAIN", kP);
  }

  public TuneablePIDGains(String name, double kP, double kD) {
    this(kP, 0.0, kD);

    this.name = name;
    tuneableP = tnBuilder("P_GAIN", kP);
    tuneableD = tnBuilder("D_GAIN", kD);
  }

  public TuneablePIDGains(String name, double kP, double kI, double kD) {
    this(kP, kI, kD);

    this.name = name;
    tuneableP = tnBuilder("P_GAIN", kP);
    tuneableI = tnBuilder("I_GAIN", kI);
    tuneableD = tnBuilder("D_GAIN", kD);
  }

  private TuneablePIDGains(double kP, double kI, double kD) {
    super(kP, kI, kD);
  }

  @Override
  public double getP() {
    return 
      Constants.tuningMode ? (tuneableP == null? kP : tuneableP.get()) : kP;
  }

  @Override
  public double getI() {
    return 
      Constants.tuningMode ? (tuneableI == null? kI : tuneableI.get()) : kI;
  }

  @Override
  public double getD() {
    return 
      Constants.tuningMode ? (tuneableD == null? kD : tuneableD.get()) : kD;
  }

  public boolean hasChanged() {
    return (
      tuneableP == null ? false : tuneableP.hasChanged() ||
      tuneableI == null ? false : tuneableI.hasChanged() ||
      tuneableD == null ? false : tuneableD.hasChanged()
    );
  }
}
