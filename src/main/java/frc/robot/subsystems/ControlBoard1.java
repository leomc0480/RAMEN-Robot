package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControlBoard1 extends SubsystemBase {

  private static final int X_RightAxis = 0;

  //Hardware ----------------------------------------------------------------->
  private final XboxController operatorControl = new XboxController(1); //declaración del control dentro del puerto 1

  //INPUTS ------------------------------------------------------------------>

  //OUTPUTS ----------------------------------------------------------------->

  //Logic ----------------------------------------------------------------->
  int Y_LeftAxis = 1, X_LeftAxis = 2, A_Button = 1;
  
  public ControlBoard1(){} //Constructor del subsistema para todas las variables

  //------------------// Funciones del subsistema //-------------------------------//

  //funcion principal de Drive con argumentos de entrada de controles
  public boolean getXButton(){
    return operatorControl.getRawButton(3);
  }

  public boolean getYButton(){
    return operatorControl.getRawButton(4);
  }

  public double getControlYAxis(){
    return operatorControl.getRawAxis(Y_LeftAxis);
    
  }
  public double getControlXAxis(){
    return operatorControl.getRawAxis(X_RightAxis);
  }

  
  public boolean getControlAButton(){ //función que regresa el estado del botón A
      return operatorControl.getRawButton(A_Button); //mandar llamar el ID del control correspondiente al boton A (1)
  }

  //todas las funciones de arriba se pueden heredar en otros archivos

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
