within FBM.Controls.ControlHeating.Interfaces;
record ControlPara
  parameter Modelica.SIunits.Temperature TRoo_nominal=273.15 + 21
    "Room temperature at nominal condition"
                                           annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.Temperature TSupNom
    "Nominal heating curve supply temperature"
                                              annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
    "Nominal difference between supply and return water temperatures"
                                                                     annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.TemperatureDifference dTHeaterSet(min=0) = 2
    "Difference between heating curve setpoint and heater setpoint"
                                                                   annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.Time timeFilter=43200
    "Time constant for filter on ambient temperature"
                                                     annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.Temperature TSupMin=273.15 + 30
    "Minimum supply temperature if enabled"
                                           annotation(Dialog(group= "Heating controler parameters"));
  parameter Boolean minSup=true
    "true to limit the supply temperature on the lower side"
                                                            annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.TemperatureDifference dTOutHeaBal=0
    "Offset for heating curve"
                              annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.Temperature TOut_nominal=273.15 - 8
    "Outside temperature"
                         annotation(Dialog(group= "Heating controler parameters"));
  parameter Modelica.SIunits.TemperatureDifference corFac_val = 2
    "correction term for TSet of the heating curve"
                                                   annotation(Dialog(group= "Heating controler parameters"));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ControlPara;
