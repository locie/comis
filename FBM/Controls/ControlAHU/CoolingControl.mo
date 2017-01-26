within FBM.Controls.ControlAHU;
block CoolingControl
  "Controller for the free cooling and the mechanical cooling"
   extends Modelica.Blocks.Icons.Block;
   parameter Modelica.SIunits.Temperature TRooCoo = 25+273.15
    "Set point for mechanical cooling";
   parameter Modelica.SIunits.Temperature TRooFre = 22+273.15
    "Maximum temperature above which free cooling is enabled";
   parameter Modelica.SIunits.Temperature TOutFre = 16+273.15
    "Outside temperature above which free cooling is allowed";
   parameter Modelica.SIunits.TemperatureDifference dT = 1
    "Dead-band for free cooling";
   parameter Real Kp(min=0) = 1 "Proportional band for mechanical cooling";
   Modelica.Blocks.Interfaces.RealInput TRoo(unit="K") "Room air temperature"
     annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
   Modelica.Blocks.Interfaces.RealInput TOut(unit="K")
    "Outside air temperature"
     annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
   Modelica.Blocks.Interfaces.RealOutput TSupCoo
    "Control signal for set point for leaving air temperature of cooling coil"
     annotation (Placement(transformation(extent={{100,50},{120,70}}),
         iconTransformation(extent={{100,50},{120,70}})));
   Modelica.Blocks.Interfaces.RealOutput yF
    "Control signal for free cooling, 1 if free cooling should be provided"
     annotation (Placement(transformation(extent={{100,-10},{120,10}}),
         iconTransformation(extent={{100,-10},{120,10}})));
   Modelica.Blocks.Interfaces.RealOutput yHex
    "Control signal for heat recovery damper"
     annotation (Placement(transformation(extent={{100,-70},{120,-50}}),
         iconTransformation(extent={{100,-70},{120,-50}})));
initial equation
   yF   = 0;
   yHex = 1;
algorithm
   when TRoo > TRooFre and TOut > TOutFre and TOut < TRoo - dT then
     yF   := 1;
     yHex := 0;
   elsewhen  TOut < TOutFre-dT or TOut > TRoo then
     yF   := 0;
     yHex := 1;
   end when;
   TSupCoo :=273.15 + Buildings.Utilities.Math.Functions.smoothLimit(
               x=30 - 20*Kp*(TRoo - TRooCoo),
               l=10,
               u=30,
               deltaX=0.1);
   annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},
             {100,100}}), graphics={
         Text(
           extent={{-94,38},{-64,80}},
           lineColor={0,0,255},
           textString="TRoo"),
         Text(
           extent={{-94,-82},{-64,-40}},
           lineColor={0,0,255},
           textString="TOut"),
         Text(
           extent={{66,42},{86,74}},
           lineColor={0,0,255},
           textString="yC"),
         Text(
           extent={{-32,100},{24,124}},
           lineColor={0,0,255},
           textString="%name"),
         Text(
           extent={{66,-16},{86,16}},
           lineColor={0,0,255},
           textString="yF"),
         Text(
           extent={{68,-74},{88,-42}},
           lineColor={0,0,255},
          textString="yHex")}),Documentation(info="<html>
<p>
This block computes a control signal for free cooling and for mechanical cooling.
</p>
</html>",
        revisions="<html>
<ul>
<li>
February 27, 2015, by Michael Wetter:<br/>
Changed controller to output setpoint for supply air temperature for cooling coil.
</li>
</ul>
</html>"));
end CoolingControl;
