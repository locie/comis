within FBM.Controls.ControlHeating;
model RunningMeanTemperatureEN15251
  "Calculate the running mean temperature of 7 days, acccording to norm EN15251"
  parameter Real[7] TAveDayIni(unit="K", displayUnit="degC") = ones(7).* 283.15
    "Initial running mean temperature";
  // Interface
   discrete Modelica.Blocks.Interfaces.RealOutput TRm(unit="K",displayUnit = "degC")
    "Running mean average temperature"
     annotation (Placement(transformation(extent={{96,-10},{116,10}})));
protected
  discrete Real[7] TAveDay(unit="K",displayUnit = "degC")
    "Vector with the average day temperatures of the previous nTermRm days";
  parameter Real coeTRm[7] = {1, 0.8, 0.6, 0.5, 0.4, 0.3, 0.2}./3.8
    "weighTAmb.yg coefficient for the running average";
  Real intTAmb "integral of TAmb.y";
  Modelica.Blocks.Interfaces.RealInput TAmb( final unit="K",
                                                     displayUnit="degC")
    "Needed to connect to conditional connector";
public
  Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
        transformation(extent={{-102,62},{-62,102}}), iconTransformation(extent=
           {{-152,40},{-132,60}})));
equation
  der(intTAmb) =  TAmb;
algorithm
  when initial() then
    // initialization of the discrete variables
    TAveDay:=TAveDayIni;
    TRm:=TAveDayIni[1];
  elsewhen sample(24*3600,24*3600) then
    // Update of TAveDay
    for i in 2:7 loop
      TAveDay[i] := pre(TAveDay[i-1]);
    end for;
    TAveDay[1] := intTAmb / 24/3600;
    TRm :=TAveDay*coeTRm;
  end when;
equation
    // reinitialisation of the intTAmb
  when sample(24*3600,24*3600) then
    reinit(intTAmb,0);
  end when;
   connect(weaBus.TDryBul, TAmb);
    annotation (Placement(transformation(extent={{-80,-10},{-60,10}})),
              Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-100,
            -100},{100,100}})),
    experiment(StopTime=864000),
    __Dymola_experimentSetupOutput,
    Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,100}}),
        graphics={
        Rectangle(
          extent={{100,100},{-100,-100}},
          lineColor={100,100,100},
          fillPattern=FillPattern.Solid,
          fillColor={255,255,255}),
        Line(
          points={{0,100},{98,0},{0,-100}},
          color={100,100,100},
          smooth=Smooth.None),
        Text(
          extent={{-100,140},{100,100}},
          lineColor={0,0,255},
          textString="%name"),
        Text(
          extent={{-48,32},{58,-26}},
          lineColor={0,0,255},
          textString="EN15251")}),
Documentation(revisions="<html>
<ul>
<li>
January 19, 2015, by Damien Picard:<br/>
First implementation.
</li>
</ul>
</html>"));
end RunningMeanTemperatureEN15251;
