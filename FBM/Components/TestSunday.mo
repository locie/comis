within FBM.Components;
model TestSunday "output a boolean : 1 in working day 0 however"
  parameter Integer FirstDayofsimulation= 5
    "From 1 for monday to 7 for sunday";
  parameter Integer nOut=1 "Number of days to predict";
  Buildings.Controls.Sources.DayType dayTypMon(
    iStart=FirstDayofsimulation,
    nout=nOut,
    days={Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.Holiday,
        Buildings.Controls.Types.Day.Holiday})
    "Model that outputs the type of the day in a week (working day or holliday)"
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  Modelica.Blocks.Interfaces.BooleanOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
equation
  if (dayTypMon.y[nOut] == Buildings.Controls.Types.Day.WorkingDay) then
        y = true;
 else    y =false;
end if;
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-100,100},{100,-100}},
          lineColor={162,29,33},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-96,96},{96,-96}},
          lineColor={162,29,33},
          fillColor={188,36,38},
          fillPattern=FillPattern.Solid),
        Text(
          extent={{-48,32},{60,-32}},
          lineColor={0,0,0},
          fillColor={188,36,38},
          fillPattern=FillPattern.Solid,
          textString="Sunday")}),                                Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestSunday;
