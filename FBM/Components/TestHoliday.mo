within FBM.Components;
model TestHoliday "output a boolean : 1 in working day 0 however"
  parameter Integer FirstDayofsimulation= 5
    "From 1 for monday to 7 for sunday";
  parameter Integer nOut=1 "Number of days to predict";
  Modelica.Blocks.Interfaces.BooleanOutput y
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Buildings.Controls.Sources.DayType dayTypHoliday(
    nout=nOut,
    iStart=1,
    days={Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.NonWorkingDay,Buildings.Controls.Types.Day.NonWorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay,
        Buildings.Controls.Types.Day.WorkingDay,Buildings.Controls.Types.Day.WorkingDay})
    "Model that outputs the type of the day in a year (normal, holliday,day off)"
    annotation (Placement(transformation(extent={{-22,-12},{-2,8}})));
equation
  if (dayTypHoliday.y[nOut] == Buildings.Controls.Types.Day.WorkingDay) then
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
          textString="Holiday")}),                               Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end TestHoliday;
