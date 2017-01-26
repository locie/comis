within FBM.Controls.ControlHeating;
model Ctrl_Heating_DHW
  "Heating curve control for heating (without Thermal Energy Storage) and separate DHW storage tank"
  extends Interfaces.Partial_Ctrl_Heating_TES;
equation
  TTopSet = TDHWSet + dTSafetyTop;
  TBotSet = TDHWSet + dTSafetyBot;
  // onOff refers to TES loading.  There is no on/off control for the emission system: the
  // heat pump gets the heating curve + 2K as setpoint temperature and will go on only if
  // there is a flowrate through the HP.
  if noEvent(TTankTop < TTopSet and TTankBot < (TBotSet - dTSafetyBot)) then
    // top too cold, system MUST be on except if bottom is still very hot (temp inversion?)
    onOff = 1;
    THPSet = TTopSet + dTHPTankSet;
  elseif noEvent(TTankBot < TBotSet and onOff > 0.5) then
    // HP running, top is fine, bottom too cold: keep running
    onOff = 1;
    THPSet = TTopSet + dTHPTankSet;
  else
    // all other cases: shut down
    onOff = 0;
    THPSet = heatingCurve.TSup + 2;
  end if;
  annotation (Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-80,-80},
            {100,80}}),
                      graphics), Documentation(info="<html>
<p><b>Description</b> </p>
<p>Heating curve based control of a heater + TES charging control for DHW. The set point temperature for the heater is higher than the heating curve output in order to make sure that the heating curve temperature is met also when thermal losses are present in the circuit. The heater set temperature is the maximum of the requirements for space heating and DHW: if tank charging is occurring, the DHW temperature requirements will normally be higher than for space heating. </p>
<p>This controller tries to limit and even avoid the creating of events.</p>
<p><h4>Model use</h4></p>
<p><ol>
<li>Use this controller in a heating system with a separate DHW storage tank where you want to follow a heating curve for space heating.</li>
</ol></p>
</html>",revisions="<html>
<p><ul>
<li>2016 November, Wilfried Thomaré: Redefining for the Buildings library</li>
</ul></p>
</html>"));
end Ctrl_Heating_DHW;
