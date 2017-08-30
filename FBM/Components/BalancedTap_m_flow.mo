within FBM.Components;
model BalancedTap_m_flow
  "DHW consumption with input for flowrate at 60 degC"
  extends FBM.Components.BaseClasses.BalancedTap(idealSource(m_flow_small=1E-4*
          abs(m_flow_nominal)));
  Modelica.Blocks.Interfaces.RealInput mDHW60C(unit="kg/s") = mFlo60C
    "Mass flowrate of DHW at 60 degC in kg/s"
    annotation (Placement(transformation(extent={{-120,80},{-80,120}}),
        iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={20,100})));
  annotation (Documentation(info="<html>
  <p><b>Description</b> </p>
  <p><b> Component import from the IDEAS library and adapted for the Buildings library </b></p>  
<p>Model of a domestic hot water (DHW) system composed mainly of a thermostatic mixing valve. The DHW flow rate is passed as a realInput.</p>
<p>This model is an extension of the <a href=\"modelica://IDEAS.Thermal.Components.Domestic_Hot_Water.partial_DHW\">Partial_DHW</a> model, see there for the documentation.</p>
<p><h4>Examples</h4></p>
<p>An example of this model is given in <a href=\"modelica://IDEAS.Thermal.Components.Examples.StorageTank_DHW_HP\">IDEAS.Thermal.Components.Examples.StorageTank_DHW_HP</a>.</p>
</html>"));
end BalancedTap_m_flow;
