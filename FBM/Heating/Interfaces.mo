within FBM.Heating;
package Interfaces "Partial models for heating systems"
extends Modelica.Icons.InterfacesPackage;
  partial model Partial_HydraulicHeating "Hydraulic multi-zone heating "
    replaceable package Medium = Buildings.Media.Water;
    extends FBM.Interfaces.BaseClasses.HeatingSystem(
      isHea = true,
      isCoo = false,
      nConvPorts = nZones,
      nRadPorts = nZones,
      nTemSen = nZones,
      nEmbPorts=0,
      nZones=1);
    // --- Parameter: General parameters for the design (nominal) conditions and heat curve
    parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
      "Nominal power, can be seen as the max power of the emission system per zone";
    parameter Boolean minSup=true
      "true to limit the supply temperature on the lower side";
      parameter Modelica.SIunits.Temperature TSupMin=273.15 + 30
      "Minimum supply temperature if enabled";
    parameter Modelica.SIunits.Temperature TSupNom=273.15 + 45
      "Nominal supply temperature";
    parameter Modelica.SIunits.TemperatureDifference dTSupRetNom=10
      "Nominal DT in the heating system";
    parameter Modelica.SIunits.Temperature[nZones] TRoomNom={294.15 for i in 1:
        nZones} "Nominal room temperature";
    parameter Modelica.SIunits.TemperatureDifference corFac_val = 0
      "correction term for TSet of the heating curve";
    parameter Modelica.SIunits.Time timeFilter=43200
      "Time constant for the filter of ambient temperature for computation of heating curve";
    final parameter Modelica.SIunits.MassFlowRate[nZones] m_flow_nominal = QNom/(4180.6*dTSupRetNom)
      "Nominal mass flow rates";


      // -- Pipes and valve parameters
    parameter Modelica.SIunits.Thickness InsuPipeThickness=0.02
                                                               "Thickness of the pipe insulation";
    parameter Modelica.SIunits.Length Pipelength=5
                                                  "pipe length of the supply OR return branch";
    parameter Modelica.SIunits.Pressure dp=3000 "Pressure drop over a single pipe"
      annotation(Dialog(group = "Pipes",
                       enable = includePipes));
   parameter Modelica.SIunits.ThermalConductivity InsuHeatCondu=0.04;


   parameter Real fraKSupply(min=0, max=1) = 0.7
      "Fraction Kv(port_3->port_2)/Kv(port_1->port_2)";
   parameter Real[2] lSupply(each min=0, each max=1) = {0.01, 0.01}
      "Valve leakage, l=Kv(y=0)/Kv(y=1)";
  parameter Buildings.Fluid.Types.CvTypes CvDataSupply=Buildings.Fluid.Types.CvTypes.OpPoint
      "Selection of flow coefficient"
     annotation(Dialog(group = "Flow Coefficient Supply Valve"));
    parameter Real KvSupply(
      fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.Kv then true else false)
      "Kv (metric) flow coefficient [m3/h/(bar)^(1/2)]"
    annotation(Dialog(group = "Flow Coefficient Supply Valve",
                      enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Kv)));
    parameter Real CvSupply(
      fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.Cv then true else false)
      "Cv (US) flow coefficient [USG/min/(psi)^(1/2)]"
    annotation(Dialog(group = "Flow Coefficient Supply Valve",
                      enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Cv)));
    parameter Modelica.SIunits.Area AvSupply(
      fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.Av then true else false)
      "Av (metric) flow coefficient"
     annotation(Dialog(group = "Flow Coefficient Supply Valve",
                       enable = (CvDataSupply==Buildings.Fluid.Types.CvTypes.Av)));
    parameter Real deltaMSupply = 0.02
      "Fraction of nominal flow rate where linearization starts, if y=1"
      annotation(Dialog(group="Pressure-flow linearization"));
    parameter Modelica.SIunits.Pressure dpValve_nominalSupply(displayUnit="Pa",
                                                        min=0,
                                                        fixed= if CvDataSupply==Buildings.Fluid.Types.CvTypes.OpPoint then true else false)
      "Nominal pressure drop of fully open valve, used if CvData=Buildings.Fluid.Types.CvTypes.OpPoint"
      annotation(Dialog(group="Nominal condition",
                 enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
    parameter Modelica.SIunits.Density rhoStdSupply=Medium.density_pTX(101325, 273.15+4, Medium.X_default)
      "Inlet density for which valve coefficients are defined"
    annotation(Dialog(group="Nominal condition", tab="Advanced"));
  protected
    parameter Real Kv_SISupply(
      min=0,
      fixed= false)
      "Flow coefficient for fully open valve in SI units, Kv=m_flow/sqrt(dp) [kg/s/(Pa)^(1/2)]"
    annotation(Dialog(group = "Flow Coefficient Supply Valve",
                      enable = (CvData==Buildings.Fluid.Types.CvTypes.OpPoint)));
    // --- production components of hydraulic circuit
    replaceable Buildings.Fluid.Boilers.BoilerPolynomial heater(
      redeclare replaceable package Medium = Medium,
      m_flow_nominal=sum(m_flow_nominal),
      Q_flow_nominal=sum(QNom),
      dp_nominal=0,
      fue=Buildings.Fluid.Data.Fuels.NaturalGasHigherHeatingValue()) "Heater (boiler, heat pump, ...)"
      annotation (Placement(transformation(extent={{-134,32},{-114,12}})));
    // --- distribution components of hydraulic circuit
    Buildings.Fluid.Movers.FlowControlled_m_flow[nZones] pumpRad(
      m_flow_nominal=m_flow_nominal,
      redeclare each replaceable package Medium = Medium,
      each tau=30,
      each filteredSpeed=false,
      each nominalValuesDefineDefaultPressureCurve=true)
                annotation (Placement(transformation(extent={{98,72},{122,48}})));
    Buildings.Fluid.Actuators.Valves.ThreeWayLinear idealCtrlMixer(
      m_flow_nominal=sum(m_flow_nominal),
      redeclare replaceable package Medium = Medium,
      CvData=Buildings.Fluid.Types.CvTypes.Kv,
      Kv=KvSupply,
      fraK=fraKSupply,
      l=lSupply,
      deltaM=deltaMSupply)                           annotation (Placement(transformation(extent={{34,46},{56,70}})));
    Buildings.Fluid.FixedResistances.Pipe pipeReturn(
      redeclare replaceable package Medium = Medium,
      m_flow_nominal=sum(m_flow_nominal),
      nSeg=3,
      thicknessIns=InsuPipeThickness,
      lambdaIns=InsuHeatCondu,
      length=Pipelength)
             annotation (Placement(transformation(extent={{2,-88},{-18,-96}})));
    Buildings.Fluid.FixedResistances.Pipe pipeSupply(
      redeclare replaceable package Medium = Medium,
      m_flow_nominal=sum(m_flow_nominal),
      nSeg=3,
      thicknessIns=InsuPipeThickness,
      lambdaIns=InsuHeatCondu,
      length=Pipelength)
             annotation (Placement(transformation(extent={{-16,54},{4,62}})));
    Buildings.Fluid.FixedResistances.Pipe[nZones] pipeReturnEmission(
      redeclare each replaceable package Medium = Medium,
      m_flow_nominal=m_flow_nominal,
      each nSeg=3,
      each thicknessIns=InsuPipeThickness,
      each lambdaIns=InsuHeatCondu,
      each length=Pipelength)
      annotation (Placement(transformation(extent={{148,-88},{128,-96}})));
    // --- emission components of hydraulic circuit
    replaceable Buildings.Fluid.HeatExchangers.Radiators.RadiatorEN442_2[
                                                  nZones] emission(
      redeclare each replaceable package Medium = Medium)
      annotation (Placement(transformation(extent={{140,50},{170,70}})));

    // --- boudaries
    Modelica.Thermal.HeatTransfer.Sources.FixedTemperature fixedTemperature(T=293.15)
      "fixed temperature to simulate heat losses of hydraulic components"
      annotation (Placement(transformation(
          extent={{-7,-7},{7,7}},
          rotation=0,
          origin={-149,45})));
    Buildings.Fluid.Sources.FixedBoundary absolutePressure(redeclare
        replaceable package
                Medium =
          Medium, use_T=false,
      nPorts=1)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-106,-114})));
    // --- controllers
    replaceable FBM.Controls.ControlHeating.Ctrl_Heating ctrl_Heating(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom,
      TSupMin=TSupMin,
      minSup=minSup,
      corFac_val=corFac_val,
      THeaterSet(start=293.15)) constrainedby
      FBM.Controls.ControlHeating.Interfaces.Partial_Ctrl_Heating(
      heatingCurve(timeFilter=timeFilter),
      TSupNom=TSupNom,
      dTSupRetNom=dTSupRetNom)
      "Controller for the heater and the emission set point "
      annotation (Placement(transformation(extent={{-176,56},{-156,76}})));
    Modelica.Blocks.Logical.Hysteresis[nZones] heatingControl(each uLow=-0.5,
        each uHigh=0.5) "onoff controller for the pumps of the emission circuits"
      annotation (Placement(transformation(extent={{-140,-86},{-120,-66}})));
    Modelica.Blocks.Sources.RealExpression TSet_max(y=max(TSet))
      "maximum value of set point temperature" annotation (Placement(
          transformation(
          extent={{-21,-10},{21,10}},
          rotation=90,
          origin={-184,41})));
    Modelica.Blocks.Math.Add add[nZones](each k1=-1, each k2=+1)
      annotation (Placement(transformation(extent={{-174,-82},{-160,-68}})));
    // --- Interface
    // --- Sensors
    Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_in(redeclare
        replaceable package
                Medium = Medium, m_flow_nominal=sum(m_flow_nominal))
      "Inlet temperature of the emission system"
      annotation (Placement(transformation(extent={{66,48},{86,68}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTemHea_out(redeclare
        replaceable package Medium =
                         Medium, m_flow_nominal=sum(m_flow_nominal))
      "Outlet temperature of the heater"
      annotation (Placement(transformation(extent={{-62,48},{-42,68}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTemEm_out(redeclare
        replaceable package Medium =
                         Medium, m_flow_nominal=sum(m_flow_nominal))
      "Outlet temperature of the emission system" annotation (Placement(
          transformation(
          extent={{8,-8},{-8,8}},
          rotation=0,
          origin={90,-92})));
    Buildings.Fluid.FixedResistances.SplitterFixedResistanceDpM spl(
      redeclare replaceable package Medium = Medium,
      m_flow_nominal={sum(m_flow_nominal),sum(m_flow_nominal),-sum(m_flow_nominal)},
      dp_nominal={0,0,0})
      annotation (Placement(transformation(extent={{76,-88},{68,-96}})));
    Buildings.Fluid.MixingVolumes.MixingVolume vol(
      redeclare replaceable package Medium = Medium,
      m_flow_nominal=sum(m_flow_nominal),
      V=sum(m_flow_nominal)*30/1000,
      nPorts=1 + nZones)
      annotation (Placement(transformation(extent={{104,-92},{124,-72}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal[nZones](realTrue=
          m_flow_nominal)
      annotation (Placement(transformation(extent={{-104,-86},{-84,-66}})));
    Buildings.Controls.Continuous.LimPID conVal(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k=0.1,
      Ti=120)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={14,86})));
  public
    Modelica.Blocks.Logical.OnOffController onOffController(bandwidth=10)
      annotation (Placement(transformation(extent={{-152,76},{-132,96}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1
      annotation (Placement(transformation(extent={{-114,76},{-94,96}})));
  protected
    Buildings.Controls.Continuous.LimPID conVal1(
      Ti=120,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=1)
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-74,86})));
  initial equation
    if  CvDataSupply == Buildings.Fluid.Types.CvTypes.OpPoint then
      Kv_SISupply =           sum(m_flow_nominal)/sqrt(dpValve_nominalSupply);
      KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
      CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
      AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
    elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Kv then
      Kv_SISupply =           KvSupply*rhoStdSupply/3600/sqrt(1E5)
        "Unit conversion m3/(h*sqrt(bar)) to kg/(s*sqrt(Pa))";
      CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
      AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
    elseif CvDataSupply == Buildings.Fluid.Types.CvTypes.Cv then
      Kv_SISupply =           CvSupply*rhoStdSupply*0.0631/1000/sqrt(6895)
        "Unit conversion USG/(min*sqrt(psi)) to kg/(s*sqrt(Pa))";
      KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
      AvSupply    =           Kv_SISupply/sqrt(rhoStdSupply);
      dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
    else
      assert(CvDataSupply == Buildings.Fluid.Types.CvTypes.Av, "Invalid value for CvData.
Obtained CvData = "   + String(CvDataSupply) + ".");
      Kv_SISupply =           AvSupply*sqrt(rhoStdSupply);
      KvSupply    =           Kv_SISupply/(rhoStdSupply/3600/sqrt(1E5));
      CvSupply    =           Kv_SISupply/(rhoStdSupply*0.0631/1000/sqrt(6895));
      dpValve_nominalSupply =  (sum(m_flow_nominal)/Kv_SISupply)^2;
    end if;
  equation
      // connections that are function of the number of circuits
    for i in 1:nZones loop
      connect(pipeReturnEmission[i].heatPort, fixedTemperature.port) annotation (
          Line(
          points={{138,-94},{138,44},{-142,44},{-142,45}},
          color={191,0,0},
          smooth=Smooth.None));
      connect(senTemEm_in.port_b, pumpRad[i].port_a) annotation (Line(
          points={{86,58},{86,60},{98,60}},
          color={0,127,255},
          smooth=Smooth.None));
    end for;
    // general connections for any configuration
    connect(fixedTemperature.port, heater.heatPort) annotation (Line(
        points={{-142,45},{-142,14.8},{-124,14.8}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(pipeSupply.heatPort, fixedTemperature.port) annotation (Line(
        points={{-6,60},{-6,80},{-18,80},{-18,44},{-142,44},{-142,45}},
        color={191,0,0},
        smooth=Smooth.None));
    connect(pipeReturn.port_b, heater.port_a) annotation (Line(
        points={{-18,-92},{-134,-92},{-134,22}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipeSupply.port_b, idealCtrlMixer.port_1) annotation (Line(
        points={{4,58},{34,58}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(absolutePressure.ports[1], heater.port_a) annotation (Line(
        points={{-106,-104},{-106,-92},{-134,-92},{-134,22}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(emission.port_b, pipeReturnEmission.port_a) annotation (Line(
        points={{170,60},{188,60},{188,-92},{148,-92}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pumpRad.port_b, emission.port_a) annotation (Line(
        points={{122,60},{140,60}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(TSet_max.y, ctrl_Heating.TRoo_in1) annotation (Line(
        points={{-184,64.1},{-184,70},{-176,70}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(add.y, heatingControl.u) annotation (Line(
        points={{-159.3,-75},{-146,-75},{-146,-76},{-142,-76}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(idealCtrlMixer.port_2, senTemEm_in.port_a) annotation (Line(
        points={{56,58},{66,58}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(heater.port_b, senTemHea_out.port_a) annotation (Line(
        points={{-114,22},{-112,22},{-112,58},{-62,58}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(senTemHea_out.port_b, pipeSupply.port_a) annotation (Line(
        points={{-42,58},{-16,58}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(TSensor, add.u1) annotation (Line(
        points={{-204,-60},{-190,-60},{-190,-70.8},{-175.4,-70.8}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(senTemEm_out.port_b, spl.port_1) annotation (Line(
        points={{82,-92},{76,-92}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(spl.port_2, pipeReturn.port_a) annotation (Line(
        points={{68,-92},{2,-92}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(idealCtrlMixer.port_3, spl.port_3) annotation (Line(
        points={{45,46},{44,46},{44,34},{92,34},{92,-78},{72,-78},{72,-88}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(pipeReturnEmission.port_b, vol.ports[1:nZones]) annotation (Line(
        points={{128,-92},{114,-92}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(vol.ports[end], senTemEm_out.port_a) annotation (Line(
        points={{114,-92},{98,-92}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(add.u2, TSet) annotation (Line(
        points={{-175.4,-79.2},{-194,-79.2},{-194,-104},{20,-104}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(heatingControl.y, booleanToReal.u) annotation (Line(points={{-119,-76},
            {-106,-76}},              color={255,0,255}));
    connect(booleanToReal.y, pumpRad.m_flow_in) annotation (Line(points={{-83,-76},
            {6,-76},{109.76,-76},{109.76,45.6}},
                                               color={0,0,127}));
    connect(fixedTemperature.port, pipeReturn.heatPort) annotation (Line(points={{-142,45},
            {-142,45},{-142,-94},{-8,-94}},              color={191,0,0}));
    connect(ctrl_Heating.THeaCur, conVal.u_s) annotation (Line(points={{-156,70},{
            -26,70},{-26,86},{2,86}}, color={0,0,127}));
    connect(conVal.y, idealCtrlMixer.y) annotation (Line(points={{25,86},{46,86},{
            46,72.4},{45,72.4}}, color={0,0,127}));
    connect(senTemEm_in.T, conVal.u_m)
      annotation (Line(points={{76,69},{14,69},{14,74}}, color={0,0,127}));
    connect(weaBus, ctrl_Heating.weaBus) annotation (Line(
        points={{-180,92},{-182,92},{-182,72.2},{-183.6,72.2}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(onOffController.y, booleanToReal1.u)
      annotation (Line(points={{-131,86},{-116,86}}, color={255,0,255}));
    connect(onOffController.reference, ctrl_Heating.THeaterSet)
      annotation (Line(points={{-154,92},{-157,92},{-157,67}}, color={0,0,127}));
    connect(booleanToReal1.y, conVal1.u_s)
      annotation (Line(points={{-93,86},{-86,86}}, color={0,0,127}));
    connect(heater.y, conVal1.u_m) annotation (Line(points={{-136,14},{-106,14},{-106,
            34},{-76,34},{-76,58},{-74,58},{-74,74}}, color={0,0,127}));
    connect(conVal1.y, heater.y) annotation (Line(points={{-63,86},{-76,86},{-76,34},
            {-106,34},{-106,14},{-136,14}}, color={0,0,127}));
    connect(onOffController.u, heater.T) annotation (Line(points={{-154,80},{
            -134,80},{-134,14},{-113,14}}, color={0,0,127}));
      annotation(Dialog(group = "Nominal condition"),
                Diagram(coordinateSystem(preserveAspectRatio=false, extent={{-200,
              -100},{200,100}}), graphics={Rectangle(
            extent={{-98,30},{88,-64}},
            lineColor={135,135,135},
            lineThickness=1), Text(
            extent={{36,30},{86,20}},
            lineColor={135,135,135},
            lineThickness=1,
            fillColor={0,0,255},
            fillPattern=FillPattern.Solid,
            textString="Thermal Energy Storage")}), Icon(coordinateSystem(
            preserveAspectRatio=false, extent={{-200,-100},{200,100}}), graphics));
  end Partial_HydraulicHeating;

  partial model Partial_IdealHeating
    "Ideal, non-hydraulic heating with limited power"
    parameter Integer nZones(min=1)
      "Number of conditioned thermal zones in the building";
    parameter Real fractionRad[nZones]=0.3*ones(nZones)
      "Fraction of radiative to total power";
    parameter Real COP=3 "virtual COP to get a PEl as output";
    parameter Modelica.SIunits.Time t=10
      "Time needed to reach temperature setpoint";
    parameter Modelica.SIunits.Power[nZones] QNom(each min=0) = ones(nZones)*5000
      "Nominal power, can be seen as the max power of the emission system per zone";
    parameter Real[nZones] VZones = 50*ones(nZones)
      "Conditioned volumes of the zones";
    final parameter Modelica.SIunits.HeatCapacity[nZones] C=1012*1.204*VZones*5
      "Heat capacity of the conditioned zones (air capacity with a correction factor of 5)";
    Modelica.SIunits.Power[nZones] QHeatZone;
  end Partial_IdealHeating;
end Interfaces;
