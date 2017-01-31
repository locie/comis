within FBM;
package AHUSystems "Collection of systems dedicated to ventilation service"
  model CTA_VAVReheat_Elec "VAV Reheat AHU with electric VAV heater"
     extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(redeclare
        replaceable package Medium =
          Buildings.Media.Water);
     extends FBM.AHUSystems.BaseClasses.VAVReheatParameter;
    replaceable package MediumA =
        Buildings.Media.Air(T_default=293.15) "Medium model for Air";
    package MediumW = Buildings.Media.Water "Medium model for water";
    Buildings.Fluid.Sources.Outside
                          amb(redeclare package Medium = MediumA, nPorts=2)
      "Ambient conditions"
      annotation (Placement(transformation(extent={{-84,20},{-62,42}})));
    Buildings.Fluid.FixedResistances.PressureDrop fil(
      m_flow_nominal=m_flow_nominal,
      redeclare package Medium = MediumA,
      dp_nominal=200 + 200 + 100,
      from_dp=false,
      linearized=false) "Filter"
      annotation (Placement(transformation(extent={{70,-40},{90,-20}})));
    Buildings.Fluid.HeatExchangers.DryEffectivenessNTU heaCoi(
      redeclare package Medium1 = MediumA,
      redeclare package Medium2 = MediumW,
      allowFlowReversal2=false,
      configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
      dp1_nominal=0,
      m1_flow_nominal=m_flow_nominal_Air,
      m2_flow_nominal=m_flow_nominal_Air*1000*(10 - (-20))/4200/10,
      Q_flow_nominal=m_flow_nominal_Air*1006*(16.7 - 8.5),
      dp2_nominal=dpHotDeck,
      T_a1_nominal=281.65,
      T_a2_nominal=328.15) "Heating coil"
      annotation (Placement(transformation(extent={{108,-46},{128,-26}})));
    Buildings.Fluid.FixedResistances.PressureDrop dpSupDuc(
      redeclare package Medium = MediumA,
      dp_nominal=20,
      m_flow_nominal=m_flow_nominal_Air) "Pressure drop for supply duct"
      annotation (Placement(transformation(extent={{330,-40},{350,-20}})));
    Buildings.Fluid.FixedResistances.PressureDrop dpRetDuc(
      redeclare package Medium = MediumA,
      dp_nominal=20,
      m_flow_nominal=m_flow_nominal_Air) "Pressure drop for return duct"
      annotation (Placement(transformation(extent={{332,120},{312,140}})));
    Buildings.Fluid.Movers.SpeedControlled_y fanSup(
      redeclare package Medium = MediumA,
      tau=60,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow={0,m_flow_nominal_Air/1.225*2}, dp={850,0})))
                                                                 "Supply air fan"
      annotation (Placement(transformation(extent={{234,-40},{254,-20}})));
    Buildings.Fluid.Movers.SpeedControlled_y fanRet(
      redeclare package Medium = MediumA,
      tau=60,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow=m_flow_nominal_Air/1.225*{0,2}, dp=1.5*110*{2,0})))
                                                                 "Return air fan"
      annotation (Placement(transformation(extent={{184,120},{164,140}})));
    Modelica.Blocks.Routing.RealPassThrough TOut(y(
        final quantity="ThermodynamicTemperature",
        final unit="K",
        displayUnit="degC",
        min=0))
      annotation (Placement(transformation(extent={{-150,184},{-130,204}})));
    Modelica.Blocks.Sources.Constant TSupSetHea(k=TSupSetHeat)
                               "Supply air temperature setpoint for heating"
      annotation (Placement(transformation(extent={{-172,120},{-152,140}})));
    Buildings.Fluid.Sensors.RelativePressure dpRetFan(
        redeclare package Medium = MediumA) "Pressure difference over return fan"
                                              annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={254,16})));
    Buildings.Examples.VAVReheat.Controls.FanVFD conFan(
      xSet_nominal(displayUnit="Pa") = 410,
      r_N_min=r_N_min,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
      "Controller for fan"
      annotation (Placement(transformation(extent={{182,24},{202,44}})));
    Buildings.Fluid.Sensors.VolumeFlowRate senSupFlo(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Sensor for supply fan flow rate"
      annotation (Placement(transformation(extent={{288,-40},{308,-20}})));
    Buildings.Controls.SetPoints.OccupancySchedule occSch(occupancy=3600*{OccOn,
          OccOff})
      "Occupancy schedule"
      annotation (Placement(transformation(extent={{346,152},{326,172}})));
    Buildings.Examples.VAVReheat.Controls.ModeSelector
                          modeSelector(
      delTRooOnOff=delTRooOnOff,
      TRooSetHeaOcc=TRooSetHeaOcc,
      TRooSetCooOcc=TRooSetCooOcc,
      TSetHeaCoiOut=TSetHeaCoiOut)
      annotation (Placement(transformation(extent={{-128,162},{-108,182}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TCoiHeaOut(redeclare package
        Medium = MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Heating coil outlet temperature"
      annotation (Placement(transformation(extent={{144,-40},{164,-20}})));
    Buildings.Examples.VAVReheat.Controls.Economizer
                        conEco(
      dT=1,
      Ti=600,
      k=0.1,
      VOut_flow_min=0.3*m_flow_nominal_Air/1.2)
             "Controller for economizer"
      annotation (Placement(transformation(extent={{-60,124},{-40,144}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TRet(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Return air temperature sensor"
      annotation (Placement(transformation(extent={{110,120},{90,140}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TMix(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Mixed air temperature sensor"
      annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
    Buildings.Examples.VAVReheat.Controls.RoomTemperatureSetpoint
                                     TSetRoo(
      THeaOn=THeaOn,
      THeaOff=THeaOff,
      TCooOn=TCooOn,
      TCooOff=TCooOff)
      annotation (Placement(transformation(extent={{348,178},{328,198}})));
    Buildings.Fluid.Actuators.Valves.TwoWayLinear valHea(
      redeclare package Medium = MediumW,
      CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
      from_dp=true,
      m_flow_nominal=m_flow_nominal_Air*1000*40/4200/10,
      dpValve_nominal=dpVal_nominal)
                            "Heating coil valve"
                                         annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={140,-54})));
    Buildings.Fluid.Actuators.Dampers.MixingBox eco(
      redeclare package Medium = MediumA,
      dpOut_nominal=10,
      dpRec_nominal=10,
      dpExh_nominal=10,
      mOut_flow_nominal=m_flow_nominal_Air,
      mRec_flow_nominal=m_flow_nominal_Air,
      mExh_flow_nominal=m_flow_nominal_Air)
                        "Economizer"
      annotation (Placement(transformation(extent={{-14,56},{30,12}})));
    Buildings.Fluid.Sensors.VolumeFlowRate VOut1(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Outside air volume flow rate"
      annotation (Placement(transformation(extent={{-46,12},{-24,34}})));
    Buildings.Examples.VAVReheat.Controls.DuctStaticPressureSetpoint
                                        pSetDuc(
      nin=nReheat,
      pMin=pMin,
      pMax=pMax,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
               "Duct static pressure setpoint"
      annotation (Placement(transformation(extent={{128,24},{148,44}})));
    FBM.Components.VAVBranch_elec[nReheat]            ReHeater(
      redeclare package MediumA = MediumA,
      redeclare package MediumW = MediumW,
      VRoo={VRoom[i] for i in 1:nReheat},
      m_flow_nominal={m_flow_room[i] for i in 1:nReheat}) "Zone of building"
      annotation (Placement(transformation(extent={{316,12},{384,80}})));
    Buildings.Examples.VAVReheat.Controls.FanVFD
                    conFanRet(
                          xSet_nominal(displayUnit="m3/s") = m_flow_nominal_Air/1.2, r_N_min=
          r_N_min,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
                          "Controller for fan"
      annotation (Placement(transformation(extent={{124,150},{144,170}})));
    Buildings.Fluid.Sensors.VolumeFlowRate senRetFlo(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Sensor for return fan flow rate"
      annotation (Placement(transformation(extent={{308,120},{288,140}})));
    Buildings.Examples.VAVReheat.Controls.CoolingCoilTemperatureSetpoint
                                            TSetCoo(TCooOn=TCooOn, TCooOff=
          TCooOff)                                  "Setpoint for cooling coil"
      annotation (Placement(transformation(extent={{-132,138},{-112,158}})));
    Buildings.Examples.VAVReheat.Controls.ControlBus
                        controlBus
      annotation (Placement(transformation(extent={{290,184},{310,204}}),
          iconTransformation(extent={{290,184},{310,204}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-188,166},{-148,206}}), iconTransformation(
            extent={{-154,178},{-134,198}})));
    Modelica.Blocks.Interfaces.RealInput[nReheat] TRoo(unit="K", displayUnit="degC")
      "Measured room temperature"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={260,222})));
    Modelica.Fluid.Interfaces.FluidPort_a[nReheat] port_a2(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{390,-90},{410,-70}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nReheat] port_b1(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{390,120},{410,140}})));
    Buildings.Utilities.Math.Min min(nin=nReheat)
                                            "Computes lowest room temperature"
      annotation (Placement(transformation(extent={{266,170},{286,190}})));
    Buildings.Utilities.Math.Average ave(nin=nReheat)
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{266,146},{286,166}})));
    Buildings.Fluid.Movers.SpeedControlled_y pumHotDeck(
      redeclare package Medium = MediumW,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow=(m_flow_nominal_Air*1000*40/4200/10)/1000*{0,2}, dp=
              dp_nominal*{2,0}))) "Pump that serves the radiators" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={74,-64})));
    Buildings.Fluid.Sensors.RelativePressure dpSen(redeclare package Medium =
          MediumW)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=180,
          origin={74,-80})));
    Modelica.Blocks.Math.Gain gain(k=1/dp_nominal)
      "Gain used to normalize pressure measurement signal"
      annotation (Placement(transformation(extent={{94,-106},{114,-86}})));
    Buildings.Controls.Continuous.PIDHysteresisTimer
                                           conPum(
      yMax=1,
      Td=60,
      yMin=0.05,
      eOn=0.5,
      Ti=15,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=1)   "Controller for pump"
      annotation (Placement(transformation(extent={{146,-92},{126,-72}})));
    Modelica.Blocks.Sources.Constant pumHotDeckOn(k=1) "Pump on signal"
      annotation (Placement(transformation(extent={{212,-68},{192,-48}})));
    Modelica.Blocks.Logical.Hysteresis hysPum(           uLow=0.01, uHigh=0.5)
      "Hysteresis for pump"
      annotation (Placement(transformation(extent={{236,-80},{216,-60}})));
    Modelica.Blocks.Logical.Switch swiPum "Pump switch"
      annotation (Placement(transformation(extent={{178,-92},{158,-72}})));
    Modelica.Blocks.Sources.Constant pumHotDeckOff(k=0) "Pump off signal"
      annotation (Placement(transformation(extent={{212,-100},{192,-80}})));
    Modelica.Blocks.Continuous.LimPID PID(
      controllerType=Modelica.Blocks.Types.SimpleController.PI,
      k=0.01,
      Ti=600,
      yMax=1,
      yMin=0) annotation (Placement(transformation(extent={{-20,-14},{0,6}})));
  equation
    connect(fil.port_b,heaCoi. port_a1) annotation (Line(
        points={{90,-30},{108,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(fanRet.port_a,dpRetFan. port_b) annotation (Line(
        points={{184,130},{254,130},{254,26}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dot));
    connect(fanSup.port_b,dpRetFan. port_a) annotation (Line(
        points={{254,-30},{254,6}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dot));
    connect(senSupFlo.port_b,dpSupDuc. port_a) annotation (Line(
        points={{308,-30},{330,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(controlBus,modeSelector. cb) annotation (Line(
        points={{300,194},{-124,194},{-124,178.818},{-124.818,178.818}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(occSch.tNexOcc,controlBus. dTNexOcc) annotation (Line(
        points={{325,168},{320,168},{320,194},{300,194}},
        color={0,0,127},
        pattern=LinePattern.Dash),
                             Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TOut.y,controlBus. TOut) annotation (Line(
        points={{-129,194},{300,194}},
        color={255,213,170},
        smooth=Smooth.None,
        thickness=0.5),      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(occSch.occupied,controlBus. occupied) annotation (Line(
        points={{325,156},{300,156},{300,194}},
        color={255,0,255},
        pattern=LinePattern.Dash),
                             Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(controlBus, conFan.controlBus) annotation (Line(
        points={{300,194},{156,194},{156,42},{185,42}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TRet.T,conEco. TRet) annotation (Line(
        points={{100,141},{100,182},{-72,182},{-72,141.333},{-61.3333,141.333}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(TMix.T,conEco. TMix) annotation (Line(
        points={{50,-19},{50,178},{-70,178},{-70,137.333},{-61.3333,137.333}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(conEco.TSupHeaSet,TSupSetHea. y) annotation (Line(
        points={{-61.3333,129.333},{-148,129.333},{-148,130},{-151,130}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(controlBus,conEco. controlBus) annotation (Line(
        points={{300,194},{-86,194},{-86,134.667},{-56,134.667}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TSetRoo.controlBus,controlBus)  annotation (Line(
        points={{336,194},{300,194}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(fil.port_a,TMix. port_b) annotation (Line(
        points={{70,-30},{60,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(conFan.y, fanSup.y) annotation (Line(
        points={{203,34},{243.8,34},{243.8,-18}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(valHea.port_b,heaCoi. port_a2) annotation (Line(
        points={{140,-44},{140,-42},{128,-42}},
        color={0,127,0},
        smooth=Smooth.None,
        thickness=0.5));
    connect(eco.port_Exh,amb. ports[1]) annotation (Line(
        points={{-14,47.2},{-48,47.2},{-48,33.2},{-62,33.2}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(amb.ports[2],VOut1. port_a) annotation (Line(
        points={{-62,28.8},{-62,28},{-50,28},{-50,24},{-46,24},{-46,23}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(VOut1.port_b,eco. port_Out) annotation (Line(
        points={{-24,23},{-24,20.8},{-14,20.8}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(dpRetFan.p_rel, conFan.u_m) annotation (Line(
        points={{245,16},{192,16},{192,22}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(eco.port_Sup,TMix. port_a) annotation (Line(
        points={{30,20.8},{36,20.8},{36,-30},{40,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(pSetDuc.y, conFan.u) annotation (Line(
        points={{149,34},{180,34}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(heaCoi.port_b1,TCoiHeaOut. port_a) annotation (Line(
        points={{128,-30},{144,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(controlBus,conFanRet. controlBus) annotation (Line(
        points={{300,194},{100,194},{100,168},{127,168}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(senSupFlo.V_flow,conFanRet. u) annotation (Line(
        points={{298,-19},{298,86},{110,86},{110,160},{122,160}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(senRetFlo.port_b,fanRet. port_a) annotation (Line(
        points={{288,130},{184,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(senRetFlo.V_flow,conFanRet. u_m) annotation (Line(
        points={{298,141},{298,144},{134,144},{134,148}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(conFanRet.y,fanRet. y) annotation (Line(
        points={{145,160},{174.2,160},{174.2,142}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(dpRetDuc.port_b,senRetFlo. port_a) annotation (Line(
        points={{312,130},{308,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TRet.port_b,eco. port_Ret) annotation (Line(
        points={{90,130},{34,130},{34,47.2},{30,47.2}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TRet.port_a,fanRet. port_b) annotation (Line(
        points={{110,130},{164,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TSetCoo.TSet,conEco. TSupCooSet) annotation (Line(
        points={{-111,148},{-104,148},{-104,126},{-82,126},{-82,125.333},{
            -61.3333,125.333}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(TSupSetHea.y,TSetCoo. TSetHea) annotation (Line(
        points={{-151,130},{-142,130},{-142,148},{-134,148}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(modeSelector.cb,TSetCoo. controlBus) annotation (Line(
        points={{-124.818,178.818},{-124,178.818},{-124,140},{-123.8,140}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(conEco.VOut_flow,VOut1. V_flow) annotation (Line(
        points={{-61.3333,133.333},{-64,133.333},{-64,118},{-35,118},{-35,35.1}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(amb.weaBus, weaBus) annotation (Line(
        points={{-84,31.22},{-168,31.22},{-168,186}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(weaBus.TDryBul,pSetDuc. TOut) annotation (Line(
        points={{-168,186},{116,186},{116,42},{126,42}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(conEco.yOA,eco. y) annotation (Line(
        points={{-39.3333,136},{-36,136},{-36,-10},{8,-10},{8,7.6}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(weaBus.TDryBul, TOut.u) annotation (Line(
        points={{-168,186},{-160,186},{-160,194},{-152,194}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(heaCoi.port_b2, port_b) annotation (Line(
        points={{108,-42},{100,-42},{100,0}},
        color={0,127,255},
        thickness=0.5));
       connect(ReHeater.yDam, pSetDuc.u) annotation (Line(points={{386.267,
            34.6667},{390,34.6667},{390,34},{396,34},{396,110},{110,110},{110,
            34},{126,34}},
          color={0,0,127},
        pattern=LinePattern.Dash));
            for i in 1:nReheat loop
             connect(controlBus, ReHeater[i].controlBus) annotation (Line(
        points={{300,194},{274,194},{274,28.32},{316,28.32}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
        connect(dpSupDuc.port_b, ReHeater[i].port_a) annotation (Line(points={{350,-30},
              {350,28.7733}},         color={0,127,255}));
                connect(dpRetDuc.port_a, port_b1[i]) annotation (Line(points={{332,130},{366,130},
            {400,130}}, color={0,127,255}));
            end for;
    connect(TRoo, ReHeater.TRoo) annotation (Line(points={{260,222},{292,222},{
            292,57.3333},{311.467,57.3333}},
                                         color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ReHeater.port_b, port_a2) annotation (Line(points={{350,80},{350,80},{
            350,88},{392,88},{392,-80},{400,-80}}, color={0,127,255}));
    connect(TRoo, min.u) annotation (Line(
        points={{260,222},{260,222},{260,180},{264,180}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ave.u, TRoo) annotation (Line(
        points={{264,156},{260,156},{260,222}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ave.y, controlBus.TRooAve) annotation (Line(
        points={{287,156},{300,156},{300,194}},
        color={0,0,127},
        pattern=LinePattern.Dash), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(min.y, controlBus.TRooMin) annotation (Line(
        points={{287,180},{300,180},{300,194}},
        color={0,0,127},
        pattern=LinePattern.Dash), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TCoiHeaOut.port_b, fanSup.port_a) annotation (Line(
        points={{164,-30},{200,-30},{234,-30}},
        color={0,127,255},
        thickness=0.5));
    connect(fanSup.port_b, senSupFlo.port_a) annotation (Line(points={{254,-30},
            {272,-30},{288,-30}}, color={0,127,255}));
    connect(pumHotDeck.port_b, dpSen.port_a) annotation (Line(
        points={{84,-64},{86,-64},{86,-80},{84,-80}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(dpSen.port_b, pumHotDeck.port_a) annotation (Line(
        points={{64,-80},{62,-80},{62,-64},{64,-64}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(gain.u,dpSen. p_rel) annotation (Line(
        points={{92,-96},{74,-96},{74,-89}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(valHea.port_a, pumHotDeck.port_b) annotation (Line(points={{140,-64},
            {112,-64},{84,-64}}, color={0,127,255}));
    connect(pumHotDeck.port_a, port_a) annotation (Line(points={{64,-64},{-101,
            -64},{-101,0},{-100,0}}, color={0,127,255}));
    connect(gain.y, conPum.u_m) annotation (Line(
        points={{115,-96},{115,-96},{114,-96},{134,-96},{136,-96},{136,-94}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(hysPum.y,swiPum. u2) annotation (Line(
        points={{215,-70},{192,-70},{192,-82},{180,-82}},
        color={255,0,255},
        pattern=LinePattern.Dash));
    connect(pumHotDeckOn.y, swiPum.u1) annotation (Line(
        points={{191,-58},{186,-58},{186,-58},{184,-58},{184,-74},{180,-74}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(pumHotDeckOff.y, swiPum.u3) annotation (Line(
        points={{191,-90},{184,-90},{180,-90}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(conPum.u_s, swiPum.y) annotation (Line(
        points={{148,-82},{157,-82}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(valHea.y_actual, hysPum.u) annotation (Line(
        points={{133,-49},{252.5,-49},{252.5,-70},{238,-70}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(conPum.y, pumHotDeck.y) annotation (Line(
        points={{125,-82},{112,-82},{112,-52},{73.8,-52}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(PID.y, valHea.y) annotation (Line(points={{1,-4},{16,-4},{16,-54},{
            128,-54}}, color={0,0,127}));
    connect(TSupSetHea.y, PID.u_s) annotation (Line(points={{-151,130},{-36,130},
            {-36,-4},{-22,-4}}, color={0,0,127}));
    connect(PID.u_m, TCoiHeaOut.T) annotation (Line(
        points={{-10,-16},{-10,-16},{-10,-20},{-10,-19},{72,-19},{154,-19}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,
              -120},{400,220}}),
                           graphics={
          Rectangle(
            extent={{-180,220},{400,-120}},
            lineColor={0,0,127},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-170,218},{-116,158}},
            lineColor={255,220,220},
            lineThickness=1,
            fillPattern=FillPattern.Sphere,
            fillColor={255,255,0}),
          Line(
            points={{-144,158},{-144,148},{-144,92},{-80,92}},
            color={0,0,127},
            thickness=0.5),
          Rectangle(
            extent={{-80,102},{-50,60}},
            lineColor={0,0,127},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-78,92},{-52,68}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-52,68},{2,68}},
            color={255,170,85},
            thickness=0.5),
          Rectangle(
            extent={{2,72},{24,48}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{2,50},{22,70},{24,72}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{6,68},{10,68}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{8,66},{8,70}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{24,68},{84,68}},
            color={255,85,85},
            thickness=0.5),
          Ellipse(
            extent={{84,80},{108,58}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{88,78},{88,62},{108,70},{88,78}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{108,68},{272,68}},
            color={255,85,85},
            thickness=0.5),
          Rectangle(
            extent={{272,88},{294,64}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{272,64},{292,84},{294,86}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{276,82},{276,86}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{274,84},{278,84}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{294,70},{324,70},{324,22}},
            color={255,0,0},
            thickness=0.5),
          Rectangle(
            extent={{-11,12},{11,-12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid,
            origin={333,12},
            rotation=90),
          Line(
            points={{324,18},{330,12},{326,8},{332,4}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{334,20},{340,14},{336,10},{342,6}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{324,0},{324,-102},{390,-102}},
            color={255,0,0},
            thickness=0.5),
          Line(
            points={{390,130},{112,130}},
            color={255,0,0},
            thickness=0.5),
          Ellipse(
            extent={{88,140},{112,118}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{108,138},{108,122},{88,130},{108,138}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{88,130},{-24,130},{-50,96}},
            color={255,0,0},
            thickness=0.5),
          Line(
            points={{-50,96},{-80,68}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-80,70},{-156,70},{-156,162}},
            color={255,170,85},
            thickness=0.5),
          Rectangle(
            extent={{260,104},{364,-14}},
            lineColor={0,0,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{260,78},{366,150}},
            lineColor={0,0,0},
            pattern=LinePattern.Dash,
            lineThickness=0.5,
            textString="VAVBranch * nReheater"),
          Line(
            points={{-90,0},{8,0},{8,48},{6,48}},
            color={255,0,0},
            thickness=1),
          Line(
            points={{8,36},{268,36},{276,36},{276,64}},
            color={255,0,0},
            thickness=1),
          Line(
            points={{92,0},{18,0},{18,48}},
            color={0,0,255},
            thickness=1),
          Line(
            points={{18,32},{286,32},{286,64}},
            color={0,0,255},
            thickness=1),
          Rectangle(
            extent={{208,196},{246,156}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{260,202},{260,180},{246,180}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{208,176},{182,176},{182,2},{322,2}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{182,80},{274,80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Rectangle(
            extent={{-12,-28},{26,-68}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-36,12},{-36,-12},{-20,0},{-36,12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-4,12},{-4,-12},{-20,0},{-4,12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Line(
            points={{30,68},{30,-40},{26,-40}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-12,-50},{-24,-50},{-24,-4}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Rectangle(
            extent={{128,114},{154,90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{142,130},{142,114},{142,114}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{142,90},{142,70}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{128,102},{100,102},{100,116}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,80},{100,102}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Text(
            extent={{266,214},{324,176}},
            lineColor={0,0,0},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid,
            textString="TRoom"),
          Rectangle(
            extent={{38,72},{60,48}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{38,48},{58,70},{60,72}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{40,68},{44,68}},
            color={0,0,0},
            thickness=0.5),
          Ellipse(
            extent={{-80,12},{-56,-10}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-76,8},{-76,-8},{-56,0},{-76,8}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid)}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-180,-120},{400,220}})),Documentation(info="<html>
          <p>
          CTA based on the model <a href=\"modelica://Buildings.Examples.VAVReheat.ClosedLoop\">VAVReheat</a> of the Buldings library
          </p>
          <p><h4>Model use</h4></p>
          <p><ol>
          <p>Firstly, give to each zone, wich is deserved by the CTA, a number and keep it in mind for the connection to the multi-input/output </p>
<li>Set the parameter <i>nReheat</i> egal to the number of zone deserved by an individual re-heater.</li>
<li>Connect <i>TRoo</i>.  </li>
<li>Connect <i>port_a</i> to the  hot water source and <i>port_b</i> to the hot water sink. </li>
<li> Connect <i>port_Supply</i> and <i>port_Return</i> to the air volume of each zone
<li> Set other parameters or let there to the default value <b> Except for VRoom and m_flow_room</b> wich need to be fullfill</li>

</ol></p>
</html>"));
  end CTA_VAVReheat_Elec;

  model CTA_VAVReheat_NoEco
     extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(redeclare
        replaceable package Medium =
          Buildings.Media.Water);
     extends FBM.AHUSystems.BaseClasses.VAVReheatParameter;
    replaceable package MediumA =
        Buildings.Media.Air(T_default=293.15) "Medium model for Air";
    package MediumW = Buildings.Media.Water "Medium model for water";
    Buildings.Fluid.Sources.Outside
                          amb(redeclare package Medium = MediumA, nPorts=2)
      "Ambient conditions"
      annotation (Placement(transformation(extent={{-100,28},{-78,50}})));
    Buildings.Fluid.FixedResistances.PressureDrop fil(
      m_flow_nominal=m_flow_nominal,
      redeclare package Medium = MediumA,
      dp_nominal=200 + 200 + 100,
      from_dp=false,
      linearized=false) "Filter"
      annotation (Placement(transformation(extent={{70,-40},{90,-20}})));
    Buildings.Fluid.HeatExchangers.DryEffectivenessNTU heaCoi(
      redeclare package Medium1 = MediumA,
      redeclare package Medium2 = MediumW,
      allowFlowReversal2=false,
      configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
      dp1_nominal=0,
      dp2_nominal=0,
      m1_flow_nominal=m_flow_nominal_Air,
      m2_flow_nominal=m_flow_nominal_Air*1000*(10 - (-20))/4200/10,
      Q_flow_nominal=m_flow_nominal_Air*1006*(16.7 - 8.5),
      T_a1_nominal=281.65,
      T_a2_nominal=323.15) "Heating coil"
      annotation (Placement(transformation(extent={{108,-46},{128,-26}})));
    Buildings.Fluid.FixedResistances.PressureDrop dpSupDuc(
      redeclare package Medium = MediumA,
      dp_nominal=20,
      m_flow_nominal=m_flow_nominal_Air) "Pressure drop for supply duct"
      annotation (Placement(transformation(extent={{330,-40},{350,-20}})));
    Buildings.Fluid.FixedResistances.PressureDrop dpRetDuc(
      redeclare package Medium = MediumA,
      dp_nominal=20,
      m_flow_nominal=m_flow_nominal_Air) "Pressure drop for return duct"
      annotation (Placement(transformation(extent={{332,120},{312,140}})));
    Buildings.Fluid.Movers.SpeedControlled_y fanSup(
      redeclare package Medium = MediumA,
      tau=60,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow={0,m_flow_nominal_Air/1.2*2}, dp={850,0})))
                                                                 "Supply air fan"
      annotation (Placement(transformation(extent={{234,-40},{254,-20}})));
    Buildings.Fluid.Movers.SpeedControlled_y fanRet(
      redeclare package Medium = MediumA,
      tau=60,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow=m_flow_nominal_Air/1.2*{0,2}, dp=1.5*110*{2,0})))
                                                                 "Return air fan"
      annotation (Placement(transformation(extent={{184,120},{164,140}})));
    Modelica.Blocks.Routing.RealPassThrough TOut(y(
        final quantity="ThermodynamicTemperature",
        final unit="K",
        displayUnit="degC",
        min=0))
      annotation (Placement(transformation(extent={{-152,148},{-132,168}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TSup(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      annotation (Placement(transformation(extent={{262,-40},{282,-20}})));
    Modelica.Blocks.Sources.Constant TSupSetHea(k=TSupSetHeat)
                               "Supply air temperature setpoint for heating"
      annotation (Placement(transformation(extent={{-90,-108},{-70,-88}})));
    Buildings.Controls.Continuous.LimPID heaCoiCon(
      yMax=1,
      yMin=0,
      Td=60,
      initType=Modelica.Blocks.Types.InitPID.InitialState,
      Ti=600,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=0.05) "Controller for heating coil"
      annotation (Placement(transformation(extent={{10,-108},{30,-88}})));
    Buildings.Fluid.Sensors.RelativePressure dpRetFan(
        redeclare package Medium = MediumA) "Pressure difference over return fan"
                                              annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={254,60})));
    Buildings.Examples.VAVReheat.Controls.FanVFD
                    conFanSup(xSet_nominal(displayUnit="Pa") = 410, r_N_min=
          r_N_min,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
      "Controller for fan"
      annotation (Placement(transformation(extent={{162,24},{182,44}})));
    Buildings.Fluid.Sensors.VolumeFlowRate senSupFlo(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Sensor for supply fan flow rate"
      annotation (Placement(transformation(extent={{288,-40},{308,-20}})));
    Buildings.Controls.SetPoints.OccupancySchedule occSch(occupancy=3600*{OccOn,
          OccOff})
      "Occupancy schedule"
      annotation (Placement(transformation(extent={{-194,-36},{-174,-16}})));
    Buildings.Examples.VAVReheat.Controls.ModeSelector
                          modeSelector(
      delTRooOnOff=delTRooOnOff,
      TRooSetHeaOcc=TRooSetHeaOcc,
      TRooSetCooOcc=TRooSetCooOcc,
      TSetHeaCoiOut=TSetHeaCoiOut)
      annotation (Placement(transformation(extent={{-192,-92},{-172,-72}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TCoiHeaOut(redeclare package
        Medium = MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Heating coil outlet temperature"
      annotation (Placement(transformation(extent={{144,-40},{164,-20}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TRet(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Return air temperature sensor"
      annotation (Placement(transformation(extent={{110,120},{90,140}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TMix(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Mixed air temperature sensor"
      annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
    Buildings.Examples.VAVReheat.Controls.RoomTemperatureSetpoint
                                     TSetRoo(
      THeaOn=THeaOn,
      THeaOff=THeaOff,
      TCooOn=TCooOn,
      TCooOff=TCooOff)
      annotation (Placement(transformation(extent={{-194,-64},{-174,-44}})));
    Buildings.Fluid.Actuators.Valves.TwoWayLinear valHea(
      redeclare package Medium = MediumW,
      CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
      dpValve_nominal=6000,
      from_dp=true,
      dpFixed_nominal=6000,
      m_flow_nominal=m_flow_nominal_Air*1000*40/4200/10)
                            "Heating coil valve"
                                         annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={140,-70})));
    Buildings.Examples.VAVReheat.Controls.DuctStaticPressureSetpoint
                                        pSetDuc(
      nin=nReheat,
      pMin=pMin,
      pMax=pMax,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
               "Duct static pressure setpoint"
      annotation (Placement(transformation(extent={{102,24},{122,44}})));
    Buildings.Examples.VAVReheat.ThermalZones.VAVBranch[nReheat]
                                                      ReHeater(
      redeclare package MediumA = MediumA,
      redeclare package MediumW = MediumW,
      VRoo={VRoom[i] for i in 1:nReheat},
      m_flow_nominal={m_flow_room[i] for i in 1:nReheat})
                                          "Zone for core of buildings (azimuth will be neglected)"
      annotation (Placement(transformation(extent={{316,12},{384,80}})));
    Buildings.Examples.VAVReheat.Controls.FanVFD
                    conFanRet(
                          xSet_nominal(displayUnit="m3/s") = m_flow_nominal_Air/1.2, r_N_min=
          r_N_min,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
                          "Controller for fan"
      annotation (Placement(transformation(extent={{124,150},{144,170}})));
    Buildings.Fluid.Sensors.VolumeFlowRate senRetFlo(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Sensor for return fan flow rate"
      annotation (Placement(transformation(extent={{308,120},{288,140}})));
    Buildings.Examples.VAVReheat.Controls.ControlBus
                        controlBus
      annotation (Placement(transformation(extent={{-122,148},{-102,168}}),
          iconTransformation(extent={{-122,148},{-102,168}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-208,166},{-168,206}}), iconTransformation(
            extent={{218,166},{238,186}})));
    Modelica.Blocks.Interfaces.RealInput[nReheat] TRoo(unit="K", displayUnit="degC")
      "Measured room temperature"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={260,222})));
    Modelica.Fluid.Interfaces.FluidPort_a[nReheat] port_a2(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{390,-90},{410,-70}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nReheat] port_b1(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{390,120},{410,140}})));
    Buildings.Utilities.Math.Min min(nin=nReheat)
                                            "Computes lowest room temperature"
      annotation (Placement(transformation(extent={{222,188},{202,208}})));
    Buildings.Utilities.Math.Average ave(nin=nReheat)
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{222,164},{202,184}})));
  equation
    connect(fil.port_b,heaCoi. port_a1) annotation (Line(
        points={{90,-30},{108,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TSupSetHea.y,heaCoiCon. u_s) annotation (Line(
        points={{-69,-98},{8,-98}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fanRet.port_a,dpRetFan. port_b) annotation (Line(
        points={{184,130},{254,130},{254,70}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dot));
    connect(fanSup.port_b,dpRetFan. port_a) annotation (Line(
        points={{254,-30},{254,50}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dot));
    connect(senSupFlo.port_b,dpSupDuc. port_a) annotation (Line(
        points={{308,-30},{330,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(controlBus,modeSelector. cb) annotation (Line(
        points={{-112,158},{-70,158},{-70,-75.1818},{-188.818,-75.1818}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(occSch.tNexOcc,controlBus. dTNexOcc) annotation (Line(
        points={{-173,-20},{-112,-20},{-112,158}},
        color={0,0,127},
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TOut.y,controlBus. TOut) annotation (Line(
        points={{-131,158},{-112,158}},
        color={255,213,170},
        smooth=Smooth.None,
        thickness=0.5),      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(occSch.occupied,controlBus. occupied) annotation (Line(
        points={{-173,-32},{-112,-32},{-112,158}},
        color={255,0,255},
        smooth=Smooth.None), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(controlBus,conFanSup. controlBus) annotation (Line(
        points={{-112,158},{156,158},{156,42},{165,42}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TSetRoo.controlBus,controlBus)  annotation (Line(
        points={{-182,-48},{-112,-48},{-112,158}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(TSup.port_a,fanSup. port_b) annotation (Line(
        points={{262,-30},{254,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TSup.port_b,senSupFlo. port_a) annotation (Line(
        points={{282,-30},{288,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(fil.port_a,TMix. port_b) annotation (Line(
        points={{70,-30},{60,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(conFanSup.y,fanSup. y) annotation (Line(
        points={{183,34},{243.8,34},{243.8,-18}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(TCoiHeaOut.T,heaCoiCon. u_m) annotation (Line(
        points={{154,-19},{154,-10},{168,-10},{168,-110},{20,-110}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(heaCoiCon.y,valHea. y) annotation (Line(
        points={{31,-98},{118,-98},{118,-70},{128,-70}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(valHea.port_b,heaCoi. port_a2) annotation (Line(
        points={{140,-60},{140,-42},{128,-42}},
        color={0,127,0},
        smooth=Smooth.None,
        thickness=0.5));
    connect(dpRetFan.p_rel,conFanSup. u_m) annotation (Line(
        points={{245,60},{214,60},{214,8},{172,8},{172,22}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(pSetDuc.y,conFanSup. u) annotation (Line(
        points={{123,34},{160,34}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(heaCoi.port_b1,TCoiHeaOut. port_a) annotation (Line(
        points={{128,-30},{144,-30}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(controlBus,conFanRet. controlBus) annotation (Line(
        points={{-112,158},{100,158},{100,168},{127,168}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(senSupFlo.V_flow,conFanRet. u) annotation (Line(
        points={{298,-19},{298,86},{110,86},{110,160},{122,160}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(senRetFlo.port_b,fanRet. port_a) annotation (Line(
        points={{288,130},{184,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(senRetFlo.V_flow,conFanRet. u_m) annotation (Line(
        points={{298,141},{298,144},{134,144},{134,148}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(conFanRet.y,fanRet. y) annotation (Line(
        points={{145,160},{174.2,160},{174.2,142}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(dpRetDuc.port_b,senRetFlo. port_a) annotation (Line(
        points={{312,130},{308,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TRet.port_a,fanRet. port_b) annotation (Line(
        points={{110,130},{164,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(amb.weaBus, weaBus) annotation (Line(
        points={{-100,39.22},{-188,39.22},{-188,186}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(weaBus.TDryBul,pSetDuc. TOut) annotation (Line(
        points={{-188,186},{92,186},{92,42},{100,42}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus.TDryBul, TOut.u) annotation (Line(
        points={{-188,186},{-168,186},{-168,158},{-154,158}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(valHea.port_a, port_a) annotation (Line(
        points={{140,-80},{-101,-80},{-101,0},{-100,0}},
        color={0,127,255},
        thickness=0.5));
    connect(heaCoi.port_b2, port_b) annotation (Line(
        points={{108,-42},{100,-42},{100,0}},
        color={0,127,255},
        thickness=0.5));
       connect(ReHeater.yDam, pSetDuc.u) annotation (Line(points={{386.267,
            34.6667},{392,34.6667},{392,34},{398,34},{398,110},{74,110},{74,34},
            {100,34}},
          color={0,0,127},
        pattern=LinePattern.Dash));
            for i in 1:nReheat loop
             connect(controlBus, ReHeater[i].controlBus) annotation (Line(
        points={{-112,158},{274,158},{274,28.32},{316,28.32}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
        connect(dpSupDuc.port_b, ReHeater[i].port_a) annotation (Line(points={{350,-30},
              {350,28.7733}},         color={0,127,255}));
                connect(dpRetDuc.port_a, port_b1[i]) annotation (Line(points={{332,130},{366,130},
            {400,130}}, color={0,127,255}));
            end for;
    connect(TRoo, ReHeater.TRoo) annotation (Line(points={{260,222},{292,222},{
            292,57.3333},{311.467,57.3333}},
                                         color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ReHeater.port_b, port_a2) annotation (Line(points={{350,80},{350,80},{
            350,88},{392,88},{392,-80},{400,-80}}, color={0,127,255}));
    connect(TRoo, min.u) annotation (Line(
        points={{260,222},{260,222},{260,198},{224,198}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ave.u, TRoo) annotation (Line(
        points={{224,174},{260,174},{260,222}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ave.y, controlBus.TRooAve) annotation (Line(
        points={{201,174},{-112,174},{-112,158}},
        color={0,0,127},
        pattern=LinePattern.Dash), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(min.y, controlBus.TRooMin) annotation (Line(
        points={{201,198},{-111.5,198},{-111.5,158},{-112,158}},
        color={0,0,127},
        pattern=LinePattern.Dash), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TCoiHeaOut.port_b, fanSup.port_a) annotation (Line(
        points={{164,-30},{200,-30},{234,-30}},
        color={0,127,255},
        thickness=0.5));
    connect(amb.ports[1], TMix.port_a) annotation (Line(points={{-78,41.2},{-19,
            41.2},{-19,-30},{40,-30}}, color={0,127,255}));
    connect(amb.ports[2], TRet.port_b) annotation (Line(points={{-78,36.8},{-19,
            36.8},{-19,130},{90,130}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-200,
              -120},{400,220}}),
                           graphics={
          Rectangle(
            extent={{-200,220},{400,-120}},
            lineColor={0,0,127},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-194,216},{-140,156}},
            lineColor={255,220,220},
            lineThickness=1,
            fillPattern=FillPattern.Sphere,
            fillColor={255,255,0}),
          Line(
            points={{-168,156},{-168,152},{-168,92},{-80,92}},
            color={0,0,127},
            thickness=0.5),
          Rectangle(
            extent={{-80,102},{-50,60}},
            lineColor={0,0,127},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-78,92},{-52,68}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-52,68},{2,68}},
            color={255,170,85},
            thickness=0.5),
          Rectangle(
            extent={{2,72},{24,48}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{2,50},{22,70},{24,72}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{6,68},{10,68}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{8,66},{8,70}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{24,68},{84,68}},
            color={255,85,85},
            thickness=0.5),
          Ellipse(
            extent={{84,80},{108,58}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{88,78},{88,62},{108,70},{88,78}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{108,68},{272,68}},
            color={255,85,85},
            thickness=0.5),
          Rectangle(
            extent={{272,88},{294,64}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{272,64},{292,84},{294,86}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{276,82},{276,86}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{274,84},{278,84}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{294,70},{324,70},{324,22}},
            color={255,0,0},
            thickness=0.5),
          Rectangle(
            extent={{-11,12},{11,-12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid,
            origin={333,12},
            rotation=90),
          Line(
            points={{324,18},{330,12},{326,8},{332,4}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{334,20},{340,14},{336,10},{342,6}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{324,0},{324,-102},{390,-102}},
            color={255,0,0},
            thickness=0.5),
          Line(
            points={{390,130},{112,130}},
            color={255,0,0},
            thickness=0.5),
          Ellipse(
            extent={{88,140},{112,118}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{108,138},{108,122},{88,130},{108,138}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{88,130},{-24,130},{-50,96}},
            color={255,0,0},
            thickness=0.5),
          Line(
            points={{-50,96},{-80,68}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-80,70},{-182,70},{-182,160}},
            color={255,170,85},
            thickness=0.5),
          Rectangle(
            extent={{260,104},{364,-14}},
            lineColor={0,0,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{260,78},{366,150}},
            lineColor={0,0,0},
            pattern=LinePattern.Dash,
            lineThickness=0.5,
            textString="VAVBranch * nReheater"),
          Line(
            points={{-90,0},{8,0},{8,48},{6,48}},
            color={255,0,0},
            thickness=1),
          Line(
            points={{8,36},{268,36},{276,36},{276,64}},
            color={255,0,0},
            thickness=1),
          Line(
            points={{92,0},{18,0},{18,48}},
            color={0,0,255},
            thickness=1),
          Line(
            points={{18,32},{286,32},{286,64}},
            color={0,0,255},
            thickness=1),
          Rectangle(
            extent={{208,196},{246,156}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{260,202},{260,180},{246,180}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{208,176},{182,176},{182,2},{322,2}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{182,80},{274,80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Rectangle(
            extent={{-12,-28},{26,-68}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-36,12},{-36,-12},{-20,0},{-36,12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-4,12},{-4,-12},{-20,0},{-4,12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Line(
            points={{30,68},{30,-40},{26,-40}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-12,-50},{-24,-50},{-24,-4}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Rectangle(
            extent={{128,114},{154,90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{142,130},{142,114},{142,114}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{142,90},{142,70}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{128,102},{100,102},{100,116}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,80},{100,102}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Text(
            extent={{266,214},{324,176}},
            lineColor={0,0,0},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid,
            textString="TRoom"),
          Rectangle(
            extent={{38,72},{60,48}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{38,48},{58,70},{60,72}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{40,68},{44,68}},
            color={0,0,0},
            thickness=0.5)}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-200,-120},{400,220}})),Documentation(info="<html>
          <p>
          CTA based on the model <a href=\"modelica://Buildings.Examples.VAVReheat.ClosedLoop\">VAVReheat</a> of the Buldings library
          </p>
          <p><h4>Model use</h4></p>
          <p><ol>
          <p>Firstly, give to each zone, wich is deserved by the CTA, a number and keep it in mind for the connection to the multi-input/output </p>
<li>Set the parameter <i>nReheat</i> egal to the number of zone deserved by an individual re-heater.</li>
<li>Connect <i>TRoo</i>.  </li>
<li>Connect <i>port_a</i> to the  hot water source and <i>port_b</i> to the hot water sink. </li>
<li> Connect <i>port_Supply</i> and <i>port_Return</i> to the air volume of each zone
<li> Set other parameters or let there to the default value <b> Except for VRoom and m_flow_room</b> wich need to be fullfill</li>

</ol></p>
</html>"));
  end CTA_VAVReheat_NoEco;

  model DoubleFlux_FreeCooling_MechanicalCooling
     replaceable package MediumA = Buildings.Media.Air(T_default=293.15)
      "Medium model for air";
    extends FBM.AHUSystems.BaseClasses.DoubleFlowParameter;
    Buildings.Fluid.Sources.Outside out(
      redeclare package Medium = MediumA,
      nPorts=2) "Outside air conditions"
      annotation (Placement(transformation(extent={{-86,-40},{-66,-20}})));
    Buildings.Fluid.HeatExchangers.ConstantEffectiveness hex(
      redeclare package Medium1 = MediumA,
      redeclare package Medium2 = MediumA,
      dp1_nominal=100,
      dp2_nominal=100,
      eps=0.9,
      m1_flow_nominal=VBuilding*1.225*Tau/3600,
      m2_flow_nominal=VBuilding*1.225*Tau/3600)
               "Heat recovery"
      annotation (Placement(transformation(extent={{18,-34},{38,-14}})));
    Buildings.Fluid.Movers.FlowControlled_m_flow fanSup(
      redeclare package Medium = MediumA,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_nominal=VBuilding*1.225*Tau/3600)                   "Supply air fan"
      annotation (Placement(transformation(extent={{-34,-22},{-14,-2}})));
    Modelica.Blocks.Sources.Constant m_flow_out(k=VBuilding*1.225*Tau/3600)
      "Outside air mass flow rate"
      annotation (Placement(transformation(extent={{-52,2},{-32,22}})));
    Buildings.Fluid.Movers.FlowControlled_m_flow fanRet(
      redeclare package Medium = MediumA,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      m_flow_nominal=VBuilding*1.225*Tau/3600)                   "Return air fan"
      annotation (Placement(transformation(extent={{-14,-62},{-34,-42}})));
    FBM.Controls.ControlAHU.CoolingControl cooCon(
      TRooCoo=TRooCoo,
      TRooFre=TRooFre,
      TOutFre=TOutFre,
      dT=dT,
      Kp=Kp) "Controller for cooling"
      annotation (Placement(transformation(extent={{-16,26},{4,46}})));
    Buildings.Fluid.Actuators.Dampers.Exponential damSupByp(
      redeclare package Medium = MediumA,
      allowFlowReversal=false,
      m_flow_nominal=VBuilding*1.225*Tau/3600)
      "Supply air damper that bypasses the heat recovery"
      annotation (Placement(transformation(extent={{4,0},{24,20}})));
    Buildings.Fluid.HeatExchangers.HeaterCooler_T coo(
      redeclare package Medium = MediumA,
      dp_nominal=0,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      Q_flow_maxHeat=0,
      m_flow_nominal=VBuilding*1.225*Tau/3600)
                        "Coil for mechanical cooling"
      annotation (Placement(transformation(extent={{38,-12},{58,8}})));
    Buildings.Fluid.Actuators.Dampers.Exponential damHex(
      redeclare package Medium = MediumA,
      allowFlowReversal=false,
      m_flow_nominal=VBuilding*1.225*Tau/3600)
      "Supply air damper that closes the heat recovery"
      annotation (Placement(transformation(extent={{-6,-22},{14,-2}})));
    Buildings.Fluid.Actuators.Dampers.Exponential damRetByp(
      redeclare package Medium = MediumA,
      allowFlowReversal=false,
      m_flow_nominal=VBuilding*1.225*Tau/3600)
      "Return air damper that bypasses the heat recovery"
      annotation (Placement(transformation(extent={{24,-62},{4,-42}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-104,10},{-64,50}}),    iconTransformation(
            extent={{-92,32},{-72,52}})));
    Modelica.Blocks.Interfaces.RealInput[nReheat] TRoo(unit="K", displayUnit="degC")
      "Measured room temperature"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={-62,62})));
    Buildings.Utilities.Math.Average ave(nin=nReheat)
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{-48,32},{-28,52}})));
    Buildings.Fluid.FixedResistances.PressureDrop[nReheat] dpFacReturn(
      each from_dp=false,
      redeclare package Medium = MediumA,
      each dp_nominal=10,
      m_flow_nominal={VRoom[i]*1.225*Tau/3600 for i in 1:nReheat})
      "Pressure drop at facade" annotation (Placement(transformation(extent={{10,-10},
              {-10,10}}, origin={80,-30})));
    Modelica.Fluid.Interfaces.FluidPort_b[nReheat] port_b1(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{90,-12},{110,8}})));
    Modelica.Fluid.Interfaces.FluidPort_a[nReheat] port_a1(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{90,-40},{110,-20}})));
    Buildings.Fluid.FixedResistances.PressureDrop[nReheat] dpFacSupply(
      each from_dp=false,
      redeclare package Medium = MediumA,
      each dp_nominal=10,
      m_flow_nominal={VRoom[i]*1.225*Tau/3600 for i in 1:nReheat})
      "Pressure drop at facade" annotation (Placement(transformation(extent={{-10,
              -10},{10,10}}, origin={76,-2})));
  equation
    connect(fanSup.m_flow_in,m_flow_out. y) annotation (Line(
        points={{-24.2,0},{-24.2,6},{-24,6},{-24,12},{-31,12}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fanRet.port_a,hex. port_b2) annotation (Line(
        points={{-14,-52},{-8,-52},{-8,-30},{18,-30}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(out.ports[1],fanSup. port_a)  annotation (Line(
        points={{-66,-28},{-46,-28},{-46,-12},{-34,-12}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(fanRet.port_b,out. ports[2])  annotation (Line(
        points={{-34,-52},{-46,-52},{-46,-32},{-66,-32}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(m_flow_out.y,fanRet. m_flow_in) annotation (Line(
        points={{-31,12},{-24,12},{-24,-36},{-23.8,-36},{-23.8,-40}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(fanSup.port_b,damSupByp. port_a)
                                       annotation (Line(
        points={{-14,-12},{-10,-12},{-10,10},{4,10}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(cooCon.yF,damSupByp. y)
                              annotation (Line(
        points={{5,36},{14,36},{14,22}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(hex.port_b1,coo. port_a) annotation (Line(
        points={{38,-18},{38,-2}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(damSupByp.port_b,coo. port_a)
                                    annotation (Line(
        points={{24,10},{32,10},{32,-2},{38,-2}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(damHex.port_b,hex. port_a1) annotation (Line(
        points={{14,-12},{18,-12},{18,-18}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(damHex.port_a,fanSup. port_b) annotation (Line(
        points={{-6,-12},{-14,-12}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(damRetByp.port_b,fanRet. port_a) annotation (Line(
        points={{4,-52},{-14,-52}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(damHex.y,cooCon. yHex) annotation (Line(
        points={{4,0},{4,30},{5,30}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(damRetByp.y,cooCon. yF) annotation (Line(
        points={{14,-40},{14,36},{5,36}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(cooCon.TSupCoo,coo. TSet) annotation (Line(
        points={{5,42},{34,42},{34,4},{36,4}},
        color={0,0,127},
        smooth=Smooth.None));
    connect(weaBus, out.weaBus) annotation (Line(
        points={{-84,30},{-86,30},{-86,-29.8}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(weaBus.TDryBul, cooCon.TOut) annotation (Line(
        points={{-84,30},{-52,30},{-18,30}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TRoo, ave.u) annotation (Line(points={{-62,62},{-62,42},{-50,42}},
                  color={0,0,127}));
    connect(ave.y, cooCon.TRoo)
      annotation (Line(points={{-27,42},{-24,42},{-18,42}}, color={0,0,127}));
       connect(dpFacReturn.port_a,port_a1)
      annotation (Line(points={{90,-30},{95,-30},{100,-30}}, color={0,127,255}));
      for i in 1:nReheat loop
    connect(hex.port_a2, dpFacReturn[i].port_b)
      annotation (Line(points={{38,-30},{54,-30},{70,-30}}, color={0,127,255}));
    connect(damRetByp.port_a, dpFacReturn[i].port_b) annotation (Line(points={{24,
            -52},{54,-52},{54,-30},{70,-30}}, color={0,127,255}));
    connect(coo.port_b, dpFacSupply[i].port_a)
      annotation (Line(points={{58,-2},{62,-2},{66,-2}}, color={0,127,255}));
      end for;
    connect(dpFacSupply.port_b, port_b1)
      annotation (Line(points={{86,-2},{93,-2},{100,-2}}, color={0,127,255}));
       annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -80},{100,60}}), graphics={
          Rectangle(
            extent={{-100,60},{100,-80}},
            lineColor={0,0,127},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-98,58},{-66,26}},
            lineColor={255,220,220},
            lineThickness=1,
            fillPattern=FillPattern.Sphere,
            fillColor={255,255,0}),
          Rectangle(
            extent={{-44,8},{-14,-34}},
            lineColor={0,0,127},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-78,26},{-78,24},{-78,-8},{-44,-8}},
            color={0,0,127},
            thickness=0.5),
          Line(
            points={{-44,-26},{-86,-26},{-86,26}},
            color={255,170,85},
            thickness=0.5),
          Line(points={{-44,-14},{-14,-14}}, color={0,0,127}),
          Line(
            points={{-28,-26},{-44,-26},{-28,-26}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-14,-8},{-30,-8},{-14,-8}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-32,-8},{-44,-8},{-30,-8}},
            color={0,0,127},
            thickness=0.5),
          Line(
            points={{-16,-26},{-28,-26},{-14,-26}},
            color={255,0,0},
            thickness=0.5),
          Polygon(
            points={{2,-4},{2,4},{-2,0},{2,-4}},
            lineColor={191,0,0},
            fillColor={191,0,0},
            fillPattern=FillPattern.Solid,
            origin={-38,-10},
            rotation=270),
          Line(
            points={{-38,-26},{-38,-12}},
            color={191,0,0},
            thickness=0.5),
          Line(
            points={{2,-8},{-14,-8},{94,-8}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-2,-26},{-14,-26},{92,-26}},
            color={255,0,0},
            thickness=0.5),
          Ellipse(
            extent={{-72,4},{-48,-18}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-68,-16},{-68,2},{-48,-8},{-68,-16}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{58,-14},{82,-36}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{78,-34},{78,-18},{58,-26},{78,-34}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{2,-4},{2,4},{-2,0},{2,-4}},
            lineColor={191,0,0},
            fillColor={191,0,0},
            fillPattern=FillPattern.Solid,
            origin={-22,-10},
            rotation=270),
          Line(
            points={{-22,-26},{-22,-12}},
            color={191,0,0},
            thickness=0.5)}),                                         Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-80},{100,60}})));
  end DoubleFlux_FreeCooling_MechanicalCooling;

  package BaseClasses
    extends Modelica.Icons.BasesPackage;
    record VAVReheatParameter
        // --- Control-Command parameters
        parameter Real OccOn(min=1, max=24)= 6 "Hour when the occupancy mode is turn-on for the CTA";
        parameter Real OccOff(min=1, max=24) = 19 "Hour when the occupancy mode is turn-off for the CTA";
       parameter Modelica.SIunits.Temperature THeaOn=293.15
        "Heating setpoint during on";
      parameter Modelica.SIunits.Temperature THeaOff=285.15
        "Heating setpoint during off";
      parameter Modelica.SIunits.Temperature TCooOn=297.15
        "Cooling setpoint during on";
      parameter Modelica.SIunits.Temperature TCooOff=303.15
        "Cooling setpoint during off";
      parameter Modelica.SIunits.TemperatureDifference delTRooOnOff(min=0.1)=1
        "Deadband in room temperature between occupied on and occupied off";
      parameter Modelica.SIunits.Temperature TRooSetHeaOcc=293.15
        "Set point for room air temperature during heating mode";
      parameter Modelica.SIunits.Temperature TRooSetCooOcc=299.15
        "Set point for room air temperature during cooling mode";
      parameter Modelica.SIunits.Temperature TSetHeaCoiOut=303.15
        "Set point for air outlet temperature at central heating coil";
        parameter Modelica.SIunits.Temperature TSupSetHeat=283.15
        "Supply air temperature setpoint for heating";
        parameter Real r_N_min=0.2 "Minimum normalized fan speed";
          parameter Modelica.SIunits.AbsolutePressure pMin(displayUnit="Pa") = 50
        "Minimum duct static pressure setpoint";
      parameter Modelica.SIunits.AbsolutePressure pMax(displayUnit="Pa") = 410
        "Maximum duct static pressure setpoint";
       // -- Design parameters
        parameter Integer nReheat(min=1)
        "Number of conditioned thermal zones deserved by the system";
        parameter Modelica.SIunits.Volume[nReheat] VRoom "Zone volume deserved by each reheater";
        parameter Modelica.SIunits.MassFlowRate[nReheat] m_flow_room "Mass flow rate  deserved by each reheater";
      // -- Pressure drop parameters
     parameter Modelica.SIunits.PressureDifference dpVal_nominal = 1000
        "Pressure difference of thermostatic valve of the Hot Deck";
     parameter Modelica.SIunits.PressureDifference dpHotDeck = 6000
        "Pressure difference of water flow in the Hot Deck";
      // Room model
    protected
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_Air = sum(m_flow_room);
        parameter Modelica.SIunits.Volume VBuilding = sum(VRoom);
        parameter Modelica.SIunits.PressureDifference dp_nominal=
        dpVal_nominal + dpHotDeck
        "Pressure difference of loop";
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end VAVReheatParameter;

    record DoubleFlowParameter
        // --- Control-Command parameters
       // -- Design parameters
        parameter Integer nReheat(min=1)
        "Number of conditioned thermal zones deserved by the system";
        parameter Modelica.SIunits.Volume[nReheat] VRoom "Zone volume deserved by each reheater";
        parameter Modelica.SIunits.MassFlowRate[nReheat] m_flow_room "Mass flow rate  deserved by each reheater";
        parameter Real Tau=0.37 "Ratio of air volume extrated by the CTA";
        // -- Free cooling parameters
       parameter Modelica.SIunits.Temperature TRooCoo = 25+273.15
        "Set point for mechanical cooling";
       parameter Modelica.SIunits.Temperature TRooFre = 22+273.15
        "Maximum temperature above which free cooling is enabled";
       parameter Modelica.SIunits.Temperature TOutFre = 16+273.15
        "Outside temperature above which free cooling is allowed";
       parameter Modelica.SIunits.TemperatureDifference dT = 1
        "Dead-band for free cooling";
       parameter Real Kp(min=0) = 1 "Proportional band for mechanical cooling";
    protected
      parameter Modelica.SIunits.MassFlowRate m_flow_nominal_Air = sum(m_flow_room);
        parameter Modelica.SIunits.Volume VBuilding = sum(VRoom);
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end DoubleFlowParameter;
  end BaseClasses;

  package Examples
    extends Modelica.Icons.ExamplesPackage;
    model DoubleFlow "On Going"
      extends Modelica.Icons.Example;
         replaceable package MediumA = Buildings.Media.Air(T_default=293.15)
        "Medium model for air";
      DoubleFlux_FreeCooling_MechanicalCooling
        doubleFlux_FreeCooling_MechanicalCooling(nReheat=1,
        VRoom={75},
        m_flow_room={75*1.225*1/3600},
        Tau=1)
        annotation (Placement(transformation(extent={{10,2},{62,42}})));
      Modelica.Blocks.Sources.TimeTable TRoom(table=[0,16.76; 600,16.76; 1200,16.76;
            1800,16.76; 2400,16.76; 3000,16.76; 3600,16.73; 4200,16.74; 4800,16.72;
            5400,16.72; 6000,16.72; 6600,16.72; 7200,16.72; 7800,16.69; 8400,16.69;
            9000,16.68; 9600,16.68; 10200,16.67; 10800,16.65; 11400,16.64; 12000,16.64;
            12600,16.64; 13200,16.64; 13800,16.64; 14400,16.64; 15000,16.64; 15600,16.63;
            16200,16.6; 16800,16.6; 17400,16.6; 18000,16.6; 18600,16.6; 19200,16.6;
            19800,16.6; 20400,16.6; 21000,16.6; 21600,16.6; 22200,16.6; 22800,16.59;
            23400,16.56; 24000,16.56; 24600,16.56; 25200,16.57; 25800,16.56; 26400,16.57;
            27000,16.55; 27600,16.52; 28200,16.53; 28800,16.52; 29400,16.53; 30000,16.52;
            30600,16.53; 31200,16.52; 31800,16.52; 32400,16.52; 33000,16.52; 33600,16.53;
            34200,16.52; 34800,16.58; 35400,16.53; 36000,16.56; 36600,16.56; 37200,16.58;
            37800,16.58; 38400,16.6; 39000,16.6; 39600,16.6; 40200,16.6; 40800,16.64;
            41400,16.64; 42000,16.64; 42600,16.68; 43200,16.72; 43800,16.76; 44400,16.84;
            45000,16.88; 45600,16.96; 46200,17.01; 46800,17.32; 47400,17.05; 48000,17.06;
            48600,17.09; 49200,17.13; 49800,17.17; 50400,17.17; 51000,17.17; 51600,17.2;
            52200,17.21; 52800,17.25; 53400,17.21; 54000,17.21; 54600,17.21; 55200,17.17;
            55800,17.13; 56400,17.09; 57000,17.08; 57600,17.05; 58200,17.01; 58800,16.97;
            59400,16.95; 60000,16.92; 60600,16.92; 61200,16.9; 61800,16.85; 62400,16.86;
            63000,16.8; 63600,16.8; 64200,16.82; 64800,16.76; 65400,16.76; 66000,16.72;
            66600,16.72; 67200,16.71; 67800,16.69; 68400,16.69; 69000,16.64; 69600,16.64;
            70200,16.6; 70800,16.6; 71400,16.6; 72000,16.56; 72600,16.56; 73200,16.52;
            73800,16.52; 74400,16.48; 75000,16.48; 75600,16.44; 76200,16.44; 76800,16.44;
            77400,16.41; 78000,16.44; 78600,16.4; 79200,16.36; 79800,16.36; 80400,16.36;
            81000,16.32; 81600,16.32; 82200,16.32; 82800,16.32; 83400,16.3; 84000,16.28;
            84600,16.28; 85200,16.27; 85800,16.24; 86400,16.24; 87000,16.24; 87600,16.24;
            88200,16.24; 88800,16.22; 89400,16.19; 90000,16.19; 90600,16.19; 91200,16.18;
            91800,16.16; 92400,16.15; 93000,16.15; 93600,16.15; 94200,16.15; 94800,16.11;
            95400,16.11; 96000,16.11; 96600,16.11; 97200,16.11; 97800,16.11; 98400,16.11;
            99000,16.11; 99600,16.11; 100200,16.07; 100800,16.07; 101400,16.07; 102000,
            16.07; 102600,16.07; 103200,16.07; 103800,16.06; 104400,16.03; 105000,16.03;
            105600,16.03; 106200,16.03; 106800,16.03; 107400,16.02; 108000,15.99; 108600,
            15.99; 109200,15.99; 109800,15.99; 110400,15.99; 111000,15.99; 111600,15.99;
            112200,15.95; 112800,15.95; 113400,15.95; 114000,15.95; 114600,15.95; 115200,
            15.95; 115800,15.95; 116400,15.95; 117000,15.95; 117600,15.94; 118200,15.96;
            118800,15.91; 119400,15.92; 120000,15.91; 120600,15.91; 121200,15.91; 121800,
            15.91; 122400,15.91; 123000,16.21; 123600,15.91; 124200,15.91; 124800,15.92;
            125400,15.94; 126000,15.94; 126600,15.95; 127200,15.95; 127800,15.95; 128400,
            15.95; 129000,15.95; 129600,15.95; 130200,15.95; 130800,15.95; 131400,15.95;
            132000,15.95; 132600,15.95; 133200,15.95; 133800,15.96; 134400,15.96; 135000,
            15.96; 135600,15.95; 136200,15.95; 136800,15.94; 137400,15.91; 138000,15.92;
            138600,15.91; 139200,15.91; 139800,15.91; 140400,15.91; 141000,15.91; 141600,
            15.9; 142200,15.87; 142800,15.87; 143400,15.87; 144000,15.87; 144600,15.87;
            145200,15.87; 145800,15.83; 146400,15.83; 147000,15.81; 147600,15.79; 148200,
            15.79; 148800,15.79; 149400,15.79; 150000,15.78; 150600,15.75; 151200,15.75;
            151800,15.76; 152400,15.74; 153000,15.74; 153600,15.74; 154200,15.74; 154800,
            15.74; 155400,15.74; 156000,15.74; 156600,15.74; 157200,15.74; 157800,15.71;
            158400,15.7; 159000,15.72; 159600,15.7; 160200,15.7; 160800,15.72; 161400,
            15.66; 162000,15.66; 162600,15.66; 163200,15.66; 163800,15.66; 164400,15.66;
            165000,15.66; 165600,15.63; 166200,15.62; 166800,15.62; 167400,15.62; 168000,
            15.62; 168600,15.62; 169200,15.62; 169800,15.58; 170400,15.58; 171000,15.58;
            171600,15.58; 172200,15.6; 172800,15.55; 173400,15.58; 174000,15.58; 174600,
            15.54; 175200,15.54; 175800,15.54; 176400,15.54; 177000,15.54; 177600,15.54;
            178200,15.54; 178800,15.53; 179400,15.5; 180000,15.5; 180600,15.51; 181200,
            15.5; 181800,15.46; 182400,15.48; 183000,15.46; 183600,15.46; 184200,15.46;
            184800,15.46; 185400,15.46; 186000,15.46; 186600,15.46; 187200,15.46; 187800,
            15.46; 188400,15.42; 189000,15.42; 189600,15.42; 190200,15.42; 190800,15.42;
            191400,15.42; 192000,15.42; 192600,15.39; 193200,15.39; 193800,15.36; 194400,
            15.34; 195000,15.35; 195600,15.34; 196200,15.3; 196800,15.3; 197400,15.3;
            198000,15.3; 198600,15.3; 199200,15.28; 199800,15.27; 200400,15.29; 201000,
            15.26; 201600,15.26; 202200,15.26; 202800,15.26; 203400,15.25; 204000,15.22;
            204600,15.22; 205200,15.22; 205800,15.26; 206400,15.27; 207000,15.27; 207600,
            15.26; 208200,15.3; 208800,15.3; 209400,15.31; 210000,15.3; 210600,15.35;
            211200,15.34; 211800,15.34; 212400,15.34; 213000,15.36; 213600,15.38; 214200,
            15.42; 214800,15.46; 215400,15.46; 216000,15.46; 216600,15.5; 217200,15.5;
            217800,15.5; 218400,15.52; 219000,15.5; 219600,15.5; 220200,15.5; 220800,
            15.5; 221400,15.5; 222000,15.51; 222600,15.47; 223200,15.47; 223800,15.46;
            224400,15.46; 225000,15.42; 225600,15.42; 226200,15.38; 226800,15.38; 227400,
            15.34; 228000,15.34; 228600,15.34; 229200,15.34; 229800,15.3; 230400,15.3;
            231000,15.3; 231600,15.3; 232200,15.26; 232800,15.26; 233400,15.26; 234000,
            15.22; 234600,15.22; 235200,15.22; 235800,15.22; 236400,15.18; 237000,15.18;
            237600,15.14; 238200,15.14; 238800,15.14; 239400,15.14; 240000,15.13; 240600,
            15.1; 241200,15.1; 241800,15.1; 242400,15.1; 243000,15.1; 243600,15.1; 244200,
            15.1; 244800,15.1; 245400,15.1; 246000,15.09; 246600,15.06; 247200,15.05;
            247800,15.02; 248400,15.02; 249000,15.01; 249600,15.01; 250200,15; 250800,
            14.97; 251400,14.97; 252000,14.97; 252600,14.97; 253200,14.97; 253800,14.94;
            254400,14.93; 255000,14.93; 255600,14.92; 256200,14.89; 256800,14.9; 257400,
            14.85; 258000,14.85; 258600,14.85; 259200,14.82; 259800,14.81; 260400,14.81;
            261000,14.81; 261600,14.81; 262200,14.82; 262800,14.77; 263400,14.77; 264000,
            14.77; 264600,14.77; 265200,14.73; 265800,14.76; 266400,14.73; 267000,14.73;
            267600,14.73; 268200,14.73; 268800,14.73; 269400,14.73; 270000,14.73; 270600,
            14.73; 271200,14.73; 271800,14.73; 272400,14.7; 273000,14.73; 273600,14.72;
            274200,14.73; 274800,14.72; 275400,14.74; 276000,14.72; 276600,14.69; 277200,
            14.7; 277800,14.69; 278400,14.69; 279000,14.69; 279600,14.69; 280200,14.69;
            280800,14.69; 281400,14.66; 282000,14.69; 282600,14.69; 283200,14.65; 283800,
            14.66; 284400,14.77; 285000,14.93; 285600,15.01; 286200,15.11; 286800,15.22;
            287400,15.38; 288000,15.46; 288600,15.58; 289200,15.7; 289800,15.83; 290400,
            15.91; 291000,15.99; 291600,16.12; 292200,16.24; 292800,16.28; 293400,16.31;
            294000,16.41; 294600,16.48; 295200,16.56; 295800,16.6; 296400,16.69; 297000,
            16.72; 297600,16.76; 298200,16.82; 298800,16.88; 299400,16.93; 300000,16.97;
            300600,17.01; 301200,17.01; 301800,17.09; 302400,17.08; 303000,17.09; 303600,
            17.13; 304200,17.17; 304800,17.21; 305400,17.18; 306000,17.22; 306600,17.21;
            307200,17.29; 307800,17.31; 308400,17.38; 309000,17.46; 309600,17.54; 310200,
            17.62; 310800,17.62; 311400,17.67; 312000,17.73; 312600,17.78; 313200,17.74;
            313800,17.9; 314400,17.9; 315000,17.9; 315600,18.02; 316200,18.06; 316800,
            17.94; 317400,17.9; 318000,17.9; 318600,17.98; 319200,17.94; 319800,17.9;
            320400,17.86; 321000,17.86; 321600,17.82; 322200,17.8; 322800,17.78; 323400,
            17.74; 324000,17.74; 324600,17.7; 325200,17.69; 325800,17.66; 326400,17.62;
            327000,17.58; 327600,17.58; 328200,17.62; 328800,17.62; 329400,17.58; 330000,
            17.5; 330600,17.42; 331200,17.37; 331800,17.29; 332400,17.21; 333000,17.17;
            333600,17.18; 334200,17.12; 334800,17.09; 335400,17.09; 336000,17.06; 336600,
            17.06; 337200,17.05; 337800,17.05; 338400,17.01; 339000,17.01; 339600,17.02;
            340200,16.98; 340800,16.97; 341400,16.97; 342000,16.93; 342600,16.92; 343200,
            16.92; 343800,16.9; 344400,16.88; 345000,16.88; 345600,16.88; 346200,16.88;
            346800,16.88; 347400,16.84; 348000,16.84; 348600,16.84; 349200,16.84; 349800,
            16.84; 350400,16.81; 351000,16.81; 351600,16.82; 352200,16.84; 352800,16.84;
            353400,16.84; 354000,16.84; 354600,16.84; 355200,16.86; 355800,16.89; 356400,
            16.88; 357000,16.88; 357600,16.88; 358200,16.88; 358800,16.9; 359400,16.89;
            360000,16.92; 360600,16.92; 361200,16.96; 361800,17.17; 362400,17.42; 363000,
            17.54; 363600,17.71; 364200,17.84; 364800,17.94; 365400,17.98; 366000,18.06;
            366600,18.1; 367200,18.15; 367800,18.19; 368400,18.24; 369000,18.27; 369600,
            18.35; 370200,18.35; 370800,18.35; 371400,18.44; 372000,18.47; 372600,18.58;
            373200,18.67; 373800,18.72; 374400,18.79; 375000,18.84; 375600,18.92; 376200,
            18.96; 376800,19.03; 377400,19.08; 378000,19.16; 378600,19.24; 379200,19.37;
            379800,19.4; 380400,19.47; 381000,19.53; 381600,19.58; 382200,19.61; 382800,
            19.69; 383400,19.81; 384000,19.86; 384600,19.94; 385200,19.98; 385800,20.03;
            386400,20.02; 387000,20.1; 387600,20.14; 388200,20.16; 388800,20.23; 389400,
            20.29; 390000,20.34; 390600,20.34; 391200,20.42; 391800,20.52; 392400,20.54;
            393000,20.54; 393600,20.51; 394200,20.51; 394800,20.5; 395400,20.51; 396000,
            20.51; 396600,20.51; 397200,20.51; 397800,20.47; 398400,20.5; 399000,20.52;
            399600,20.58; 400200,20.59; 400800,20.63; 401400,20.68; 402000,20.67; 402600,
            20.67; 403200,20.67; 403800,20.68; 404400,20.75; 405000,20.8; 405600,20.83;
            406200,20.83; 406800,20.83; 407400,20.87; 408000,20.86; 408600,20.87; 409200,
            20.86; 409800,20.83; 410400,20.83; 411000,20.92; 411600,20.87; 412200,20.88;
            412800,20.87; 413400,20.83; 414000,20.83; 414600,20.8; 415200,20.75; 415800,
            20.62; 416400,20.49; 417000,20.34; 417600,20.26; 418200,20.14; 418800,20.02;
            419400,19.86; 420000,19.78; 420600,19.65; 421200,19.57; 421800,19.53; 422400,
            19.41; 423000,19.34; 423600,19.28; 424200,19.2; 424800,19.16; 425400,19.12;
            426000,19.05; 426600,19; 427200,18.92; 427800,18.88; 428400,18.88; 429000,
            18.84; 429600,18.8; 430200,18.76; 430800,18.72; 431400,18.68; 432000,18.64;
            432600,18.64; 433200,18.6; 433800,18.55; 434400,18.55; 435000,18.51; 435600,
            18.51; 436200,18.47; 436800,18.47; 437400,18.43; 438000,18.4; 438600,18.39;
            439200,18.35; 439800,18.35; 440400,18.35; 441000,18.35; 441600,18.35; 442200,
            18.35; 442800,18.35; 443400,18.36; 444000,18.35; 444600,18.37; 445200,18.35;
            445800,18.35; 446400,18.38; 447000,18.36; 447600,18.4; 448200,18.55; 448800,
            18.84; 449400,19.01; 450000,19.2; 450600,19.37; 451200,19.45; 451800,19.5;
            452400,19.53; 453000,19.57; 453600,19.61; 454200,19.65; 454800,19.69; 455400,
            19.74; 456000,19.78; 456600,19.82; 457200,19.86; 457800,19.9; 458400,19.98;
            459000,20.02; 459600,20.06; 460200,20.06; 460800,20.14; 461400,20.18; 462000,
            20.27; 462600,20.3; 463200,20.34; 463800,20.44; 464400,20.46; 465000,20.47;
            465600,20.76; 466200,20.56; 466800,20.54; 467400,20.54; 468000,20.55; 468600,
            20.59; 469200,20.71; 469800,20.79; 470400,20.91; 471000,20.98; 471600,21;
            472200,21.08; 472800,21.08; 473400,21.16; 474000,21.24; 474600,21.24; 475200,
            21.28; 475800,21.16; 476400,19.86; 477000,20.72; 477600,21.04; 478200,21.15;
            478800,21.2; 479400,21.28; 480000,21.32; 480600,21.32; 481200,21.4; 481800,
            21.44; 482400,21.48; 483000,21.48; 483600,21.52; 484200,21.56; 484800,21.6;
            485400,21.6; 486000,21.66; 486600,21.69; 487200,21.73; 487800,21.74; 488400,
            21.73; 489000,21.77; 489600,21.81; 490200,21.81; 490800,21.81; 491400,21.85;
            492000,21.93; 492600,21.96; 493200,21.97; 493800,21.97; 494400,21.97; 495000,
            21.97; 495600,21.97; 496200,21.97; 496800,21.96; 497400,21.96; 498000,21.93;
            498600,21.93; 499200,21.9; 499800,21.89; 500400,21.93; 501000,21.89; 501600,
            21.81; 502200,21.69; 502800,21.62; 503400,21.49; 504000,21.32; 504600,21.2;
            505200,21.08; 505800,20.99; 506400,20.83; 507000,20.71; 507600,20.63; 508200,
            20.59; 508800,20.51; 509400,20.48; 510000,20.42; 510600,20.34; 511200,20.22;
            511800,20.18; 512400,20.18; 513000,20.11; 513600,20.06; 514200,20.02; 514800,
            19.94; 515400,19.9; 516000,19.82; 516600,19.78; 517200,19.73; 517800,19.69;
            518400,19.65; 519000,19.65; 519600,19.61; 520200,19.57; 520800,19.57; 521400,
            19.53; 522000,19.53; 522600,19.49; 523200,19.49; 523800,19.45; 524400,19.41;
            525000,19.45; 525600,19.57; 526200,19.73; 526800,19.87; 527400,20.02; 528000,
            20.18; 528600,20.26; 529200,20.34; 529800,20.46; 530400,20.55; 531000,20.63;
            531600,20.72; 532200,20.77; 532800,20.83; 533400,20.92; 534000,20.96; 534600,
            20.99; 535200,21; 535800,21.08; 536400,21.12; 537000,21.12; 537600,21.15;
            538200,21.16; 538800,21.19; 539400,21.23; 540000,21.24; 540600,21.28; 541200,
            21.28; 541800,21.32; 542400,21.32; 543000,21.32; 543600,21.31; 544200,21.28;
            544800,21.32; 545400,21.36; 546000,21.4; 546600,21.46; 547200,21.52; 547800,
            21.55; 548400,21.6; 549000,21.6; 549600,21.61; 550200,21.66; 550800,21.73;
            551400,21.77; 552000,21.81; 552600,21.81; 553200,21.93; 553800,21.99; 554400,
            22.05; 555000,22.09; 555600,22.1; 556200,22.17; 556800,22.21; 557400,22.24;
            558000,22.36; 558600,22.38; 559200,22.34; 559800,22.34; 560400,22.38; 561000,
            22.42; 561600,22.42; 562200,22.42; 562800,22.46; 563400,22.46; 564000,22.46;
            564600,22.5; 565200,22.5; 565800,22.5; 566400,22.51; 567000,22.48; 567600,
            22.5; 568200,22.5; 568800,22.5; 569400,22.5; 570000,22.54; 570600,22.58;
            571200,22.58; 571800,22.58; 572400,22.64; 573000,22.74; 573600,22.78; 574200,
            22.85; 574800,22.87; 575400,22.92; 576000,22.95; 576600,23; 577200,23.07;
            577800,23.17; 578400,23.27; 579000,23.27; 579600,23.24; 580200,23.4; 580800,
            23.44; 581400,23.48; 582000,23.48; 582600,23.43; 583200,23.4; 583800,23.37;
            584400,23.32; 585000,23.35; 585600,23.4; 586200,23.4; 586800,23.43; 587400,
            23.4; 588000,23.32; 588600,23.16; 589200,23.07; 589800,22.94; 590400,22.78;
            591000,22.62; 591600,22.46; 592200,22.34; 592800,22.22; 593400,22.18; 594000,
            22.05; 594600,21.93; 595200,21.85; 595800,21.77; 596400,21.65; 597000,21.6;
            597600,21.52; 598200,21.49; 598800,21.4; 599400,21.36; 600000,21.28; 600600,
            21.28; 601200,21.2; 601800,21.16; 602400,21.15; 603000,21.12; 603600,21.08;
            604200,21.04; 604800,21; 605400,20.96; 606000,20.95; 606600,20.9; 607200,
            20.87; 607800,20.83; 608400,20.84; 609000,20.83; 609600,20.78; 610200,20.76;
            610800,20.75; 611400,20.79; 612000,20.91; 612600,21.08; 613200,21.2; 613800,
            21.32; 614400,21.48; 615000,21.6; 615600,21.69; 616200,21.82; 616800,21.89;
            617400,22.01; 618000,22.09; 618600,22.18; 619200,22.25; 619800,22.33; 620400,
            22.38; 621000,22.42; 621600,22.49; 622200,22.5; 622800,22.58; 623400,22.62;
            624000,22.63; 624600,22.7; 625200,22.72; 625800,22.74; 626400,22.78; 627000,
            22.82; 627600,22.83; 628200,22.86; 628800,22.87; 629400,22.91; 630000,22.79;
            630600,22.74; 631200,22.7; 631800,22.7; 632400,22.78; 633000,22.91; 633600,
            23.03; 634200,23.11; 634800,23.18; 635400,23.24; 636000,23.22; 636600,23.15;
            637200,23.06; 637800,22.99; 638400,23; 639000,22.91; 639600,22.91; 640200,
            22.78; 640800,22.87; 641400,23.03; 642000,23.04; 642600,22.99; 643200,23.04;
            643800,23.03; 644400,23.12; 645000,23.11; 645600,23.11; 646200,23.12; 646800,
            23.11; 647400,23.03; 648000,22.88; 648600,22.82; 649200,22.78; 649800,22.83;
            650400,22.96; 651000,22.92; 651600,23.03; 652200,23.08; 652800,23.13; 653400,
            23.11; 654000,23.15; 654600,23.12; 655200,23.15; 655800,23.2; 656400,23.19;
            657000,23.24; 657600,23.27; 658200,23.28; 658800,23.27; 659400,23.28; 660000,
            23.27; 660600,23.27; 661200,23.27; 661800,23.23; 662400,23.27; 663000,23.27;
            663600,23.27; 664200,23.31; 664800,23.32; 665400,23.32; 666000,23.28; 666600,
            23.19; 667200,23.05; 667800,22.95; 668400,22.83; 669000,22.74; 669600,22.7;
            670200,22.62; 670800,22.58; 671400,22.54; 672000,22.48; 672600,22.42; 673200,
            22.34; 673800,22.3; 674400,22.27; 675000,22.22; 675600,22.14; 676200,22.05;
            676800,21.97; 677400,21.93; 678000,21.85; 678600,21.81; 679200,21.77; 679800,
            21.72; 680400,21.65; 681000,21.6; 681600,21.52; 682200,21.48; 682800,21.44;
            683400,21.4; 684000,21.4; 684600,21.32; 685200,21.32; 685800,21.29; 686400,
            21.24; 687000,21.2; 687600,21.16; 688200,21.15; 688800,21.12; 689400,21.08;
            690000,21.07; 690600,21.04; 691200,21; 691800,21; 692400,21; 693000,20.96;
            693600,20.91; 694200,20.92; 694800,20.91; 695400,20.88; 696000,20.88; 696600,
            20.87; 697200,20.83; 697800,20.83; 698400,20.83; 699000,20.79; 699600,20.79;
            700200,20.8; 700800,20.76; 701400,20.78; 702000,20.75; 702600,20.75; 703200,
            20.75; 703800,20.71; 704400,20.71; 705000,20.7; 705600,20.67; 706200,20.67;
            706800,20.67; 707400,20.65; 708000,20.63; 708600,20.63; 709200,20.63; 709800,
            20.63; 710400,20.63; 711000,20.59; 711600,20.59; 712200,20.59; 712800,20.56;
            713400,20.55; 714000,20.55; 714600,20.55; 715200,20.56; 715800,20.51; 716400,
            20.51; 717000,20.52; 717600,20.51; 718200,20.52; 718800,20.51; 719400,20.48;
            720000,20.5; 720600,20.5; 721200,20.48; 721800,20.46; 722400,20.46; 723000,
            20.46; 723600,20.46; 724200,20.48; 724800,20.44; 725400,20.46; 726000,20.42;
            726600,20.42; 727200,20.42; 727800,20.46; 728400,20.42; 729000,20.46; 729600,
            20.42; 730200,20.42; 730800,20.42; 731400,20.42; 732000,20.42; 732600,20.42;
            733200,20.42; 733800,20.42; 734400,20.38; 735000,20.38; 735600,20.4; 736200,
            20.38; 736800,20.38; 737400,20.38; 738000,20.4; 738600,20.38; 739200,20.38;
            739800,20.34; 740400,20.34; 741000,20.34; 741600,20.34; 742200,20.34; 742800,
            20.34; 743400,20.34; 744000,20.34; 744600,20.38; 745200,20.38; 745800,20.38;
            746400,20.38; 747000,20.34; 747600,20.34; 748200,20.36; 748800,20.32; 749400,
            20.3; 750000,20.32; 750600,20.3; 751200,20.3; 751800,20.26; 752400,20.26;
            753000,20.24; 753600,20.26; 754200,20.22; 754800,20.23; 755400,20.22; 756000,
            20.21; 756600,20.18; 757200,20.18; 757800,20.18; 758400,20.18; 759000,20.18;
            759600,20.14; 760200,20.14; 760800,20.14; 761400,20.1; 762000,20.1; 762600,
            20.1; 763200,20.09; 763800,20.06; 764400,20.06; 765000,20.06; 765600,20.04;
            766200,20.02; 766800,20.02; 767400,20.02; 768000,19.98; 768600,19.98; 769200,
            19.98; 769800,19.94; 770400,19.94; 771000,19.94; 771600,19.9; 772200,19.9;
            772800,19.86; 773400,19.86; 774000,19.86; 774600,19.87; 775200,19.82; 775800,
            19.82; 776400,19.81; 777000,19.78; 777600,19.78; 778200,19.75; 778800,19.73;
            779400,19.73; 780000,19.73; 780600,19.73; 781200,19.74; 781800,19.72; 782400,
            19.73; 783000,19.7; 783600,19.69; 784200,19.72; 784800,19.69; 785400,19.69;
            786000,19.65; 786600,19.65; 787200,19.65; 787800,19.65; 788400,19.65; 789000,
            19.65; 789600,19.65; 790200,19.61; 790800,19.62; 791400,19.62; 792000,19.61;
            792600,19.61; 793200,19.62; 793800,19.61; 794400,19.61; 795000,19.61; 795600,
            19.61; 796200,19.57; 796800,19.61; 797400,19.6; 798000,19.59; 798600,19.57;
            799200,19.57; 799800,19.57; 800400,19.57; 801000,19.57; 801600,19.57; 802200,
            19.58; 802800,19.57; 803400,19.57; 804000,19.57; 804600,19.57; 805200,19.57;
            805800,19.57; 806400,19.57; 807000,19.57; 807600,19.57; 808200,19.57; 808800,
            19.57; 809400,19.57; 810000,19.58; 810600,19.61; 811200,19.61; 811800,19.61;
            812400,19.62; 813000,19.61; 813600,19.64; 814200,19.65; 814800,19.65; 815400,
            19.65; 816000,19.66; 816600,19.66; 817200,19.69; 817800,19.7; 818400,19.73;
            819000,19.73; 819600,19.77; 820200,19.78; 820800,19.78; 821400,19.78; 822000,
            19.78; 822600,19.83; 823200,19.82; 823800,19.82; 824400,19.83; 825000,19.82;
            825600,19.84; 826200,19.83; 826800,19.83; 827400,19.82; 828000,19.82; 828600,
            19.82; 829200,19.82; 829800,19.82; 830400,19.82; 831000,19.82; 831600,19.82;
            832200,19.82; 832800,19.82; 833400,19.82; 834000,19.82; 834600,19.82; 835200,
            19.82; 835800,19.82; 836400,19.82; 837000,19.82; 837600,19.8; 838200,19.78;
            838800,19.78; 839400,19.78; 840000,19.78; 840600,19.78; 841200,19.78; 841800,
            19.78; 842400,19.77; 843000,19.73; 843600,19.76; 844200,19.73; 844800,19.73;
            845400,19.73; 846000,19.73; 846600,19.73; 847200,19.74; 847800,19.73; 848400,
            19.73; 849000,19.69; 849600,19.69; 850200,19.69; 850800,19.69; 851400,19.7;
            852000,19.73; 852600,19.73; 853200,19.73; 853800,19.73; 854400,19.74; 855000,
            19.73; 855600,19.73; 856200,19.73; 856800,19.73; 857400,19.73; 858000,19.69;
            858600,19.69; 859200,19.7; 859800,19.7; 860400,19.69; 861000,19.69; 861600,
            19.7; 862200,19.69; 862800,19.69; 863400,19.67; 864000,19.66; 864600,19.66;
            865200,19.65; 865800,19.65; 866400,19.66; 867000,19.65; 867600,19.65; 868200,
            19.61; 868800,19.65; 869400,19.65; 870000,19.65; 870600,19.65; 871200,19.65;
            871800,19.66; 872400,19.65; 873000,19.65; 873600,19.65; 874200,19.61; 874800,
            19.61; 875400,19.63; 876000,19.61; 876600,19.61; 877200,19.65; 877800,19.65;
            878400,19.66; 879000,19.65; 879600,19.65; 880200,19.65; 880800,19.66; 881400,
            19.65; 882000,19.65; 882600,19.65; 883200,19.65; 883800,19.65; 884400,19.65;
            885000,19.65; 885600,19.65; 886200,19.68; 886800,19.65; 887400,19.65; 888000,
            19.81; 888600,20.02; 889200,20.18; 889800,20.26; 890400,20.42; 891000,20.58;
            891600,20.71; 892200,20.83; 892800,20.96; 893400,21; 894000,21.08; 894600,
            21.17; 895200,21.2; 895800,21.32; 896400,21.46; 897000,21.56; 897600,21.65;
            898200,21.69; 898800,21.77; 899400,21.84; 900000,21.89; 900600,22.01; 901200,
            22.09; 901800,22.17; 902400,22.26; 903000,22.38; 903600,22.51; 904200,22.59;
            904800,22.62; 905400,22.66; 906000,22.7; 906600,22.7; 907200,22.71; 907800,
            22.76; 908400,22.74; 909000,22.75; 909600,22.83; 910200,22.91; 910800,22.94;
            911400,22.94; 912000,23.03; 912600,23.07; 913200,23.11; 913800,23.11; 914400,
            23.15; 915000,23.27; 915600,23.4; 916200,23.49; 916800,23.56; 917400,23.6;
            918000,23.64; 918600,23.72; 919200,23.76; 919800,23.88; 920400,23.84; 921000,
            23.76; 921600,23.73; 922200,23.72; 922800,23.68; 923400,23.68; 924000,23.68;
            924600,23.67; 925200,23.68; 925800,23.65; 926400,23.64; 927000,23.64; 927600,
            23.68; 928200,23.72; 928800,23.72; 929400,23.72; 930000,23.72; 930600,23.72;
            931200,23.72; 931800,23.72; 932400,23.72; 933000,23.72; 933600,23.65; 934200,
            23.48; 934800,23.36; 935400,23.18; 936000,23.03; 936600,22.91; 937200,22.78;
            937800,22.67; 938400,22.57; 939000,22.46; 939600,22.38; 940200,22.26; 940800,
            22.18; 941400,22.09; 942000,22.02; 942600,21.97; 943200,21.89; 943800,21.85;
            944400,21.81; 945000,21.77; 945600,21.69; 946200,21.65; 946800,21.61; 947400,
            21.6; 948000,21.6; 948600,21.56; 949200,21.56; 949800,21.52; 950400,21.51;
            951000,21.44; 951600,21.44; 952200,21.39; 952800,21.4; 953400,21.36; 954000,
            21.36; 954600,21.32; 955200,21.28; 955800,21.32; 956400,21.32; 957000,21.32;
            957600,21.44; 958200,21.6; 958800,21.73; 959400,21.89; 960000,22.05; 960600,
            22.18; 961200,22.3; 961800,22.44; 962400,22.58; 963000,22.7; 963600,22.78;
            964200,22.83; 964800,22.91; 965400,22.99; 966000,23.03; 966600,23.07; 967200,
            23.11; 967800,23.11; 968400,23.15; 969000,23.23; 969600,23.25; 970200,23.27;
            970800,23.32; 971400,23.35; 972000,23.36; 972600,23.36; 973200,23.4; 973800,
            23.4; 974400,23.4; 975000,23.36; 975600,23.27; 976200,23.32; 976800,23.32;
            977400,23.27; 978000,23.23; 978600,23.19; 979200,23.16; 979800,23.11; 980400,
            23.14; 981000,23.08; 981600,23.07; 982200,23.03; 982800,23.03; 983400,23.03;
            984000,23.03; 984600,22.95; 985200,22.95; 985800,22.95; 986400,22.99; 987000,
            22.99; 987600,22.95; 988200,23.03; 988800,23.07; 989400,23.07; 990000,23.1;
            990600,23.11; 991200,23.11; 991800,23.12; 992400,23.19; 993000,23.19; 993600,
            23.12; 994200,23.11; 994800,23.06; 995400,23.08; 996000,23.15; 996600,23.24;
            997200,23.23; 997800,23.2; 998400,23.2; 999000,23.15; 999600,23.19; 1000200,
            23.19; 1000800,23.23; 1001400,23.24; 1002000,23.27; 1002600,23.32; 1003200,
            23.23; 1003800,23.23; 1004400,23.27; 1005000,23.24; 1005600,23.27; 1006200,
            23.24; 1006800,23.28; 1007400,23.27; 1008000,23.27; 1008600,23.24; 1009200,
            23.2; 1009800,23.19; 1010400,23.07; 1011000,23.03; 1011600,22.92; 1012200,
            22.91; 1012800,22.83; 1013400,22.79; 1014000,22.66; 1014600,22.58; 1015200,
            22.5; 1015800,22.5; 1016400,22.42; 1017000,22.35; 1017600,22.26; 1018200,
            22.26; 1018800,22.21; 1019400,22.14; 1020000,22.09; 1020600,22.04; 1021200,
            22.02; 1021800,21.97; 1022400,21.89; 1023000,21.81; 1023600,21.77; 1024200,
            21.69; 1024800,21.7; 1025400,21.62; 1026000,21.6; 1026600,21.6; 1027200,
            21.56; 1027800,21.48; 1028400,21.48; 1029000,21.4; 1029600,21.4; 1030200,
            21.36; 1030800,21.32; 1031400,21.32; 1032000,21.28; 1032600,21.28; 1033200,
            21.25; 1033800,21.2; 1034400,21.16; 1035000,21.16; 1035600,21.12; 1036200,
            21.12; 1036800,21.08; 1037400,21.08; 1038000,21.07; 1038600,21.04; 1039200,
            21; 1039800,20.99; 1040400,20.96; 1041000,20.96; 1041600,20.96; 1042200,
            20.95; 1042800,20.91; 1043400,20.87; 1044000,20.87; 1044600,20.91; 1045200,
            20.96; 1045800,20.91; 1046400,20.96; 1047000,20.96; 1047600,20.96; 1048200,
            20.96; 1048800,20.96; 1049400,20.96; 1050000,21; 1050600,20.99; 1051200,
            20.96; 1051800,20.96; 1052400,20.96; 1053000,20.96; 1053600,20.96; 1054200,
            20.96; 1054800,20.96; 1055400,20.96; 1056000,20.95; 1056600,20.94; 1057200,
            20.94; 1057800,20.95; 1058400,20.91; 1059000,20.95; 1059600,20.92; 1060200,
            20.96; 1060800,20.96; 1061400,21; 1062000,21.16; 1062600,21.27; 1063200,
            21.36; 1063800,21.44; 1064400,21.48; 1065000,21.56; 1065600,21.68; 1066200,
            21.81; 1066800,21.94; 1067400,22.01; 1068000,22.1; 1068600,22.18; 1069200,
            22.22; 1069800,22.22; 1070400,22.22; 1071000,22.23; 1071600,22.26; 1072200,
            22.3; 1072800,22.38; 1073400,22.42; 1074000,22.47; 1074600,22.54; 1075200,
            22.58; 1075800,22.7; 1076400,22.78; 1077000,22.83; 1077600,22.83; 1078200,
            22.87; 1078800,22.83; 1079400,22.78; 1080000,22.78; 1080600,22.82; 1081200,
            22.91; 1081800,22.88; 1082400,22.87; 1083000,22.87; 1083600,22.96; 1084200,
            22.96; 1084800,22.99; 1085400,23.03; 1086000,23.07; 1086600,23.16; 1087200,
            23.23; 1087800,23.15; 1088400,23.16; 1089000,23.23; 1089600,23.27; 1090200,
            23.28; 1090800,23.31; 1091400,23.32; 1092000,23.36; 1092600,23.36; 1093200,
            23.34; 1093800,23.36; 1094400,23.39; 1095000,23.4; 1095600,23.32; 1096200,
            23.27; 1096800,23.23; 1097400,23.18; 1098000,23.11; 1098600,23.07; 1099200,
            23; 1099800,22.87; 1100400,22.7; 1101000,22.58; 1101600,22.46; 1102200,22.4;
            1102800,22.3; 1103400,22.22; 1104000,22.14; 1104600,22.05; 1105200,21.97;
            1105800,21.89; 1106400,21.81; 1107000,21.77; 1107600,21.69; 1108200,21.6;
            1108800,21.56; 1109400,21.48; 1110000,21.44; 1110600,21.41; 1111200,21.32;
            1111800,21.24; 1112400,21.2; 1113000,21.16; 1113600,21.08; 1114200,21.02;
            1114800,20.99; 1115400,20.93; 1116000,20.87; 1116600,20.83; 1117200,20.79;
            1117800,20.79; 1118400,20.75; 1119000,20.71; 1119600,20.67; 1120200,20.64;
            1120800,20.63; 1121400,20.59; 1122000,20.55; 1122600,20.51; 1123200,20.51;
            1123800,20.46; 1124400,20.44; 1125000,20.42; 1125600,20.38; 1126200,20.36;
            1126800,20.34; 1127400,20.33; 1128000,20.3; 1128600,20.26; 1129200,20.26;
            1129800,20.23; 1130400,20.22; 1131000,20.22; 1131600,20.18; 1132200,20.18;
            1132800,20.18; 1133400,20.14; 1134000,20.14; 1134600,20.1; 1135200,20.1;
            1135800,20.06; 1136400,20.06; 1137000,20.02; 1137600,20.02; 1138200,20.02;
            1138800,20.02; 1139400,20.02; 1140000,19.98; 1140600,19.98; 1141200,19.98;
            1141800,19.99; 1142400,19.98; 1143000,19.98; 1143600,19.98; 1144200,19.97;
            1144800,19.94; 1145400,19.96; 1146000,19.94; 1146600,19.94; 1147200,19.94;
            1147800,20.02; 1148400,20.3; 1149000,20.47; 1149600,20.63; 1150200,20.88;
            1150800,20.87; 1151400,21.04; 1152000,21.11; 1152600,21.16; 1153200,21.26;
            1153800,21.32; 1154400,21.36; 1155000,21.33; 1155600,21.38; 1156200,21.73;
            1156800,22.06; 1157400,22.38; 1158000,22.58; 1158600,22.78; 1159200,22.97;
            1159800,23.11; 1160400,23.27; 1161000,23.44; 1161600,23.56; 1162200,23.73;
            1162800,23.84; 1163400,23.77; 1164000,23.72; 1164600,23.72; 1165200,23.72;
            1165800,23.8; 1166400,23.82; 1167000,23.84; 1167600,23.72; 1168200,23.8;
            1168800,23.8; 1169400,23.88; 1170000,23.93; 1170600,24.01; 1171200,24.09;
            1171800,24.18; 1172400,24.32; 1173000,24.38; 1173600,24.37; 1174200,24.37;
            1174800,24.29; 1175400,24.16; 1176000,23.98; 1176600,23.93; 1177200,23.79;
            1177800,23.68; 1178400,23.58; 1179000,23.44; 1179600,23.43; 1180200,23.47;
            1180800,23.44; 1181400,23.45; 1182000,23.44; 1182600,23.4; 1183200,23.35;
            1183800,23.24; 1184400,23.24; 1185000,23.17; 1185600,23.11; 1186200,23.07;
            1186800,22.99; 1187400,22.87; 1188000,22.74; 1188600,22.62; 1189200,22.5;
            1189800,22.42; 1190400,22.34; 1191000,22.26; 1191600,22.18; 1192200,22.09;
            1192800,22.05; 1193400,21.97; 1194000,21.89; 1194600,21.81; 1195200,21.81;
            1195800,21.77; 1196400,21.69; 1197000,21.65; 1197600,21.6; 1198200,21.48;
            1198800,21.44; 1199400,21.41; 1200000,21.33; 1200600,21.28; 1201200,21.24;
            1201800,21.16; 1202400,21.12; 1203000,21.12; 1203600,21.04; 1204200,21;
            1204800,20.96; 1205400,20.96; 1206000,20.91; 1206600,20.88; 1207200,20.83;
            1207800,20.79; 1208400,20.79; 1209000,20.75; 1209600,20.71; 1210200,20.67;
            1210800,20.67; 1211400,20.63; 1212000,20.63; 1212600,20.59; 1213200,20.55;
            1213800,20.54; 1214400,20.51; 1215000,20.51; 1215600,20.48; 1216200,20.46;
            1216800,20.46; 1217400,20.42; 1218000,20.42; 1218600,20.42; 1219200,20.39;
            1219800,20.38; 1220400,20.38; 1221000,20.34; 1221600,20.34; 1222200,20.34;
            1222800,20.34; 1223400,20.34; 1224000,20.34; 1224600,20.36; 1225200,20.3;
            1225800,20.3; 1226400,20.3; 1227000,20.3; 1227600,20.3; 1228200,20.3; 1228800,
            20.3; 1229400,20.29; 1230000,20.26; 1230600,20.26; 1231200,20.26; 1231800,
            20.26; 1232400,20.24; 1233000,20.22; 1233600,20.22; 1234200,20.34; 1234800,
            20.51; 1235400,20.67; 1236000,20.75; 1236600,20.87; 1237200,20.95; 1237800,
            21.07; 1238400,21.16; 1239000,21.22; 1239600,21.32; 1240200,21.36; 1240800,
            21.44; 1241400,21.5; 1242000,21.6; 1242600,21.65; 1243200,21.69; 1243800,
            21.76; 1244400,21.85; 1245000,21.85; 1245600,21.93; 1246200,21.97; 1246800,
            22.01; 1247400,22.09; 1248000,22.1; 1248600,22.14; 1249200,22.18; 1249800,
            22.27; 1250400,22.29; 1251000,22.42; 1251600,22.46; 1252200,22.5; 1252800,
            22.5; 1253400,22.5; 1254000,22.54; 1254600,22.58; 1255200,22.54; 1255800,
            22.54; 1256400,22.5; 1257000,22.54; 1257600,22.66; 1258200,22.7; 1258800,
            22.7; 1259400,22.74; 1260000,22.75; 1260600,22.83; 1261200,22.89; 1261800,
            22.8; 1262400,22.76; 1263000,22.7; 1263600,22.78; 1264200,22.78; 1264800,
            22.78; 1265400,22.78; 1266000,22.75; 1266600,22.77; 1267200,22.82; 1267800,
            22.83; 1268400,22.78; 1269000,22.74; 1269600,22.7; 1270200,22.66; 1270800,
            22.64; 1271400,22.58; 1272000,22.58; 1272600,22.54; 1273200,22.54; 1273800,
            22.5; 1274400,22.51; 1275000,22.46; 1275600,22.42; 1276200,22.42; 1276800,
            22.35; 1277400,22.23; 1278000,22.15; 1278600,22.05; 1279200,21.97; 1279800,
            21.89; 1280400,21.82; 1281000,21.75; 1281600,21.72; 1282200,21.65; 1282800,
            21.6; 1283400,21.52; 1284000,21.44; 1284600,21.4; 1285200,21.32; 1285800,
            21.28; 1286400,21.2; 1287000,21.16; 1287600,21.12; 1288200,21.04; 1288800,
            21; 1289400,20.97; 1290000,20.88; 1290600,20.83; 1291200,20.79; 1291800,
            20.75; 1292400,20.72; 1293000,20.67; 1293600,20.63; 1294200,20.59; 1294800,
            20.58; 1295400,20.55; 1296000,20.51; 1296600,20.49; 1297200,20.46; 1297800,
            20.42; 1298400,20.38; 1299000,20.38; 1299600,20.34; 1300200,20.33; 1300800,
            20.3; 1301400,20.26; 1302000,20.26; 1302600,20.22; 1303200,20.23; 1303800,
            20.19; 1304400,20.18; 1305000,20.14; 1305600,20.14; 1306200,20.11; 1306800,
            20.1; 1307400,20.1; 1308000,20.06; 1308600,20.05; 1309200,20.02; 1309800,
            20.02; 1310400,19.99; 1311000,19.98; 1311600,19.98; 1312200,19.94; 1312800,
            19.94; 1313400,19.9; 1314000,19.9; 1314600,19.9; 1315200,19.86; 1315800,
            19.86; 1316400,19.85; 1317000,19.82; 1317600,19.82; 1318200,19.82; 1318800,
            19.78; 1319400,19.78; 1320000,19.78; 1320600,19.78; 1321200,19.73; 1321800,
            19.73; 1322400,19.73; 1323000,19.73; 1323600,19.97; 1324200,19.69; 1324800,
            19.69; 1325400,19.69; 1326000,19.69; 1326600,19.65; 1327200,19.65; 1327800,
            19.65; 1328400,19.65; 1329000,19.66; 1329600,19.65; 1330200,19.69; 1330800,
            19.69; 1331400,19.74; 1332000,19.69; 1332600,19.73; 1333200,19.74; 1333800,
            19.73; 1334400,19.76; 1335000,19.78; 1335600,19.78; 1336200,19.78; 1336800,
            19.78; 1337400,19.78; 1338000,19.78; 1338600,19.78; 1339200,19.82; 1339800,
            19.82; 1340400,19.82; 1341000,19.82; 1341600,19.86; 1342200,19.85; 1342800,
            19.86; 1343400,19.9; 1344000,19.9; 1344600,19.98; 1345200,19.98; 1345800,
            20.02; 1346400,20.02; 1347000,19.98; 1347600,19.94; 1348200,19.9; 1348800,
            19.88; 1349400,19.86; 1350000,19.82; 1350600,19.82; 1351200,19.78; 1351800,
            19.78; 1352400,19.73; 1353000,19.73; 1353600,19.7; 1354200,19.69; 1354800,
            19.65; 1355400,19.65; 1356000,19.61; 1356600,19.57; 1357200,19.57; 1357800,
            19.53; 1358400,19.53; 1359000,19.53; 1359600,19.5; 1360200,19.49; 1360800,
            19.47; 1361400,19.45; 1362000,19.45; 1362600,19.42; 1363200,19.41; 1363800,
            19.37; 1364400,19.37; 1365000,19.37; 1365600,19.34; 1366200,19.34; 1366800,
            19.33; 1367400,19.32; 1368000,19.28; 1368600,19.28; 1369200,19.28; 1369800,
            19.24; 1370400,19.24; 1371000,19.24; 1371600,19.24; 1372200,19.2; 1372800,
            19.2; 1373400,19.21; 1374000,19.2; 1374600,19.16; 1375200,19.16; 1375800,
            19.16; 1376400,19.18; 1377000,19.16; 1377600,19.12; 1378200,19.12; 1378800,
            19.08; 1379400,19.08; 1380000,19.06; 1380600,19.04; 1381200,19.04; 1381800,
            19.04; 1382400,19.02; 1383000,19; 1383600,19; 1384200,19; 1384800,19; 1385400,
            18.98; 1386000,18.96; 1386600,18.96; 1387200,18.96; 1387800,18.96; 1388400,
            18.97; 1389000,18.94; 1389600,18.92; 1390200,18.92; 1390800,18.92; 1391400,
            18.92; 1392000,18.92; 1392600,18.91; 1393200,18.91; 1393800,18.88; 1394400,
            18.89; 1395000,18.88; 1395600,18.88; 1396200,18.88; 1396800,18.88; 1397400,
            18.89; 1398000,18.89; 1398600,18.86; 1399200,18.84; 1399800,18.84; 1400400,
            18.84; 1401000,18.85; 1401600,18.84; 1402200,18.84; 1402800,18.81; 1403400,
            18.82; 1404000,18.8; 1404600,18.81; 1405200,18.8; 1405800,18.8; 1406400,
            18.8; 1407000,18.8; 1407600,18.8; 1408200,18.8; 1408800,18.76; 1409400,18.76;
            1410000,18.76; 1410600,18.77; 1411200,18.76; 1411800,18.77; 1412400,18.76;
            1413000,18.76; 1413600,18.76; 1414200,18.78; 1414800,18.8; 1415400,18.8;
            1416000,18.82; 1416600,18.84; 1417200,18.84; 1417800,18.85; 1418400,18.88;
            1419000,18.88; 1419600,18.89; 1420200,18.92; 1420800,18.92; 1421400,18.96;
            1422000,19; 1422600,19; 1423200,19.04; 1423800,19.04; 1424400,19.08; 1425000,
            19.12; 1425600,19.15; 1426200,19.16; 1426800,19.2; 1427400,19.2; 1428000,
            19.24; 1428600,19.24; 1429200,19.29; 1429800,19.33; 1430400,19.37; 1431000,
            19.37; 1431600,19.4; 1432200,19.45; 1432800,19.45; 1433400,19.49; 1434000,
            19.5; 1434600,19.53; 1435200,19.53; 1435800,19.57; 1436400,19.53; 1437000,
            19.53; 1437600,19.52; 1438200,19.49; 1438800,19.45; 1439400,19.41; 1440000,
            19.37; 1440600,19.37; 1441200,19.33; 1441800,19.28; 1442400,19.24; 1443000,
            19.2; 1443600,19.2; 1444200,19.16; 1444800,19.14; 1445400,19.12; 1446000,
            19.08; 1446600,19.08; 1447200,19.04; 1447800,19.04; 1448400,19; 1449000,
            19; 1449600,18.96; 1450200,18.93; 1450800,18.93; 1451400,18.88; 1452000,
            18.88; 1452600,18.88; 1453200,18.85; 1453800,18.84; 1454400,18.84; 1455000,
            18.8; 1455600,18.8; 1456200,18.8; 1456800,18.77; 1457400,18.76; 1458000,
            18.72; 1458600,18.72; 1459200,18.72; 1459800,18.68; 1460400,18.68; 1461000,
            18.68; 1461600,18.65; 1462200,18.64; 1462800,18.64; 1463400,18.61; 1464000,
            18.6; 1464600,18.6; 1465200,18.6; 1465800,18.56; 1466400,18.55; 1467000,
            18.56; 1467600,18.52; 1468200,18.51; 1468800,18.51; 1469400,18.51; 1470000,
            18.51; 1470600,18.51; 1471200,18.47; 1471800,18.47; 1472400,18.47; 1473000,
            18.47; 1473600,18.43; 1474200,18.43; 1474800,18.43; 1475400,18.43; 1476000,
            18.43; 1476600,18.4; 1477200,18.39; 1477800,18.39; 1478400,18.39; 1479000,
            18.39; 1479600,18.35; 1480200,18.35; 1480800,18.35; 1481400,18.36; 1482000,
            18.32; 1482600,18.31; 1483200,18.31; 1483800,18.31; 1484400,18.32; 1485000,
            18.34; 1485600,18.35; 1486200,18.35; 1486800,18.35; 1487400,18.35; 1488000,
            18.35; 1488600,18.35; 1489200,18.35; 1489800,18.35; 1490400,18.35; 1491000,
            18.35; 1491600,18.33; 1492200,18.35; 1492800,18.35; 1493400,18.35; 1494000,
            18.65; 1494600,18.88; 1495200,19.06; 1495800,19.38; 1496400,19.62; 1497000,
            19.69; 1497600,19.73; 1498200,19.83; 1498800,20.22; 1499400,20.59; 1500000,
            20.91; 1500600,21.13; 1501200,21.36; 1501800,21.56; 1502400,21.74; 1503000,
            21.9; 1503600,21.93; 1504200,22.06; 1504800,22.14; 1505400,22.26; 1506000,
            22.38; 1506600,22.42; 1507200,22.46; 1507800,22.54; 1508400,22.66; 1509000,
            22.77; 1509600,22.84; 1510200,22.83; 1510800,22.78; 1511400,22.75; 1512000,
            22.74; 1512600,22.72; 1513200,22.74; 1513800,22.87; 1514400,22.94; 1515000,
            22.91; 1515600,22.86; 1516200,22.78; 1516800,22.78; 1517400,22.72; 1518000,
            22.62; 1518600,22.54; 1519200,22.46; 1519800,22.43; 1520400,22.38; 1521000,
            22.34; 1521600,22.3; 1522200,22.3; 1522800,22.22; 1523400,22.22; 1524000,
            22.18; 1524600,22.18; 1525200,22.14; 1525800,22.18; 1526400,22.14; 1527000,
            22.1; 1527600,22.09; 1528200,22.05; 1528800,22.01; 1529400,21.97; 1530000,
            21.89; 1530600,21.86; 1531200,21.97; 1531800,21.93; 1532400,21.97; 1533000,
            21.85; 1533600,21.73; 1534200,21.6; 1534800,21.52; 1535400,21.44; 1536000,
            21.46; 1536600,21.44; 1537200,21.4; 1537800,21.32; 1538400,21.2; 1539000,
            21.08; 1539600,21.03; 1540200,20.99; 1540800,20.96; 1541400,20.91; 1542000,
            20.87; 1542600,20.83; 1543200,20.83; 1543800,20.82; 1544400,20.75; 1545000,
            20.71; 1545600,20.67; 1546200,20.63; 1546800,20.59; 1547400,20.55; 1548000,
            20.51; 1548600,20.51; 1549200,20.5; 1549800,20.44; 1550400,20.42; 1551000,
            20.38; 1551600,20.38; 1552200,20.34; 1552800,20.34; 1553400,20.3; 1554000,
            20.3; 1554600,20.26; 1555200,20.28; 1555800,20.26; 1556400,20.25; 1557000,
            20.22; 1557600,20.22; 1558200,20.22; 1558800,20.18; 1559400,20.18; 1560000,
            20.19; 1560600,20.18; 1561200,20.18; 1561800,20.18; 1562400,20.14; 1563000,
            20.14; 1563600,20.14; 1564200,20.14; 1564800,20.14; 1565400,20.15; 1566000,
            20.14; 1566600,20.1; 1567200,20.11; 1567800,20.1; 1568400,20.1; 1569000,
            20.1; 1569600,20.1; 1570200,20.1; 1570800,20.13; 1571400,20.14; 1572000,
            20.1; 1572600,20.06; 1573200,20.02; 1573800,20.01; 1574400,20.02; 1575000,
            20.07; 1575600,20.1; 1576200,20.02; 1576800,19.98; 1577400,19.98; 1578000,
            19.98; 1578600,20.02; 1579200,20.11; 1579800,20.06; 1580400,20.02; 1581000,
            20.02; 1581600,20.03; 1582200,20.05; 1582800,20.19; 1583400,20.42; 1584000,
            20.59; 1584600,20.71; 1585200,20.83; 1585800,20.91; 1586400,20.96; 1587000,
            21.07; 1587600,21.12; 1588200,21.24; 1588800,21.32; 1589400,21.36; 1590000,
            21.44; 1590600,21.56; 1591200,21.65; 1591800,21.73; 1592400,21.65; 1593000,
            21.74; 1593600,21.69; 1594200,21.75; 1594800,21.81; 1595400,21.93; 1596000,
            21.93; 1596600,21.9; 1597200,21.89; 1597800,21.85; 1598400,21.85; 1599000,
            21.86; 1599600,21.88; 1600200,22.01; 1600800,22.09; 1601400,22.17; 1602000,
            22.22; 1602600,22.22; 1603200,22.3; 1603800,22.34; 1604400,22.46; 1605000,
            22.46; 1605600,22.44; 1606200,22.46; 1606800,22.5; 1607400,22.56; 1608000,
            22.58; 1608600,22.62; 1609200,22.61; 1609800,22.54; 1610400,22.58; 1611000,
            22.62; 1611600,22.66; 1612200,22.66; 1612800,22.7; 1613400,22.62; 1614000,
            22.58; 1614600,22.54; 1615200,22.62; 1615800,22.51; 1616400,22.27; 1617000,
            22.22; 1617600,22.26; 1618200,22.29; 1618800,22.23; 1619400,22.18; 1620000,
            22.26; 1620600,22.3; 1621200,22.22; 1621800,22.1; 1622400,21.97; 1623000,
            21.89; 1623600,21.86; 1624200,21.81; 1624800,21.81; 1625400,21.81; 1626000,
            21.81; 1626600,21.74; 1627200,21.6; 1627800,21.52; 1628400,21.44; 1629000,
            21.36; 1629600,21.32; 1630200,21.32; 1630800,21.28; 1631400,21.2; 1632000,
            21.16; 1632600,21.12; 1633200,21.08; 1633800,21; 1634400,21; 1635000,20.96;
            1635600,20.87; 1636200,20.83; 1636800,20.83; 1637400,20.79; 1638000,20.75;
            1638600,20.75; 1639200,20.71; 1639800,20.68; 1640400,20.67; 1641000,20.67;
            1641600,20.63; 1642200,20.59; 1642800,20.6; 1643400,20.59; 1644000,20.55;
            1644600,20.52; 1645200,20.52; 1645800,20.51; 1646400,20.51; 1647000,20.46;
            1647600,20.46; 1648200,20.46; 1648800,20.51; 1649400,20.59; 1650000,20.63;
            1650600,20.63; 1651200,20.63; 1651800,20.63; 1652400,20.55; 1653000,20.55;
            1653600,20.51; 1654200,20.51; 1654800,20.46; 1655400,20.44; 1656000,20.38;
            1656600,20.38; 1657200,20.42; 1657800,20.42; 1658400,20.42; 1659000,20.42;
            1659600,20.38; 1660200,20.34; 1660800,20.34; 1661400,20.39; 1662000,20.46;
            1662600,20.52; 1663200,20.48; 1663800,20.42; 1664400,20.42; 1665000,20.38;
            1665600,20.38; 1666200,20.48; 1666800,20.67; 1667400,20.87; 1668000,20.96;
            1668600,21.04; 1669200,21.11; 1669800,21.24; 1670400,21.32; 1671000,21.47;
            1671600,21.56; 1672200,21.56; 1672800,21.7; 1673400,22.01; 1674000,22.26;
            1674600,22.42; 1675200,22.59; 1675800,22.7; 1676400,22.87; 1677000,22.99;
            1677600,23.07; 1678200,23.23; 1678800,23.38; 1679400,23.48; 1680000,23.6;
            1680600,23.68; 1681200,23.8; 1681800,23.84; 1682400,23.88; 1683000,23.92;
            1683600,23.9; 1684200,23.92; 1684800,23.92; 1685400,23.89; 1686000,23.88;
            1686600,23.96; 1687200,24.04; 1687800,24.12; 1688400,24.21; 1689000,24.21;
            1689600,24.18; 1690200,24.12; 1690800,24.09; 1691400,24.09; 1692000,24.09;
            1692600,24.09; 1693200,24.08; 1693800,24.09; 1694400,24.1; 1695000,24.05;
            1695600,23.96; 1696200,23.88; 1696800,23.88; 1697400,23.72; 1698000,23.76;
            1698600,23.8; 1699200,23.72; 1699800,23.62; 1700400,23.6; 1701000,23.52;
            1701600,23.52; 1702200,23.44; 1702800,23.4; 1703400,23.36; 1704000,23.36;
            1704600,23.23; 1705200,23.2; 1705800,23.15; 1706400,23.11; 1707000,23.04;
            1707600,23.03; 1708200,22.99; 1708800,22.94; 1709400,22.91; 1710000,22.84;
            1710600,22.7; 1711200,22.58; 1711800,22.46; 1712400,22.41; 1713000,22.3;
            1713600,22.22; 1714200,22.14; 1714800,22.05; 1715400,21.97; 1716000,21.89;
            1716600,21.81; 1717200,21.73; 1717800,21.68; 1718400,21.6; 1719000,21.53;
            1719600,21.5; 1720200,21.45; 1720800,21.39; 1721400,21.32; 1722000,21.28;
            1722600,21.24; 1723200,21.2; 1723800,21.16; 1724400,21.12; 1725000,21.08;
            1725600,21.04; 1726200,21; 1726800,20.96; 1727400,20.95; 1728000,20.92;
            1728600,20.87; 1729200,20.84; 1729800,20.83; 1730400,20.79; 1731000,20.79;
            1731600,20.75; 1732200,20.71; 1732800,20.72; 1733400,20.7; 1734000,20.67;
            1734600,20.65; 1735200,20.63; 1735800,20.63; 1736400,20.64; 1737000,20.59;
            1737600,20.59; 1738200,20.59; 1738800,20.55; 1739400,20.51; 1740000,20.52;
            1740600,20.51; 1741200,20.51; 1741800,20.46; 1742400,20.46; 1743000,20.46;
            1743600,20.48; 1744200,20.46; 1744800,20.42; 1745400,20.42; 1746000,20.41;
            1746600,20.38; 1747200,20.38; 1747800,20.38; 1748400,20.38; 1749000,20.34;
            1749600,20.34; 1750200,20.36; 1750800,20.34; 1751400,20.32; 1752000,20.3;
            1752600,20.3; 1753200,20.3; 1753800,20.38; 1754400,20.55; 1755000,20.67;
            1755600,20.79; 1756200,20.84; 1756800,20.87; 1757400,20.91; 1758000,21;
            1758600,21.09; 1759200,21.16; 1759800,21.24; 1760400,21.32; 1761000,21.36;
            1761600,21.45; 1762200,21.44; 1762800,21.58; 1763400,21.65; 1764000,21.78;
            1764600,21.74; 1765200,21.72; 1765800,21.81; 1766400,21.81; 1767000,21.97;
            1767600,21.94; 1768200,21.93; 1768800,22.05; 1769400,22.09; 1770000,22.17;
            1770600,22.18; 1771200,22.19; 1771800,22.09; 1772400,22.09; 1773000,22.13;
            1773600,22.14; 1774200,22.14; 1774800,22.13; 1775400,22.17; 1776000,22.14;
            1776600,22.18; 1777200,22.18; 1777800,22.22; 1778400,22.22; 1779000,22.22;
            1779600,22.23; 1780200,22.3; 1780800,22.38; 1781400,22.46; 1782000,22.5;
            1782600,22.47; 1783200,22.54; 1783800,22.58; 1784400,22.5; 1785000,22.54;
            1785600,22.59; 1786200,22.62; 1786800,22.7; 1787400,22.67; 1788000,22.66;
            1788600,22.58; 1789200,22.46; 1789800,22.39; 1790400,22.33; 1791000,22.3;
            1791600,22.22; 1792200,22.19; 1792800,22.1; 1793400,22.05; 1794000,21.98;
            1794600,21.96; 1795200,21.89; 1795800,21.81; 1796400,21.77; 1797000,21.73;
            1797600,21.65; 1798200,21.6; 1798800,21.56; 1799400,21.48; 1800000,21.44;
            1800600,21.4; 1801200,21.35; 1801800,21.28; 1802400,21.24; 1803000,21.2;
            1803600,21.16; 1804200,21.1; 1804800,21.08; 1805400,21; 1806000,21; 1806600,
            20.96; 1807200,20.91; 1807800,20.86; 1808400,20.83; 1809000,20.79; 1809600,
            20.79; 1810200,20.75; 1810800,20.71; 1811400,20.67; 1812000,20.63; 1812600,
            20.63; 1813200,20.59; 1813800,20.55; 1814400,20.51; 1815000,20.51; 1815600,
            20.48; 1816200,20.43; 1816800,20.42; 1817400,20.38; 1818000,20.34; 1818600,
            20.34; 1819200,20.3; 1819800,20.26; 1820400,20.26; 1821000,20.22; 1821600,
            20.21; 1822200,20.18; 1822800,20.15; 1823400,20.14; 1824000,20.14; 1824600,
            20.1; 1825200,20.1; 1825800,20.07; 1826400,20.06; 1827000,20.06; 1827600,
            20.02; 1828200,20.02; 1828800,20.02; 1829400,19.99; 1830000,19.98; 1830600,
            19.98; 1831200,19.97; 1831800,19.94; 1832400,19.94; 1833000,19.94; 1833600,
            19.9; 1834200,19.9; 1834800,19.9; 1835400,19.9; 1836000,19.88; 1836600,19.86;
            1837200,19.86; 1837800,19.86; 1838400,19.86; 1839000,19.82; 1839600,19.78;
            1840200,20; 1840800,20.18; 1841400,20.3; 1842000,20.46; 1842600,20.55; 1843200,
            20.67; 1843800,20.83; 1844400,20.91; 1845000,21.01; 1845600,21.16; 1846200,
            21.28; 1846800,21.4; 1847400,21.48; 1848000,21.57; 1848600,21.65; 1849200,
            21.77; 1849800,21.81; 1850400,21.93; 1851000,21.97; 1851600,22.02; 1852200,
            22.08; 1852800,22.12; 1853400,22.14; 1854000,22.26; 1854600,22.36; 1855200,
            22.42; 1855800,22.26; 1856400,22.18; 1857000,22.14; 1857600,22.1; 1858200,
            22.09; 1858800,22.1; 1859400,21.9; 1860000,22.04; 1860600,22.3; 1861200,
            22.46; 1861800,22.54; 1862400,22.58; 1863000,22.62; 1863600,22.7; 1864200,
            22.78; 1864800,23.05; 1865400,22.87; 1866000,22.91; 1866600,22.99; 1867200,
            22.99; 1867800,23.03; 1868400,23.03; 1869000,23.03; 1869600,22.95; 1870200,
            22.91; 1870800,22.87; 1871400,22.91; 1872000,22.95; 1872600,22.9; 1873200,
            22.83; 1873800,22.83; 1874400,22.78; 1875000,22.78; 1875600,22.74; 1876200,
            22.75; 1876800,22.62; 1877400,22.56; 1878000,22.46; 1878600,22.38; 1879200,
            22.3; 1879800,22.25; 1880400,22.17; 1881000,22.09; 1881600,22.01; 1882200,
            21.94; 1882800,21.85; 1883400,21.81; 1884000,21.73; 1884600,21.7; 1885200,
            21.61; 1885800,21.56; 1886400,21.49; 1887000,21.44; 1887600,21.4; 1888200,
            21.37; 1888800,21.33; 1889400,21.28; 1890000,21.21; 1890600,21.16; 1891200,
            21.12; 1891800,21.08; 1892400,21.07; 1893000,21.01; 1893600,20.97; 1894200,
            20.96; 1894800,20.91; 1895400,20.87; 1896000,20.88; 1896600,20.83; 1897200,
            20.82; 1897800,20.79; 1898400,20.75; 1899000,20.76; 1899600,20.71; 1900200,
            20.68; 1900800,20.68; 1901400,20.67; 1902000,20.65; 1902600,20.63; 1903200,
            20.61; 1903800,20.59; 1904400,20.55; 1905000,20.55; 1905600,20.54; 1906200,
            20.51; 1906800,20.51; 1907400,20.5; 1908000,20.46; 1908600,20.46; 1909200,
            20.46; 1909800,20.44; 1910400,20.42; 1911000,20.42; 1911600,20.38; 1912200,
            20.37; 1912800,20.34; 1913400,20.34; 1914000,20.34; 1914600,20.31; 1915200,
            20.3; 1915800,20.3; 1916400,20.3; 1917000,20.3; 1917600,20.27; 1918200,20.26;
            1918800,20.24; 1919400,20.23; 1920000,20.22; 1920600,20.22; 1921200,20.23;
            1921800,20.22; 1922400,20.22; 1923000,20.22; 1923600,20.23; 1924200,20.18;
            1924800,20.18; 1925400,20.18; 1926000,20.18; 1926600,20.18; 1927200,20.17;
            1927800,20.15; 1928400,20.14; 1929000,20.15; 1929600,20.14; 1930200,20.14;
            1930800,20.14; 1931400,20.15; 1932000,20.14; 1932600,20.18; 1933200,20.16;
            1933800,20.14; 1934400,20.18; 1935000,20.18; 1935600,20.18; 1936200,20.18;
            1936800,20.22; 1937400,20.22; 1938000,20.22; 1938600,20.22; 1939200,20.26;
            1939800,20.26; 1940400,20.3; 1941000,20.3; 1941600,20.34; 1942200,20.36;
            1942800,20.38; 1943400,20.42; 1944000,20.46; 1944600,20.46; 1945200,20.5;
            1945800,20.56; 1946400,20.59; 1947000,20.65; 1947600,20.67; 1948200,20.71;
            1948800,20.75; 1949400,20.81; 1950000,20.87; 1950600,20.97; 1951200,21.04;
            1951800,21.08; 1952400,21.12; 1953000,21.13; 1953600,21.17; 1954200,21.16;
            1954800,21.24; 1955400,21.3; 1956000,21.32; 1956600,21.2; 1957200,21.2;
            1957800,21.2; 1958400,21.16; 1959000,21.12; 1959600,21.08; 1960200,21.07;
            1960800,21; 1961400,20.96; 1962000,20.96; 1962600,20.91; 1963200,20.83;
            1963800,20.83; 1964400,20.8; 1965000,20.75; 1965600,20.71; 1966200,20.67;
            1966800,20.63; 1967400,20.59; 1968000,20.56; 1968600,20.55; 1969200,20.51;
            1969800,20.47; 1970400,20.46; 1971000,20.42; 1971600,20.42; 1972200,20.38;
            1972800,20.36; 1973400,20.33; 1974000,20.3; 1974600,20.26; 1975200,20.26;
            1975800,20.22; 1976400,20.21; 1977000,20.18; 1977600,20.14; 1978200,20.14;
            1978800,20.1; 1979400,20.06; 1980000,20.06; 1980600,20.02; 1981200,19.98;
            1981800,19.98; 1982400,19.94; 1983000,19.91; 1983600,19.9; 1984200,19.86;
            1984800,19.86; 1985400,19.82; 1986000,19.82; 1986600,19.78; 1987200,19.78;
            1987800,19.73; 1988400,19.7; 1989000,19.69; 1989600,19.69; 1990200,19.65;
            1990800,19.65; 1991400,19.65; 1992000,19.62; 1992600,19.59; 1993200,19.57;
            1993800,19.57; 1994400,19.57; 1995000,19.56; 1995600,19.53; 1996200,19.53;
            1996800,19.5; 1997400,19.49; 1998000,19.49; 1998600,19.48; 1999200,19.45;
            1999800,19.45; 2000400,19.45; 2001000,19.43; 2001600,19.41; 2002200,19.41;
            2002800,19.37; 2003400,19.37; 2004000,19.37; 2004600,19.37; 2005200,19.37;
            2005800,19.33; 2006400,19.34; 2007000,19.33; 2007600,19.34; 2008200,19.28;
            2008800,19.28; 2009400,19.28; 2010000,19.26; 2010600,19.25; 2011200,19.24;
            2011800,19.24; 2012400,19.22; 2013000,19.24; 2013600,19.22; 2014200,19.2;
            2014800,19.2; 2015400,19.21; 2016000,19.21; 2016600,19.2; 2017200,19.22;
            2017800,19.22; 2018400,19.24; 2019000,19.26; 2019600,19.24; 2020200,19.24;
            2020800,19.24; 2021400,19.28; 2022000,19.28; 2022600,19.34; 2023200,19.33;
            2023800,19.38; 2024400,19.38; 2025000,19.41; 2025600,19.41; 2026200,19.45;
            2026800,19.49; 2027400,19.53; 2028000,19.57; 2028600,19.61; 2029200,19.66;
            2029800,19.69; 2030400,19.73; 2031000,19.78; 2031600,19.82; 2032200,19.9;
            2032800,19.94; 2033400,19.98; 2034000,20.02; 2034600,20.1; 2035200,20.14;
            2035800,20.21; 2036400,20.5; 2037000,20.3; 2037600,20.38; 2038200,20.42;
            2038800,20.43; 2039400,20.49; 2040000,20.56; 2040600,20.63; 2041200,20.71;
            2041800,20.75; 2042400,20.79; 2043000,20.67; 2043600,20.67; 2044200,20.67;
            2044800,20.63; 2045400,20.59; 2046000,20.58; 2046600,20.55; 2047200,20.51;
            2047800,20.46; 2048400,20.42; 2049000,20.4; 2049600,20.34; 2050200,20.3;
            2050800,20.26; 2051400,20.27; 2052000,20.22; 2052600,20.18; 2053200,20.14;
            2053800,20.11; 2054400,20.06; 2055000,20.02; 2055600,20.02; 2056200,19.98;
            2056800,19.94; 2057400,19.9; 2058000,19.86; 2058600,19.86; 2059200,19.82;
            2059800,19.81; 2060400,19.78; 2061000,19.73; 2061600,19.73; 2062200,19.69;
            2062800,19.65; 2063400,19.65; 2064000,19.61; 2064600,19.59; 2065200,19.57;
            2065800,19.56; 2066400,19.53; 2067000,19.53; 2067600,19.52; 2068200,19.49;
            2068800,19.45; 2069400,19.45; 2070000,19.44; 2070600,19.41; 2071200,19.38;
            2071800,19.37; 2072400,19.37; 2073000,19.37; 2073600,19.33; 2074200,19.33;
            2074800,19.33; 2075400,19.28; 2076000,19.28; 2076600,19.26; 2077200,19.24;
            2077800,19.24; 2078400,19.24; 2079000,19.21; 2079600,19.2; 2080200,19.2;
            2080800,19.2; 2081400,19.2; 2082000,19.16; 2082600,19.16; 2083200,19.16;
            2083800,19.16; 2084400,19.12; 2085000,19.12; 2085600,19.12; 2086200,19.1;
            2086800,19.08; 2087400,19.08; 2088000,19.08; 2088600,19.04; 2089200,19.04;
            2089800,19.04; 2090400,19.04; 2091000,19; 2091600,19.04; 2092200,19.04;
            2092800,19.05; 2093400,19.04; 2094000,19.01; 2094600,19; 2095200,19; 2095800,
            19.01; 2096400,18.96; 2097000,18.96; 2097600,18.96; 2098200,18.96; 2098800,
            18.92; 2099400,19.18; 2100000,19.42; 2100600,19.53; 2101200,19.65; 2101800,
            19.81; 2102400,19.9; 2103000,20.02; 2103600,20.1; 2104200,20.22; 2104800,
            20.3; 2105400,20.42; 2106000,20.55; 2106600,20.63; 2107200,20.74; 2107800,
            20.79; 2108400,20.87; 2109000,20.98; 2109600,21.08; 2110200,21.16; 2110800,
            21.24; 2111400,21.32; 2112000,21.32; 2112600,21.44; 2113200,21.54; 2113800,
            21.65; 2114400,21.7; 2115000,21.69; 2115600,21.72; 2116200,21.73; 2116800,
            21.61; 2117400,21.68; 2118000,21.82; 2118600,21.48; 2119200,21.66; 2119800,
            21.69; 2120400,21.72; 2121000,21.76; 2121600,21.81; 2122200,21.81; 2122800,
            21.85; 2123400,21.9; 2124000,21.93; 2124600,21.97; 2125200,22.05; 2125800,
            22.14; 2126400,22.14; 2127000,22.22; 2127600,22.27; 2128200,22.34; 2128800,
            22.26; 2129400,22.38; 2130000,22.46; 2130600,22.45; 2131200,22.38; 2131800,
            22.3; 2132400,22.17; 2133000,22.05; 2133600,21.97; 2134200,21.97; 2134800,
            22.05; 2135400,22.05; 2136000,22.06; 2136600,22.09; 2137200,22.09; 2137800,
            22.13; 2138400,22.14; 2139000,22.09; 2139600,22.05; 2140200,22.09; 2140800,
            22.01; 2141400,21.93; 2142000,21.85; 2142600,21.81; 2143200,21.73; 2143800,
            21.65; 2144400,21.6; 2145000,21.52; 2145600,21.44; 2146200,21.36; 2146800,
            21.32; 2147400,21.23; 2148000,21.16; 2148600,21.08; 2149200,21.04; 2149800,
            20.96; 2150400,20.89; 2151000,20.83; 2151600,20.79; 2152200,20.75; 2152800,
            20.67; 2153400,20.63; 2154000,20.63; 2154600,20.55; 2155200,20.51; 2155800,
            20.46; 2156400,20.46; 2157000,20.38; 2157600,20.34; 2158200,20.34; 2158800,
            20.3; 2159400,20.27; 2160000,20.26; 2160600,20.22; 2161200,20.18; 2161800,
            20.19; 2162400,20.16; 2163000,20.1; 2163600,20.1; 2164200,20.09; 2164800,
            20.02; 2165400,20.02; 2166000,20.02; 2166600,19.98; 2167200,19.98; 2167800,
            19.94; 2168400,19.94; 2169000,19.9; 2169600,19.9; 2170200,19.88; 2170800,
            19.86; 2171400,19.86; 2172000,19.82; 2172600,19.82; 2173200,19.82; 2173800,
            19.82; 2174400,19.78; 2175000,19.78; 2175600,19.77; 2176200,19.74; 2176800,
            19.73; 2177400,19.73; 2178000,19.71; 2178600,19.69; 2179200,19.69; 2179800,
            19.69; 2180400,19.7; 2181000,19.69; 2181600,19.69; 2182200,19.69; 2182800,
            19.7; 2183400,19.66; 2184000,19.69; 2184600,19.69; 2185200,19.78; 2185800,
            20.02; 2186400,20.14; 2187000,20.26; 2187600,20.38; 2188200,20.5; 2188800,
            20.59; 2189400,20.67; 2190000,20.79; 2190600,20.83; 2191200,20.87; 2191800,
            20.95; 2192400,21.04; 2193000,21.16; 2193600,21.16; 2194200,21.24; 2194800,
            21.28; 2195400,21.36; 2196000,21.44; 2196600,21.52; 2197200,21.64; 2197800,
            21.77; 2198400,21.93; 2199000,22.05; 2199600,22.09; 2200200,22.22; 2200800,
            22.34; 2201400,22.3; 2202000,22.26; 2202600,22.26; 2203200,22.33; 2203800,
            22.35; 2204400,22.38; 2205000,22.38; 2205600,22.58; 2206200,22.66; 2206800,
            22.62; 2207400,22.78; 2208000,22.83; 2208600,22.87; 2209200,22.92; 2209800,
            22.84; 2210400,22.95; 2211000,22.91; 2211600,22.85; 2212200,22.78; 2212800,
            22.83; 2213400,22.82; 2214000,22.78; 2214600,22.83; 2215200,22.91; 2215800,
            22.91; 2216400,22.91; 2217000,22.91; 2217600,22.95; 2218200,22.99; 2218800,
            23.03; 2219400,22.99; 2220000,23.03; 2220600,22.99; 2221200,22.94; 2221800,
            22.87; 2222400,22.88; 2223000,22.84; 2223600,22.83; 2224200,22.7; 2224800,
            22.64; 2225400,22.54; 2226000,22.46; 2226600,22.39; 2227200,22.3; 2227800,
            22.22; 2228400,22.14; 2229000,22.05; 2229600,22.01; 2230200,21.93; 2230800,
            21.85; 2231400,21.77; 2232000,21.73; 2232600,21.65; 2233200,21.61; 2233800,
            21.52; 2234400,21.44; 2235000,21.4; 2235600,21.36; 2236200,21.28; 2236800,
            21.24; 2237400,21.2; 2238000,21.12; 2238600,21.08; 2239200,21.04; 2239800,
            21; 2240400,20.96; 2241000,20.91; 2241600,20.87; 2242200,20.83; 2242800,
            20.81; 2243400,20.78; 2244000,20.75; 2244600,20.71; 2245200,20.68; 2245800,
            20.68; 2246400,20.67; 2247000,20.63; 2247600,20.6; 2248200,20.56; 2248800,
            20.55; 2249400,20.51; 2250000,20.51; 2250600,20.46; 2251200,20.46; 2251800,
            20.42; 2252400,20.42; 2253000,20.38; 2253600,20.36; 2254200,20.34; 2254800,
            20.3; 2255400,20.3; 2256000,20.28; 2256600,20.26; 2257200,20.26; 2257800,
            20.26; 2258400,20.26; 2259000,20.22; 2259600,20.23; 2260200,20.22; 2260800,
            20.22; 2261400,20.22; 2262000,20.18; 2262600,20.18; 2263200,20.18; 2263800,
            20.18; 2264400,20.18; 2265000,20.18; 2265600,20.18; 2266200,20.14; 2266800,
            20.14; 2267400,20.14; 2268000,20.14; 2268600,20.14; 2269200,20.14; 2269800,
            20.15; 2270400,20.13; 2271000,20.26; 2271600,20.46; 2272200,20.64; 2272800,
            20.75; 2273400,20.87; 2274000,20.96; 2274600,21.04; 2275200,21.2; 2275800,
            21.37; 2276400,21.44; 2277000,21.6; 2277600,21.73; 2278200,21.85; 2278800,
            21.93; 2279400,21.93; 2280000,21.93; 2280600,22.02; 2281200,22.09; 2281800,
            22.09; 2282400,22.13; 2283000,22.26; 2283600,22.26; 2284200,22.26; 2284800,
            22.31; 2285400,22.41; 2286000,22.45; 2286600,22.62; 2287200,22.59; 2287800,
            22.58; 2288400,22.58; 2289000,22.46; 2289600,22.46; 2290200,22.72; 2290800,
            22.58; 2291400,22.7; 2292000,22.83; 2292600,23; 2293200,23.12; 2293800,23.15;
            2294400,23.19; 2295000,23.32; 2295600,23.28; 2296200,23.39; 2296800,23.44;
            2297400,23.52; 2298000,23.55; 2298600,23.6; 2299200,23.68; 2299800,23.68;
            2300400,23.68; 2301000,23.72; 2301600,23.73; 2302200,23.68; 2302800,23.68;
            2303400,23.68; 2304000,23.72; 2304600,23.72; 2305200,23.68; 2305800,23.57;
            2306400,23.52; 2307000,23.48; 2307600,23.44; 2308200,23.4; 2308800,23.36;
            2309400,23.31; 2310000,23.28; 2310600,23.27; 2311200,23.27; 2311800,23.15;
            2312400,23.11; 2313000,23.03; 2313600,22.95; 2314200,22.87; 2314800,22.78;
            2315400,22.7; 2316000,22.62; 2316600,22.5; 2317200,22.42; 2317800,22.34;
            2318400,22.26; 2319000,22.23; 2319600,22.14; 2320200,22.09; 2320800,22.01;
            2321400,21.93; 2322000,21.89; 2322600,21.81; 2323200,21.73; 2323800,21.69;
            2324400,21.6; 2325000,21.58; 2325600,21.52; 2326200,21.48; 2326800,21.4;
            2327400,21.38; 2328000,21.32; 2328600,21.3; 2329200,21.25; 2329800,21.2;
            2330400,21.16; 2331000,21.12; 2331600,21.12; 2332200,21.09; 2332800,21.04;
            2333400,21; 2334000,21.21; 2334600,20.97; 2335200,20.92; 2335800,20.89;
            2336400,20.87; 2337000,20.83; 2337600,20.83; 2338200,20.79; 2338800,20.79;
            2339400,20.78; 2340000,20.75; 2340600,20.74; 2341200,20.71; 2341800,20.71;
            2342400,20.67; 2343000,20.67; 2343600,20.67; 2344200,20.67; 2344800,20.63;
            2345400,20.63; 2346000,20.63; 2346600,20.61; 2347200,20.59; 2347800,20.59;
            2348400,20.59; 2349000,20.59; 2349600,20.58; 2350200,20.55; 2350800,20.55;
            2351400,20.55; 2352000,20.55; 2352600,20.55; 2353200,20.54; 2353800,20.51;
            2354400,20.51; 2355000,20.51; 2355600,20.51; 2356200,20.51; 2356800,20.51;
            2357400,20.51; 2358000,20.72; 2358600,20.8; 2359200,20.96; 2359800,21.08;
            2360400,21.2; 2361000,21.32; 2361600,21.44; 2362200,21.57; 2362800,21.68;
            2363400,21.69; 2364000,21.72; 2364600,21.77; 2365200,21.85; 2365800,21.89;
            2366400,21.97; 2367000,22.02; 2367600,22.09; 2368200,22.14; 2368800,22.18;
            2369400,22.26; 2370000,22.26; 2370600,22.3; 2371200,22.3; 2371800,22.38;
            2372400,22.47; 2373000,22.56; 2373600,22.57; 2374200,22.66; 2374800,22.68;
            2375400,22.7; 2376000,22.72; 2376600,22.66; 2377200,22.66; 2377800,22.7;
            2378400,22.7; 2379000,22.74; 2379600,22.88; 2380200,23.03; 2380800,23.16;
            2381400,23.15; 2382000,23.17; 2382600,23.32; 2383200,23.44; 2383800,23.28;
            2384400,23.07; 2385000,23.03; 2385600,22.95; 2386200,22.91; 2386800,23;
            2387400,23.07; 2388000,23.03; 2388600,23.03; 2389200,22.99; 2389800,22.99;
            2390400,23.12; 2391000,23.03; 2391600,22.91; 2392200,23.14; 2392800,23.27;
            2393400,23.32; 2394000,23.4; 2394600,23.44; 2395200,23.4; 2395800,23.37;
            2396400,23.27; 2397000,23.19; 2397600,23.11; 2398200,23.03; 2398800,22.95;
            2399400,22.91; 2400000,22.83; 2400600,22.74; 2401200,22.7; 2401800,22.64;
            2402400,22.58; 2403000,22.5; 2403600,22.46; 2404200,22.38; 2404800,22.34;
            2405400,22.26; 2406000,22.22; 2406600,22.19; 2407200,22.11; 2407800,22.09;
            2408400,22.01; 2409000,21.98; 2409600,21.93; 2410200,21.89; 2410800,21.85;
            2411400,21.81; 2412000,21.78; 2412600,21.73; 2413200,21.72; 2413800,21.65;
            2414400,21.65; 2415000,21.6; 2415600,21.6; 2416200,21.56; 2416800,21.53;
            2417400,21.52; 2418000,21.48; 2418600,21.48; 2419200,21.48; 2419800,21.46;
            2420400,21.44; 2421000,21.41; 2421600,21.4; 2422200,21.36; 2422800,21.36;
            2423400,21.36; 2424000,21.32; 2424600,21.32; 2425200,21.32; 2425800,21.32;
            2426400,21.32; 2427000,21.28; 2427600,21.24; 2428200,21.24; 2428800,21.24;
            2429400,21.23; 2430000,21.22; 2430600,21.23; 2431200,21.24; 2431800,21.2;
            2432400,21.2; 2433000,21.21; 2433600,21.2; 2434200,21.2; 2434800,21.2; 2435400,
            21.2; 2436000,21.2; 2436600,21.2; 2437200,21.16; 2437800,21.16; 2438400,
            21.18; 2439000,21.16; 2439600,21.16; 2440200,21.16; 2440800,21.16; 2441400,
            21.16; 2442000,21.16; 2442600,21.16; 2443200,21.15; 2443800,21.19; 2444400,
            21.32; 2445000,21.44; 2445600,21.52; 2446200,21.64; 2446800,21.73; 2447400,
            21.81; 2448000,21.93; 2448600,22.09; 2449200,22.18; 2449800,22.29; 2450400,
            22.38; 2451000,22.46; 2451600,22.5; 2452200,22.58; 2452800,22.7; 2453400,
            22.74; 2454000,22.78; 2454600,22.87; 2455200,22.95; 2455800,23.02; 2456400,
            23.07; 2457000,23.11; 2457600,23.19; 2458200,23.19; 2458800,23.2; 2459400,
            23.26; 2460000,23.27; 2460600,23.27; 2461200,23.27; 2461800,23.23; 2462400,
            23.31; 2463000,23.32; 2463600,23.32; 2464200,23.32; 2464800,23.32; 2465400,
            23.32; 2466000,23.4; 2466600,23.36; 2467200,23.35; 2467800,23.47; 2468400,
            23.48; 2469000,23.44; 2469600,23.48; 2470200,23.56; 2470800,23.6; 2471400,
            23.65; 2472000,23.64; 2472600,23.64; 2473200,23.74; 2473800,23.77; 2474400,
            23.64; 2475000,23.64; 2475600,23.68; 2476200,23.73; 2476800,23.68; 2477400,
            23.68; 2478000,23.68; 2478600,23.68; 2479200,23.71; 2479800,23.6; 2480400,
            23.61; 2481000,23.72; 2481600,23.74; 2482200,23.73; 2482800,23.71; 2483400,
            23.56; 2484000,23.48; 2484600,23.4; 2485200,23.32; 2485800,23.27; 2486400,
            23.15; 2487000,23.11; 2487600,23.03; 2488200,22.95; 2488800,22.9; 2489400,
            22.8; 2490000,22.74; 2490600,22.7; 2491200,22.62; 2491800,22.54; 2492400,
            22.47; 2493000,22.43; 2493600,22.34; 2494200,22.27; 2494800,22.22; 2495400,
            22.18; 2496000,22.09; 2496600,22.05; 2497200,22.01; 2497800,21.97; 2498400,
            21.89; 2499000,21.85; 2499600,21.81; 2500200,21.8; 2500800,21.76; 2501400,
            21.73; 2502000,21.69; 2502600,21.65; 2503200,21.64; 2503800,21.6; 2504400,
            21.58; 2505000,21.53; 2505600,21.52; 2506200,21.51; 2506800,21.48; 2507400,
            21.48; 2508000,21.44; 2508600,21.42; 2509200,21.4; 2509800,21.4; 2510400,
            21.37; 2511000,21.36; 2511600,21.32; 2512200,21.32; 2512800,21.32; 2513400,
            21.28; 2514000,21.28; 2514600,21.24; 2515200,21.24; 2515800,21.21; 2516400,
            21.2; 2517000,21.19; 2517600,21.16; 2518200,21.15; 2518800,21.12; 2519400,
            21.12; 2520000,21.12; 2520600,21.08; 2521200,21.08; 2521800,21.07; 2522400,
            21.04; 2523000,21.05; 2523600,21.01; 2524200,21; 2524800,20.98; 2525400,
            20.96; 2526000,20.96; 2526600,20.92; 2527200,20.91; 2527800,20.92; 2528400,
            20.88; 2529000,20.87; 2529600,20.87; 2530200,20.83; 2530800,20.83; 2531400,
            20.83; 2532000,20.83; 2532600,20.83; 2533200,20.8; 2533800,20.8; 2534400,
            20.79; 2535000,20.79; 2535600,20.79; 2536200,20.79; 2536800,20.8; 2537400,
            20.8; 2538000,20.79; 2538600,20.84; 2539200,20.83; 2539800,20.83; 2540400,
            20.83; 2541000,20.87; 2541600,20.87; 2542200,20.91; 2542800,20.96; 2543400,
            21; 2544000,21; 2544600,21; 2545200,21.04; 2545800,21.04; 2546400,21.08;
            2547000,21.12; 2547600,21.12; 2548200,21.12; 2548800,21.12; 2549400,21.16;
            2550000,21.16; 2550600,21.2; 2551200,21.28; 2551800,21.32; 2552400,21.4;
            2553000,21.44; 2553600,21.49; 2554200,21.52; 2554800,21.56; 2555400,21.63;
            2556000,21.64; 2556600,21.65; 2557200,21.74; 2557800,21.71; 2558400,21.69;
            2559000,21.69; 2559600,21.68; 2560200,21.65; 2560800,21.6; 2561400,21.56;
            2562000,21.56; 2562600,21.52; 2563200,21.5; 2563800,21.48; 2564400,21.44;
            2565000,21.44; 2565600,21.43; 2566200,21.4; 2566800,21.38; 2567400,21.32;
            2568000,21.32; 2568600,21.32; 2569200,21.31; 2569800,21.28; 2570400,21.28;
            2571000,21.27; 2571600,21.24; 2572200,21.2; 2572800,21.2; 2573400,21.2;
            2574000,21.16; 2574600,21.16; 2575200,21.12; 2575800,21.12; 2576400,21.08;
            2577000,21.07; 2577600,21.04; 2578200,21.03; 2578800,21; 2579400,21; 2580000,
            20.96; 2580600,20.96; 2581200,20.96; 2581800,20.91; 2582400,20.91; 2583000,
            20.91; 2583600,20.87; 2584200,20.87; 2584800,20.83; 2585400,20.83; 2586000,
            20.83; 2586600,20.83; 2587200,20.79; 2587800,20.79; 2588400,20.79; 2589000,
            20.76; 2589600,20.75; 2590200,20.76; 2590800,20.71; 2591400,20.71; 2592000,
            20.68; 2592600,20.67; 2593200,20.67; 2593800,20.67; 2594400,20.63; 2595000,
            20.63; 2595600,20.63; 2596200,20.63; 2596800,20.63; 2597400,20.64; 2598000,
            20.6; 2598600,20.59; 2599200,20.59; 2599800,20.57; 2600400,20.55; 2601000,
            20.55; 2601600,20.55; 2602200,20.55; 2602800,20.55; 2603400,20.54; 2604000,
            20.51; 2604600,20.51; 2605200,20.51; 2605800,20.51; 2606400,20.51; 2607000,
            20.51; 2607600,20.5; 2608200,20.52; 2608800,20.5; 2609400,20.49; 2610000,
            20.46; 2610600,20.46; 2611200,20.48; 2611800,20.46; 2612400,20.46; 2613000,
            20.46; 2613600,20.46; 2614200,20.46; 2614800,20.46; 2615400,20.46; 2616000,
            20.43; 2616600,20.42; 2617200,20.44; 2617800,20.44; 2618400,20.44; 2619000,
            20.42; 2619600,20.42; 2620200,20.42; 2620800,20.44; 2621400,20.42; 2622000,
            20.42; 2622600,20.42; 2623200,20.42; 2623800,20.42; 2624400,20.42; 2625000,
            20.42; 2625600,20.45; 2626200,20.46; 2626800,20.46; 2627400,20.46; 2628000,
            20.46; 2628600,20.46; 2629200,20.46; 2629800,20.46; 2630400,20.46; 2631000,
            20.5; 2631600,20.5; 2632200,20.46; 2632800,20.46; 2633400,20.46; 2634000,
            20.46; 2634600,20.46; 2635200,20.46; 2635800,20.46; 2636400,20.46; 2637000,
            20.43; 2637600,20.46; 2638200,20.46; 2638800,20.46; 2639400,20.46; 2640000,
            20.46; 2640600,20.46; 2641200,20.46; 2641800,20.46; 2642400,20.46; 2643000,
            20.46; 2643600,20.46; 2644200,20.46; 2644800,20.46; 2645400,20.45; 2646000,
            20.42; 2646600,20.42; 2647200,20.42; 2647800,20.42; 2648400,20.42; 2649000,
            20.42; 2649600,20.42; 2650200,20.41; 2650800,20.42; 2651400,20.38; 2652000,
            20.38; 2652600,20.38; 2653200,20.4; 2653800,20.34; 2654400,20.34; 2655000,
            20.34; 2655600,20.34; 2656200,20.34; 2656800,20.34; 2657400,20.35; 2658000,
            20.32; 2658600,20.33; 2659200,20.3; 2659800,20.31; 2660400,20.32; 2661000,
            20.32; 2661600,20.3; 2662200,20.3; 2662800,20.3; 2663400,20.3; 2664000,20.3;
            2664600,20.3; 2665200,20.3; 2665800,20.3; 2666400,20.3; 2667000,20.3; 2667600,
            20.3; 2668200,20.3; 2668800,20.3; 2669400,20.32; 2670000,20.3; 2670600,20.3;
            2671200,20.31; 2671800,20.3; 2672400,20.3; 2673000,20.3; 2673600,20.3; 2674200,
            20.33; 2674800,20.32; 2675400,20.34; 2676000,20.34; 2676600,20.33; 2677200,
            20.36; 2677800,20.34; 2678400,20.34; 2679000,20.34; 2679600,20.34; 2680200,
            20.34; 2680800,20.34; 2681400,20.34; 2682000,20.34; 2682600,20.34; 2683200,
            20.34; 2683800,20.34; 2684400,20.34; 2685000,20.34; 2685600,20.34; 2686200,
            20.34; 2686800,20.34; 2687400,20.34; 2688000,20.34; 2688600,20.34; 2689200,
            20.34; 2689800,20.34; 2690400,20.34; 2691000,20.34; 2691600,20.34; 2692200,
            20.34; 2692800,20.34; 2693400,20.34; 2694000,20.34; 2694600,20.34; 2695200,
            20.34; 2695800,20.35; 2696400,20.38; 2697000,20.38; 2697600,20.38; 2698200,
            20.38; 2698800,20.39; 2699400,20.42; 2700000,20.42; 2700600,20.42; 2701200,
            20.42; 2701800,20.42; 2702400,20.39; 2703000,20.55; 2703600,20.75; 2704200,
            20.87; 2704800,21.01; 2705400,21.12; 2706000,21.2; 2706600,21.32; 2707200,
            21.43; 2707800,21.58; 2708400,21.64; 2709000,21.6; 2709600,21.65; 2710200,
            21.69; 2710800,21.78; 2711400,21.93; 2712000,22.01; 2712600,22.05; 2713200,
            22.14; 2713800,22.17; 2714400,22.29; 2715000,22.34; 2715600,22.41; 2716200,
            22.47; 2716800,22.7; 2717400,22.82; 2718000,22.91; 2718600,22.95; 2719200,
            22.99; 2719800,23; 2720400,22.91; 2721000,22.83; 2721600,22.62; 2722200,
            21.8; 2722800,22.16; 2723400,22.14; 2724000,22.42; 2724600,22.54; 2725200,
            22.58; 2725800,22.72; 2726400,22.8; 2727000,22.79; 2727600,22.91; 2728200,
            23.02; 2728800,23.15; 2729400,23.23; 2730000,23.27; 2730600,23.32; 2731200,
            23.27; 2731800,23.39; 2732400,23.44; 2733000,23.49; 2733600,23.52; 2734200,
            23.57; 2734800,23.6; 2735400,23.6; 2736000,23.6; 2736600,23.52; 2737200,
            23.52; 2737800,23.4; 2738400,23.27; 2739000,23.24; 2739600,23.27; 2740200,
            23.24; 2740800,23.23; 2741400,23.19; 2742000,23.16; 2742600,23.15; 2743200,
            23.12; 2743800,23.11; 2744400,23.07; 2745000,22.99; 2745600,22.91; 2746200,
            22.8; 2746800,22.74; 2747400,22.66; 2748000,22.58; 2748600,22.5; 2749200,
            22.42; 2749800,22.38; 2750400,22.3; 2751000,22.26; 2751600,22.18; 2752200,
            22.09; 2752800,22.05; 2753400,22.02; 2754000,21.93; 2754600,21.89; 2755200,
            21.84; 2755800,21.77; 2756400,21.73; 2757000,21.69; 2757600,21.65; 2758200,
            21.6; 2758800,21.56; 2759400,21.52; 2760000,21.48; 2760600,21.44; 2761200,
            21.4; 2761800,21.36; 2762400,21.32; 2763000,21.32; 2763600,21.28; 2764200,
            21.28; 2764800,21.24; 2765400,21.2; 2766000,21.2; 2766600,21.16; 2767200,
            21.12; 2767800,21.12; 2768400,21.12; 2769000,21.08; 2769600,21.09; 2770200,
            21.07; 2770800,21.04; 2771400,21.04; 2772000,21.01; 2772600,21; 2773200,
            20.99; 2773800,20.96; 2774400,20.96; 2775000,20.96; 2775600,20.96; 2776200,
            20.96; 2776800,20.95; 2777400,20.95; 2778000,20.94; 2778600,20.92; 2779200,
            20.92; 2779800,20.91; 2780400,20.91; 2781000,20.92; 2781600,20.91; 2782200,
            20.91; 2782800,20.91; 2783400,20.9; 2784000,20.89; 2784600,20.87; 2785200,
            20.88; 2785800,20.87; 2786400,20.87; 2787000,20.87; 2787600,20.88; 2788200,
            20.87; 2788800,20.87; 2789400,20.96; 2790000,21.16; 2790600,21.3; 2791200,
            21.36; 2791800,21.44; 2792400,21.49; 2793000,21.65; 2793600,21.81; 2794200,
            21.86; 2794800,21.93; 2795400,21.97; 2796000,22.06; 2796600,22.14; 2797200,
            22.24; 2797800,22.3; 2798400,22.38; 2799000,22.44; 2799600,22.5; 2800200,
            22.55; 2800800,22.62; 2801400,22.66; 2802000,22.64; 2802600,22.78; 2803200,
            22.87; 2803800,22.91; 2804400,22.95; 2805000,23.07; 2805600,23.08; 2806200,
            23.07; 2806800,23.11; 2807400,23.11; 2808000,23.15; 2808600,23.15; 2809200,
            23.31; 2809800,23.4; 2810400,23.55; 2811000,23.6; 2811600,23.68; 2812200,
            23.69; 2812800,23.72; 2813400,23.75; 2814000,23.8; 2814600,23.8; 2815200,
            23.84; 2815800,23.9; 2816400,23.92; 2817000,23.91; 2817600,24.01; 2818200,
            24.01; 2818800,23.94; 2819400,23.88; 2820000,23.84; 2820600,23.88; 2821200,
            23.94; 2821800,24.01; 2822400,24; 2823000,24.02; 2823600,24.02; 2824200,
            23.96; 2824800,24.01; 2825400,23.96; 2826000,23.85; 2826600,23.72; 2827200,
            23.72; 2827800,23.67; 2828400,23.71; 2829000,23.68; 2829600,23.56; 2830200,
            23.44; 2830800,23.4; 2831400,23.32; 2832000,23.23; 2832600,23.15; 2833200,
            23.11; 2833800,23.03; 2834400,22.95; 2835000,22.91; 2835600,22.82; 2836200,
            22.75; 2836800,22.69; 2837400,22.62; 2838000,22.54; 2838600,22.5; 2839200,
            22.42; 2839800,22.39; 2840400,22.3; 2841000,22.26; 2841600,22.22; 2842200,
            22.18; 2842800,22.1; 2843400,22.05; 2844000,22.01; 2844600,21.97; 2845200,
            21.94; 2845800,21.89; 2846400,21.85; 2847000,21.81; 2847600,21.8; 2848200,
            21.73; 2848800,21.74; 2849400,21.69; 2850000,21.65; 2850600,21.64; 2851200,
            21.6; 2851800,21.6; 2852400,21.56; 2853000,21.56; 2853600,21.52; 2854200,
            21.5; 2854800,21.48; 2855400,21.44; 2856000,21.44; 2856600,21.46; 2857200,
            21.42; 2857800,21.4; 2858400,21.4; 2859000,21.38; 2859600,21.36; 2860200,
            21.37; 2860800,21.32; 2861400,21.32; 2862000,21.32; 2862600,21.3; 2863200,
            21.28; 2863800,21.28; 2864400,21.26; 2865000,21.25; 2865600,21.24; 2866200,
            21.24; 2866800,21.22; 2867400,21.2; 2868000,21.2; 2868600,21.2; 2869200,
            21.16; 2869800,21.16; 2870400,21.16; 2871000,21.16; 2871600,21.16; 2872200,
            21.16; 2872800,21.16; 2873400,21.16; 2874000,21.17; 2874600,21.16; 2875200,
            21.16; 2875800,21.16; 2876400,21.16; 2877000,21.36; 2877600,21.52; 2878200,
            21.65; 2878800,21.77; 2879400,21.89; 2880000,21.97; 2880600,22.01; 2881200,
            22.08; 2881800,22.22; 2882400,22.3; 2883000,22.42; 2883600,22.49; 2884200,
            22.5; 2884800,22.54; 2885400,22.62; 2886000,22.62; 2886600,22.74; 2887200,
            22.78; 2887800,22.88; 2888400,22.87; 2889000,22.92; 2889600,22.95; 2890200,
            23; 2890800,23.03; 2891400,23.11; 2892000,23.11; 2892600,23.11; 2893200,
            23.15; 2893800,23.16; 2894400,23.15; 2895000,23.16; 2895600,23.19; 2896200,
            23.19; 2896800,23.15; 2897400,23.11; 2898000,23.11; 2898600,23.03; 2899200,
            23.06; 2899800,23.19; 2900400,23.23; 2901000,23.2; 2901600,23.25; 2902200,
            23.28; 2902800,23.2; 2903400,23.36; 2904000,23.4; 2904600,23.4; 2905200,
            23.44; 2905800,23.44; 2906400,23.48; 2907000,23.47; 2907600,23.48; 2908200,
            23.48; 2908800,23.52; 2909400,23.57; 2910000,23.55; 2910600,23.53; 2911200,
            23.49; 2911800,23.52; 2912400,23.56; 2913000,23.53; 2913600,23.44; 2914200,
            23.36; 2914800,23.27; 2915400,23.23; 2916000,23.23; 2916600,23.19; 2917200,
            23.14; 2917800,23.11; 2918400,23.03; 2919000,22.92; 2919600,22.83; 2920200,
            22.74; 2920800,22.64; 2921400,22.58; 2922000,22.5; 2922600,22.42; 2923200,
            22.38; 2923800,22.3; 2924400,22.22; 2925000,22.14; 2925600,22.08; 2926200,
            22.01; 2926800,21.97; 2927400,21.89; 2928000,21.81; 2928600,21.77; 2929200,
            21.73; 2929800,21.65; 2930400,21.61; 2931000,21.6; 2931600,21.56; 2932200,
            21.48; 2932800,21.46; 2933400,21.4; 2934000,21.4; 2934600,21.36; 2935200,
            21.32; 2935800,21.28; 2936400,21.28; 2937000,21.24; 2937600,21.2; 2938200,
            21.16; 2938800,21.16; 2939400,21.12; 2940000,21.12; 2940600,21.08; 2941200,
            21.08; 2941800,21.07; 2942400,21.04; 2943000,21; 2943600,21; 2944200,20.99;
            2944800,20.96; 2945400,20.96; 2946000,20.94; 2946600,20.91; 2947200,20.91;
            2947800,20.88; 2948400,20.87; 2949000,20.88; 2949600,20.87; 2950200,20.87;
            2950800,20.83; 2951400,20.84; 2952000,20.83; 2952600,20.83; 2953200,20.83;
            2953800,20.83; 2954400,20.83; 2955000,20.8; 2955600,20.79; 2956200,20.82;
            2956800,20.79; 2957400,20.8; 2958000,20.79; 2958600,20.79; 2959200,20.79;
            2959800,20.79; 2960400,20.8; 2961000,20.79; 2961600,20.76; 2962200,20.75;
            2962800,20.75; 2963400,20.99; 2964000,21.16; 2964600,21.44; 2965200,21.52;
            2965800,21.56; 2966400,21.65; 2967000,21.65; 2967600,21.69; 2968200,21.77;
            2968800,21.81; 2969400,21.93; 2970000,22.09; 2970600,22.18; 2971200,22.26;
            2971800,22.39; 2972400,22.5; 2973000,22.58; 2973600,22.62; 2974200,22.7;
            2974800,22.78; 2975400,22.82; 2976000,22.86; 2976600,22.94; 2977200,23.03;
            2977800,23.03; 2978400,23.12; 2979000,23.15; 2979600,23.15; 2980200,23.15;
            2980800,23.11; 2981400,23.07; 2982000,23.03; 2982600,23.03; 2983200,23.08;
            2983800,23.18; 2984400,23.23; 2985000,23.24; 2985600,23.23; 2986200,23.2;
            2986800,23.24; 2987400,23.32; 2988000,23.4; 2988600,23.4; 2989200,23.4;
            2989800,23.48; 2990400,23.56; 2991000,23.55; 2991600,23.56; 2992200,23.56;
            2992800,23.6; 2993400,23.57; 2994000,23.56; 2994600,23.56; 2995200,23.6;
            2995800,23.64; 2996400,23.64; 2997000,23.6; 2997600,23.56; 2998200,23.52;
            2998800,23.48; 2999400,23.4; 3000000,23.27; 3000600,23.16; 3001200,23.11;
            3001800,23.03; 3002400,22.95; 3003000,22.9; 3003600,22.83; 3004200,22.78;
            3004800,22.7; 3005400,22.62; 3006000,22.58; 3006600,22.5; 3007200,22.42;
            3007800,22.38; 3008400,22.3; 3009000,22.26; 3009600,22.22; 3010200,22.14;
            3010800,22.09; 3011400,22.05; 3012000,22.01; 3012600,21.94; 3013200,21.89;
            3013800,21.85; 3014400,21.81; 3015000,21.78; 3015600,21.73; 3016200,21.69;
            3016800,21.65; 3017400,21.6; 3018000,21.56; 3018600,21.52; 3019200,21.52;
            3019800,21.48; 3020400,21.44; 3021000,21.42; 3021600,21.4; 3022200,21.36;
            3022800,21.33; 3023400,21.32; 3024000,21.32; 3024600,21.29; 3025200,21.24;
            3025800,21.24; 3026400,21.2; 3027000,21.2; 3027600,21.2; 3028200,21.16;
            3028800,21.16; 3029400,21.13; 3030000,21.12; 3030600,21.12; 3031200,21.13;
            3031800,21.08; 3032400,21.08; 3033000,21.08; 3033600,21.08; 3034200,21.07;
            3034800,21.04; 3035400,21.04; 3036000,21.05; 3036600,21.04; 3037200,21.04;
            3037800,21.04; 3038400,21.03; 3039000,21.02; 3039600,21; 3040200,21; 3040800,
            21; 3041400,21; 3042000,21; 3042600,20.99; 3043200,21; 3043800,20.98; 3044400,
            20.96; 3045000,20.96; 3045600,20.96; 3046200,20.97; 3046800,20.96; 3047400,
            20.96; 3048000,20.96; 3048600,20.95; 3049200,20.92; 3049800,20.91; 3050400,
            20.96; 3051000,21.12; 3051600,21.28; 3052200,21.4; 3052800,21.56; 3053400,
            21.65; 3054000,21.6; 3054600,21.77; 3055200,21.93; 3055800,22.01; 3056400,
            22.09; 3057000,22.18; 3057600,22.26; 3058200,22.38; 3058800,22.46; 3059400,
            22.58; 3060000,22.62; 3060600,22.72; 3061200,22.78; 3061800,22.83; 3062400,
            22.91; 3063000,22.94; 3063600,23.04; 3064200,23.07; 3064800,23.12; 3065400,
            23.15; 3066000,23.11; 3066600,23.07; 3067200,23.07; 3067800,23.19; 3068400,
            23.27; 3069000,23.32; 3069600,23.36; 3070200,22.91; 3070800,19.82; 3071400,
            20.14; 3072000,19.17; 3072600,21.07; 3073200,22.02; 3073800,22.38; 3074400,
            22.58; 3075000,22.74; 3075600,22.83; 3076200,22.91; 3076800,23; 3077400,
            23.02; 3078000,23.07; 3078600,23.08; 3079200,23.15; 3079800,23.19; 3080400,
            23.19; 3081000,23.19; 3081600,23.15; 3082200,23.12; 3082800,23.19; 3083400,
            23.19; 3084000,23.15; 3084600,23.11; 3085200,23.11; 3085800,23.11; 3086400,
            23.11; 3087000,23.15; 3087600,23.16; 3088200,23.19; 3088800,23.16; 3089400,
            23.08; 3090000,23.03; 3090600,22.95; 3091200,22.91; 3091800,22.83; 3092400,
            22.78; 3093000,22.7; 3093600,22.66; 3094200,22.59; 3094800,22.5; 3095400,
            22.46; 3096000,22.38; 3096600,22.34; 3097200,22.27; 3097800,22.22; 3098400,
            22.14; 3099000,22.09; 3099600,22.05; 3100200,21.97; 3100800,21.93; 3101400,
            21.88; 3102000,21.84; 3102600,21.81; 3103200,21.73; 3103800,21.74; 3104400,
            21.65; 3105000,21.63; 3105600,21.6; 3106200,21.58; 3106800,21.52; 3107400,
            21.5; 3108000,21.44; 3108600,21.4; 3109200,21.36; 3109800,21.36; 3110400,
            21.32; 3111000,21.29; 3111600,21.25; 3112200,21.25; 3112800,21.2; 3113400,
            21.16; 3114000,21.12; 3114600,21.12; 3115200,21.1; 3115800,21.08; 3116400,
            21.08; 3117000,21.04; 3117600,21; 3118200,21; 3118800,20.97; 3119400,20.95;
            3120000,20.91; 3120600,20.88; 3121200,20.87; 3121800,20.86; 3122400,20.83;
            3123000,20.83; 3123600,20.79; 3124200,20.79; 3124800,20.75; 3125400,20.75;
            3126000,20.71; 3126600,20.71; 3127200,20.67; 3127800,20.67; 3128400,20.67;
            3129000,20.63; 3129600,20.63; 3130200,20.63; 3130800,20.59; 3131400,20.58;
            3132000,20.55; 3132600,20.55; 3133200,20.55; 3133800,20.53; 3134400,20.51;
            3135000,20.51; 3135600,20.51; 3136200,20.51; 3136800,20.51; 3137400,20.52;
            3138000,20.46; 3138600,20.46; 3139200,20.46; 3139800,20.5; 3140400,20.46;
            3141000,20.48; 3141600,20.52; 3142200,20.51; 3142800,20.51; 3143400,20.54;
            3144000,20.55; 3144600,20.55; 3145200,20.6; 3145800,20.59; 3146400,20.63;
            3147000,20.63; 3147600,20.67; 3148200,20.67; 3148800,20.71; 3149400,20.74;
            3150000,20.75; 3150600,20.79; 3151200,20.83; 3151800,20.86; 3152400,20.87;
            3153000,20.91; 3153600,20.95; 3154200,20.97; 3154800,21.04; 3155400,21.08;
            3156000,21.12; 3156600,21.16; 3157200,21.2; 3157800,21.28; 3158400,21.33;
            3159000,21.4; 3159600,21.48; 3160200,21.56; 3160800,21.6; 3161400,21.62;
            3162000,21.69; 3162600,21.73; 3163200,21.77; 3163800,21.81; 3164400,21.85;
            3165000,21.89; 3165600,21.93; 3166200,21.92; 3166800,21.89; 3167400,21.88;
            3168000,21.89; 3168600,21.85; 3169200,21.81; 3169800,21.8; 3170400,21.73;
            3171000,21.7; 3171600,21.65; 3172200,21.64; 3172800,21.6; 3173400,21.53;
            3174000,21.5; 3174600,21.48; 3175200,21.46; 3175800,21.4; 3176400,21.36;
            3177000,21.32; 3177600,21.28; 3178200,21.28; 3178800,21.24; 3179400,21.21;
            3180000,21.16; 3180600,21.13; 3181200,21.12; 3181800,21.08; 3182400,21.05;
            3183000,21; 3183600,20.98; 3184200,20.96; 3184800,20.92; 3185400,20.91;
            3186000,20.88; 3186600,20.83; 3187200,20.83; 3187800,20.79; 3188400,20.76;
            3189000,20.76; 3189600,20.73; 3190200,20.71; 3190800,20.67; 3191400,20.67;
            3192000,20.67; 3192600,20.63; 3193200,20.61; 3193800,20.59; 3194400,20.6;
            3195000,20.55; 3195600,20.55; 3196200,20.51; 3196800,20.51; 3197400,20.51;
            3198000,20.5; 3198600,20.5; 3199200,20.47; 3199800,20.46; 3200400,20.46;
            3201000,20.42; 3201600,20.42; 3202200,20.42; 3202800,20.42; 3203400,20.39;
            3204000,20.38; 3204600,20.38; 3205200,20.38; 3205800,20.38; 3206400,20.36;
            3207000,20.34; 3207600,20.34; 3208200,20.34; 3208800,20.36; 3209400,20.31;
            3210000,20.3; 3210600,20.3; 3211200,20.3; 3211800,20.3; 3212400,20.32; 3213000,
            20.3; 3213600,20.31; 3214200,20.29; 3214800,20.31; 3215400,20.3; 3216000,
            20.3; 3216600,20.28; 3217200,20.26; 3217800,20.26; 3218400,20.26; 3219000,
            20.26; 3219600,20.26; 3220200,20.26; 3220800,20.26; 3221400,20.26; 3222000,
            20.26; 3222600,20.26; 3223200,20.26; 3223800,20.26; 3224400,20.26; 3225000,
            20.26; 3225600,20.26; 3226200,20.26; 3226800,20.26; 3227400,20.26; 3228000,
            20.27; 3228600,20.3; 3229200,20.3; 3229800,20.31; 3230400,20.3; 3231000,
            20.3; 3231600,20.3; 3232200,20.3; 3232800,20.32; 3233400,20.3; 3234000,20.3;
            3234600,20.3; 3235200,20.3; 3235800,20.3; 3236400,20.3; 3237000,20.31; 3237600,
            20.31; 3238200,20.26; 3238800,20.26; 3239400,20.26; 3240000,20.26; 3240600,
            20.24; 3241200,20.22; 3241800,20.22; 3242400,20.22; 3243000,20.22; 3243600,
            20.18; 3244200,20.19; 3244800,20.22; 3245400,20.18; 3246000,20.18; 3246600,
            20.18; 3247200,20.18; 3247800,20.18; 3248400,20.18; 3249000,20.18; 3249600,
            20.18; 3250200,20.2; 3250800,20.22; 3251400,20.2; 3252000,20.18; 3252600,
            20.18; 3253200,20.18; 3253800,20.18; 3254400,20.19; 3255000,20.18; 3255600,
            20.18; 3256200,20.14; 3256800,20.14; 3257400,20.14; 3258000,20.15; 3258600,
            20.13; 3259200,20.1; 3259800,20.1; 3260400,20.1; 3261000,20.1; 3261600,20.1;
            3262200,20.06; 3262800,20.06; 3263400,20.05; 3264000,20.06; 3264600,20.03;
            3265200,20.02; 3265800,20.02; 3266400,20.02; 3267000,20.02; 3267600,20;
            3268200,19.99; 3268800,19.98; 3269400,19.98; 3270000,19.98; 3270600,19.98;
            3271200,19.99; 3271800,19.94; 3272400,19.94; 3273000,19.9; 3273600,19.91;
            3274200,19.9; 3274800,19.9; 3275400,19.9; 3276000,19.89; 3276600,19.86;
            3277200,19.86; 3277800,19.86; 3278400,19.86; 3279000,19.82; 3279600,19.82;
            3280200,19.82; 3280800,19.78; 3281400,19.81; 3282000,19.78; 3282600,19.78;
            3283200,19.78; 3283800,19.75; 3284400,19.73; 3285000,19.73; 3285600,19.7;
            3286200,19.69; 3286800,19.69; 3287400,19.7; 3288000,19.67; 3288600,19.66;
            3289200,19.65; 3289800,19.65; 3290400,19.65; 3291000,19.61; 3291600,19.61;
            3292200,19.57; 3292800,19.57; 3293400,19.55; 3294000,19.53; 3294600,19.53;
            3295200,19.53; 3295800,19.53; 3296400,19.53; 3297000,19.53; 3297600,19.5;
            3298200,19.5; 3298800,19.49; 3299400,19.49; 3300000,19.49; 3300600,19.49;
            3301200,19.49; 3301800,19.49; 3302400,19.49; 3303000,19.49; 3303600,19.49;
            3304200,19.5; 3304800,19.49; 3305400,19.49; 3306000,19.49; 3306600,19.49;
            3307200,19.49; 3307800,19.46; 3308400,19.7; 3309000,19.9; 3309600,20.08;
            3310200,20.34; 3310800,20.75; 3311400,20.87; 3312000,21; 3312600,21.08;
            3313200,21.12; 3313800,21.15; 3314400,21.28; 3315000,21.33; 3315600,21.44;
            3316200,21.52; 3316800,21.6; 3317400,21.69; 3318000,21.74; 3318600,21.76;
            3319200,21.84; 3319800,21.93; 3320400,21.97; 3321000,21.97; 3321600,22.01;
            3322200,22.01; 3322800,22.02; 3323400,22.05; 3324000,22.01; 3324600,21.97;
            3325200,21.97; 3325800,21.97; 3326400,21.97; 3327000,21.96; 3327600,21.85;
            3328200,21.77; 3328800,21.85; 3329400,21.85; 3330000,21.9; 3330600,21.97;
            3331200,22.14; 3331800,22.06; 3332400,22.08; 3333000,22.18; 3333600,22.18;
            3334200,22.18; 3334800,22.27; 3335400,22.3; 3336000,22.36; 3336600,22.42;
            3337200,22.46; 3337800,22.46; 3338400,22.5; 3339000,22.5; 3339600,22.54;
            3340200,22.58; 3340800,22.55; 3341400,22.42; 3342000,22.46; 3342600,22.38;
            3343200,22.31; 3343800,22.26; 3344400,22.18; 3345000,22.15; 3345600,22.09;
            3346200,22.09; 3346800,22.05; 3347400,22; 3348000,21.97; 3348600,21.93;
            3349200,21.89; 3349800,21.85; 3350400,21.81; 3351000,21.81; 3351600,21.77;
            3352200,21.69; 3352800,21.65; 3353400,21.65; 3354000,21.6; 3354600,21.6;
            3355200,21.53; 3355800,21.48; 3356400,21.44; 3357000,21.4; 3357600,21.36;
            3358200,21.32; 3358800,21.28; 3359400,21.24; 3360000,21.2; 3360600,21.16;
            3361200,21.12; 3361800,21.08; 3362400,21.08; 3363000,21; 3363600,20.98;
            3364200,20.96; 3364800,20.96; 3365400,20.91; 3366000,20.91; 3366600,20.87;
            3367200,20.83; 3367800,20.83; 3368400,20.83; 3369000,20.79; 3369600,20.79;
            3370200,20.75; 3370800,20.75; 3371400,20.75; 3372000,20.71; 3372600,20.71;
            3373200,20.67; 3373800,20.67; 3374400,20.67; 3375000,20.67; 3375600,20.66;
            3376200,20.63; 3376800,20.63; 3377400,20.59; 3378000,20.59; 3378600,20.59;
            3379200,20.55; 3379800,20.55; 3380400,20.51; 3381000,20.53; 3381600,20.53;
            3382200,20.51; 3382800,20.5; 3383400,20.5; 3384000,20.46; 3384600,20.46;
            3385200,20.46; 3385800,20.42; 3386400,20.42; 3387000,20.42; 3387600,20.42;
            3388200,20.42; 3388800,20.42; 3389400,20.42; 3390000,20.42; 3390600,20.44;
            3391200,20.42; 3391800,20.42; 3392400,20.39; 3393000,20.42; 3393600,20.42;
            3394200,20.42; 3394800,20.55; 3395400,20.79; 3396000,20.91; 3396600,21.04;
            3397200,21.16; 3397800,21.25; 3398400,21.36; 3399000,21.4; 3399600,21.45;
            3400200,21.46; 3400800,21.44; 3401400,21.52; 3402000,21.58; 3402600,21.6;
            3403200,21.6; 3403800,21.64; 3404400,21.69; 3405000,21.69; 3405600,21.77;
            3406200,21.77; 3406800,21.81; 3407400,21.85; 3408000,21.81; 3408600,21.85;
            3409200,21.85; 3409800,21.85; 3410400,21.89; 3411000,21.9; 3411600,21.9;
            3412200,21.93; 3412800,21.93; 3413400,21.94; 3414000,21.93; 3414600,21.98;
            3415200,22.05; 3415800,22.22; 3416400,22.18; 3417000,22.38; 3417600,22.46;
            3418200,22.54; 3418800,22.62; 3419400,22.66; 3420000,22.74; 3420600,22.78;
            3421200,22.78; 3421800,22.78; 3422400,22.91; 3423000,22.99; 3423600,22.95;
            3424200,22.99; 3424800,23.03; 3425400,23.15; 3426000,23.11; 3426600,23.06;
            3427200,23.07; 3427800,23.11; 3428400,23.11; 3429000,23.15; 3429600,23.16;
            3430200,23.19; 3430800,23.19; 3431400,23.15; 3432000,23.2; 3432600,23.23;
            3433200,23.15; 3433800,23.15; 3434400,23.2; 3435000,23.17; 3435600,23.11;
            3436200,23.03; 3436800,22.91; 3437400,22.8; 3438000,22.67; 3438600,22.58;
            3439200,22.5; 3439800,22.42; 3440400,22.34; 3441000,22.26; 3441600,22.22;
            3442200,22.13; 3442800,22.06; 3443400,21.97; 3444000,21.93; 3444600,21.85;
            3445200,21.81; 3445800,21.76; 3446400,21.69; 3447000,21.64; 3447600,21.59;
            3448200,21.54; 3448800,21.48; 3449400,21.44; 3450000,21.4; 3450600,21.36;
            3451200,21.32; 3451800,21.28; 3452400,21.22; 3453000,21.2; 3453600,21.2;
            3454200,21.16; 3454800,21.12; 3455400,21.11; 3456000,21.08; 3456600,21.04;
            3457200,21.04; 3457800,21.01; 3458400,20.98; 3459000,20.96; 3459600,20.96;
            3460200,20.91; 3460800,20.9; 3461400,20.87; 3462000,20.87; 3462600,20.84;
            3463200,20.83; 3463800,20.82; 3464400,20.79; 3465000,20.79; 3465600,20.78;
            3466200,20.75; 3466800,20.75; 3467400,20.76; 3468000,20.75; 3468600,20.71;
            3469200,20.67; 3469800,20.68; 3470400,20.69; 3471000,20.67; 3471600,20.67;
            3472200,20.67; 3472800,20.68; 3473400,20.67; 3474000,20.63; 3474600,20.64;
            3475200,20.63; 3475800,20.63; 3476400,20.63; 3477000,20.63; 3477600,20.63;
            3478200,20.63; 3478800,20.63; 3479400,20.62; 3480000,20.62; 3480600,20.63;
            3481200,20.58; 3481800,20.83; 3482400,20.96; 3483000,21.12; 3483600,21.2;
            3484200,21.28; 3484800,21.36; 3485400,21.4; 3486000,21.48; 3486600,21.52;
            3487200,21.6; 3487800,21.65; 3488400,21.72; 3489000,21.76; 3489600,21.81;
            3490200,21.85; 3490800,21.85; 3491400,21.94; 3492000,22; 3492600,21.97;
            3493200,21.98; 3493800,22.05; 3494400,22.05; 3495000,22.09; 3495600,22.14;
            3496200,22.22; 3496800,22.22; 3497400,22.18; 3498000,22.22; 3498600,22.3;
            3499200,22.34; 3499800,22.34; 3500400,22.26; 3501000,22.34; 3501600,22.38;
            3502200,22.42; 3502800,22.46; 3503400,22.46; 3504000,22.42; 3504600,22.47;
            3505200,22.47; 3505800,22.49; 3506400,22.5; 3507000,22.54; 3507600,22.58;
            3508200,22.58; 3508800,22.63; 3509400,22.58; 3510000,22.58; 3510600,22.55;
            3511200,22.54; 3511800,22.6; 3512400,22.54; 3513000,22.62; 3513600,22.74;
            3514200,22.74; 3514800,22.83; 3515400,22.78; 3516000,22.87; 3516600,22.84;
            3517200,22.78; 3517800,22.78; 3518400,22.74; 3519000,22.78; 3519600,22.74;
            3520200,22.74; 3520800,22.74; 3521400,22.74; 3522000,22.7; 3522600,22.65;
            3523200,22.64; 3523800,22.52; 3524400,22.42; 3525000,22.34; 3525600,22.26;
            3526200,22.22; 3526800,22.14; 3527400,22.05; 3528000,21.97; 3528600,21.93;
            3529200,21.85; 3529800,21.81; 3530400,21.73; 3531000,21.66; 3531600,21.64;
            3532200,21.56; 3532800,21.48; 3533400,21.44; 3534000,21.4; 3534600,21.36;
            3535200,21.32; 3535800,21.28; 3536400,21.24; 3537000,21.2; 3537600,21.16;
            3538200,21.13; 3538800,21.12; 3539400,21.04; 3540000,21.02; 3540600,20.99;
            3541200,20.96; 3541800,20.92; 3542400,20.89; 3543000,20.87; 3543600,20.83;
            3544200,20.83; 3544800,20.79; 3545400,20.78; 3546000,20.75; 3546600,20.76;
            3547200,20.71; 3547800,20.67; 3548400,20.67; 3549000,20.67; 3549600,20.63;
            3550200,20.63; 3550800,20.59; 3551400,20.59; 3552000,20.59; 3552600,20.55;
            3553200,20.55; 3553800,20.55; 3554400,20.55; 3555000,20.51; 3555600,20.51;
            3556200,20.51; 3556800,20.51; 3557400,20.5; 3558000,20.49; 3558600,20.46;
            3559200,20.46; 3559800,20.46; 3560400,20.43; 3561000,20.44; 3561600,20.42;
            3562200,20.42; 3562800,20.42; 3563400,20.42; 3564000,20.42; 3564600,20.38;
            3565200,20.4; 3565800,20.38; 3566400,20.38; 3567000,20.38; 3567600,20.51;
            3568200,21.07; 3568800,21.36; 3569400,21.6; 3570000,21.73; 3570600,21.86;
            3571200,21.89; 3571800,21.81; 3572400,21.85; 3573000,21.89; 3573600,21.96;
            3574200,22.1; 3574800,22.18; 3575400,22.26; 3576000,22.34; 3576600,22.3;
            3577200,22.26; 3577800,22.34; 3578400,22.43; 3579000,22.54; 3579600,22.5;
            3580200,22.61; 3580800,22.7; 3581400,22.78; 3582000,22.82; 3582600,22.87;
            3583200,22.83; 3583800,22.74; 3584400,22.75; 3585000,22.83; 3585600,22.83;
            3586200,22.83; 3586800,22.87; 3587400,23.03; 3588000,23.11; 3588600,23.22;
            3589200,23.27; 3589800,23.2; 3590400,23.27; 3591000,23.27; 3591600,23.31;
            3592200,23.32; 3592800,23.27; 3593400,23.27; 3594000,23.27; 3594600,23.23;
            3595200,23.23; 3595800,23.23; 3596400,23.23; 3597000,23.24; 3597600,23.27;
            3598200,23.27; 3598800,23.44; 3599400,23.61; 3600000,23.76; 3600600,23.88;
            3601200,23.92; 3601800,23.96; 3602400,23.94; 3603000,23.92; 3603600,23.84;
            3604200,23.76; 3604800,23.76; 3605400,23.76; 3606000,23.76; 3606600,23.76;
            3607200,23.7; 3607800,23.72; 3608400,23.72; 3609000,23.68; 3609600,23.68;
            3610200,23.6; 3610800,23.6; 3611400,23.56; 3612000,23.48; 3612600,23.4;
            3613200,23.36; 3613800,23.27; 3614400,23.11; 3615000,23.03; 3615600,22.95;
            3616200,22.95; 3616800,22.88; 3617400,22.83; 3618000,22.76; 3618600,22.7;
            3619200,22.62; 3619800,22.54; 3620400,22.51; 3621000,22.42; 3621600,22.34;
            3622200,22.3; 3622800,22.26; 3623400,22.22; 3624000,22.19; 3624600,22.14;
            3625200,22.09; 3625800,22.31; 3626400,22.05; 3627000,22.01; 3627600,21.97;
            3628200,21.97; 3628800,21.93; 3629400,21.93; 3630000,21.9; 3630600,21.85;
            3631200,21.85; 3631800,21.81; 3632400,21.81; 3633000,21.77; 3633600,21.74;
            3634200,21.77; 3634800,21.73; 3635400,21.74; 3636000,21.89; 3636600,22.05;
            3637200,22.18; 3637800,22.26; 3638400,22.38; 3639000,22.46; 3639600,22.54;
            3640200,22.63; 3640800,22.62; 3641400,22.66; 3642000,22.66; 3642600,22.62;
            3643200,22.62; 3643800,22.64; 3644400,22.62; 3645000,22.62; 3645600,22.62;
            3646200,22.62; 3646800,22.66; 3647400,22.66; 3648000,22.66; 3648600,22.67;
            3649200,22.62; 3649800,22.64; 3650400,22.62; 3651000,22.62; 3651600,22.66;
            3652200,22.74; 3652800,22.78; 3653400,22.75; 3654000,22.74; 3654600,22.84;
            3655200,22.96; 3655800,23.03; 3656400,23.11; 3657000,23.15; 3657600,23.16;
            3658200,23.32; 3658800,23.35; 3659400,23.4; 3660000,23.48; 3660600,23.56;
            3661200,23.68; 3661800,23.72; 3662400,23.8; 3663000,23.81; 3663600,23.84;
            3664200,23.89; 3664800,23.84; 3665400,23.88; 3666000,23.88; 3666600,23.92;
            3667200,23.94; 3667800,23.92; 3668400,23.96; 3669000,23.88; 3669600,23.8;
            3670200,23.77; 3670800,23.77; 3671400,23.76; 3672000,23.73; 3672600,23.72;
            3673200,23.74; 3673800,23.76; 3674400,23.78; 3675000,23.8; 3675600,23.86;
            3676200,23.84; 3676800,23.88; 3677400,23.86; 3678000,23.88; 3678600,23.88;
            3679200,23.92; 3679800,23.92; 3680400,23.96; 3681000,23.93; 3681600,24.01;
            3682200,24.01; 3682800,24.1; 3683400,24.09; 3684000,24.21; 3684600,24.33;
            3685200,24.41; 3685800,24.41; 3686400,24.45; 3687000,24.46; 3687600,24.45;
            3688200,24.37; 3688800,24.37; 3689400,24.37; 3690000,24.41; 3690600,24.49;
            3691200,24.5; 3691800,24.54; 3692400,24.57; 3693000,24.58; 3693600,24.58;
            3694200,24.58; 3694800,24.58; 3695400,24.58; 3696000,24.58; 3696600,24.58;
            3697200,24.58; 3697800,24.58; 3698400,24.54; 3699000,24.45; 3699600,24.37;
            3700200,24.21; 3700800,24.05; 3701400,23.92; 3702000,23.8; 3702600,23.72;
            3703200,23.66; 3703800,23.56; 3704400,23.48; 3705000,23.4; 3705600,23.3;
            3706200,23.19; 3706800,23.11; 3707400,23.07; 3708000,23.03; 3708600,22.96;
            3709200,22.91; 3709800,22.83; 3710400,22.78; 3711000,22.74; 3711600,22.7;
            3712200,22.66; 3712800,22.58; 3713400,22.5; 3714000,22.46; 3714600,22.42;
            3715200,22.38; 3715800,22.34; 3716400,22.3; 3717000,22.26; 3717600,22.22;
            3718200,22.18; 3718800,22.14; 3719400,22.1; 3720000,22.09; 3720600,22.03;
            3721200,22.01; 3721800,21.97; 3722400,21.95; 3723000,21.94; 3723600,21.89;
            3724200,21.85; 3724800,21.81; 3725400,21.81; 3726000,21.79; 3726600,21.77;
            3727200,21.73; 3727800,21.73; 3728400,21.69; 3729000,21.69; 3729600,21.65;
            3730200,21.66; 3730800,21.6; 3731400,21.6; 3732000,21.6; 3732600,21.56;
            3733200,21.56; 3733800,21.52; 3734400,21.48; 3735000,21.48; 3735600,21.44;
            3736200,21.45; 3736800,21.4; 3737400,21.4; 3738000,21.4; 3738600,21.36;
            3739200,21.32; 3739800,21.28; 3740400,21.24; 3741000,21.2; 3741600,21.2;
            3742200,21.16; 3742800,21.17; 3743400,21.12; 3744000,21.13; 3744600,21.12;
            3745200,21.12; 3745800,21.08; 3746400,21.08; 3747000,21.04; 3747600,21.05;
            3748200,21.04; 3748800,21; 3749400,21; 3750000,21; 3750600,20.99; 3751200,
            20.96; 3751800,20.91; 3752400,20.91; 3753000,20.91; 3753600,20.88; 3754200,
            20.91; 3754800,20.91; 3755400,20.96; 3756000,20.99; 3756600,21; 3757200,
            21; 3757800,21.04; 3758400,21.04; 3759000,21.08; 3759600,21.12; 3760200,
            21.16; 3760800,21.25; 3761400,21.28; 3762000,21.28; 3762600,21.24; 3763200,
            21.24; 3763800,21.28; 3764400,21.28; 3765000,21.25; 3765600,21.2; 3766200,
            21.21; 3766800,21.2; 3767400,21.21; 3768000,21.2; 3768600,21.2; 3769200,
            21.2; 3769800,21.2; 3770400,21.2; 3771000,21.2; 3771600,21.2; 3772200,21.16;
            3772800,21.12; 3773400,21.12; 3774000,21.12; 3774600,21.08; 3775200,21.08;
            3775800,21.08; 3776400,21.07; 3777000,21.09; 3777600,21.04; 3778200,21.04;
            3778800,21.03; 3779400,21.02; 3780000,21.04; 3780600,21; 3781200,21; 3781800,
            21; 3782400,21; 3783000,20.99; 3783600,20.96; 3784200,20.98; 3784800,20.97;
            3785400,20.98; 3786000,20.96; 3786600,20.96; 3787200,20.96; 3787800,20.96;
            3788400,20.96; 3789000,20.97; 3789600,20.96; 3790200,20.96; 3790800,20.96;
            3791400,20.93; 3792000,20.92; 3792600,20.92; 3793200,20.91; 3793800,20.91;
            3794400,20.92; 3795000,20.91; 3795600,20.91; 3796200,20.91; 3796800,20.87;
            3797400,20.87; 3798000,20.87; 3798600,20.87; 3799200,20.88; 3799800,20.94;
            3800400,20.98; 3801000,21; 3801600,21.04; 3802200,21.05; 3802800,21.08;
            3803400,21.12; 3804000,21.12; 3804600,21.15; 3805200,21.16; 3805800,21.16;
            3806400,21.21; 3807000,21.2; 3807600,21.2; 3808200,21.2; 3808800,21.24;
            3809400,21.24; 3810000,21.28; 3810600,21.32; 3811200,21.36; 3811800,21.36;
            3812400,21.36; 3813000,21.4; 3813600,21.4; 3814200,21.4; 3814800,21.44;
            3815400,21.45; 3816000,21.44; 3816600,21.44; 3817200,21.44; 3817800,21.47;
            3818400,21.48; 3819000,21.49; 3819600,21.48; 3820200,21.48; 3820800,21.48;
            3821400,21.48; 3822000,21.48; 3822600,21.52; 3823200,21.52; 3823800,21.52;
            3824400,21.54; 3825000,21.49; 3825600,21.52; 3826200,21.52; 3826800,21.52;
            3827400,21.52; 3828000,21.52; 3828600,21.52; 3829200,21.52; 3829800,21.52;
            3830400,21.52; 3831000,21.48; 3831600,21.44; 3832200,21.44; 3832800,21.4;
            3833400,21.44; 3834000,21.44; 3834600,21.44; 3835200,21.44; 3835800,21.44;
            3836400,21.48; 3837000,21.48; 3837600,21.54; 3838200,21.56; 3838800,21.6;
            3839400,21.6; 3840000,21.64; 3840600,21.65; 3841200,21.67; 3841800,21.73;
            3842400,21.77; 3843000,21.82; 3843600,21.85; 3844200,21.89; 3844800,21.97;
            3845400,22.06; 3846000,22.05; 3846600,22.05; 3847200,22.05; 3847800,22.06;
            3848400,22.1; 3849000,22.14; 3849600,22.13; 3850200,22.14; 3850800,22.14;
            3851400,22.17; 3852000,22.18; 3852600,22.18; 3853200,22.21; 3853800,22.32;
            3854400,22.36; 3855000,22.42; 3855600,22.46; 3856200,22.46; 3856800,22.42;
            3857400,22.42; 3858000,22.42; 3858600,22.42; 3859200,22.42; 3859800,22.42;
            3860400,22.41; 3861000,22.38; 3861600,22.34; 3862200,22.34; 3862800,22.3;
            3863400,22.26; 3864000,22.26; 3864600,22.24; 3865200,22.22; 3865800,22.18;
            3866400,22.18; 3867000,22.14; 3867600,22.14; 3868200,22.13; 3868800,22.09;
            3869400,22.09; 3870000,22.09; 3870600,22.09; 3871200,22.05; 3871800,22.05;
            3872400,22.03; 3873000,22.01; 3873600,22.01; 3874200,21.97; 3874800,21.97;
            3875400,21.97; 3876000,21.97; 3876600,21.93; 3877200,21.93; 3877800,21.93;
            3878400,21.93; 3879000,21.93; 3879600,21.89; 3880200,21.89; 3880800,21.88;
            3881400,21.85; 3882000,21.85; 3882600,21.85; 3883200,21.85; 3883800,21.84;
            3884400,21.81; 3885000,21.81; 3885600,21.81; 3886200,21.81; 3886800,21.81;
            3887400,21.81; 3888000,21.81; 3888600,21.77; 3889200,21.79; 3889800,21.77;
            3890400,21.78; 3891000,21.77; 3891600,21.77; 3892200,21.77; 3892800,21.77;
            3893400,21.77; 3894000,21.73; 3894600,21.77; 3895200,21.75; 3895800,21.76;
            3896400,21.77; 3897000,21.77; 3897600,21.76; 3898200,21.78; 3898800,21.76;
            3899400,21.73; 3900000,21.73; 3900600,21.77; 3901200,21.76; 3901800,21.77;
            3902400,21.73; 3903000,21.73; 3903600,21.89; 3904200,22.18; 3904800,22.3;
            3905400,22.46; 3906000,22.66; 3906600,22.88; 3907200,22.95; 3907800,22.99;
            3908400,23; 3909000,23.03; 3909600,23.07; 3910200,23.11; 3910800,23.11;
            3911400,23.15; 3912000,23.19; 3912600,23.23; 3913200,23.28; 3913800,23.31;
            3914400,23.27; 3915000,23.23; 3915600,23.28; 3916200,23.33; 3916800,23.4;
            3917400,23.44; 3918000,23.48; 3918600,23.56; 3919200,23.6; 3919800,23.69;
            3920400,23.72; 3921000,23.76; 3921600,23.8; 3922200,23.84; 3922800,23.92;
            3923400,23.94; 3924000,23.96; 3924600,24.01; 3925200,24.05; 3925800,24.09;
            3926400,24.09; 3927000,24.09; 3927600,24.09; 3928200,24.05; 3928800,24.1;
            3929400,24.09; 3930000,24.09; 3930600,24.09; 3931200,24.09; 3931800,24.13;
            3932400,24.13; 3933000,24.17; 3933600,24.17; 3934200,24.17; 3934800,24.2;
            3935400,24.21; 3936000,24.22; 3936600,24.21; 3937200,24.22; 3937800,24.25;
            3938400,24.3; 3939000,24.21; 3939600,24.25; 3940200,24.21; 3940800,24.21;
            3941400,24.17; 3942000,24.17; 3942600,24.17; 3943200,24.16; 3943800,24.14;
            3944400,24.13; 3945000,24.13; 3945600,24.13; 3946200,24.13; 3946800,24.06;
            3947400,24.01; 3948000,23.96; 3948600,23.96; 3949200,23.96; 3949800,23.92;
            3950400,23.88; 3951000,23.88; 3951600,23.88; 3952200,23.84; 3952800,23.8;
            3953400,23.8; 3954000,23.76; 3954600,23.72; 3955200,23.68; 3955800,23.68;
            3956400,23.65; 3957000,23.64; 3957600,23.54; 3958200,23.4; 3958800,23.28;
            3959400,23.18; 3960000,23.07; 3960600,22.99; 3961200,22.91; 3961800,22.79;
            3962400,22.7; 3963000,22.58; 3963600,22.52; 3964200,22.46; 3964800,22.42;
            3965400,22.34; 3966000,22.3; 3966600,22.22; 3967200,22.19; 3967800,22.14;
            3968400,22.09; 3969000,22.05; 3969600,22.01; 3970200,21.97; 3970800,21.94;
            3971400,21.89; 3972000,21.89; 3972600,21.85; 3973200,21.81; 3973800,21.81;
            3974400,21.79; 3975000,21.77; 3975600,21.77; 3976200,21.73; 3976800,21.69;
            3977400,21.69; 3978000,21.69; 3978600,21.65; 3979200,21.65; 3979800,21.63;
            3980400,21.65; 3981000,21.65; 3981600,21.65; 3982200,21.65; 3982800,21.65;
            3983400,21.65; 3984000,21.65; 3984600,21.65; 3985200,21.65; 3985800,21.65;
            3986400,21.65; 3987000,21.65; 3987600,21.66; 3988200,21.65; 3988800,21.65;
            3989400,21.65; 3990000,21.72; 3990600,21.94; 3991200,22.09; 3991800,22.26;
            3992400,22.42; 3993000,22.54; 3993600,22.6; 3994200,22.58; 3994800,22.6;
            3995400,22.58; 3996000,22.62; 3996600,22.62; 3997200,22.62; 3997800,22.64;
            3998400,22.66; 3999000,22.74; 3999600,22.77; 4000200,22.78; 4000800,22.83;
            4001400,22.91; 4002000,23.03; 4002600,23.15; 4003200,23.2; 4003800,23.27;
            4004400,23.4; 4005000,23.48; 4005600,23.56; 4006200,23.64; 4006800,23.72;
            4007400,23.72; 4008000,23.8; 4008600,23.88; 4009200,23.95; 4009800,24.01;
            4010400,24.05; 4011000,24.1; 4011600,24.17; 4012200,24.21; 4012800,24.21;
            4013400,24.25; 4014000,24.25; 4014600,24.25; 4015200,24.25; 4015800,24.21;
            4016400,24.21; 4017000,24.21; 4017600,24.21; 4018200,24.22; 4018800,24.21;
            4019400,24.21; 4020000,24.21; 4020600,24.13; 4021200,24.21; 4021800,24.25;
            4022400,24.25; 4023000,24.3; 4023600,24.34; 4024200,24.34; 4024800,24.41;
            4025400,24.62; 4026000,24.74; 4026600,24.86; 4027200,24.9; 4027800,24.9;
            4028400,24.9; 4029000,24.9; 4029600,24.86; 4030200,24.86; 4030800,24.85;
            4031400,24.89; 4032000,24.86; 4032600,24.87; 4033200,24.78; 4033800,24.71;
            4034400,24.66; 4035000,24.58; 4035600,24.57; 4036200,24.54; 4036800,24.5;
            4037400,24.45; 4038000,24.45; 4038600,24.44; 4039200,24.41; 4039800,24.41;
            4040400,24.41; 4041000,24.38; 4041600,24.37; 4042200,24.33; 4042800,24.33;
            4043400,24.3; 4044000,24.22; 4044600,24.14; 4045200,24.05; 4045800,23.92;
            4046400,23.84; 4047000,23.72; 4047600,23.64; 4048200,23.52; 4048800,23.44;
            4049400,23.36; 4050000,23.36; 4050600,23.27; 4051200,23.25; 4051800,23.19;
            4052400,23.15; 4053000,23.11; 4053600,23.07; 4054200,23.03; 4054800,23.03;
            4055400,22.95; 4056000,22.95; 4056600,22.91; 4057200,22.92; 4057800,22.91;
            4058400,22.87; 4059000,22.86; 4059600,22.83; 4060200,22.83; 4060800,22.78;
            4061400,22.74; 4062000,22.74; 4062600,22.74; 4063200,22.74; 4063800,22.72;
            4064400,22.7; 4065000,22.7; 4065600,22.7; 4066200,22.7; 4066800,22.66; 4067400,
            22.66; 4068000,22.66; 4068600,22.68; 4069200,22.66; 4069800,22.66; 4070400,
            22.62; 4071000,22.64; 4071600,22.62; 4072200,22.62; 4072800,22.62; 4073400,
            22.62; 4074000,22.62; 4074600,22.62; 4075200,22.62; 4075800,22.62; 4076400,
            22.66; 4077000,22.91; 4077600,23.12; 4078200,23.36; 4078800,23.56; 4079400,
            23.69; 4080000,23.68; 4080600,23.6; 4081200,23.56; 4081800,23.44; 4082400,
            23.4; 4083000,23.39; 4083600,23.4; 4084200,23.48; 4084800,23.51; 4085400,
            23.56; 4086000,23.56; 4086600,23.58; 4087200,23.56; 4087800,23.48; 4088400,
            23.4; 4089000,23.36; 4089600,23.36; 4090200,23.4; 4090800,23.48; 4091400,
            23.58; 4092000,23.64; 4092600,23.68; 4093200,23.7; 4093800,23.68; 4094400,
            23.68; 4095000,23.68; 4095600,23.72; 4096200,23.72; 4096800,23.72; 4097400,
            23.72; 4098000,23.77; 4098600,23.8; 4099200,23.8; 4099800,23.84; 4100400,
            23.88; 4101000,23.92; 4101600,23.9; 4102200,23.84; 4102800,23.8; 4103400,
            23.85; 4104000,23.96; 4104600,23.96; 4105200,23.96; 4105800,23.94; 4106400,
            24; 4107000,24.01; 4107600,24.01; 4108200,24.05; 4108800,24.05; 4109400,
            24.06; 4110000,24.05; 4110600,24.06; 4111200,24.08; 4111800,24.01; 4112400,
            24.04; 4113000,24.05; 4113600,24.09; 4114200,24.12; 4114800,24.12; 4115400,
            24.08; 4116000,24.09; 4116600,24.09; 4117200,24.13; 4117800,24.13; 4118400,
            24.13; 4119000,24.12; 4119600,24.09; 4120200,24.01; 4120800,23.96; 4121400,
            23.92; 4122000,23.92; 4122600,23.84; 4123200,23.81; 4123800,23.76; 4124400,
            23.76; 4125000,23.68; 4125600,23.68; 4126200,23.6; 4126800,23.6; 4127400,
            23.56; 4128000,23.57; 4128600,23.52; 4129200,23.49; 4129800,23.48; 4130400,
            23.44; 4131000,23.41; 4131600,23.36; 4132200,23.35; 4132800,23.32; 4133400,
            23.27; 4134000,23.16; 4134600,23.07; 4135200,23.03; 4135800,22.99; 4136400,
            22.94; 4137000,22.94; 4137600,22.91; 4138200,22.88; 4138800,22.83; 4139400,
            22.78; 4140000,22.78; 4140600,22.74; 4141200,22.74; 4141800,22.7; 4142400,
            22.66; 4143000,22.63; 4143600,22.64; 4144200,22.62; 4144800,22.58; 4145400,
            22.58; 4146000,22.54; 4146600,22.5; 4147200,22.5; 4147800,22.46; 4148400,
            22.46; 4149000,22.46; 4149600,22.42; 4150200,22.42; 4150800,22.42; 4151400,
            22.42; 4152000,22.38; 4152600,22.38; 4153200,22.42; 4153800,22.42; 4154400,
            22.42; 4155000,22.46; 4155600,22.46; 4156200,22.46; 4156800,22.46; 4157400,
            22.48; 4158000,22.5; 4158600,22.5; 4159200,22.5; 4159800,22.5; 4160400,22.5;
            4161000,22.5; 4161600,22.5; 4162200,22.5; 4162800,22.67; 4163400,22.91;
            4164000,23.15; 4164600,23.23; 4165200,23.27; 4165800,23.23; 4166400,23.15;
            4167000,23.03; 4167600,22.96; 4168200,22.87; 4168800,22.87; 4169400,22.95;
            4170000,23.03; 4170600,23.12; 4171200,23.11; 4171800,23.11; 4172400,23.1;
            4173000,23.07; 4173600,23.11; 4174200,23.11; 4174800,23.15; 4175400,23.15;
            4176000,23.2; 4176600,23.28; 4177200,23.4; 4177800,23.48; 4178400,23.52;
            4179000,23.56; 4179600,23.68; 4180200,23.68; 4180800,23.76; 4181400,23.8;
            4182000,23.92; 4182600,23.92; 4183200,23.96; 4183800,24.04; 4184400,24.01;
            4185000,24.12; 4185600,24.13; 4186200,24.13; 4186800,24.09; 4187400,24.17;
            4188000,24.2; 4188600,24.17; 4189200,24.17; 4189800,24.13; 4190400,24.13;
            4191000,24.05; 4191600,24.09; 4192200,24.13; 4192800,24.17; 4193400,24.26;
            4194000,24.25; 4194600,24.31; 4195200,24.41; 4195800,24.41; 4196400,24.5;
            4197000,24.5; 4197600,24.5; 4198200,24.45; 4198800,24.49; 4199400,24.46;
            4200000,24.5; 4200600,24.54; 4201200,24.54; 4201800,24.54; 4202400,24.58;
            4203000,24.54; 4203600,24.54; 4204200,24.62; 4204800,24.58; 4205400,24.58;
            4206000,24.61; 4206600,24.61; 4207200,24.54; 4207800,24.58; 4208400,24.5;
            4209000,24.45; 4209600,24.41; 4210200,24.37; 4210800,24.37; 4211400,24.29;
            4212000,24.25; 4212600,24.17; 4213200,24.13; 4213800,24.08; 4214400,24.05;
            4215000,24.21; 4215600,24.01; 4216200,24.01; 4216800,23.92; 4217400,23.8;
            4218000,23.76; 4218600,23.68; 4219200,23.6; 4219800,23.56; 4220400,23.48;
            4221000,23.49; 4221600,23.44; 4222200,23.4; 4222800,23.32; 4223400,23.27;
            4224000,23.23; 4224600,23.15; 4225200,23.11; 4225800,23.11; 4226400,22.99;
            4227000,22.95; 4227600,22.91; 4228200,22.91; 4228800,22.87; 4229400,22.87;
            4230000,22.83; 4230600,22.78; 4231200,22.78; 4231800,22.77; 4232400,22.75;
            4233000,22.74; 4233600,22.7; 4234200,22.66; 4234800,22.66; 4235400,22.62;
            4236000,22.62; 4236600,22.58; 4237200,22.58; 4237800,22.55; 4238400,22.54;
            4239000,22.55; 4239600,22.59; 4240200,22.55; 4240800,22.58; 4241400,22.58;
            4242000,22.58; 4242600,22.62; 4243200,22.59; 4243800,22.62; 4244400,22.62;
            4245000,22.63; 4245600,22.62; 4246200,22.62; 4246800,22.64; 4247400,22.62;
            4248000,22.62; 4248600,22.62; 4249200,22.62; 4249800,22.89; 4250400,23.07;
            4251000,23.23; 4251600,23.37; 4252200,23.37; 4252800,23.35; 4253400,23.24;
            4254000,23.15; 4254600,23.07; 4255200,22.99; 4255800,23.03; 4256400,23.11;
            4257000,23.19; 4257600,23.23; 4258200,23.24; 4258800,23.24; 4259400,23.19;
            4260000,23.15; 4260600,23.11; 4261200,23.11; 4261800,23.11; 4262400,23.19;
            4263000,23.21; 4263600,23.36; 4264200,23.45; 4264800,23.52; 4265400,23.59;
            4266000,23.56; 4266600,23.6; 4267200,23.64; 4267800,23.64; 4268400,23.69;
            4269000,23.72; 4269600,23.73; 4270200,23.77; 4270800,23.77; 4271400,23.84;
            4272000,23.92; 4272600,23.93; 4273200,23.88; 4273800,23.92; 4274400,23.93;
            4275000,23.92; 4275600,23.92; 4276200,23.92; 4276800,23.92; 4277400,23.92;
            4278000,23.92; 4278600,23.92; 4279200,23.97; 4279800,24; 4280400,24.09;
            4281000,24.17; 4281600,24.22; 4282200,24.29; 4282800,24.34; 4283400,24.41;
            4284000,24.4; 4284600,24.37; 4285200,24.41; 4285800,24.42; 4286400,24.41;
            4287000,24.41; 4287600,24.41; 4288200,24.37; 4288800,24.29; 4289400,24.21;
            4290000,24.22; 4290600,24.17; 4291200,24.17; 4291800,24.25; 4292400,24.17;
            4293000,24.17; 4293600,24.21; 4294200,24.13; 4294800,24.2; 4295400,24.21;
            4296000,24.17; 4296600,24.17; 4297200,24.13; 4297800,24.09; 4298400,24.05;
            4299000,24.01; 4299600,23.98; 4300200,23.93; 4300800,23.9; 4301400,23.84;
            4302000,23.8; 4302600,23.76; 4303200,23.72; 4303800,23.68; 4304400,23.6;
            4305000,23.56; 4305600,23.48; 4306200,23.4; 4306800,23.32; 4307400,23.24;
            4308000,23.15; 4308600,23.08; 4309200,23.03; 4309800,22.91; 4310400,22.95;
            4311000,22.9; 4311600,22.84; 4312200,22.8; 4312800,22.78; 4313400,22.74;
            4314000,22.71; 4314600,22.66; 4315200,22.62; 4315800,22.58; 4316400,22.54;
            4317000,22.5; 4317600,22.46; 4318200,22.42; 4318800,22.34; 4319400,22.3;
            4320000,22.26; 4320600,22.26; 4321200,22.18; 4321800,22.13; 4322400,22.05;
            4323000,22; 4323600,21.95; 4324200,21.89; 4324800,21.85; 4325400,21.81;
            4326000,21.75; 4326600,21.72; 4327200,21.7; 4327800,21.64; 4328400,21.6;
            4329000,21.54; 4329600,21.52; 4330200,21.48; 4330800,21.44; 4331400,21.4;
            4332000,21.36; 4332600,21.32; 4333200,21.32; 4333800,21.28; 4334400,21.24;
            4335000,21.2; 4335600,21.2; 4336200,21.16; 4336800,21.13; 4337400,21.12;
            4338000,21.08; 4338600,21.08; 4339200,21.04; 4339800,21; 4340400,21; 4341000,
            20.97; 4341600,20.96; 4342200,20.91; 4342800,20.92; 4343400,20.87; 4344000,
            20.87; 4344600,20.87; 4345200,20.84; 4345800,20.83; 4346400,20.83; 4347000,
            20.83; 4347600,20.83; 4348200,20.83; 4348800,20.79; 4349400,20.79; 4350000,
            20.79; 4350600,20.82; 4351200,20.8; 4351800,20.79; 4352400,20.79; 4353000,
            20.79; 4353600,20.79; 4354200,20.8; 4354800,20.83; 4355400,20.83; 4356000,
            20.87; 4356600,20.87; 4357200,20.91; 4357800,20.92; 4358400,20.91; 4359000,
            20.96; 4359600,20.97; 4360200,20.97; 4360800,20.96; 4361400,21; 4362000,
            21.01; 4362600,21.04; 4363200,21.08; 4363800,21.11; 4364400,21.12; 4365000,
            21.16; 4365600,21.16; 4366200,21.2; 4366800,21.2; 4367400,21.24; 4368000,
            21.24; 4368600,21.28; 4369200,21.32; 4369800,21.32; 4370400,21.32; 4371000,
            21.3; 4371600,21.28; 4372200,21.28; 4372800,21.24; 4373400,21.24; 4374000,
            21.22; 4374600,21.2; 4375200,21.2; 4375800,21.16; 4376400,21.16; 4377000,
            21.16; 4377600,21.16; 4378200,21.12; 4378800,21.12; 4379400,21.11; 4380000,
            21.08; 4380600,21.05; 4381200,21.04; 4381800,21.03; 4382400,21; 4383000,
            20.98; 4383600,20.96; 4384200,20.95; 4384800,20.92; 4385400,20.91; 4386000,
            20.91; 4386600,20.87; 4387200,20.87; 4387800,20.87; 4388400,20.84; 4389000,
            20.83; 4389600,20.83; 4390200,20.83; 4390800,20.83; 4391400,20.83; 4392000,
            20.8; 4392600,20.8; 4393200,20.79; 4393800,20.79; 4394400,20.79; 4395000,
            20.79; 4395600,20.79; 4396200,20.79; 4396800,20.75; 4397400,20.75; 4398000,
            20.75; 4398600,20.75; 4399200,20.75; 4399800,20.71; 4400400,20.72; 4401000,
            20.71; 4401600,20.71; 4402200,20.69; 4402800,20.67; 4403400,20.67; 4404000,
            20.75; 4404600,20.85; 4405200,20.96; 4405800,21.04; 4406400,21.08; 4407000,
            21.15; 4407600,21.18; 4408200,21.23; 4408800,21.28; 4409400,21.32; 4410000,
            21.32; 4410600,21.36; 4411200,21.41; 4411800,21.44; 4412400,21.44; 4413000,
            21.48; 4413600,21.48; 4414200,21.52; 4414800,21.52; 4415400,21.52; 4416000,
            21.58; 4416600,21.56; 4417200,21.56; 4417800,21.6; 4418400,21.6; 4419000,
            21.6; 4419600,21.6; 4420200,21.6; 4420800,21.64; 4421400,21.65; 4422000,
            21.63; 4422600,21.64; 4423200,21.65; 4423800,21.65; 4424400,21.66; 4425000,
            21.65; 4425600,21.65; 4426200,21.65; 4426800,21.65; 4427400,21.66; 4428000,
            21.68; 4428600,21.69; 4429200,21.69; 4429800,21.69; 4430400,21.69; 4431000,
            21.69; 4431600,21.69; 4432200,21.7; 4432800,21.72; 4433400,21.73; 4434000,
            21.76; 4434600,21.77; 4435200,21.78; 4435800,21.77; 4436400,21.81; 4437000,
            21.81; 4437600,21.81; 4438200,21.84; 4438800,21.81; 4439400,21.85; 4440000,
            21.85; 4440600,21.89; 4441200,21.89; 4441800,21.93; 4442400,21.93; 4443000,
            21.89; 4443600,21.88; 4444200,21.85; 4444800,21.85; 4445400,21.84; 4446000,
            21.81; 4446600,21.81; 4447200,21.85; 4447800,21.85; 4448400,21.86; 4449000,
            21.89; 4449600,21.89; 4450200,21.9; 4450800,21.98; 4451400,22.02; 4452000,
            22.09; 4452600,22.12; 4453200,22.18; 4453800,22.22; 4454400,22.26; 4455000,
            22.3; 4455600,22.34; 4456200,22.38; 4456800,22.42; 4457400,22.45; 4458000,
            22.46; 4458600,22.51; 4459200,22.58; 4459800,22.58; 4460400,22.62; 4461000,
            22.66; 4461600,22.74; 4462200,22.78; 4462800,22.91; 4463400,22.87; 4464000,
            22.8; 4464600,22.7; 4465200,22.65; 4465800,22.62; 4466400,22.57; 4467000,
            22.5; 4467600,22.46; 4468200,22.42; 4468800,22.34; 4469400,22.3; 4470000,
            22.27; 4470600,22.25; 4471200,22.14; 4471800,22.09; 4472400,22.09; 4473000,
            22.14; 4473600,22.17; 4474200,22.17; 4474800,22.14; 4475400,22.14; 4476000,
            22.17; 4476600,22.17; 4477200,22.14; 4477800,22.17; 4478400,22.14; 4479000,
            22.14; 4479600,22.14; 4480200,22.14; 4480800,22.14; 4481400,22.1; 4482000,
            22.09; 4482600,22.1; 4483200,22.1; 4483800,22.09; 4484400,22.09; 4485000,
            22.09; 4485600,22.09; 4486200,22.09; 4486800,22.09; 4487400,22.09; 4488000,
            22.1; 4488600,22.06; 4489200,22.08; 4489800,22.08; 4490400,22.05; 4491000,
            22.05; 4491600,22.05; 4492200,22.05; 4492800,22.05; 4493400,22.05; 4494000,
            22.05; 4494600,22.03; 4495200,22.05; 4495800,22.05; 4496400,22.01; 4497000,
            22.05; 4497600,22.01; 4498200,22.01; 4498800,22.01; 4499400,22.01; 4500000,
            22.01; 4500600,22.01; 4501200,22.01; 4501800,22.01; 4502400,22.01; 4503000,
            22.01; 4503600,22.01; 4504200,22.01; 4504800,22; 4505400,21.97; 4506000,
            22.02; 4506600,21.97; 4507200,21.97; 4507800,21.97; 4508400,22.06; 4509000,
            22.34; 4509600,22.58; 4510200,22.78; 4510800,22.95; 4511400,23.06; 4512000,
            23.03; 4512600,22.96; 4513200,22.83; 4513800,22.7; 4514400,22.62; 4515000,
            22.7; 4515600,22.78; 4516200,22.87; 4516800,22.91; 4517400,22.97; 4518000,
            23; 4518600,22.91; 4519200,22.87; 4519800,22.86; 4520400,22.87; 4521000,
            22.94; 4521600,22.99; 4522200,22.95; 4522800,23.03; 4523400,23.08; 4524000,
            23.15; 4524600,23.15; 4525200,23.19; 4525800,23.27; 4526400,23.28; 4527000,
            23.38; 4527600,23.44; 4528200,23.51; 4528800,23.56; 4529400,23.6; 4530000,
            23.64; 4530600,23.68; 4531200,23.72; 4531800,23.73; 4532400,23.76; 4533000,
            23.72; 4533600,23.69; 4534200,23.68; 4534800,23.64; 4535400,23.64; 4536000,
            23.61; 4536600,23.68; 4537200,23.72; 4537800,23.76; 4538400,23.8; 4539000,
            23.8; 4539600,23.86; 4540200,23.84; 4540800,23.85; 4541400,23.84; 4542000,
            23.88; 4542600,23.92; 4543200,23.96; 4543800,23.97; 4544400,23.96; 4545000,
            23.94; 4545600,23.96; 4546200,23.94; 4546800,23.92; 4547400,23.92; 4548000,
            23.92; 4548600,23.96; 4549200,24.01; 4549800,23.96; 4550400,23.98; 4551000,
            23.96; 4551600,23.95; 4552200,23.88; 4552800,23.84; 4553400,23.8; 4554000,
            23.8; 4554600,23.76; 4555200,23.77; 4555800,23.81; 4556400,23.8; 4557000,
            23.86; 4557600,23.84; 4558200,23.8; 4558800,23.8; 4559400,23.8; 4560000,
            23.76; 4560600,23.72; 4561200,23.68; 4561800,23.68; 4562400,23.6; 4563000,
            23.56; 4563600,23.48; 4564200,23.43; 4564800,23.36; 4565400,23.27; 4566000,
            23.19; 4566600,23.11; 4567200,23.03; 4567800,22.99; 4568400,22.95; 4569000,
            22.91; 4569600,22.87; 4570200,22.83; 4570800,22.78; 4571400,22.74; 4572000,
            22.7; 4572600,22.66; 4573200,22.66; 4573800,22.62; 4574400,22.62; 4575000,
            22.58; 4575600,22.54; 4576200,22.53; 4576800,22.5; 4577400,22.5; 4578000,
            22.46; 4578600,22.47; 4579200,22.42; 4579800,22.42; 4580400,22.42; 4581000,
            22.38; 4581600,22.39; 4582200,22.38; 4582800,22.37; 4583400,22.34; 4584000,
            22.33; 4584600,22.3; 4585200,22.3; 4585800,22.3; 4586400,22.27; 4587000,
            22.26; 4587600,22.26; 4588200,22.26; 4588800,22.26; 4589400,22.26; 4590000,
            22.26; 4590600,22.26; 4591200,22.22; 4591800,22.23; 4592400,22.22; 4593000,
            22.22; 4593600,22.22; 4594200,22.18; 4594800,22.22; 4595400,22.38; 4596000,
            22.55; 4596600,22.62; 4597200,22.71; 4597800,22.82; 4598400,22.83; 4599000,
            22.83; 4599600,22.83; 4600200,22.83; 4600800,22.83; 4601400,22.78; 4602000,
            22.81; 4602600,22.83; 4603200,22.87; 4603800,22.89; 4604400,22.95; 4605000,
            23.08; 4605600,23.24; 4606200,23.27; 4606800,23.4; 4607400,23.48; 4608000,
            23.56; 4608600,23.6; 4609200,23.72; 4609800,23.76; 4610400,23.8; 4611000,
            23.84; 4611600,23.88; 4612200,23.84; 4612800,23.88; 4613400,23.88; 4614000,
            23.92; 4614600,23.96; 4615200,24.02; 4615800,24.05; 4616400,24.1; 4617000,
            24.12; 4617600,24.09; 4618200,24.09; 4618800,24.09; 4619400,24.13; 4620000,
            24.05; 4620600,24.03; 4621200,24.01; 4621800,24; 4622400,23.96; 4623000,
            24.01; 4623600,24.04; 4624200,24.05; 4624800,24.09; 4625400,24.17; 4626000,
            24.21; 4626600,24.25; 4627200,24.29; 4627800,24.33; 4628400,24.32; 4629000,
            24.33; 4629600,24.33; 4630200,24.37; 4630800,24.37; 4631400,24.37; 4632000,
            24.33; 4632600,24.3; 4633200,24.29; 4633800,24.29; 4634400,24.29; 4635000,
            24.25; 4635600,24.31; 4636200,24.29; 4636800,24.25; 4637400,24.33; 4638000,
            24.3; 4638600,24.29; 4639200,24.3; 4639800,24.27; 4640400,24.21; 4641000,
            24.17; 4641600,24.18; 4642200,24.21; 4642800,24.21; 4643400,24.21; 4644000,
            24.21; 4644600,24.21; 4645200,24.17; 4645800,24.21; 4646400,24.17; 4647000,
            24.17; 4647600,24.17; 4648200,24.13; 4648800,24.09; 4649400,24.05; 4650000,
            24.01; 4650600,23.92; 4651200,23.8; 4651800,23.77; 4652400,23.68; 4653000,
            23.6; 4653600,23.52; 4654200,23.46; 4654800,23.4; 4655400,23.36; 4656000,
            23.35; 4656600,23.32; 4657200,23.27; 4657800,23.23; 4658400,23.2; 4659000,
            23.19; 4659600,23.15; 4660200,23.15; 4660800,23.11; 4661400,23.11; 4662000,
            23.07; 4662600,23.07; 4663200,23.03; 4663800,23.02; 4664400,23.02; 4665000,
            22.99; 4665600,22.95; 4666200,22.95; 4666800,22.95; 4667400,22.94; 4668000,
            22.91; 4668600,22.91; 4669200,22.91; 4669800,22.91; 4670400,22.87; 4671000,
            22.87; 4671600,22.86; 4672200,22.83; 4672800,22.83; 4673400,22.83; 4674000,
            22.82; 4674600,22.8; 4675200,22.78; 4675800,22.78; 4676400,22.78; 4677000,
            22.78; 4677600,22.78; 4678200,22.74; 4678800,22.74; 4679400,22.75; 4680000,
            22.74; 4680600,22.74; 4681200,22.78; 4681800,23.03; 4682400,23.23; 4683000,
            23.4; 4683600,23.52; 4684200,23.48; 4684800,23.43; 4685400,23.36; 4686000,
            23.27; 4686600,23.15; 4687200,23.11; 4687800,23.17; 4688400,23.27; 4689000,
            23.32; 4689600,23.36; 4690200,23.39; 4690800,23.4; 4691400,23.27; 4692000,
            23.24; 4692600,23.23; 4693200,23.23; 4693800,23.32; 4694400,23.4; 4695000,
            23.48; 4695600,23.48; 4696200,23.49; 4696800,23.57; 4697400,23.55; 4698000,
            23.44; 4698600,23.48; 4699200,23.59; 4699800,23.64; 4700400,23.64; 4701000,
            23.68; 4701600,23.76; 4702200,23.72; 4702800,23.73; 4703400,23.76; 4704000,
            23.8; 4704600,23.84; 4705200,23.84; 4705800,23.88; 4706400,23.89; 4707000,
            23.92; 4707600,23.88; 4708200,23.91; 4708800,23.92; 4709400,23.92; 4710000,
            23.93; 4710600,23.88; 4711200,23.92; 4711800,23.96; 4712400,24.05; 4713000,
            24.09; 4713600,24.08; 4714200,24.13; 4714800,24.13; 4715400,24.17; 4716000,
            24.17; 4716600,24.21; 4717200,24.21; 4717800,24.17; 4718400,24.09; 4719000,
            24.08; 4719600,24.13; 4720200,24.25; 4720800,24.25; 4721400,24.26; 4722000,
            24.29; 4722600,24.25; 4723200,24.21; 4723800,24.14; 4724400,24.13; 4725000,
            24.09; 4725600,24.1; 4726200,24.13; 4726800,24.09; 4727400,24.05; 4728000,
            24.01; 4728600,23.96; 4729200,23.99; 4729800,23.96; 4730400,23.92; 4731000,
            23.92; 4731600,23.88; 4732200,23.83; 4732800,23.8; 4733400,23.76; 4734000,
            23.73; 4734600,23.68; 4735200,23.64; 4735800,23.64; 4736400,23.56; 4737000,
            23.48; 4737600,23.4; 4738200,23.32; 4738800,23.28; 4739400,23.15; 4740000,
            23.09; 4740600,23.03; 4741200,22.96; 4741800,22.91; 4742400,22.87; 4743000,
            22.84; 4743600,22.78; 4744200,22.74; 4744800,22.7; 4745400,22.66; 4746000,
            22.62; 4746600,22.58; 4747200,22.58; 4747800,22.54; 4748400,22.5; 4749000,
            22.46; 4749600,22.46; 4750200,22.42; 4750800,22.42; 4751400,22.41; 4752000,
            22.38; 4752600,22.38; 4753200,22.35; 4753800,22.34; 4754400,22.34; 4755000,
            22.3; 4755600,22.27; 4756200,22.29; 4756800,22.26; 4757400,22.26; 4758000,
            22.26; 4758600,22.26; 4759200,22.42; 4759800,22.54; 4760400,22.66; 4761000,
            22.78; 4761600,22.9; 4762200,22.99; 4762800,23.03; 4763400,23.06; 4764000,
            23.03; 4764600,22.99; 4765200,22.91; 4765800,22.84; 4766400,22.83; 4767000,
            22.88; 4767600,22.92; 4768200,22.91; 4768800,22.91; 4769400,22.92; 4770000,
            22.91; 4770600,22.91; 4771200,22.91; 4771800,22.91; 4772400,22.91; 4773000,
            22.92; 4773600,22.91; 4774200,22.91; 4774800,22.91; 4775400,22.91; 4776000,
            22.91; 4776600,22.91; 4777200,22.91; 4777800,22.95; 4778400,23.03; 4779000,
            23.07; 4779600,23.16; 4780200,23.26; 4780800,23.35; 4781400,23.44; 4782000,
            23.53; 4782600,23.6; 4783200,23.64; 4783800,23.68; 4784400,23.68; 4785000,
            23.72; 4785600,23.72; 4786200,23.72; 4786800,23.76; 4787400,23.8; 4788000,
            23.84; 4788600,23.88; 4789200,23.92; 4789800,23.92; 4790400,23.94; 4791000,
            23.98; 4791600,24.05; 4792200,24.1; 4792800,24.09; 4793400,24.05; 4794000,
            24.01; 4794600,23.96; 4795200,23.93; 4795800,24; 4796400,24.01; 4797000,
            24.09; 4797600,24.21; 4798200,24.3; 4798800,24.34; 4799400,24.33; 4800000,
            24.37; 4800600,24.34; 4801200,24.34; 4801800,24.29; 4802400,24.27; 4803000,
            24.29; 4803600,24.29; 4804200,24.33; 4804800,24.33; 4805400,24.33; 4806000,
            24.33; 4806600,24.33; 4807200,24.29; 4807800,24.29; 4808400,24.33; 4809000,
            24.33; 4809600,24.38; 4810200,24.33; 4810800,24.29; 4811400,24.25; 4812000,
            24.22; 4812600,24.12; 4813200,24.17; 4813800,24.13; 4814400,24.05; 4815000,
            24.01; 4815600,23.92; 4816200,23.92; 4816800,23.88; 4817400,23.84; 4818000,
            23.88; 4818600,23.88; 4819200,23.88; 4819800,23.88; 4820400,23.88; 4821000,
            23.84; 4821600,23.84; 4822200,23.76; 4822800,23.73; 4823400,23.68; 4824000,
            23.6; 4824600,23.56; 4825200,23.44; 4825800,23.4; 4826400,23.36; 4827000,
            23.31; 4827600,23.24; 4828200,23.15; 4828800,23.11; 4829400,23.03; 4830000,
            23; 4830600,22.95; 4831200,22.88; 4831800,22.83; 4832400,22.78; 4833000,
            22.8; 4833600,22.74; 4834200,22.7; 4834800,22.66; 4835400,22.62; 4836000,
            22.62; 4836600,22.58; 4837200,22.55; 4837800,22.52; 4838400,22.5; 4839000,
            22.46; 4839600,22.47; 4840200,22.42; 4840800,22.42; 4841400,22.38; 4842000,
            22.38; 4842600,22.38; 4843200,22.35; 4843800,22.34; 4844400,22.38; 4845000,
            22.38; 4845600,22.39; 4846200,22.42; 4846800,22.42; 4847400,22.45; 4848000,
            22.46; 4848600,22.47; 4849200,22.46; 4849800,22.46; 4850400,22.46; 4851000,
            22.46; 4851600,22.46; 4852200,22.46; 4852800,22.46; 4853400,22.46; 4854000,
            22.62; 4854600,22.87; 4855200,23.07; 4855800,23.24; 4856400,23.32; 4857000,
            23.27; 4857600,23.19; 4858200,23.11; 4858800,23; 4859400,22.9; 4860000,22.83;
            4860600,22.93; 4861200,23.07; 4861800,23.15; 4862400,23.19; 4863000,23.2;
            4863600,23.16; 4864200,23.32; 4864800,23.03; 4865400,22.99; 4866000,23.03;
            4866600,23.07; 4867200,23.11; 4867800,23.23; 4868400,23.36; 4869000,23.4;
            4869600,23.44; 4870200,23.46; 4870800,23.48; 4871400,23.44; 4872000,23.45;
            4872600,23.52; 4873200,23.56; 4873800,23.56; 4874400,23.6; 4875000,23.68;
            4875600,23.72; 4876200,23.72; 4876800,23.77; 4877400,23.85; 4878000,23.97;
            4878600,23.92; 4879200,23.92; 4879800,23.88; 4880400,23.88; 4881000,23.88;
            4881600,23.92; 4882200,23.92; 4882800,24.05; 4883400,24.21; 4884000,24.25;
            4884600,24.25; 4885200,24.29; 4885800,24.33; 4886400,24.25; 4887000,24.25;
            4887600,24.29; 4888200,24.34; 4888800,24.38; 4889400,24.41; 4890000,24.41;
            4890600,24.38; 4891200,24.11; 4891800,24.21; 4892400,24.29; 4893000,24.31;
            4893600,24.33; 4894200,24.33; 4894800,24.37; 4895400,24.37; 4896000,24.25;
            4896600,24.33; 4897200,24.21; 4897800,24.13; 4898400,24.13; 4899000,24.12;
            4899600,24.1; 4900200,24.13; 4900800,24.05; 4901400,23.92; 4902000,23.88;
            4902600,23.84; 4903200,23.8; 4903800,23.76; 4904400,23.72; 4905000,23.65;
            4905600,23.59; 4906200,23.52; 4906800,23.48; 4907400,23.44; 4908000,23.4;
            4908600,23.4; 4909200,23.32; 4909800,23.24; 4910400,23.15; 4911000,23.08;
            4911600,23.02; 4912200,22.98; 4912800,22.91; 4913400,22.83; 4914000,22.8;
            4914600,22.74; 4915200,22.7; 4915800,22.64; 4916400,22.58; 4917000,22.54;
            4917600,22.46; 4918200,22.44; 4918800,22.38; 4919400,22.38; 4920000,22.34;
            4920600,22.3; 4921200,22.26; 4921800,22.22; 4922400,22.18; 4923000,22.14;
            4923600,22.09; 4924200,22.05; 4924800,21.97; 4925400,21.97; 4926000,21.89;
            4926600,21.85; 4927200,21.85; 4927800,21.81; 4928400,21.77; 4929000,21.72;
            4929600,21.66; 4930200,21.61; 4930800,21.6; 4931400,21.56; 4932000,21.52;
            4932600,21.52; 4933200,21.48; 4933800,21.44; 4934400,21.45; 4935000,21.4;
            4935600,21.36; 4936200,21.37; 4936800,21.32; 4937400,21.32; 4938000,21.32;
            4938600,21.28; 4939200,21.27; 4939800,21.24; 4940400,21.23; 4941000,21.2;
            4941600,21.2; 4942200,21.16; 4942800,21.16; 4943400,21.12; 4944000,21.12;
            4944600,21.12; 4945200,21.09; 4945800,21.08; 4946400,21.04; 4947000,21.04;
            4947600,21.04; 4948200,21; 4948800,21; 4949400,21; 4950000,20.99; 4950600,
            20.96; 4951200,20.96; 4951800,20.96; 4952400,20.96; 4953000,20.96; 4953600,
            20.96; 4954200,20.96; 4954800,20.96; 4955400,20.96; 4956000,20.99; 4956600,
            21; 4957200,21; 4957800,21; 4958400,21.02; 4959000,21.04; 4959600,21.04;
            4960200,21.04; 4960800,21.08; 4961400,21.08; 4962000,21.12; 4962600,21.12;
            4963200,21.16; 4963800,21.16; 4964400,21.17; 4965000,21.2; 4965600,21.2;
            4966200,21.24; 4966800,21.24; 4967400,21.24; 4968000,21.26; 4968600,21.26;
            4969200,21.27; 4969800,21.24; 4970400,21.24; 4971000,21.24; 4971600,21.21;
            4972200,21.2; 4972800,21.2; 4973400,21.2; 4974000,21.16; 4974600,21.16;
            4975200,21.17; 4975800,21.16; 4976400,21.16; 4977000,21.17; 4977600,21.12;
            4978200,21.12; 4978800,21.12; 4979400,21.12; 4980000,21.12; 4980600,21.12;
            4981200,21.12; 4981800,21.1; 4982400,21.08; 4983000,21.08; 4983600,21.08;
            4984200,21.08; 4984800,21.04; 4985400,21.04; 4986000,21.02; 4986600,21;
            4987200,20.99; 4987800,20.96; 4988400,20.96; 4989000,20.96; 4989600,20.95;
            4990200,20.91; 4990800,20.91; 4991400,20.88; 4992000,20.87; 4992600,20.87;
            4993200,20.87; 4993800,20.85; 4994400,20.83; 4995000,20.83; 4995600,20.83;
            4996200,20.79; 4996800,20.79; 4997400,20.79; 4998000,20.8; 4998600,20.79;
            4999200,20.75; 4999800,20.75; 5000400,20.75; 5001000,20.71; 5001600,20.71;
            5002200,20.71; 5002800,20.67; 5003400,20.67; 5004000,20.67; 5004600,20.67;
            5005200,20.66; 5005800,20.63; 5006400,20.63; 5007000,20.63; 5007600,20.63;
            5008200,20.59; 5008800,20.7; 5009400,20.79; 5010000,20.87; 5010600,20.91;
            5011200,20.96; 5011800,21; 5012400,21.04; 5013000,21.06; 5013600,21.08;
            5014200,21.12; 5014800,21.12; 5015400,21.16; 5016000,21.16; 5016600,21.2;
            5017200,21.2; 5017800,21.2; 5018400,21.24; 5019000,21.24; 5019600,21.27;
            5020200,21.28; 5020800,21.28; 5021400,21.28; 5022000,21.28; 5022600,21.28;
            5023200,21.33; 5023800,21.32; 5024400,21.32; 5025000,21.32; 5025600,21.32;
            5026200,21.36; 5026800,21.36; 5027400,21.36; 5028000,21.36; 5028600,21.36;
            5029200,21.36; 5029800,21.36; 5030400,21.36; 5031000,21.36; 5031600,21.37;
            5032200,21.4; 5032800,21.4; 5033400,21.4; 5034000,21.36; 5034600,21.36;
            5035200,21.4; 5035800,21.4; 5036400,21.4; 5037000,21.4; 5037600,21.4; 5038200,
            21.4; 5038800,21.4; 5039400,21.45; 5040000,21.44; 5040600,21.44; 5041200,
            21.44; 5041800,21.44; 5042400,21.44; 5043000,21.48; 5043600,21.48; 5044200,
            21.48; 5044800,21.52; 5045400,21.56; 5046000,21.56; 5046600,21.64; 5047200,
            21.69; 5047800,21.73; 5048400,21.76; 5049000,21.77; 5049600,21.77; 5050200,
            21.77; 5050800,21.77; 5051400,21.81; 5052000,21.85; 5052600,21.85; 5053200,
            21.89; 5053800,21.93; 5054400,21.93; 5055000,21.96; 5055600,21.97; 5056200,
            21.97; 5056800,21.97; 5057400,21.97; 5058000,21.97; 5058600,21.97; 5059200,
            22.01; 5059800,22.09; 5060400,22.09; 5061000,22.05; 5061600,22.05; 5062200,
            22.05; 5062800,22.05; 5063400,22.01; 5064000,21.97; 5064600,21.97; 5065200,
            21.97; 5065800,21.97; 5066400,21.93; 5067000,21.93; 5067600,21.93; 5068200,
            21.89; 5068800,21.87; 5069400,21.85; 5070000,21.85; 5070600,21.85; 5071200,
            21.81; 5071800,21.81; 5072400,21.78; 5073000,21.77; 5073600,21.78; 5074200,
            21.73; 5074800,21.74; 5075400,21.73; 5076000,21.73; 5076600,21.69; 5077200,
            21.69; 5077800,21.69; 5078400,21.65; 5079000,21.65; 5079600,21.65; 5080200,
            21.65; 5080800,21.66; 5081400,21.64; 5082000,21.62; 5082600,21.6; 5083200,
            21.6; 5083800,21.6; 5084400,21.59; 5085000,21.57; 5085600,21.56; 5086200,
            21.58; 5086800,21.56; 5087400,21.56; 5088000,21.56; 5088600,21.52; 5089200,
            21.54; 5089800,21.54; 5090400,21.52; 5091000,21.48; 5091600,21.48; 5092200,
            21.48; 5092800,21.48; 5093400,21.48; 5094000,21.44; 5094600,21.44; 5095200,
            21.44; 5095800,21.44; 5096400,21.44; 5097000,21.44; 5097600,21.44; 5098200,
            21.42; 5098800,21.4; 5099400,21.4; 5100000,21.4; 5100600,21.4; 5101200,21.4;
            5101800,21.4; 5102400,21.37; 5103000,21.36; 5103600,21.36; 5104200,21.36;
            5104800,21.32; 5105400,21.36; 5106000,21.32; 5106600,21.32; 5107200,21.32;
            5107800,21.33; 5108400,21.32; 5109000,21.32; 5109600,21.32; 5110200,21.32;
            5110800,21.33; 5111400,21.32; 5112000,21.32; 5112600,21.32; 5113200,21.41;
            5113800,21.62; 5114400,21.73; 5115000,21.81; 5115600,21.93; 5116200,22.05;
            5116800,22.05; 5117400,22.05; 5118000,22.05; 5118600,22.05; 5119200,22.01;
            5119800,22.03; 5120400,22.05; 5121000,22.09; 5121600,22.18; 5122200,22.18;
            5122800,22.18; 5123400,22.23; 5124000,22.27; 5124600,22.3; 5125200,22.38;
            5125800,22.5; 5126400,22.62; 5127000,22.7; 5127600,22.7; 5128200,22.82;
            5128800,22.91; 5129400,22.88; 5130000,22.9; 5130600,22.95; 5131200,23; 5131800,
            23.08; 5132400,23.04; 5133000,22.99; 5133600,23.03; 5134200,23.03; 5134800,
            23.03; 5135400,23.08; 5136000,23.15; 5136600,23.2; 5137200,23.23; 5137800,
            23.2; 5138400,23.18; 5139000,23.15; 5139600,23.15; 5140200,23.17; 5140800,
            23.19; 5141400,23.24; 5142000,23.35; 5142600,23.32; 5143200,23.32; 5143800,
            23.35; 5144400,23.36; 5145000,23.4; 5145600,23.4; 5146200,23.35; 5146800,
            23.36; 5147400,23.36; 5148000,23.4; 5148600,23.4; 5149200,23.44; 5149800,
            23.52; 5150400,23.44; 5151000,23.53; 5151600,23.49; 5152200,23.52; 5152800,
            23.56; 5153400,23.56; 5154000,23.64; 5154600,23.64; 5155200,23.65; 5155800,
            23.68; 5156400,23.68; 5157000,23.63; 5157600,23.6; 5158200,23.6; 5158800,
            23.6; 5159400,23.58; 5160000,23.52; 5160600,23.44; 5161200,23.44; 5161800,
            23.43; 5162400,23.42; 5163000,23.4; 5163600,23.4; 5164200,23.39; 5164800,
            23.36; 5165400,23.35; 5166000,23.27; 5166600,23.23; 5167200,23.15; 5167800,
            23.07; 5168400,22.99; 5169000,22.91; 5169600,22.8; 5170200,22.7; 5170800,
            22.64; 5171400,22.54; 5172000,22.42; 5172600,22.37; 5173200,22.3; 5173800,
            22.25; 5174400,22.18; 5175000,22.14; 5175600,22.09; 5176200,22.05; 5176800,
            21.97; 5177400,21.93; 5178000,21.9; 5178600,21.85; 5179200,21.81; 5179800,
            21.8; 5180400,21.77; 5181000,21.73; 5181600,21.7; 5182200,21.65; 5182800,
            21.65; 5183400,21.66; 5184000,21.6; 5184600,21.6; 5185200,21.57; 5185800,
            21.56; 5186400,21.52; 5187000,21.53; 5187600,21.55; 5188200,21.53; 5188800,
            21.52; 5189400,21.48; 5190000,21.48; 5190600,21.48; 5191200,21.65; 5191800,
            21.81; 5192400,21.98; 5193000,22.09; 5193600,22.14; 5194200,22.22; 5194800,
            22.3; 5195400,22.34; 5196000,22.38; 5196600,22.38; 5197200,22.34; 5197800,
            22.31; 5198400,22.3; 5199000,22.3; 5199600,22.3; 5200200,22.3; 5200800,22.34;
            5201400,22.34; 5202000,22.34; 5202600,22.36; 5203200,22.34; 5203800,22.35;
            5204400,22.34; 5205000,22.34; 5205600,22.38; 5206200,22.38; 5206800,22.38;
            5207400,22.38; 5208000,22.38; 5208600,22.38; 5209200,22.38; 5209800,22.42;
            5210400,22.46; 5211000,22.54; 5211600,22.62; 5212200,22.72; 5212800,22.74;
            5213400,22.83; 5214000,22.99; 5214600,23.07; 5215200,23.16; 5215800,23.27;
            5216400,23.29; 5217000,23.35; 5217600,23.39; 5218200,23.42; 5218800,23.48;
            5219400,23.49; 5220000,23.56; 5220600,23.56; 5221200,23.68; 5221800,23.72;
            5222400,23.76; 5223000,23.76; 5223600,23.84; 5224200,23.76; 5224800,23.76;
            5225400,23.76; 5226000,23.8; 5226600,23.84; 5227200,23.88; 5227800,24.01;
            5228400,24.16; 5229000,24.25; 5229600,24.4; 5230200,24.46; 5230800,24.54;
            5231400,24.58; 5232000,24.58; 5232600,24.5; 5233200,24.59; 5233800,24.58;
            5234400,24.62; 5235000,24.66; 5235600,24.66; 5236200,24.66; 5236800,24.66;
            5237400,24.69; 5238000,24.62; 5238600,24.62; 5239200,24.58; 5239800,24.53;
            5240400,24.49; 5241000,24.45; 5241600,24.45; 5242200,24.37; 5242800,24.38;
            5243400,24.38; 5244000,24.29; 5244600,24.21; 5245200,24.13; 5245800,24.21;
            5246400,24.22; 5247000,24.13; 5247600,24.13; 5248200,24.17; 5248800,24.13;
            5249400,24.09; 5250000,24.05; 5250600,24.01; 5251200,23.92; 5251800,23.88;
            5252400,23.8; 5253000,23.76; 5253600,23.72; 5254200,23.65; 5254800,23.6;
            5255400,23.56; 5256000,23.56; 5256600,23.47; 5257200,23.63; 5257800,23.32;
            5258400,23.27; 5259000,23.19; 5259600,23.19; 5260200,23.1; 5260800,23.07;
            5261400,22.99; 5262000,22.91; 5262600,22.87; 5263200,22.83; 5263800,22.78;
            5264400,22.74; 5265000,22.7; 5265600,22.66; 5266200,22.62; 5266800,22.58;
            5267400,22.58; 5268000,22.5; 5268600,22.5; 5269200,22.46; 5269800,22.42;
            5270400,22.42; 5271000,22.38; 5271600,22.35; 5272200,22.35; 5272800,22.31;
            5273400,22.26; 5274000,22.26; 5274600,22.22; 5275200,22.22; 5275800,22.19;
            5276400,22.18; 5277000,22.22; 5277600,22.22; 5278200,22.22; 5278800,22.25;
            5279400,22.26; 5280000,22.26; 5280600,22.26; 5281200,22.26; 5281800,22.26;
            5282400,22.3; 5283000,22.3; 5283600,22.3; 5284200,22.3; 5284800,22.3; 5285400,
            22.3; 5286000,22.42; 5286600,22.77; 5287200,22.99; 5287800,23.11; 5288400,
            23.11; 5289000,23.07; 5289600,22.95; 5290200,22.83; 5290800,22.74; 5291400,
            22.64; 5292000,22.66; 5292600,22.78; 5293200,22.9; 5293800,22.96; 5294400,
            22.95; 5295000,22.95; 5295600,22.91; 5296200,22.83; 5296800,22.74; 5297400,
            22.67; 5298000,22.66; 5298600,22.71; 5299200,22.82; 5299800,22.88; 5300400,
            22.91; 5301000,22.95; 5301600,23.03; 5302200,23.08; 5302800,23.07; 5303400,
            23.12; 5304000,23.11; 5304600,23.11; 5305200,23.23; 5305800,23.27; 5306400,
            23.3; 5307000,23.4; 5307600,23.44; 5308200,23.52; 5308800,23.56; 5309400,
            23.56; 5310000,23.56; 5310600,23.52; 5311200,23.48; 5311800,23.4; 5312400,
            23.36; 5313000,23.4; 5313600,23.4; 5314200,23.4; 5314800,23.49; 5315400,
            23.48; 5316000,23.57; 5316600,23.64; 5317200,23.72; 5317800,23.77; 5318400,
            23.84; 5319000,23.89; 5319600,23.8; 5320200,23.84; 5320800,23.88; 5321400,
            23.92; 5322000,23.92; 5322600,23.92; 5323200,23.92; 5323800,23.96; 5324400,
            24; 5325000,24; 5325600,24.01; 5326200,24.09; 5326800,24.04; 5327400,24.08;
            5328000,24.05; 5328600,24.09; 5329200,24.07; 5329800,24.06; 5330400,24.05;
            5331000,24.05; 5331600,24.05; 5332200,24.04; 5332800,24.01; 5333400,24.02;
            5334000,24.01; 5334600,24.01; 5335200,23.97; 5335800,23.96; 5336400,23.94;
            5337000,23.92; 5337600,23.88; 5338200,23.8; 5338800,23.76; 5339400,23.72;
            5340000,23.68; 5340600,23.61; 5341200,23.52; 5341800,23.58; 5342400,23.43;
            5343000,23.43; 5343600,23.4; 5344200,23.32; 5344800,23.22; 5345400,23.11;
            5346000,23.07; 5346600,23.03; 5347200,22.99; 5347800,22.91; 5348400,22.88;
            5349000,22.82; 5349600,22.74; 5350200,22.7; 5350800,22.64; 5351400,22.58;
            5352000,22.56; 5352600,22.5; 5353200,22.47; 5353800,22.42; 5354400,22.42;
            5355000,22.38; 5355600,22.34; 5356200,22.3; 5356800,22.26; 5357400,22.26;
            5358000,22.22; 5358600,22.22; 5359200,22.18; 5359800,22.14; 5360400,22.12;
            5361000,22.09; 5361600,22.09; 5362200,22.09; 5362800,22.09; 5363400,22.09;
            5364000,22.26; 5364600,22.46; 5365200,22.62; 5365800,22.78; 5366400,22.95;
            5367000,23.03; 5367600,22.99; 5368200,22.91; 5368800,22.78; 5369400,22.71;
            5370000,22.66; 5370600,22.74; 5371200,22.83; 5371800,22.91; 5372400,22.94;
            5373000,22.91; 5373600,22.83; 5374200,22.78; 5374800,22.74; 5375400,22.74;
            5376000,22.74; 5376600,22.74; 5377200,22.74; 5377800,22.74; 5378400,22.74;
            5379000,22.74; 5379600,22.74; 5380200,22.74; 5380800,22.75; 5381400,22.74;
            5382000,22.72; 5382600,22.7; 5383200,22.7; 5383800,22.7; 5384400,22.67;
            5385000,22.7; 5385600,22.74; 5386200,22.83; 5386800,23.01; 5387400,23.07;
            5388000,23.23; 5388600,23.28; 5389200,23.36; 5389800,23.4; 5390400,23.48;
            5391000,23.48; 5391600,23.57; 5392200,23.6; 5392800,23.64; 5393400,23.68;
            5394000,23.64; 5394600,23.64; 5395200,23.64; 5395800,23.68; 5396400,23.64;
            5397000,23.6; 5397600,23.64; 5398200,23.64; 5398800,23.68; 5399400,23.73;
            5400000,23.72; 5400600,23.77; 5401200,23.72; 5401800,23.76; 5402400,23.8;
            5403000,23.88; 5403600,23.88; 5404200,23.86; 5404800,23.84; 5405400,23.82;
            5406000,23.8; 5406600,23.76; 5407200,23.68; 5407800,23.68; 5408400,23.68;
            5409000,23.72; 5409600,23.76; 5410200,23.77; 5410800,23.72; 5411400,23.76;
            5412000,23.79; 5412600,23.81; 5413200,23.85; 5413800,23.84; 5414400,23.84;
            5415000,23.88; 5415600,23.88; 5416200,21.85; 5416800,23.24; 5417400,23.6;
            5418000,23.84; 5418600,23.84; 5419200,23.88; 5419800,23.88; 5420400,23.84;
            5421000,23.72; 5421600,23.64; 5422200,23.6; 5422800,23.56; 5423400,23.56;
            5424000,23.56; 5424600,23.52; 5425200,23.5; 5425800,23.44; 5426400,23.4;
            5427000,23.36; 5427600,23.27; 5428200,23.19; 5428800,23.13; 5429400,23.06;
            5430000,22.99; 5430600,22.87; 5431200,22.83; 5431800,22.74; 5432400,22.7;
            5433000,22.62; 5433600,22.58; 5434200,22.5; 5434800,22.46; 5435400,22.42;
            5436000,22.38; 5436600,22.34; 5437200,22.3; 5437800,22.26; 5438400,22.22;
            5439000,22.18; 5439600,22.13; 5440200,22.09; 5440800,22.05; 5441400,22.01;
            5442000,22.02; 5442600,21.97; 5443200,21.97; 5443800,21.93; 5444400,21.89;
            5445000,21.85; 5445600,21.82; 5446200,21.81; 5446800,21.81; 5447400,21.77;
            5448000,21.76; 5448600,21.77; 5449200,21.77; 5449800,21.77; 5450400,21.73;
            5451000,21.73; 5451600,21.77; 5452200,21.77; 5452800,21.73; 5453400,21.73;
            5454000,21.73; 5454600,21.73; 5455200,21.73; 5455800,21.73; 5456400,21.73;
            5457000,21.73; 5457600,21.73; 5458200,21.78; 5458800,21.89; 5459400,22.01;
            5460000,22.14; 5460600,22.26; 5461200,22.38; 5461800,22.45; 5462400,22.42;
            5463000,22.3; 5463600,22.22; 5464200,22.18; 5464800,22.14; 5465400,22.18;
            5466000,22.18; 5466600,22.22; 5467200,22.22; 5467800,22.25; 5468400,22.26;
            5469000,22.3; 5469600,22.37; 5470200,22.47; 5470800,22.46; 5471400,22.51;
            5472000,22.62; 5472600,22.74; 5473200,22.83; 5473800,22.96; 5474400,23;
            5475000,23.07; 5475600,23.16; 5476200,23.23; 5476800,23.27; 5477400,23.4;
            5478000,23.35; 5478600,23.35; 5479200,23.36; 5479800,23.43; 5480400,23.48;
            5481000,23.56; 5481600,23.56; 5482200,23.54; 5482800,23.56; 5483400,23.52;
            5484000,23.44; 5484600,23.36; 5485200,23.28; 5485800,23.27; 5486400,23.23;
            5487000,23.19; 5487600,23.2; 5488200,23.25; 5488800,23.36; 5489400,23.32;
            5490000,23.3; 5490600,23.36; 5491200,23.32; 5491800,23.31; 5492400,23.32;
            5493000,23.36; 5493600,23.36; 5494200,23.48; 5494800,23.52; 5495400,23.57;
            5496000,23.56; 5496600,23.56; 5497200,23.6; 5497800,23.6; 5498400,23.72;
            5499000,23.64; 5499600,23.64; 5500200,23.64; 5500800,23.66; 5501400,23.64;
            5502000,23.63; 5502600,23.64; 5503200,23.65; 5503800,23.6; 5504400,23.64;
            5505000,23.56; 5505600,23.4; 5506200,23.36; 5506800,23.32; 5507400,23.27;
            5508000,23.23; 5508600,23.19; 5509200,23.19; 5509800,23.15; 5510400,23.11;
            5511000,23.11; 5511600,23.07; 5512200,23.05; 5512800,23.03; 5513400,22.95;
            5514000,22.92; 5514600,22.81; 5515200,22.74; 5515800,22.66; 5516400,22.62;
            5517000,22.54; 5517600,22.46; 5518200,22.43; 5518800,22.42; 5519400,22.3;
            5520000,22.26; 5520600,22.22; 5521200,22.18; 5521800,22.14; 5522400,22.1;
            5523000,22.09; 5523600,22.05; 5524200,22.06; 5524800,22.01; 5525400,22.01;
            5526000,21.97; 5526600,21.97; 5527200,21.93; 5527800,21.89; 5528400,21.85;
            5529000,21.81; 5529600,21.81; 5530200,21.77; 5530800,21.73; 5531400,21.69;
            5532000,21.65; 5532600,21.65; 5533200,21.6; 5533800,21.6; 5534400,21.58;
            5535000,21.52; 5535600,21.52; 5536200,21.48; 5536800,21.44; 5537400,21.41;
            5538000,21.4; 5538600,21.36; 5539200,21.32; 5539800,21.32; 5540400,21.28;
            5541000,21.26; 5541600,21.24; 5542200,21.18; 5542800,21.16; 5543400,21.12;
            5544000,21.13; 5544600,21.08; 5545200,21.08; 5545800,21.04; 5546400,21;
            5547000,20.99; 5547600,20.97; 5548200,20.94; 5548800,20.91; 5549400,20.87;
            5550000,20.83; 5550600,20.81; 5551200,20.79; 5551800,20.75; 5552400,20.71;
            5553000,20.71; 5553600,20.67; 5554200,20.67; 5554800,20.63; 5555400,20.63;
            5556000,20.63; 5556600,20.62; 5557200,20.55; 5557800,20.55; 5558400,20.54;
            5559000,20.52; 5559600,20.51; 5560200,20.51; 5560800,20.51; 5561400,20.51;
            5562000,20.51; 5562600,20.51; 5563200,20.51; 5563800,20.51; 5564400,20.51;
            5565000,20.49; 5565600,20.46; 5566200,20.46; 5566800,20.46; 5567400,20.5;
            5568000,20.54; 5568600,20.55; 5569200,20.59; 5569800,20.63; 5570400,20.63;
            5571000,20.71; 5571600,20.74; 5572200,20.76; 5572800,20.83; 5573400,20.92;
            5574000,21; 5574600,21.08; 5575200,21.2; 5575800,21.32; 5576400,21.4; 5577000,
            21.48; 5577600,21.52; 5578200,21.45; 5578800,21.4; 5579400,21.32; 5580000,
            21.32; 5580600,21.31; 5581200,21.28; 5581800,21.28; 5582400,21.24; 5583000,
            21.2; 5583600,21.2; 5584200,21.2; 5584800,21.16; 5585400,21.16; 5586000,
            21.12; 5586600,21.07; 5587200,21.03; 5587800,21; 5588400,20.94; 5589000,
            20.9; 5589600,20.87; 5590200,20.75; 5590800,20.75; 5591400,20.67; 5592000,
            20.67; 5592600,20.67; 5593200,20.67; 5593800,20.63; 5594400,20.59; 5595000,
            20.55; 5595600,20.55; 5596200,20.51; 5596800,20.46; 5597400,20.42; 5598000,
            20.42; 5598600,20.38; 5599200,20.37; 5599800,20.34; 5600400,20.34; 5601000,
            20.34; 5601600,20.3; 5602200,20.32; 5602800,20.3; 5603400,20.3; 5604000,
            20.26; 5604600,20.26; 5605200,20.26; 5605800,20.22; 5606400,20.22; 5607000,
            20.2; 5607600,20.21; 5608200,20.18; 5608800,20.19; 5609400,20.19; 5610000,
            20.18; 5610600,20.18; 5611200,20.16; 5611800,20.14; 5612400,20.14; 5613000,
            20.1; 5613600,20.18; 5614200,20.3; 5614800,20.38; 5615400,20.42; 5616000,
            20.46; 5616600,20.51; 5617200,20.55; 5617800,20.59; 5618400,20.63; 5619000,
            20.64; 5619600,20.67; 5620200,20.7; 5620800,20.71; 5621400,20.72; 5622000,
            20.75; 5622600,20.78; 5623200,20.79; 5623800,20.79; 5624400,20.8; 5625000,
            20.83; 5625600,20.83; 5626200,20.83; 5626800,20.88; 5627400,20.87; 5628000,
            20.87; 5628600,20.88; 5629200,20.88; 5629800,20.92; 5630400,20.91; 5631000,
            20.9; 5631600,20.91; 5632200,20.91; 5632800,20.87; 5633400,20.9; 5634000,
            20.92; 5634600,20.92; 5635200,20.9; 5635800,20.91; 5636400,20.87; 5637000,
            20.88; 5637600,20.91; 5638200,20.88; 5638800,20.87; 5639400,20.91; 5640000,
            20.91; 5640600,20.91; 5641200,20.91; 5641800,20.91; 5642400,20.91; 5643000,
            20.92; 5643600,20.93; 5644200,20.97; 5644800,20.96; 5645400,20.96; 5646000,
            20.99; 5646600,21; 5647200,21.04; 5647800,21.04; 5648400,21.06; 5649000,
            21.08; 5649600,21.08; 5650200,21.12; 5650800,21.12; 5651400,21.16; 5652000,
            21.2; 5652600,21.28; 5653200,21.32; 5653800,21.39; 5654400,21.4; 5655000,
            21.4; 5655600,21.41; 5656200,21.44; 5656800,21.44; 5657400,21.44; 5658000,
            21.48; 5658600,21.48; 5659200,21.5; 5659800,21.48; 5660400,21.47; 5661000,
            21.44; 5661600,21.44; 5662200,21.44; 5662800,21.44; 5663400,21.48; 5664000,
            21.52; 5664600,21.54; 5665200,21.52; 5665800,21.52; 5666400,21.52; 5667000,
            21.57; 5667600,21.58; 5668200,21.56; 5668800,21.57; 5669400,21.62; 5670000,
            21.7; 5670600,21.73; 5671200,21.81; 5671800,21.82; 5672400,21.77; 5673000,
            21.7; 5673600,21.68; 5674200,21.65; 5674800,21.64; 5675400,21.62; 5676000,
            21.56; 5676600,21.56; 5677200,21.52; 5677800,21.52; 5678400,21.48; 5679000,
            21.44; 5679600,21.44; 5680200,21.4; 5680800,21.4; 5681400,21.36; 5682000,
            21.36; 5682600,21.32; 5683200,21.32; 5683800,21.32; 5684400,21.32; 5685000,
            21.28; 5685600,21.28; 5686200,21.25; 5686800,21.24; 5687400,21.21; 5688000,
            21.2; 5688600,21.2; 5689200,21.2; 5689800,21.18; 5690400,21.16; 5691000,
            21.16; 5691600,21.16; 5692200,21.16; 5692800,21.12; 5693400,21.12; 5694000,
            21.12; 5694600,21.12; 5695200,21.12; 5695800,21.13; 5696400,21.08; 5697000,
            21.08; 5697600,21.08; 5698200,21.08; 5698800,21.08; 5699400,21.07; 5700000,
            21.04; 5700600,21.04; 5701200,21.04; 5701800,21.04; 5702400,21.04; 5703000,
            21.04; 5703600,21; 5704200,21; 5704800,21; 5705400,21; 5706000,21; 5706600,
            21; 5707200,20.96; 5707800,20.96; 5708400,20.96; 5709000,20.96; 5709600,
            20.96; 5710200,20.96; 5710800,20.96; 5711400,20.96; 5712000,20.96; 5712600,
            20.92; 5713200,20.95; 5713800,20.94; 5714400,20.91; 5715000,20.94; 5715600,
            20.95; 5716200,20.91; 5716800,20.91; 5717400,20.98; 5718000,21.12; 5718600,
            21.2; 5719200,21.32; 5719800,21.46; 5720400,21.6; 5721000,21.65; 5721600,
            21.69; 5722200,21.66; 5722800,21.65; 5723400,21.64; 5724000,21.61; 5724600,
            21.6; 5725200,21.65; 5725800,21.65; 5726400,21.69; 5727000,21.73; 5727600,
            21.77; 5728200,21.85; 5728800,21.96; 5729400,22.01; 5730000,22.06; 5730600,
            22.14; 5731200,22.3; 5731800,22.42; 5732400,22.5; 5733000,22.55; 5733600,
            22.62; 5734200,22.7; 5734800,22.76; 5735400,22.83; 5736000,22.91; 5736600,
            22.96; 5737200,23.03; 5737800,23.07; 5738400,23.15; 5739000,23.19; 5739600,
            23.23; 5740200,23.27; 5740800,23.35; 5741400,23.4; 5742000,23.4; 5742600,
            23.4; 5743200,23.44; 5743800,23.4; 5744400,23.43; 5745000,23.48; 5745600,
            23.48; 5746200,23.52; 5746800,23.6; 5747400,23.72; 5748000,23.86; 5748600,
            24.02; 5749200,24.04; 5749800,24.01; 5750400,24.01; 5751000,23.99; 5751600,
            24.02; 5752200,24.05; 5752800,24.05; 5753400,24.04; 5754000,24.13; 5754600,
            24.13; 5755200,24.06; 5755800,24.01; 5756400,23.96; 5757000,23.95; 5757600,
            23.92; 5758200,23.88; 5758800,23.87; 5759400,23.88; 5760000,23.92; 5760600,
            24.01; 5761200,24; 5761800,23.97; 5762400,23.97; 5763000,23.89; 5763600,
            23.89; 5764200,23.92; 5764800,23.88; 5765400,23.8; 5766000,23.72; 5766600,
            23.68; 5767200,23.64; 5767800,23.56; 5768400,23.48; 5769000,23.44; 5769600,
            23.39; 5770200,23.32; 5770800,23.28; 5771400,23.23; 5772000,23.19; 5772600,
            23.11; 5773200,23.03; 5773800,22.95; 5774400,22.87; 5775000,22.74; 5775600,
            22.66; 5776200,22.62; 5776800,22.54; 5777400,22.46; 5778000,22.42; 5778600,
            22.34; 5779200,22.26; 5779800,22.22; 5780400,22.18; 5781000,22.09; 5781600,
            22.05; 5782200,22.01; 5782800,21.97; 5783400,21.93; 5784000,21.89; 5784600,
            21.82; 5785200,21.81; 5785800,21.77; 5786400,21.73; 5787000,21.73; 5787600,
            21.68; 5788200,21.65; 5788800,21.65; 5789400,21.6; 5790000,21.6; 5790600,
            21.6; 5791200,21.56; 5791800,21.52; 5792400,21.52; 5793000,21.48; 5793600,
            21.48; 5794200,21.51; 5794800,21.52; 5795400,21.52; 5796000,21.56; 5796600,
            21.59; 5797200,21.57; 5797800,21.6; 5798400,21.62; 5799000,21.6; 5799600,
            21.6; 5800200,21.6; 5800800,21.6; 5801400,21.64; 5802000,21.61; 5802600,
            21.63; 5803200,21.64; 5803800,21.65; 5804400,21.94; 5805000,22.09; 5805600,
            22.33; 5806200,22.5; 5806800,22.5; 5807400,22.46; 5808000,22.38; 5808600,
            22.26; 5809200,22.14; 5809800,22.09; 5810400,22.18; 5811000,22.3; 5811600,
            22.38; 5812200,22.42; 5812800,22.46; 5813400,22.42; 5814000,22.38; 5814600,
            22.34; 5815200,22.3; 5815800,22.3; 5816400,22.37; 5817000,22.41; 5817600,
            22.5; 5818200,22.62; 5818800,22.7; 5819400,22.76; 5820000,22.78; 5820600,
            22.87; 5821200,22.91; 5821800,22.95; 5822400,22.96; 5823000,23.04; 5823600,
            23.11; 5824200,23.16; 5824800,23.23; 5825400,23.28; 5826000,23.36; 5826600,
            23.36; 5827200,23.37; 5827800,23.4; 5828400,23.44; 5829000,23.44; 5829600,
            23.45; 5830200,23.44; 5830800,23.4; 5831400,23.39; 5832000,23.44; 5832600,
            23.49; 5833200,23.64; 5833800,23.68; 5834400,23.72; 5835000,23.84; 5835600,
            23.84; 5836200,23.8; 5836800,23.76; 5837400,23.74; 5838000,23.72; 5838600,
            23.76; 5839200,23.8; 5839800,23.8; 5840400,23.82; 5841000,23.8; 5841600,
            23.76; 5842200,23.76; 5842800,23.72; 5843400,23.72; 5844000,23.68; 5844600,
            23.65; 5845200,23.76; 5845800,23.78; 5846400,23.76; 5847000,23.76; 5847600,
            23.88; 5848200,23.86; 5848800,23.82; 5849400,23.86; 5850000,23.76; 5850600,
            23.92; 5851200,23.92; 5851800,23.91; 5852400,23.84; 5853000,23.8; 5853600,
            23.72; 5854200,23.68; 5854800,23.64; 5855400,23.59; 5856000,23.53; 5856600,
            23.52; 5857200,23.48; 5857800,23.44; 5858400,23.4; 5859000,23.32; 5859600,
            23.23; 5860200,23.11; 5860800,23.03; 5861400,22.95; 5862000,22.79; 5862600,
            22.74; 5863200,22.63; 5863800,22.62; 5864400,22.54; 5865000,22.46; 5865600,
            22.41; 5866200,22.39; 5866800,22.3; 5867400,22.26; 5868000,22.22; 5868600,
            22.18; 5869200,22.14; 5869800,22.09; 5870400,22.05; 5871000,22.04; 5871600,
            21.98; 5872200,21.97; 5872800,21.93; 5873400,21.86; 5874000,21.85; 5874600,
            21.81; 5875200,21.77; 5875800,21.77; 5876400,21.73; 5877000,21.73; 5877600,
            21.69; 5878200,21.7; 5878800,21.65; 5879400,21.62; 5880000,21.6; 5880600,
            21.56; 5881200,21.56; 5881800,21.56; 5882400,21.77; 5883000,21.98; 5883600,
            22.1; 5884200,22.27; 5884800,22.39; 5885400,22.46; 5886000,22.54; 5886600,
            22.58; 5887200,22.54; 5887800,22.5; 5888400,22.42; 5889000,22.38; 5889600,
            22.42; 5890200,22.42; 5890800,22.46; 5891400,22.46; 5892000,22.46; 5892600,
            22.46; 5893200,22.46; 5893800,22.51; 5894400,22.5; 5895000,22.5; 5895600,
            22.5; 5896200,22.53; 5896800,22.54; 5897400,22.58; 5898000,22.58; 5898600,
            22.58; 5899200,22.58; 5899800,22.58; 5900400,22.58; 5901000,22.62; 5901600,
            22.62; 5902200,22.64; 5902800,22.62; 5903400,22.62; 5904000,22.62; 5904600,
            22.62; 5905200,22.69; 5905800,22.7; 5906400,22.8; 5907000,22.92; 5907600,
            23.03; 5908200,23.11; 5908800,23.18; 5909400,23.15; 5910000,23.24; 5910600,
            23.32; 5911200,23.4; 5911800,23.39; 5912400,23.3; 5913000,23.26; 5913600,
            23.27; 5914200,23.23; 5914800,23.19; 5915400,23.23; 5916000,23.27; 5916600,
            23.27; 5917200,23.32; 5917800,23.33; 5918400,23.36; 5919000,23.37; 5919600,
            23.4; 5920200,23.47; 5920800,23.56; 5921400,23.71; 5922000,23.76; 5922600,
            23.8; 5923200,23.82; 5923800,23.8; 5924400,23.84; 5925000,23.85; 5925600,
            23.82; 5926200,23.85; 5926800,23.84; 5927400,23.8; 5928000,23.76; 5928600,
            23.76; 5929200,23.74; 5929800,23.72; 5930400,23.75; 5931000,23.72; 5931600,
            23.72; 5932200,23.77; 5932800,23.77; 5933400,23.76; 5934000,23.79; 5934600,
            23.8; 5935200,23.8; 5935800,23.81; 5936400,23.8; 5937000,23.8; 5937600,23.84;
            5938200,23.84; 5938800,23.8; 5939400,23.8; 5940000,23.84; 5940600,23.77;
            5941200,23.72; 5941800,23.69; 5942400,23.64; 5943000,23.56; 5943600,23.52;
            5944200,23.44; 5944800,23.4; 5945400,23.6; 5946000,23.32; 5946600,23.24;
            5947200,23.15; 5947800,23.07; 5948400,22.95; 5949000,22.82; 5949600,22.74;
            5950200,22.62; 5950800,22.54; 5951400,22.46; 5952000,22.46; 5952600,22.38;
            5953200,22.34; 5953800,22.26; 5954400,22.18; 5955000,22.14; 5955600,22.09;
            5956200,22.05; 5956800,22.01; 5957400,21.97; 5958000,21.93; 5958600,21.89;
            5959200,21.9; 5959800,21.82; 5960400,21.81; 5961000,21.77; 5961600,21.75;
            5962200,21.74; 5962800,21.69; 5963400,21.68; 5964000,21.65; 5964600,21.65;
            5965200,21.88; 5965800,21.6; 5966400,21.6; 5967000,21.6; 5967600,21.56;
            5968200,21.6; 5968800,21.81; 5969400,21.93; 5970000,22.01; 5970600,22.13;
            5971200,22.22; 5971800,22.3; 5972400,22.3; 5973000,22.27; 5973600,22.18;
            5974200,22.14; 5974800,22.09; 5975400,22.14; 5976000,22.18; 5976600,22.22;
            5977200,22.23; 5977800,22.26; 5978400,22.22; 5979000,22.21; 5979600,22.18;
            5980200,22.18; 5980800,22.17; 5981400,22.14; 5982000,22.16; 5982600,22.16;
            5983200,22.18; 5983800,22.18; 5984400,22.18; 5985000,22.18; 5985600,22.18;
            5986200,22.18; 5986800,22.25; 5987400,22.26; 5988000,22.3; 5988600,22.39;
            5989200,22.42; 5989800,22.5; 5990400,22.5; 5991000,22.58; 5991600,22.63;
            5992200,22.71; 5992800,22.78; 5993400,22.91; 5994000,23.04; 5994600,23.08;
            5995200,23.12; 5995800,23.11; 5996400,23.11; 5997000,23.12; 5997600,23.15;
            5998200,23.23; 5998800,23.27; 5999400,23.32; 6000000,23.36; 6000600,23.44;
            6001200,23.52; 6001800,23.56; 6002400,23.56; 6003000,23.6; 6003600,23.64;
            6004200,23.64; 6004800,23.64; 6005400,23.76; 6006000,23.88; 6006600,23.92;
            6007200,24.13; 6007800,24.3; 6008400,24.42; 6009000,24.54; 6009600,24.62;
            6010200,24.72; 6010800,24.78; 6011400,24.82; 6012000,24.85; 6012600,24.86;
            6013200,24.86; 6013800,24.9; 6014400,24.9; 6015000,24.94; 6015600,24.86;
            6016200,24.66; 6016800,24.62; 6017400,24.63; 6018000,24.65; 6018600,24.78;
            6019200,25.12; 6019800,24.9; 6020400,24.7; 6021000,24.62; 6021600,24.56;
            6022200,24.45; 6022800,24.37; 6023400,24.25; 6024000,24.21; 6024600,24.13;
            6025200,24.05; 6025800,24.01; 6026400,23.92; 6027000,23.9; 6027600,23.81;
            6028200,23.76; 6028800,23.73; 6029400,23.69; 6030000,23.6; 6030600,23.56;
            6031200,23.52; 6031800,23.48; 6032400,23.44; 6033000,23.36; 6033600,23.27;
            6034200,23.16; 6034800,23.07; 6035400,22.99; 6036000,22.88; 6036600,22.82;
            6037200,22.7; 6037800,22.62; 6038400,22.62; 6039000,22.54; 6039600,22.5;
            6040200,22.46; 6040800,22.41; 6041400,22.38; 6042000,22.34; 6042600,22.31;
            6043200,22.26; 6043800,22.22; 6044400,22.18; 6045000,22.14; 6045600,22.09;
            6046200,22.06; 6046800,22.04; 6047400,21.97; 6048000,21.97; 6048600,21.93;
            6049200,21.89; 6049800,21.87; 6050400,21.82; 6051000,21.81; 6051600,21.81;
            6052200,21.77; 6052800,21.73; 6053400,21.69; 6054000,21.65; 6054600,21.65;
            6055200,21.73; 6055800,21.78; 6056400,21.77; 6057000,21.77; 6057600,21.73;
            6058200,21.74; 6058800,21.73; 6059400,21.73; 6060000,21.69; 6060600,21.65;
            6061200,21.65; 6061800,21.64; 6062400,21.6; 6063000,21.56; 6063600,21.54;
            6064200,21.48; 6064800,21.47; 6065400,21.44; 6066000,21.43; 6066600,21.4;
            6067200,21.4; 6067800,21.36; 6068400,21.32; 6069000,21.32; 6069600,21.28;
            6070200,21.24; 6070800,21.2; 6071400,21.16; 6072000,21.16; 6072600,21.13;
            6073200,21.16; 6073800,21.13; 6074400,21.16; 6075000,21.16; 6075600,21.2;
            6076200,21.24; 6076800,21.28; 6077400,21.4; 6078000,21.48; 6078600,21.56;
            6079200,21.6; 6079800,21.69; 6080400,21.81; 6081000,21.85; 6081600,21.93;
            6082200,22.09; 6082800,22.3; 6083400,22.58; 6084000,22.78; 6084600,23.03;
            6085200,23.2; 6085800,23.32; 6086400,23.4; 6087000,23.48; 6087600,23.6;
            6088200,23.68; 6088800,23.68; 6089400,23.73; 6090000,23.76; 6090600,23.84;
            6091200,23.86; 6091800,23.92; 6092400,24.05; 6093000,24.14; 6093600,24.26;
            6094200,24.33; 6094800,24.41; 6095400,24.5; 6096000,24.57; 6096600,24.58;
            6097200,24.62; 6097800,24.66; 6098400,24.62; 6099000,24.66; 6099600,24.75;
            6100200,24.7; 6100800,24.7; 6101400,24.66; 6102000,24.62; 6102600,24.66;
            6103200,24.71; 6103800,24.66; 6104400,24.62; 6105000,24.58; 6105600,24.71;
            6106200,24.59; 6106800,24.57; 6107400,24.59; 6108000,24.58; 6108600,24.54;
            6109200,24.46; 6109800,24.45; 6110400,24.4; 6111000,24.35; 6111600,24.3;
            6112200,24.25; 6112800,24.25; 6113400,24.21; 6114000,24.17; 6114600,24.13;
            6115200,24.1; 6115800,24.08; 6116400,24.05; 6117000,24; 6117600,23.88; 6118200,
            23.76; 6118800,23.68; 6119400,23.61; 6120000,23.52; 6120600,23.44; 6121200,
            23.36; 6121800,23.24; 6122400,23.12; 6123000,23.07; 6123600,22.95; 6124200,
            22.91; 6124800,22.83; 6125400,22.78; 6126000,22.72; 6126600,22.62; 6127200,
            22.58; 6127800,22.51; 6128400,22.46; 6129000,22.42; 6129600,22.42; 6130200,
            22.34; 6130800,22.3; 6131400,22.26; 6132000,22.26; 6132600,22.18; 6133200,
            22.14; 6133800,22.05; 6134400,22.01; 6135000,21.98; 6135600,21.93; 6136200,
            21.89; 6136800,21.85; 6137400,21.82; 6138000,21.77; 6138600,21.73; 6139200,
            21.69; 6139800,21.65; 6140400,21.6; 6141000,21.6; 6141600,21.56; 6142200,
            21.53; 6142800,21.48; 6143400,21.44; 6144000,21.44; 6144600,21.4; 6145200,
            21.41; 6145800,21.37; 6146400,21.36; 6147000,21.36; 6147600,21.32; 6148200,
            21.32; 6148800,21.28; 6149400,21.28; 6150000,21.24; 6150600,21.24; 6151200,
            21.24; 6151800,21.2; 6152400,21.2; 6153000,21.16; 6153600,21.16; 6154200,
            21.16; 6154800,21.13; 6155400,21.12; 6156000,21.12; 6156600,21.08; 6157200,
            21.08; 6157800,21.08; 6158400,21.05; 6159000,21.04; 6159600,21.04; 6160200,
            21.02; 6160800,21.04; 6161400,21.04; 6162000,21.04; 6162600,21.04; 6163200,
            21.04; 6163800,21.06; 6164400,21.08; 6165000,21.08; 6165600,21.12; 6166200,
            21.12; 6166800,21.15; 6167400,21.16; 6168000,21.2; 6168600,21.2; 6169200,
            21.24; 6169800,21.28; 6170400,21.28; 6171000,21.32; 6171600,21.33; 6172200,
            21.36; 6172800,21.4; 6173400,21.44; 6174000,21.49; 6174600,21.5; 6175200,
            21.54; 6175800,21.6; 6176400,21.6; 6177000,21.69; 6177600,21.73; 6178200,
            21.81; 6178800,21.85; 6179400,21.89; 6180000,21.98; 6180600,21.98; 6181200,
            21.97; 6181800,21.97; 6182400,21.94; 6183000,21.93; 6183600,21.93; 6184200,
            21.88; 6184800,21.9; 6185400,21.85; 6186000,21.83; 6186600,21.81; 6187200,
            21.81; 6187800,21.81; 6188400,21.77; 6189000,21.77; 6189600,21.73; 6190200,
            21.73; 6190800,21.66; 6191400,21.64; 6192000,21.6; 6192600,21.6; 6193200,
            21.58; 6193800,21.56; 6194400,21.52; 6195000,21.52; 6195600,21.48; 6196200,
            21.44; 6196800,21.44; 6197400,21.4; 6198000,21.36; 6198600,21.32; 6199200,
            21.31; 6199800,21.28; 6200400,21.24; 6201000,21.2; 6201600,21.16; 6202200,
            21.15; 6202800,21.12; 6203400,21.08; 6204000,21.08; 6204600,21.08; 6205200,
            21.05; 6205800,21.02; 6206400,21; 6207000,20.98; 6207600,20.96; 6208200,
            20.96; 6208800,20.91; 6209400,20.92; 6210000,20.88; 6210600,20.87; 6211200,
            20.87; 6211800,20.83; 6212400,20.83; 6213000,20.79; 6213600,20.79; 6214200,
            20.79; 6214800,20.75; 6215400,20.75; 6216000,20.75; 6216600,20.74; 6217200,
            20.71; 6217800,20.7; 6218400,20.79; 6219000,20.87; 6219600,20.96; 6220200,
            20.96; 6220800,21.01; 6221400,21.04; 6222000,21.08; 6222600,21.08; 6223200,
            21.12; 6223800,21.12; 6224400,21.16; 6225000,21.16; 6225600,21.16; 6226200,
            21.16; 6226800,21.21; 6227400,21.2; 6228000,21.21; 6228600,21.24; 6229200,
            21.24; 6229800,21.24; 6230400,21.27; 6231000,21.28; 6231600,21.28; 6232200,
            21.28; 6232800,21.28; 6233400,21.28; 6234000,21.28; 6234600,21.32; 6235200,
            21.32; 6235800,21.32; 6236400,21.32; 6237000,21.33; 6237600,21.32; 6238200,
            21.32; 6238800,21.32; 6239400,21.33; 6240000,21.32; 6240600,21.33; 6241200,
            21.32; 6241800,21.32; 6242400,21.34; 6243000,21.36; 6243600,21.36; 6244200,
            21.36; 6244800,21.36; 6245400,21.36; 6246000,21.36; 6246600,21.36; 6247200,
            21.36; 6247800,21.36; 6248400,21.4; 6249000,21.4; 6249600,21.4; 6250200,
            21.4; 6250800,21.44; 6251400,21.44; 6252000,21.44; 6252600,21.44; 6253200,
            21.5; 6253800,21.48; 6254400,21.54; 6255000,21.52; 6255600,21.6; 6256200,
            21.61; 6256800,21.65; 6257400,21.69; 6258000,21.77; 6258600,21.81; 6259200,
            21.82; 6259800,21.86; 6260400,21.89; 6261000,21.97; 6261600,21.97; 6262200,
            21.97; 6262800,21.97; 6263400,22.01; 6264000,22.05; 6264600,22.14; 6265200,
            22.18; 6265800,22.22; 6266400,22.26; 6267000,22.26; 6267600,22.22; 6268200,
            22.27; 6268800,22.26; 6269400,22.26; 6270000,22.26; 6270600,22.26; 6271200,
            22.29; 6271800,22.31; 6272400,22.32; 6273000,22.32; 6273600,22.33; 6274200,
            22.3; 6274800,22.34; 6275400,22.3; 6276000,22.34; 6276600,22.34; 6277200,
            22.33; 6277800,22.3; 6278400,22.3; 6279000,22.26; 6279600,22.3; 6280200,
            22.29; 6280800,22.26; 6281400,22.22; 6282000,22.22; 6282600,22.17; 6283200,
            22.13; 6283800,22.09; 6284400,22.05; 6285000,22.01; 6285600,21.97; 6286200,
            21.93; 6286800,21.92; 6287400,21.9; 6288000,21.85; 6288600,21.85; 6289200,
            21.81; 6289800,21.81; 6290400,21.81; 6291000,21.78; 6291600,21.77; 6292200,
            21.73; 6292800,21.73; 6293400,21.74; 6294000,21.69; 6294600,21.69; 6295200,
            21.66; 6295800,21.66; 6296400,21.66; 6297000,21.65; 6297600,21.61; 6298200,
            21.6; 6298800,21.6; 6299400,21.6; 6300000,21.6; 6300600,21.56; 6301200,21.56;
            6301800,21.56; 6302400,21.56; 6303000,21.56; 6303600,21.52; 6304200,21.52;
            6304800,21.52; 6305400,21.52; 6306000,21.52; 6306600,21.52; 6307200,21.48;
            6307800,21.48; 6308400,21.48; 6309000,21.48; 6309600,21.47; 6310200,21.44;
            6310800,21.46; 6311400,21.48; 6312000,21.48; 6312600,21.44; 6313200,21.44;
            6313800,21.44; 6314400,21.44; 6315000,21.44; 6315600,21.44; 6316200,21.44;
            6316800,21.44; 6317400,21.44; 6318000,21.44; 6318600,21.41; 6319200,21.4;
            6319800,21.4; 6320400,21.4; 6321000,21.4; 6321600,21.4; 6322200,21.4; 6322800,
            21.6; 6323400,21.77; 6324000,21.85; 6324600,21.97; 6325200,22.09; 6325800,
            22.14; 6326400,22.18; 6327000,22.17; 6327600,22.15; 6328200,22.14; 6328800,
            22.14; 6329400,22.18; 6330000,22.18; 6330600,22.18; 6331200,22.22; 6331800,
            22.26; 6332400,22.35; 6333000,22.38; 6333600,22.31; 6334200,22.42; 6334800,
            22.5; 6335400,22.57; 6336000,22.7; 6336600,22.83; 6337200,22.96; 6337800,
            23.08; 6338400,23.15; 6339000,23.27; 6339600,23.31; 6340200,23.36; 6340800,
            23.43; 6341400,23.52; 6342000,23.59; 6342600,23.64; 6343200,23.68; 6343800,
            23.73; 6344400,23.77; 6345000,23.84; 6345600,23.88; 6346200,23.92; 6346800,
            23.93; 6347400,24; 6348000,23.97; 6348600,24.13; 6349200,24.29; 6349800,
            24.37; 6350400,24.21; 6351000,24.09; 6351600,22.3; 6352200,22.01; 6352800,
            22.84; 6353400,22.01; 6354000,23.11; 6354600,23.64; 6355200,23.84; 6355800,
            24.01; 6356400,24.06; 6357000,24.13; 6357600,24.13; 6358200,24.18; 6358800,
            24.21; 6359400,24.22; 6360000,24.21; 6360600,24.26; 6361200,24.3; 6361800,
            24.33; 6362400,24.37; 6363000,24.4; 6363600,24.41; 6364200,24.42; 6364800,
            24.53; 6365400,24.5; 6366000,24.38; 6366600,24.29; 6367200,24.25; 6367800,
            24.21; 6368400,24.13; 6369000,24.14; 6369600,24.09; 6370200,24.05; 6370800,
            24.09; 6371400,24.06; 6372000,24.04; 6372600,24.06; 6373200,24.02; 6373800,
            23.96; 6374400,23.88; 6375000,23.8; 6375600,23.72; 6376200,23.72; 6376800,
            23.64; 6377400,23.56; 6378000,23.46; 6378600,23.4; 6379200,23.32; 6379800,
            23.19; 6380400,23.12; 6381000,23.03; 6381600,22.91; 6382200,22.84; 6382800,
            22.71; 6383400,22.66; 6384000,22.61; 6384600,22.54; 6385200,22.48; 6385800,
            22.43; 6386400,22.38; 6387000,22.34; 6387600,22.26; 6388200,22.25; 6388800,
            22.19; 6389400,22.15; 6390000,22.09; 6390600,22.09; 6391200,22.05; 6391800,
            22.01; 6392400,21.97; 6393000,21.94; 6393600,21.93; 6394200,21.89; 6394800,
            21.86; 6395400,21.85; 6396000,21.81; 6396600,21.81; 6397200,21.81; 6397800,
            21.77; 6398400,21.75; 6399000,21.77; 6399600,21.77; 6400200,21.77; 6400800,
            21.94; 6401400,22.06; 6402000,22.17; 6402600,22.26; 6403200,22.41; 6403800,
            22.47; 6404400,22.5; 6405000,22.54; 6405600,22.58; 6406200,22.54; 6406800,
            22.55; 6407400,22.5; 6408000,22.5; 6408600,22.5; 6409200,22.5; 6409800,22.5;
            6410400,22.5; 6411000,22.5; 6411600,22.5; 6412200,22.5; 6412800,22.5; 6413400,
            22.5; 6414000,22.46; 6414600,22.49; 6415200,22.46; 6415800,22.46; 6416400,
            22.46; 6417000,22.46; 6417600,22.5; 6418200,22.5; 6418800,22.54; 6419400,
            22.76; 6420000,22.69; 6420600,22.83; 6421200,22.82; 6421800,22.84; 6422400,
            22.83; 6423000,22.9; 6423600,22.95; 6424200,22.99; 6424800,22.99; 6425400,
            23.07; 6426000,23.15; 6426600,23.23; 6427200,23.28; 6427800,23.4; 6428400,
            23.44; 6429000,23.48; 6429600,23.52; 6430200,23.6; 6430800,23.66; 6431400,
            23.66; 6432000,23.68; 6432600,23.72; 6433200,23.72; 6433800,23.74; 6434400,
            23.82; 6435000,23.8; 6435600,23.8; 6436200,23.84; 6436800,23.86; 6437400,
            23.88; 6438000,23.96; 6438600,24.04; 6439200,24.13; 6439800,24.25; 6440400,
            24.34; 6441000,24.41; 6441600,24.45; 6442200,24.49; 6442800,24.5; 6443400,
            24.5; 6444000,24.45; 6444600,24.45; 6445200,24.43; 6445800,24.45; 6446400,
            24.41; 6447000,24.33; 6447600,24.25; 6448200,24.22; 6448800,24.21; 6449400,
            24.13; 6450000,24.21; 6450600,24.24; 6451200,24.21; 6451800,24.25; 6452400,
            24.17; 6453000,24.21; 6453600,24.21; 6454200,24.21; 6454800,24.21; 6455400,
            24.17; 6456000,24.21; 6456600,24.29; 6457200,24.29; 6457800,24.25; 6458400,
            24.26; 6459000,24.21; 6459600,24.21; 6460200,24.17; 6460800,24.17; 6461400,
            24.1; 6462000,24.15; 6462600,24.13; 6463200,24.09; 6463800,24.05; 6464400,
            23.93; 6465000,23.88; 6465600,23.77; 6466200,23.73; 6466800,23.64; 6467400,
            23.64; 6468000,23.56; 6468600,23.44; 6469200,23.32; 6469800,23.27; 6470400,
            23.2; 6471000,23.16; 6471600,23.1; 6472200,23.07; 6472800,23.03; 6473400,
            22.95; 6474000,22.91; 6474600,22.88; 6475200,22.83; 6475800,22.78; 6476400,
            22.78; 6477000,22.74; 6477600,22.7; 6478200,22.66; 6478800,22.62; 6479400,
            22.62; 6480000,22.58; 6480600,22.54; 6481200,22.55; 6481800,22.51; 6482400,
            22.47; 6483000,22.46; 6483600,22.68; 6484200,22.42; 6484800,22.42; 6485400,
            22.38; 6486000,22.34; 6486600,22.34; 6487200,22.46; 6487800,22.63; 6488400,
            22.74; 6489000,22.87; 6489600,22.95; 6490200,23.04; 6490800,23.08; 6491400,
            23.03; 6492000,22.95; 6492600,22.88; 6493200,22.82; 6493800,22.81; 6494400,
            22.87; 6495000,22.91; 6495600,22.95; 6496200,22.96; 6496800,22.91; 6497400,
            22.91; 6498000,22.92; 6498600,22.87; 6499200,22.84; 6499800,22.83; 6500400,
            22.82; 6501000,22.82; 6501600,22.82; 6502200,22.8; 6502800,22.78; 6503400,
            22.78; 6504000,22.78; 6504600,22.78; 6505200,22.74; 6505800,22.74; 6506400,
            22.66; 6507000,22.62; 6507600,22.7; 6508200,22.7; 6508800,22.79; 6509400,
            22.83; 6510000,22.94; 6510600,23.03; 6511200,23.08; 6511800,23.11; 6512400,
            23.19; 6513000,23.24; 6513600,23.18; 6514200,23.23; 6514800,23.32; 6515400,
            23.33; 6516000,23.36; 6516600,23.4; 6517200,23.48; 6517800,23.52; 6518400,
            23.56; 6519000,23.68; 6519600,23.8; 6520200,23.76; 6520800,23.8; 6521400,
            23.76; 6522000,23.76; 6522600,23.72; 6523200,23.67; 6523800,23.64; 6524400,
            23.65; 6525000,23.68; 6525600,23.81; 6526200,23.88; 6526800,23.86; 6527400,
            23.89; 6528000,23.83; 6528600,23.8; 6529200,23.92; 6529800,24; 6530400,24;
            6531000,24.05; 6531600,24.05; 6532200,24.06; 6532800,24.09; 6533400,24.08;
            6534000,24.09; 6534600,24.13; 6535200,24.09; 6535800,24.08; 6536400,24.13;
            6537000,24.17; 6537600,24.17; 6538200,24.25; 6538800,24.29; 6539400,24.31;
            6540000,24.28; 6540600,24.29; 6541200,24.29; 6541800,24.28; 6542400,24.25;
            6543000,24.2; 6543600,24.21; 6544200,24.21; 6544800,24.17; 6545400,24.21;
            6546000,24.21; 6546600,24.21; 6547200,24.13; 6547800,24.09; 6548400,24.01;
            6549000,23.92; 6549600,23.84; 6550200,23.76; 6550800,23.68; 6551400,23.57;
            6552000,23.6; 6552600,23.56; 6553200,23.56; 6553800,23.52; 6554400,23.44;
            6555000,23.32; 6555600,23.23; 6556200,23.19; 6556800,23.19; 6557400,23.15;
            6558000,23.11; 6558600,23.08; 6559200,23.07; 6559800,23.03; 6560400,22.99;
            6561000,22.95; 6561600,22.95; 6562200,22.91; 6562800,22.91; 6563400,22.87;
            6564000,22.83; 6564600,22.83; 6565200,22.79; 6565800,22.78; 6566400,22.74;
            6567000,22.74; 6567600,22.7; 6568200,22.7; 6568800,22.69; 6569400,22.65;
            6570000,22.62; 6570600,22.62; 6571200,22.58; 6571800,22.58; 6572400,22.58;
            6573000,22.54; 6573600,22.55; 6574200,22.5; 6574800,22.5; 6575400,22.5;
            6576000,22.5; 6576600,22.5; 6577200,22.46; 6577800,22.46; 6578400,22.46;
            6579000,22.43; 6579600,22.43; 6580200,22.42; 6580800,22.42; 6581400,22.42;
            6582000,22.51; 6582600,22.7; 6583200,22.78; 6583800,22.87; 6584400,22.91;
            6585000,22.9; 6585600,22.82; 6586200,22.74; 6586800,22.67; 6587400,22.63;
            6588000,22.62; 6588600,22.7; 6589200,22.74; 6589800,22.79; 6590400,22.83;
            6591000,22.83; 6591600,22.79; 6592200,22.8; 6592800,22.79; 6593400,22.91;
            6594000,22.87; 6594600,22.58; 6595200,22.5; 6595800,22.78; 6596400,22.95;
            6597000,23.02; 6597600,23.11; 6598200,23.19; 6598800,23.33; 6599400,23.4;
            6600000,23.44; 6600600,23.48; 6601200,23.56; 6601800,23.64; 6602400,23.68;
            6603000,23.72; 6603600,23.76; 6604200,23.8; 6604800,23.92; 6605400,24.06;
            6606000,24.12; 6606600,24.13; 6607200,24.13; 6607800,24.17; 6608400,24.18;
            6609000,24.22; 6609600,24.25; 6610200,24.34; 6610800,24.45; 6611400,24.53;
            6612000,24.58; 6612600,24.7; 6613200,24.74; 6613800,24.79; 6614400,24.78;
            6615000,24.78; 6615600,24.82; 6616200,24.86; 6616800,24.94; 6617400,24.87;
            6618000,24.86; 6618600,24.9; 6619200,24.91; 6619800,24.78; 6620400,24.82;
            6621000,24.88; 6621600,24.86; 6622200,24.91; 6622800,24.94; 6623400,24.98;
            6624000,25.06; 6624600,25.1; 6625200,25.06; 6625800,25.02; 6626400,24.9;
            6627000,24.85; 6627600,24.83; 6628200,24.78; 6628800,24.66; 6629400,24.58;
            6630000,24.5; 6630600,24.45; 6631200,24.42; 6631800,24.44; 6632400,24.41;
            6633000,24.41; 6633600,24.38; 6634200,24.33; 6634800,24.26; 6635400,24.21;
            6636000,24.17; 6636600,24.13; 6637200,24.05; 6637800,23.96; 6638400,23.88;
            6639000,23.76; 6639600,23.68; 6640200,23.6; 6640800,23.52; 6641400,23.49;
            6642000,23.4; 6642600,23.36; 6643200,23.28; 6643800,23.24; 6644400,23.19;
            6645000,23.15; 6645600,23.1; 6646200,23.07; 6646800,23.02; 6647400,22.95;
            6648000,22.93; 6648600,22.91; 6649200,22.87; 6649800,22.83; 6650400,22.78;
            6651000,22.78; 6651600,22.74; 6652200,22.74; 6652800,22.7; 6653400,22.66;
            6654000,22.68; 6654600,22.62; 6655200,22.61; 6655800,22.58; 6656400,22.58;
            6657000,22.54; 6657600,22.55; 6658200,22.5; 6658800,22.5; 6659400,22.5;
            6660000,22.46; 6660600,22.45; 6661200,22.42; 6661800,22.42; 6662400,22.4;
            6663000,22.38; 6663600,22.38; 6664200,22.39; 6664800,22.34; 6665400,22.34;
            6666000,22.3; 6666600,22.3; 6667200,22.3; 6667800,22.3; 6668400,22.42; 6669000,
            22.5; 6669600,22.62; 6670200,22.76; 6670800,22.84; 6671400,22.91; 6672000,
            22.91; 6672600,22.87; 6673200,22.78; 6673800,22.77; 6674400,22.74; 6675000,
            22.74; 6675600,22.75; 6676200,22.76; 6676800,22.78; 6677400,22.83; 6678000,
            22.87; 6678600,22.92; 6679200,22.91; 6679800,23.15; 6680400,22.95; 6681000,
            22.99; 6681600,23.12; 6682200,23.19; 6682800,23.26; 6683400,23.4; 6684000,
            23.48; 6684600,23.6; 6685200,23.67; 6685800,23.72; 6686400,23.8; 6687000,
            23.88; 6687600,23.96; 6688200,24.01; 6688800,24.09; 6689400,24.13; 6690000,
            24.21; 6690600,24.25; 6691200,24.29; 6691800,24.34; 6692400,24.62; 6693000,
            24.41; 6693600,24.41; 6694200,24.41; 6694800,24.41; 6695400,24.41; 6696000,
            24.41; 6696600,24.41; 6697200,24.41; 6697800,24.41; 6698400,24.41; 6699000,
            24.53; 6699600,24.54; 6700200,24.58; 6700800,24.65; 6701400,24.74; 6702000,
            24.7; 6702600,24.78; 6703200,24.78; 6703800,24.79; 6704400,24.82; 6705000,
            24.86; 6705600,24.82; 6706200,24.78; 6706800,24.82; 6707400,24.78; 6708000,
            24.73; 6708600,24.74; 6709200,24.61; 6709800,24.62; 6710400,24.63; 6711000,
            24.66; 6711600,24.66; 6712200,24.62; 6712800,24.54; 6713400,24.54; 6714000,
            24.54; 6714600,24.54; 6715200,24.54; 6715800,24.51; 6716400,24.58; 6717000,
            24.58; 6717600,24.59; 6718200,24.53; 6718800,24.45; 6719400,24.41; 6720000,
            24.37; 6720600,24.33; 6721200,24.29; 6721800,24.25; 6722400,24.22; 6723000,
            24.14; 6723600,24.05; 6724200,24; 6724800,23.88; 6725400,23.8; 6726000,23.72;
            6726600,23.6; 6727200,23.52; 6727800,23.4; 6728400,23.37; 6729000,23.27;
            6729600,23.19; 6730200,23.15; 6730800,23.11; 6731400,23.07; 6732000,23.03;
            6732600,22.95; 6733200,22.91; 6733800,22.87; 6734400,22.83; 6735000,22.78;
            6735600,22.74; 6736200,22.7; 6736800,22.66; 6737400,22.62; 6738000,22.58;
            6738600,22.5; 6739200,22.42; 6739800,22.42; 6740400,22.38; 6741000,22.34;
            6741600,22.3; 6742200,22.26; 6742800,22.22; 6743400,22.18; 6744000,22.14;
            6744600,22.1; 6745200,22.06; 6745800,22.05; 6746400,22.01; 6747000,22.01;
            6747600,21.97; 6748200,21.97; 6748800,21.93; 6749400,21.89; 6750000,21.85;
            6750600,21.85; 6751200,21.81; 6751800,21.8; 6752400,21.77; 6753000,21.73;
            6753600,21.74; 6754200,21.69; 6754800,21.69; 6755400,21.65; 6756000,21.65;
            6756600,21.63; 6757200,21.56; 6757800,21.56; 6758400,21.52; 6759000,21.52;
            6759600,21.48; 6760200,21.46; 6760800,21.44; 6761400,21.44; 6762000,21.44;
            6762600,21.43; 6763200,21.4; 6763800,21.4; 6764400,21.4; 6765000,21.41;
            6765600,21.4; 6766200,21.4; 6766800,21.4; 6767400,21.4; 6768000,21.43; 6768600,
            21.44; 6769200,21.44; 6769800,21.44; 6770400,21.46; 6771000,21.48; 6771600,
            21.52; 6772200,21.52; 6772800,21.56; 6773400,21.6; 6774000,21.6; 6774600,
            21.65; 6775200,21.65; 6775800,21.67; 6776400,21.73; 6777000,21.73; 6777600,
            21.81; 6778200,21.82; 6778800,21.87; 6779400,21.93; 6780000,21.97; 6780600,
            22.02; 6781200,22.09; 6781800,22.14; 6782400,22.18; 6783000,22.26; 6783600,
            22.33; 6784200,22.42; 6784800,22.5; 6785400,22.6; 6786000,22.7; 6786600,
            22.81; 6787200,22.91; 6787800,22.98; 6788400,23.03; 6789000,23.07; 6789600,
            23.16; 6790200,23.15; 6790800,23.19; 6791400,23.23; 6792000,23.44; 6792600,
            23.23; 6793200,23.27; 6793800,23.32; 6794400,23.36; 6795000,23.4; 6795600,
            23.44; 6796200,23.51; 6796800,23.48; 6797400,23.52; 6798000,23.48; 6798600,
            23.4; 6799200,23.36; 6799800,23.27; 6800400,23.19; 6801000,23.15; 6801600,
            23.11; 6802200,23.03; 6802800,22.98; 6803400,22.91; 6804000,22.83; 6804600,
            22.78; 6805200,22.78; 6805800,22.7; 6806400,22.66; 6807000,22.62; 6807600,
            22.58; 6808200,22.54; 6808800,22.47; 6809400,22.42; 6810000,22.38; 6810600,
            22.34; 6811200,22.3; 6811800,22.22; 6812400,22.15; 6813000,22.09; 6813600,
            22.08; 6814200,22.05; 6814800,22.01; 6815400,21.97; 6816000,21.97; 6816600,
            21.93; 6817200,21.89; 6817800,21.85; 6818400,21.81; 6819000,21.81; 6819600,
            21.77; 6820200,21.73; 6820800,21.69; 6821400,21.66; 6822000,21.65; 6822600,
            21.6; 6823200,21.65; 6823800,21.73; 6824400,21.78; 6825000,21.81; 6825600,
            21.81; 6826200,21.85; 6826800,21.85; 6827400,21.85; 6828000,21.86; 6828600,
            21.86; 6829200,21.9; 6829800,21.89; 6830400,21.89; 6831000,21.89; 6831600,
            21.89; 6832200,21.89; 6832800,21.89; 6833400,21.89; 6834000,21.89; 6834600,
            21.89; 6835200,21.9; 6835800,21.89; 6836400,21.89; 6837000,21.9; 6837600,
            21.85; 6838200,21.85; 6838800,21.85; 6839400,21.85; 6840000,21.85; 6840600,
            21.86; 6841200,21.85; 6841800,21.85; 6842400,21.85; 6843000,21.84; 6843600,
            21.85; 6844200,21.81; 6844800,21.81; 6845400,21.8; 6846000,21.81; 6846600,
            21.81; 6847200,21.81; 6847800,21.81; 6848400,21.81; 6849000,21.81; 6849600,
            21.81; 6850200,21.84; 6850800,21.81; 6851400,21.85; 6852000,21.85; 6852600,
            21.85; 6853200,21.85; 6853800,21.89; 6854400,21.89; 6855000,21.93; 6855600,
            21.98; 6856200,21.97; 6856800,21.97; 6857400,22.01; 6858000,22.03; 6858600,
            22.06; 6859200,22.06; 6859800,22.09; 6860400,22.14; 6861000,22.16; 6861600,
            22.18; 6862200,22.22; 6862800,22.22; 6863400,22.26; 6864000,22.26; 6864600,
            22.29; 6865200,22.3; 6865800,22.34; 6866400,22.38; 6867000,22.47; 6867600,
            22.5; 6868200,22.54; 6868800,22.55; 6869400,22.54; 6870000,22.54; 6870600,
            22.5; 6871200,22.51; 6871800,22.54; 6872400,22.58; 6873000,22.58; 6873600,
            22.58; 6874200,22.58; 6874800,22.58; 6875400,22.58; 6876000,22.58; 6876600,
            22.58; 6877200,22.54; 6877800,22.54; 6878400,22.58; 6879000,22.58; 6879600,
            22.61; 6880200,22.58; 6880800,22.54; 6881400,22.54; 6882000,22.58; 6882600,
            22.58; 6883200,22.58; 6883800,22.58; 6884400,22.54; 6885000,22.5; 6885600,
            22.46; 6886200,22.42; 6886800,22.42; 6887400,22.38; 6888000,22.38; 6888600,
            22.33; 6889200,22.3; 6889800,22.26; 6890400,22.26; 6891000,22.22; 6891600,
            22.18; 6892200,22.19; 6892800,22.15; 6893400,22.14; 6894000,22.14; 6894600,
            22.1; 6895200,22.1; 6895800,22.09; 6896400,22.06; 6897000,22.09; 6897600,
            22.09; 6898200,22.09; 6898800,22.09; 6899400,22.09; 6900000,22.09; 6900600,
            22.09; 6901200,22.09; 6901800,22.09; 6902400,22.09; 6903000,22.09; 6903600,
            22.09; 6904200,22.09; 6904800,22.09; 6905400,22.09; 6906000,22.09; 6906600,
            22.09; 6907200,22.09; 6907800,22.09; 6908400,22.09; 6909000,22.06; 6909600,
            22.06; 6910200,22.05; 6910800,22.05; 6911400,22.05; 6912000,22.05; 6912600,
            22.05; 6913200,22.05; 6913800,22.05; 6914400,22.02; 6915000,22.06; 6915600,
            22.04; 6916200,22.01; 6916800,22.01; 6917400,22.01; 6918000,22.02; 6918600,
            22.01; 6919200,22.01; 6919800,22.01; 6920400,22.01; 6921000,22.01; 6921600,
            22.01; 6922200,21.98; 6922800,21.97; 6923400,22.01; 6924000,21.98; 6924600,
            21.97; 6925200,21.97; 6925800,21.97; 6926400,21.97; 6927000,21.97; 6927600,
            22.01; 6928200,22.22; 6928800,22.34; 6929400,22.46; 6930000,22.62; 6930600,
            22.7; 6931200,22.7; 6931800,22.66; 6932400,22.62; 6933000,22.62; 6933600,
            22.6; 6934200,22.64; 6934800,22.6; 6935400,22.62; 6936000,22.66; 6936600,
            22.74; 6937200,22.74; 6937800,22.78; 6938400,22.87; 6939000,22.96; 6939600,
            23.03; 6940200,23.15; 6940800,23.27; 6941400,23.4; 6942000,23.42; 6942600,
            23.64; 6943200,23.76; 6943800,23.8; 6944400,23.9; 6945000,23.92; 6945600,
            23.92; 6946200,24.01; 6946800,24.1; 6947400,24.13; 6948000,24.17; 6948600,
            24.21; 6949200,24.24; 6949800,24.3; 6950400,24.33; 6951000,24.29; 6951600,
            24.33; 6952200,24.45; 6952800,24.37; 6953400,24.38; 6954000,24.41; 6954600,
            24.41; 6955200,24.41; 6955800,24.41; 6956400,24.46; 6957000,24.66; 6957600,
            24.74; 6958200,24.78; 6958800,24.95; 6959400,25.02; 6960000,25.1; 6960600,
            25.14; 6961200,25.14; 6961800,25.14; 6962400,25.19; 6963000,25.22; 6963600,
            25.19; 6964200,25.19; 6964800,25.14; 6965400,25.19; 6966000,25.23; 6966600,
            25.23; 6967200,25.23; 6967800,25.26; 6968400,25.23; 6969000,25.24; 6969600,
            25.22; 6970200,25.11; 6970800,25.07; 6971400,24.98; 6972000,24.94; 6972600,
            24.83; 6973200,24.78; 6973800,24.7; 6974400,24.66; 6975000,24.66; 6975600,
            24.64; 6976200,24.65; 6976800,24.6; 6977400,24.58; 6978000,24.5; 6978600,
            24.46; 6979200,24.41; 6979800,24.37; 6980400,24.29; 6981000,24.25; 6981600,
            24.17; 6982200,24.09; 6982800,24.01; 6983400,23.92; 6984000,23.8; 6984600,
            23.72; 6985200,23.64; 6985800,23.52; 6986400,23.43; 6987000,23.36; 6987600,
            23.33; 6988200,23.27; 6988800,23.19; 6989400,23.14; 6990000,23.12; 6990600,
            23.03; 6991200,22.99; 6991800,22.95; 6992400,22.91; 6993000,22.9; 6993600,
            22.86; 6994200,22.83; 6994800,22.78; 6995400,22.78; 6996000,22.74; 6996600,
            22.74; 6997200,22.7; 6997800,22.7; 6998400,22.68; 6999000,22.64; 6999600,
            22.62; 7000200,22.62; 7000800,22.62; 7001400,22.59; 7002000,22.58; 7002600,
            22.58; 7003200,22.55; 7003800,22.54; 7004400,22.54; 7005000,22.52; 7005600,
            22.5; 7006200,22.5; 7006800,22.46; 7007400,22.46; 7008000,22.44; 7008600,
            22.42; 7009200,22.42; 7009800,22.42; 7010400,22.42; 7011000,22.38; 7011600,
            22.38; 7012200,22.38; 7012800,22.38; 7013400,22.36; 7014000,22.42; 7014600,
            22.62; 7015200,22.78; 7015800,22.88; 7016400,22.99; 7017000,23.08; 7017600,
            23.07; 7018200,22.95; 7018800,22.87; 7019400,22.83; 7020000,22.78; 7020600,
            22.79; 7021200,22.84; 7021800,22.87; 7022400,22.91; 7023000,22.94; 7023600,
            22.95; 7024200,23.03; 7024800,23.07; 7025400,23.15; 7026000,23.15; 7026600,
            23.33; 7027200,23.44; 7027800,23.52; 7028400,23.6; 7029000,23.68; 7029600,
            23.71; 7030200,23.76; 7030800,23.81; 7031400,23.84; 7032000,23.96; 7032600,
            24.13; 7033200,24.14; 7033800,24.23; 7034400,24.25; 7035000,24.3; 7035600,
            24.37; 7036200,24.38; 7036800,24.53; 7037400,24.66; 7038000,24.72; 7038600,
            24.78; 7039200,24.7; 7039800,24.7; 7040400,24.66; 7041000,24.62; 7041600,
            24.66; 7042200,24.66; 7042800,24.79; 7043400,24.82; 7044000,24.98; 7044600,
            25.07; 7045200,25.14; 7045800,25.19; 7046400,25.27; 7047000,25.31; 7047600,
            25.31; 7048200,25.32; 7048800,25.31; 7049400,25.27; 7050000,25.3; 7050600,
            25.18; 7051200,25.23; 7051800,25.27; 7052400,25.31; 7053000,25.32; 7053600,
            25.27; 7054200,25.31; 7054800,25.39; 7055400,25.31; 7056000,25.32; 7056600,
            25.27; 7057200,25.19; 7057800,25.1; 7058400,25.1; 7059000,25.06; 7059600,
            25.02; 7060200,24.98; 7060800,24.94; 7061400,24.86; 7062000,24.82; 7062600,
            24.78; 7063200,24.7; 7063800,24.66; 7064400,24.58; 7065000,24.5; 7065600,
            24.41; 7066200,24.37; 7066800,24.33; 7067400,24.25; 7068000,24.21; 7068600,
            24.13; 7069200,24.05; 7069800,24; 7070400,23.92; 7071000,23.88; 7071600,
            23.8; 7072200,23.72; 7072800,23.64; 7073400,23.54; 7074000,23.44; 7074600,
            23.4; 7075200,23.32; 7075800,23.27; 7076400,23.2; 7077000,23.15; 7077600,
            23.11; 7078200,23.07; 7078800,23.03; 7079400,22.96; 7080000,22.96; 7080600,
            22.91; 7081200,22.87; 7081800,22.83; 7082400,22.78; 7083000,22.75; 7083600,
            22.74; 7084200,22.7; 7084800,22.66; 7085400,22.62; 7086000,22.63; 7086600,
            22.58; 7087200,22.54; 7087800,22.54; 7088400,22.5; 7089000,22.49; 7089600,
            22.46; 7090200,22.47; 7090800,22.43; 7091400,22.42; 7092000,22.54; 7092600,
            22.62; 7093200,22.78; 7093800,22.87; 7094400,22.91; 7095000,23.02; 7095600,
            23.03; 7096200,23.03; 7096800,23.03; 7097400,22.99; 7098000,22.95; 7098600,
            22.95; 7099200,22.91; 7099800,22.91; 7100400,22.93; 7101000,22.95; 7101600,
            22.95; 7102200,22.91; 7102800,22.92; 7103400,22.91; 7104000,22.91; 7104600,
            22.91; 7105200,22.91; 7105800,22.91; 7106400,22.95; 7107000,22.95; 7107600,
            22.95; 7108200,22.95; 7108800,22.99; 7109400,23.03; 7110000,23.03; 7110600,
            23.11; 7111200,23.15; 7111800,23.19; 7112400,23.27; 7113000,23.32; 7113600,
            23.44; 7114200,23.57; 7114800,23.68; 7115400,23.76; 7116000,23.84; 7116600,
            23.89; 7117200,23.92; 7117800,23.96; 7118400,24.01; 7119000,24.09; 7119600,
            24.09; 7120200,24.21; 7120800,24.25; 7121400,24.27; 7122000,24.33; 7122600,
            24.29; 7123200,24.34; 7123800,24.41; 7124400,24.5; 7125000,24.58; 7125600,
            24.55; 7126200,24.48; 7126800,24.45; 7127400,24.45; 7128000,24.45; 7128600,
            24.5; 7129200,24.53; 7129800,24.57; 7130400,24.7; 7131000,24.82; 7131600,
            24.78; 7132200,24.82; 7132800,24.82; 7133400,24.9; 7134000,24.9; 7134600,
            24.94; 7135200,24.98; 7135800,24.94; 7136400,24.94; 7137000,24.92; 7137600,
            24.82; 7138200,24.86; 7138800,24.83; 7139400,24.86; 7140000,24.9; 7140600,
            24.86; 7141200,24.94; 7141800,24.88; 7142400,24.98; 7143000,24.98; 7143600,
            25.04; 7144200,24.94; 7144800,24.9; 7145400,24.86; 7146000,24.86; 7146600,
            24.87; 7147200,24.78; 7147800,24.7; 7148400,24.62; 7149000,24.54; 7149600,
            24.45; 7150200,24.41; 7150800,24.33; 7151400,24.25; 7152000,24.21; 7152600,
            24.18; 7153200,24.13; 7153800,24.05; 7154400,24.04; 7155000,23.96; 7155600,
            23.92; 7156200,23.84; 7156800,23.8; 7157400,23.76; 7158000,23.68; 7158600,
            23.61; 7159200,23.55; 7159800,23.48; 7160400,23.42; 7161000,23.36; 7161600,
            23.29; 7162200,23.24; 7162800,23.19; 7163400,23.15; 7164000,23.12; 7164600,
            23.07; 7165200,23.04; 7165800,22.99; 7166400,22.98; 7167000,22.96; 7167600,
            22.91; 7168200,22.87; 7168800,22.87; 7169400,22.83; 7170000,22.8; 7170600,
            22.78; 7171200,22.78; 7171800,22.74; 7172400,22.71; 7173000,22.7; 7173600,
            22.68; 7174200,22.66; 7174800,22.62; 7175400,22.62; 7176000,22.62; 7176600,
            22.58; 7177200,22.58; 7177800,22.54; 7178400,22.7; 7179000,22.8; 7179600,
            22.91; 7180200,22.99; 7180800,23.07; 7181400,23.12; 7182000,23.15; 7182600,
            23.15; 7183200,23.11; 7183800,23.11; 7184400,23.07; 7185000,23.07; 7185600,
            23.07; 7186200,23.03; 7186800,23.03; 7187400,23.07; 7188000,23.11; 7188600,
            23.11; 7189200,23.11; 7189800,23.11; 7190400,23.1; 7191000,23.07; 7191600,
            23.03; 7192200,23.03; 7192800,23.04; 7193400,23.05; 7194000,23.07; 7194600,
            23.07; 7195200,23.09; 7195800,23.11; 7196400,23.15; 7197000,23.19; 7197600,
            23.27; 7198200,23.37; 7198800,23.4; 7199400,23.48; 7200000,23.56; 7200600,
            23.68; 7201200,23.8; 7201800,23.89; 7202400,23.92; 7203000,23.96; 7203600,
            24; 7204200,24.13; 7204800,24.17; 7205400,24.14; 7206000,24.13; 7206600,
            24.21; 7207200,24.3; 7207800,24.37; 7208400,24.45; 7209000,24.5; 7209600,
            24.54; 7210200,24.58; 7210800,24.69; 7211400,24.66; 7212000,24.6; 7212600,
            24.56; 7213200,24.54; 7213800,24.58; 7214400,24.66; 7215000,24.62; 7215600,
            24.74; 7216200,24.86; 7216800,24.94; 7217400,24.9; 7218000,24.97; 7218600,
            24.99; 7219200,25.04; 7219800,25.06; 7220400,25.06; 7221000,25.06; 7221600,
            25.14; 7222200,25.14; 7222800,25.15; 7223400,25.14; 7224000,25.08; 7224600,
            25.1; 7225200,25.16; 7225800,25.12; 7226400,25.16; 7227000,25.23; 7227600,
            25.2; 7228200,25.14; 7228800,25.1; 7229400,25.08; 7230000,25.1; 7230600,
            24.98; 7231200,24.74; 7231800,24.67; 7232400,24.58; 7233000,24.53; 7233600,
            24.45; 7234200,24.41; 7234800,24.37; 7235400,24.25; 7236000,24.17; 7236600,
            24.09; 7237200,24.05; 7237800,24.01; 7238400,23.92; 7239000,23.88; 7239600,
            23.8; 7240200,23.73; 7240800,23.68; 7241400,23.64; 7242000,23.56; 7242600,
            23.5; 7243200,23.44; 7243800,23.4; 7244400,23.32; 7245000,23.27; 7245600,
            23.19; 7246200,23.11; 7246800,23.07; 7247400,23.03; 7248000,22.99; 7248600,
            22.91; 7249200,22.87; 7249800,22.83; 7250400,22.78; 7251000,22.7; 7251600,
            22.66; 7252200,22.62; 7252800,22.58; 7253400,22.54; 7254000,22.5; 7254600,
            22.46; 7255200,22.43; 7255800,22.42; 7256400,22.38; 7257000,22.34; 7257600,
            22.3; 7258200,22.26; 7258800,22.26; 7259400,22.25; 7260000,22.22; 7260600,
            22.18; 7261200,22.18; 7261800,22.14; 7262400,22.13; 7263000,22.09; 7263600,
            22.09; 7264200,22.05; 7264800,22.05; 7265400,22.01; 7266000,22; 7266600,
            21.98; 7267200,21.97; 7267800,21.97; 7268400,21.97; 7269000,21.97; 7269600,
            21.97; 7270200,21.97; 7270800,21.97; 7271400,21.93; 7272000,21.93; 7272600,
            21.93; 7273200,21.93; 7273800,21.93; 7274400,21.92; 7275000,21.93; 7275600,
            21.89; 7276200,21.87; 7276800,21.85; 7277400,21.85; 7278000,21.89; 7278600,
            21.89; 7279200,21.85; 7279800,21.85; 7280400,21.85; 7281000,21.85; 7281600,
            21.85; 7282200,21.85; 7282800,21.85; 7283400,21.86; 7284000,21.86; 7284600,
            21.88; 7285200,21.92; 7285800,21.98; 7286400,22.09; 7287000,22.26; 7287600,
            22.38; 7288200,22.46; 7288800,22.62; 7289400,22.73; 7290000,22.83; 7290600,
            23; 7291200,23.06; 7291800,23.19; 7292400,23.27; 7293000,23.33; 7293600,
            23.36; 7294200,23.4; 7294800,23.48; 7295400,23.57; 7296000,23.68; 7296600,
            23.72; 7297200,23.79; 7297800,23.8; 7298400,23.78; 7299000,23.72; 7299600,
            23.64; 7300200,23.76; 7300800,23.76; 7301400,23.76; 7302000,23.8; 7302600,
            23.86; 7303200,23.92; 7303800,23.96; 7304400,23.96; 7305000,23.97; 7305600,
            24.05; 7306200,24.09; 7306800,24.09; 7307400,24.09; 7308000,24.14; 7308600,
            24.13; 7309200,24.2; 7309800,24.25; 7310400,24.25; 7311000,24.26; 7311600,
            24.28; 7312200,24.25; 7312800,24.21; 7313400,24.21; 7314000,24.17; 7314600,
            24.13; 7315200,24.13; 7315800,24.13; 7316400,24.17; 7317000,24.09; 7317600,
            24.05; 7318200,24.05; 7318800,24.01; 7319400,23.93; 7320000,23.88; 7320600,
            23.83; 7321200,23.76; 7321800,23.72; 7322400,23.64; 7323000,23.56; 7323600,
            23.56; 7324200,23.48; 7324800,23.44; 7325400,23.4; 7326000,23.36; 7326600,
            23.32; 7327200,23.27; 7327800,23.19; 7328400,23.14; 7329000,23.07; 7329600,
            23.03; 7330200,22.95; 7330800,22.93; 7331400,22.9; 7332000,22.83; 7332600,
            22.78; 7333200,22.74; 7333800,22.7; 7334400,22.66; 7335000,22.62; 7335600,
            22.58; 7336200,22.54; 7336800,22.5; 7337400,22.47; 7338000,22.42; 7338600,
            22.38; 7339200,22.36; 7339800,22.34; 7340400,22.3; 7341000,22.3; 7341600,
            22.27; 7342200,22.22; 7342800,22.22; 7343400,22.2; 7344000,22.19; 7344600,
            22.14; 7345200,22.1; 7345800,22.09; 7346400,22.08; 7347000,22.05; 7347600,
            22.05; 7348200,22.01; 7348800,21.98; 7349400,21.97; 7350000,21.97; 7350600,
            21.93; 7351200,21.93; 7351800,21.89; 7352400,21.86; 7353000,21.84; 7353600,
            21.81; 7354200,21.81; 7354800,21.77; 7355400,21.77; 7356000,21.73; 7356600,
            21.73; 7357200,21.69; 7357800,21.65; 7358400,21.65; 7359000,21.61; 7359600,
            21.6; 7360200,21.6; 7360800,21.56; 7361400,21.52; 7362000,21.52; 7362600,
            21.48; 7363200,21.48; 7363800,21.44; 7364400,21.44; 7365000,21.44; 7365600,
            21.41; 7366200,21.41; 7366800,21.4; 7367400,21.36; 7368000,21.36; 7368600,
            21.37; 7369200,21.36; 7369800,21.36; 7370400,21.37; 7371000,21.4; 7371600,
            21.4; 7372200,21.4; 7372800,21.4; 7373400,21.44; 7374000,21.44; 7374600,
            21.45; 7375200,21.48; 7375800,21.49; 7376400,21.52; 7377000,21.52; 7377600,
            21.56; 7378200,21.6; 7378800,21.61; 7379400,21.65; 7380000,21.69; 7380600,
            21.73; 7381200,21.77; 7381800,21.81; 7382400,21.81; 7383000,21.85; 7383600,
            21.89; 7384200,21.96; 7384800,21.97; 7385400,22.06; 7386000,22.09; 7386600,
            22.14; 7387200,22.26; 7387800,22.3; 7388400,22.38; 7389000,22.46; 7389600,
            22.58; 7390200,22.66; 7390800,22.78; 7391400,22.87; 7392000,22.98; 7392600,
            23.06; 7393200,23.12; 7393800,23.19; 7394400,23.27; 7395000,23.36; 7395600,
            23.44; 7396200,23.49; 7396800,23.52; 7397400,23.56; 7398000,23.6; 7398600,
            23.68; 7399200,23.73; 7399800,23.76; 7400400,23.88; 7401000,23.92; 7401600,
            24; 7402200,24.05; 7402800,24.08; 7403400,24.01; 7404000,23.95; 7404600,
            23.88; 7405200,23.84; 7405800,23.76; 7406400,23.72; 7407000,23.62; 7407600,
            23.56; 7408200,23.48; 7408800,23.4; 7409400,23.32; 7410000,23.27; 7410600,
            23.23; 7411200,23.15; 7411800,23.11; 7412400,23.03; 7413000,22.99; 7413600,
            22.94; 7414200,22.87; 7414800,22.84; 7415400,22.74; 7416000,22.7; 7416600,
            22.62; 7417200,22.58; 7417800,22.54; 7418400,22.5; 7419000,22.42; 7419600,
            22.38; 7420200,22.34; 7420800,22.3; 7421400,22.27; 7422000,22.25; 7422600,
            22.22; 7423200,22.14; 7423800,22.12; 7424400,22.09; 7425000,22.05; 7425600,
            22.01; 7426200,21.97; 7426800,21.97; 7427400,21.97; 7428000,21.93; 7428600,
            21.93; 7429200,21.89; 7429800,21.92; 7430400,21.89; 7431000,21.89; 7431600,
            21.89; 7432200,21.88; 7432800,21.85; 7433400,21.86; 7434000,21.77; 7434600,
            21.77; 7435200,21.78; 7435800,21.73; 7436400,21.73; 7437000,21.73; 7437600,
            21.69; 7438200,21.65; 7438800,21.65; 7439400,21.65; 7440000,21.61; 7440600,
            21.6; 7441200,21.6; 7441800,21.6; 7442400,21.56; 7443000,21.56; 7443600,
            21.56; 7444200,21.56; 7444800,21.52; 7445400,21.52; 7446000,21.54; 7446600,
            21.52; 7447200,21.48; 7447800,21.48; 7448400,21.48; 7449000,21.48; 7449600,
            21.48; 7450200,21.48; 7450800,21.48; 7451400,21.48; 7452000,21.48; 7452600,
            21.48; 7453200,21.5; 7453800,21.52; 7454400,21.52; 7455000,21.52; 7455600,
            21.52; 7456200,21.55; 7456800,21.56; 7457400,21.56; 7458000,21.6; 7458600,
            21.6; 7459200,21.6; 7459800,21.6; 7460400,21.62; 7461000,21.6; 7461600,21.62;
            7462200,21.64; 7462800,21.65; 7463400,21.65; 7464000,21.65; 7464600,21.66;
            7465200,21.68; 7465800,21.69; 7466400,21.68; 7467000,21.7; 7467600,21.73;
            7468200,21.73; 7468800,21.77; 7469400,21.77; 7470000,21.77; 7470600,21.77;
            7471200,21.77; 7471800,21.77; 7472400,21.77; 7473000,21.77; 7473600,21.77;
            7474200,21.77; 7474800,21.77; 7475400,21.81; 7476000,21.81; 7476600,21.81;
            7477200,21.81; 7477800,21.78; 7478400,21.77; 7479000,21.76; 7479600,21.77;
            7480200,21.81; 7480800,21.81; 7481400,21.81; 7482000,21.81; 7482600,21.81;
            7483200,21.89; 7483800,21.97; 7484400,22.09; 7485000,22.09; 7485600,22.09;
            7486200,22.05; 7486800,22.05; 7487400,22.02; 7488000,22.01; 7488600,22.01;
            7489200,21.97; 7489800,21.97; 7490400,21.93; 7491000,21.93; 7491600,21.89;
            7492200,21.85; 7492800,21.81; 7493400,21.81; 7494000,21.77; 7494600,21.73;
            7495200,21.73; 7495800,21.69; 7496400,21.66; 7497000,21.64; 7497600,21.6;
            7498200,21.6; 7498800,21.58; 7499400,21.52; 7500000,21.52; 7500600,21.48;
            7501200,21.48; 7501800,21.44; 7502400,21.44; 7503000,21.4; 7503600,21.4;
            7504200,21.36; 7504800,21.36; 7505400,21.33; 7506000,21.32; 7506600,21.28;
            7507200,21.28; 7507800,21.24; 7508400,21.24; 7509000,21.21; 7509600,21.2;
            7510200,21.16; 7510800,21.16; 7511400,21.14; 7512000,21.12; 7512600,21.12;
            7513200,21.12; 7513800,21.12; 7514400,21.08; 7515000,21.07; 7515600,21.06;
            7516200,21.05; 7516800,21.04; 7517400,21.04; 7518000,21.04; 7518600,21.03;
            7519200,21; 7519800,21; 7520400,21; 7521000,21; 7521600,21; 7522200,21;
            7522800,21; 7523400,20.96; 7524000,20.96; 7524600,20.96; 7525200,20.96;
            7525800,20.96; 7526400,20.91; 7527000,20.93; 7527600,20.96; 7528200,20.91;
            7528800,20.91; 7529400,20.91; 7530000,20.91; 7530600,20.96; 7531200,20.99;
            7531800,21; 7532400,20.98; 7533000,20.96; 7533600,20.96; 7534200,20.92;
            7534800,20.91; 7535400,20.91; 7536000,20.91; 7536600,21; 7537200,21.08;
            7537800,21.08; 7538400,21.08; 7539000,21.04; 7539600,21.01; 7540200,21.04;
            7540800,21.04; 7541400,21.06; 7542000,21.16; 7542600,21.2; 7543200,21.24;
            7543800,21.24; 7544400,21.2; 7545000,21.2; 7545600,21.21; 7546200,21.21;
            7546800,21.2; 7547400,21.24; 7548000,21.32; 7548600,21.32; 7549200,21.32;
            7549800,21.32; 7550400,21.28; 7551000,21.28; 7551600,21.24; 7552200,21.23;
            7552800,21.25; 7553400,21.36; 7554000,21.4; 7554600,21.36; 7555200,21.32;
            7555800,21.32; 7556400,21.31; 7557000,21.28; 7557600,21.24; 7558200,21.24;
            7558800,21.28; 7559400,21.36; 7560000,21.4; 7560600,21.45; 7561200,21.4;
            7561800,21.4; 7562400,21.36; 7563000,21.36; 7563600,21.32; 7564200,21.32;
            7564800,21.32; 7565400,21.32; 7566000,21.4; 7566600,21.44; 7567200,21.44;
            7567800,21.44; 7568400,21.44; 7569000,21.4; 7569600,21.4; 7570200,21.4;
            7570800,21.44; 7571400,21.44; 7572000,21.52; 7572600,21.52; 7573200,21.52;
            7573800,21.49; 7574400,21.5; 7575000,21.48; 7575600,21.44; 7576200,21.4;
            7576800,21.4; 7577400,21.38; 7578000,21.4; 7578600,21.4; 7579200,21.36;
            7579800,21.36; 7580400,21.32; 7581000,21.32; 7581600,21.32; 7582200,21.28;
            7582800,21.32; 7583400,21.32; 7584000,21.32; 7584600,21.32; 7585200,21.31;
            7585800,21.32; 7586400,21.32; 7587000,21.28; 7587600,21.25; 7588200,21.24;
            7588800,21.2; 7589400,21.19; 7590000,21.16; 7590600,21.16; 7591200,21.16;
            7591800,21.12; 7592400,21.16; 7593000,21.16; 7593600,21.15; 7594200,21.12;
            7594800,21.12; 7595400,21.12; 7596000,21.12; 7596600,21.12; 7597200,21.12;
            7597800,21.12; 7598400,21.13; 7599000,21.08; 7599600,21.08; 7600200,21.08;
            7600800,21.08; 7601400,21.08; 7602000,21.08; 7602600,21.08; 7603200,21.08;
            7603800,21.08; 7604400,21.07; 7605000,21.05; 7605600,21.04; 7606200,21.04;
            7606800,21.05; 7607400,21.04; 7608000,21.04; 7608600,21.04; 7609200,21.08;
            7609800,21.08; 7610400,21.08; 7611000,21.12; 7611600,21.12; 7612200,21.13;
            7612800,21.12; 7613400,21.12; 7614000,21.12; 7614600,21.12; 7615200,21.12;
            7615800,21.09; 7616400,21.12; 7617000,21.16; 7617600,21.13; 7618200,21.08;
            7618800,21.08; 7619400,21.04; 7620000,21; 7620600,21; 7621200,21.04; 7621800,
            21.08; 7622400,21.17; 7623000,21.5; 7623600,21.7; 7624200,21.85; 7624800,
            22.01; 7625400,22.19; 7626000,22.26; 7626600,22.3; 7627200,22.22; 7627800,
            22.34; 7628400,22.37; 7629000,22.47; 7629600,22.54; 7630200,22.62; 7630800,
            22.66; 7631400,22.74; 7632000,22.78; 7632600,22.87; 7633200,22.95; 7633800,
            23.03; 7634400,23.08; 7635000,23.15; 7635600,23.11; 7636200,23.15; 7636800,
            23.15; 7637400,23.19; 7638000,23.19; 7638600,23.19; 7639200,23.19; 7639800,
            23.23; 7640400,23.23; 7641000,23.23; 7641600,23.19; 7642200,23.19; 7642800,
            23.15; 7643400,23.15; 7644000,23.11; 7644600,23.11; 7645200,23.11; 7645800,
            23.15; 7646400,23.19; 7647000,23.27; 7647600,23.24; 7648200,23.24; 7648800,
            23.27; 7649400,23.28; 7650000,23.32; 7650600,23.35; 7651200,23.32; 7651800,
            23.39; 7652400,23.41; 7653000,23.44; 7653600,23.44; 7654200,23.47; 7654800,
            23.51; 7655400,23.48; 7656000,23.48; 7656600,23.46; 7657200,23.4; 7657800,
            23.36; 7658400,23.27; 7659000,23.23; 7659600,23.23; 7660200,23.23; 7660800,
            23.23; 7661400,23.15; 7662000,23.11; 7662600,23.07; 7663200,23.07; 7663800,
            22.95; 7664400,22.91; 7665000,22.87; 7665600,22.83; 7666200,22.78; 7666800,
            22.74; 7667400,22.7; 7668000,22.66; 7668600,22.64; 7669200,22.58; 7669800,
            22.5; 7670400,22.43; 7671000,22.39; 7671600,22.34; 7672200,22.3; 7672800,
            22.26; 7673400,22.22; 7674000,22.14; 7674600,22.09; 7675200,22.05; 7675800,
            22.05; 7676400,22.01; 7677000,21.97; 7677600,21.93; 7678200,21.9; 7678800,
            21.89; 7679400,21.85; 7680000,21.85; 7680600,21.81; 7681200,21.81; 7681800,
            21.77; 7682400,21.77; 7683000,21.73; 7683600,21.73; 7684200,21.7; 7684800,
            21.69; 7685400,21.65; 7686000,21.65; 7686600,21.65; 7687200,21.61; 7687800,
            21.5975; 7688400,21.585; 7689000,21.5725; 7689600,21.56; 7690200,21.52;
            7690800,21.52; 7691400,21.52; 7692000,21.48; 7692600,21.48; 7693200,21.45;
            7693800,21.44; 7694400,21.44; 7695000,21.44; 7695600,21.45; 7696200,21.4;
            7696800,21.44; 7697400,21.43; 7698000,21.44; 7698600,21.44; 7699200,21.48;
            7699800,21.48; 7700400,21.52; 7701000,21.49; 7701600,21.52; 7702200,21.52;
            7702800,21.6; 7703400,21.66; 7704000,21.66; 7704600,21.6; 7705200,21.56;
            7705800,21.52; 7706400,21.48; 7707000,21.52; 7707600,21.57; 7708200,21.59;
            7708800,21.65; 7709400,21.73; 7710000,21.77; 7710600,21.81; 7711200,21.89;
            7711800,22.02; 7712400,22.17; 7713000,22.26; 7713600,22.42; 7714200,22.46;
            7714800,22.62; 7715400,22.62; 7716000,22.7; 7716600,22.78; 7717200,22.82;
            7717800,22.88; 7718400,22.95; 7719000,23.03; 7719600,23.03; 7720200,23.07;
            7720800,23.18; 7721400,23.32; 7722000,23.4; 7722600,23.47; 7723200,23.52;
            7723800,23.44; 7724400,23.55; 7725000,23.48; 7725600,23.43; 7726200,23.44;
            7726800,23.44; 7727400,23.6; 7728000,23.65; 7728600,23.72; 7729200,23.77;
            7729800,23.84; 7730400,23.92; 7731000,24.01; 7731600,24.05; 7732200,24.17;
            7732800,24.21; 7733400,24.29; 7734000,24.33; 7734600,24.29; 7735200,24.29;
            7735800,24.34; 7736400,24.42; 7737000,24.5; 7737600,24.54; 7738200,24.58;
            7738800,24.52; 7739400,24.58; 7740000,24.54; 7740600,24.5; 7741200,24.62;
            7741800,24.62; 7742400,24.62; 7743000,24.58; 7743600,24.53; 7744200,24.42;
            7744800,24.42; 7745400,24.37; 7746000,24.29; 7746600,24.24; 7747200,24.17;
            7747800,24.12; 7748400,24.05; 7749000,24.05; 7749600,24.01; 7750200,23.92;
            7750800,23.81; 7751400,23.73; 7752000,23.68; 7752600,23.68; 7753200,23.6;
            7753800,23.52; 7754400,23.44; 7755000,23.36; 7755600,23.32; 7756200,23.27;
            7756800,23.15; 7757400,23.11; 7758000,23.08; 7758600,23.03; 7759200,22.95;
            7759800,22.91; 7760400,22.87; 7761000,22.8; 7761600,22.78; 7762200,22.74;
            7762800,22.7; 7763400,22.66; 7764000,22.62; 7764600,22.58; 7765200,22.54;
            7765800,22.5; 7766400,22.47; 7767000,22.46; 7767600,22.42; 7768200,22.42;
            7768800,22.38; 7769400,22.38; 7770000,22.36; 7770600,22.34; 7771200,22.3;
            7771800,22.26; 7772400,22.26; 7773000,22.22; 7773600,22.21; 7774200,22.1825;
            7774800,22.155; 7775400,22.1275; 7776000,22.1; 7776600,22.1; 7777200,22.09;
            7777800,22.09; 7778400,22.05; 7779000,22.02; 7779600,22.01; 7780200,21.97;
            7780800,21.97; 7781400,21.97; 7782000,21.97; 7782600,21.93; 7783200,21.96;
            7783800,21.98; 7784400,21.97; 7785000,21.94; 7785600,21.93; 7786200,21.93;
            7786800,21.93; 7787400,21.89; 7788000,21.89; 7788600,21.89; 7789200,21.85;
            7789800,21.85; 7790400,21.85; 7791000,21.81; 7791600,21.81; 7792200,21.81;
            7792800,21.77; 7793400,21.77; 7794000,21.77; 7794600,21.81; 7795200,21.81;
            7795800,21.85; 7796400,21.85; 7797000,21.94; 7797600,22.01; 7798200,22.02;
            7798800,22.1; 7799400,22.25; 7800000,22.3; 7800600,22.42; 7801200,22.5;
            7801800,22.63; 7802400,22.7; 7803000,22.78; 7803600,22.91; 7804200,23.08;
            7804800,23.07; 7805400,23.15; 7806000,23.27; 7806600,23.4; 7807200,23.44;
            7807800,23.52; 7808400,23.56; 7809000,23.61; 7809600,23.6; 7810200,23.6;
            7810800,23.6; 7811400,23.65; 7812000,23.6; 7812600,23.6; 7813200,23.6; 7813800,
            23.61; 7814400,23.6; 7815000,23.65; 7815600,23.72; 7816200,23.81; 7816800,
            23.92; 7817400,23.97; 7818000,24.05; 7818600,24.08; 7819200,24.04; 7819800,
            24.06; 7820400,24.09; 7821000,24.16; 7821600,24.17; 7822200,24.21; 7822800,
            24.17; 7823400,24.13; 7824000,24.13; 7824600,24.17; 7825200,24.13; 7825800,
            24.18; 7826400,24.13; 7827000,24.14; 7827600,24.13; 7828200,24.05; 7828800,
            24.09; 7829400,24.05; 7830000,24.05; 7830600,24.09; 7831200,24.05; 7831800,
            24.05; 7832400,23.96; 7833000,23.88; 7833600,23.76; 7834200,23.69; 7834800,
            23.64; 7835400,23.6; 7836000,23.54; 7836600,23.48; 7837200,23.44; 7837800,
            23.36; 7838400,23.27; 7839000,23.23; 7839600,23.23; 7840200,23.16; 7840800,
            23.12; 7841400,23.03; 7842000,22.99; 7842600,22.96; 7843200,22.9; 7843800,
            22.87; 7844400,22.83; 7845000,22.78; 7845600,22.74; 7846200,22.7; 7846800,
            22.66; 7847400,22.62; 7848000,22.62; 7848600,22.58; 7849200,22.54; 7849800,
            22.54; 7850400,22.5; 7851000,22.46; 7851600,22.46; 7852200,22.42; 7852800,
            22.42; 7853400,22.42; 7854000,22.42; 7854600,22.38; 7855200,22.38; 7855800,
            22.34; 7856400,22.34; 7857000,22.34; 7857600,22.34; 7858200,22.32; 7858800,
            22.3; 7859400,22.3; 7860000,22.3; 7860600,22.29; 7861200,22.28; 7861800,
            22.27; 7862400,22.26; 7863000,22.26; 7863600,22.22; 7864200,22.22; 7864800,
            22.22; 7865400,22.18; 7866000,22.18; 7866600,22.18; 7867200,22.17; 7867800,
            22.16; 7868400,22.14; 7869000,22.14; 7869600,22.14; 7870200,22.09; 7870800,
            22.09; 7871400,22.09; 7872000,22.09; 7872600,22.05; 7873200,22.09; 7873800,
            22.09; 7874400,22.09; 7875000,22.09; 7875600,22.06; 7876200,22.08; 7876800,
            22.06; 7877400,22.09; 7878000,22.07; 7878600,22.05; 7879200,22.05; 7879800,
            22.02; 7880400,22.01; 7881000,22.01; 7881600,22.01; 7882200,22.09; 7882800,
            22.14; 7883400,22.26; 7884000,22.34; 7884600,22.41; 7885200,22.46; 7885800,
            22.51; 7886400,22.59; 7887000,22.7; 7887600,22.8; 7888200,22.94; 7888800,
            23; 7889400,23.11; 7890000,23.11; 7890600,23.16; 7891200,23.24; 7891800,
            23.36; 7892400,23.44; 7893000,23.52; 7893600,23.58; 7894200,23.62; 7894800,
            23.68; 7895400,23.73; 7896000,23.72; 7896600,23.68; 7897200,23.68; 7897800,
            23.66; 7898400,23.64; 7899000,23.64; 7899600,23.64; 7900200,23.6; 7900800,
            23.72; 7901400,23.8; 7902000,23.8; 7902600,23.84; 7903200,23.84; 7903800,
            23.88; 7904400,23.92; 7905000,23.92; 7905600,24.01; 7906200,24.04; 7906800,
            24.05; 7907400,24.02; 7908000,23.98; 7908600,23.96; 7909200,24.01; 7909800,
            24; 7910400,24.02; 7911000,24.05; 7911600,24.2; 7912200,24.25; 7912800,24.23;
            7913400,24.21; 7914000,24.21; 7914600,24.17; 7915200,24.09; 7915800,24.05;
            7916400,24.01; 7917000,23.96; 7917600,23.92; 7918200,23.88; 7918800,23.84;
            7919400,23.82; 7920000,23.76; 7920600,23.72; 7921200,23.65; 7921800,23.56;
            7922400,23.56; 7923000,23.49; 7923600,23.44; 7924200,23.4; 7924800,23.32;
            7925400,23.23; 7926000,23.18; 7926600,23.12; 7927200,23.06; 7927800,22.96;
            7928400,22.91; 7929000,22.83; 7929600,22.8; 7930200,22.78; 7930800,22.7;
            7931400,22.68; 7932000,22.64; 7932600,22.59; 7933200,22.54; 7933800,22.46;
            7934400,22.42; 7935000,22.38; 7935600,22.34; 7936200,22.3; 7936800,22.3;
            7937400,22.26; 7938000,22.27; 7938600,22.22; 7939200,22.17; 7939800,22.17;
            7940400,22.14; 7941000,22.13; 7941600,22.09; 7942200,22.09; 7942800,22.05;
            7943400,22.05; 7944000,22; 7944600,21.97; 7945200,21.93; 7945800,21.93;
            7946400,21.89; 7947000,21.87; 7947600,21.85; 7948200,21.83; 7948800,21.81;
            7949400,21.8; 7950000,21.77; 7950600,21.73; 7951200,21.73; 7951800,21.72;
            7952400,21.69; 7953000,21.69; 7953600,21.65; 7954200,21.66; 7954800,21.64;
            7955400,21.6; 7956000,21.6; 7956600,21.57; 7957200,21.56; 7957800,21.56;
            7958400,21.52; 7959000,21.52; 7959600,21.48; 7960200,21.48; 7960800,21.48;
            7961400,21.48; 7962000,21.44; 7962600,21.44; 7963200,21.44; 7963800,21.45;
            7964400,21.44; 7965000,21.4; 7965600,21.4; 7966200,21.4; 7966800,21.38;
            7967400,21.36; 7968000,21.36; 7968600,21.37; 7969200,21.36; 7969800,21.36;
            7970400,21.39; 7971000,21.38; 7971600,21.36; 7972200,21.38; 7972800,21.36;
            7973400,21.36; 7974000,21.4; 7974600,21.41; 7975200,21.4; 7975800,21.4;
            7976400,21.4; 7977000,21.42; 7977600,21.44; 7978200,21.48; 7978800,21.48;
            7979400,21.52; 7980000,21.56; 7980600,21.6; 7981200,21.65; 7981800,21.69;
            7982400,21.76; 7983000,21.82; 7983600,21.84; 7984200,21.85; 7984800,21.93;
            7985400,21.97; 7986000,22.05; 7986600,22.07; 7987200,22.14; 7987800,22.18;
            7988400,22.26; 7989000,22.27; 7989600,22.34; 7990200,22.39; 7990800,22.42;
            7991400,22.42; 7992000,22.46; 7992600,22.5; 7993200,22.54; 7993800,22.58;
            7994400,22.6; 7995000,22.61; 7995600,22.62; 7996200,22.66; 7996800,22.7;
            7997400,22.78; 7998000,22.8; 7998600,22.86; 7999200,22.91; 7999800,22.92;
            8000400,23.02; 8001000,23.07; 8001600,23.11; 8002200,23.19; 8002800,23.19;
            8003400,23.19; 8004000,23.15; 8004600,23.15; 8005200,23.15; 8005800,23.12;
            8006400,23.11; 8007000,23.11; 8007600,23.08; 8008200,23.03; 8008800,22.99;
            8009400,22.95; 8010000,22.87; 8010600,22.83; 8011200,22.76; 8011800,22.74;
            8012400,22.7; 8013000,22.66; 8013600,22.62; 8014200,22.59; 8014800,22.55;
            8015400,22.5; 8016000,22.43; 8016600,22.42; 8017200,22.38; 8017800,22.35;
            8018400,22.3; 8019000,22.3; 8019600,22.26; 8020200,22.22; 8020800,22.18;
            8021400,22.17; 8022000,22.14; 8022600,22.09; 8023200,22.07; 8023800,22.05;
            8024400,22.01; 8025000,21.97; 8025600,21.97; 8026200,21.93; 8026800,21.92;
            8027400,21.89; 8028000,21.85; 8028600,21.85; 8029200,21.81; 8029800,21.85;
            8030400,21.85; 8031000,21.89; 8031600,21.89; 8032200,21.89; 8032800,21.92;
            8033400,21.9225; 8034000,21.925; 8034600,21.9275; 8035200,21.93; 8035800,
            21.93; 8036400,21.93; 8037000,21.93; 8037600,21.93; 8038200,21.93; 8038800,
            21.89; 8039400,21.9; 8040000,21.92; 8040600,21.93; 8041200,21.89; 8041800,
            21.89; 8042400,21.89; 8043000,21.89; 8043600,21.9; 8044200,21.9; 8044800,
            21.85; 8045400,21.85; 8046000,21.82; 8046600,21.81; 8047200,21.81; 8047800,
            21.81; 8048400,21.81; 8049000,21.81; 8049600,21.81; 8050200,21.81; 8050800,
            21.81; 8051400,21.81; 8052000,21.81; 8052600,21.82; 8053200,21.81; 8053800,
            21.81; 8054400,21.81; 8055000,21.81; 8055600,21.81; 8056200,21.85; 8056800,
            21.85; 8057400,21.85; 8058000,21.85; 8058600,21.88; 8059200,21.89; 8059800,
            21.89; 8060400,21.89; 8061000,21.92; 8061600,21.93; 8062200,21.93; 8062800,
            21.97; 8063400,21.97; 8064000,21.97; 8064600,21.94; 8065200,21.97; 8065800,
            21.97; 8066400,21.97; 8067000,21.97; 8067600,21.97; 8068200,22; 8068800,
            22.04; 8069400,22.01; 8070000,22.04; 8070600,22.05; 8071200,22.03; 8071800,
            22.01; 8072400,22.01; 8073000,22.01; 8073600,22.05; 8074200,22.06; 8074800,
            22.06; 8075400,22.05; 8076000,22.05; 8076600,22.05; 8077200,22.05; 8077800,
            22.01; 8078400,22.02; 8079000,22.03; 8079600,22.05; 8080200,22.06; 8080800,
            22.05; 8081400,22.05; 8082000,22.01; 8082600,22.01; 8083200,22.01; 8083800,
            22.02; 8084400,21.98; 8085000,21.97; 8085600,21.98; 8086200,21.97; 8086800,
            21.98; 8087400,21.97; 8088000,21.97; 8088600,21.97; 8089200,21.98; 8089800,
            21.97; 8090400,21.97; 8091000,21.97; 8091600,21.96; 8092200,21.95; 8092800,
            21.93; 8093400,21.93; 8094000,21.89; 8094600,21.89; 8095200,21.89; 8095800,
            21.85; 8096400,21.85; 8097000,21.85; 8097600,21.81; 8098200,21.81; 8098800,
            21.78; 8099400,21.77; 8100000,21.77; 8100600,21.73; 8101200,21.73; 8101800,
            21.7; 8102400,21.69; 8103000,21.7; 8103600,21.69; 8104200,21.65; 8104800,
            21.65; 8105400,21.65; 8106000,21.65; 8106600,21.62; 8107200,21.6; 8107800,
            21.6; 8108400,21.63; 8109000,21.6; 8109600,21.6; 8110200,21.62; 8110800,
            21.64; 8111400,21.66; 8112000,21.65; 8112600,21.65; 8113200,21.65; 8113800,
            21.65; 8114400,21.7; 8115000,21.69; 8115600,21.69; 8116200,21.72; 8116800,
            21.7; 8117400,21.69; 8118000,21.69; 8118600,21.71; 8119200,21.72; 8119800,
            21.7125; 8120400,21.705; 8121000,21.6975; 8121600,21.69; 8122200,21.68;
            8122800,21.68; 8123400,21.65; 8124000,21.65; 8124600,21.65; 8125200,21.65;
            8125800,21.65; 8126400,21.64; 8127000,21.64; 8127600,21.64; 8128200,21.64;
            8128800,21.6; 8129400,21.6; 8130000,21.6; 8130600,21.6; 8131200,21.6; 8131800,
            21.6; 8132400,21.6; 8133000,21.6; 8133600,21.6; 8134200,21.6; 8134800,21.65;
            8135400,21.66; 8136000,21.65; 8136600,21.65; 8137200,21.6; 8137800,21.56;
            8138400,21.54; 8139000,21.52; 8139600,21.52; 8140200,21.54; 8140800,21.6;
            8141400,21.69; 8142000,21.75; 8142600,21.85; 8143200,21.93; 8143800,21.97;
            8144400,22.01; 8145000,22.09; 8145600,22.21; 8146200,22.3; 8146800,22.45;
            8147400,22.46; 8148000,22.55; 8148600,22.58; 8149200,22.62; 8149800,22.62;
            8150400,22.67; 8151000,22.74; 8151600,22.83; 8152200,22.86; 8152800,22.92;
            8153400,23; 8154000,22.99; 8154600,22.99; 8155200,23.03; 8155800,23.07;
            8156400,23.11; 8157000,23.11; 8157600,23.15; 8158200,23.17; 8158800,23.2;
            8159400,23.31; 8160000,23.36; 8160600,23.4; 8161200,23.44; 8161800,23.52;
            8162400,23.52; 8163000,23.52; 8163600,23.56; 8164200,23.56; 8164800,23.6;
            8165400,23.64; 8166000,23.64; 8166600,23.69; 8167200,23.75; 8167800,23.72;
            8168400,23.76; 8169000,23.76; 8169600,23.78; 8170200,23.76; 8170800,23.8;
            8171400,23.88; 8172000,23.88; 8172600,23.92; 8173200,23.96; 8173800,24;
            8174400,23.96; 8175000,23.96; 8175600,23.96; 8176200,23.92; 8176800,23.94;
            8177400,23.92; 8178000,23.92; 8178600,23.88; 8179200,23.84; 8179800,23.8;
            8180400,23.72; 8181000,23.69; 8181600,23.68; 8182200,23.56; 8182800,23.56;
            8183400,23.48; 8184000,23.44; 8184600,23.41; 8185200,23.33; 8185800,23.24;
            8186400,23.15; 8187000,23.11; 8187600,23.03; 8188200,22.95; 8188800,22.91;
            8189400,22.86; 8190000,22.78; 8190600,22.74; 8191200,22.7; 8191800,22.62;
            8192400,22.54; 8193000,22.5; 8193600,22.42; 8194200,22.42; 8194800,22.38;
            8195400,22.34; 8196000,22.3; 8196600,22.26; 8197200,22.19; 8197800,22.14;
            8198400,22.14; 8199000,22.09; 8199600,22.09; 8200200,22.05; 8200800,22.05;
            8201400,22.02; 8202000,22.01; 8202600,21.98; 8203200,21.97; 8203800,21.97;
            8204400,21.96; 8205000,21.93; 8205600,21.93; 8206200,21.9125; 8206800,21.895;
            8207400,21.8775; 8208000,21.86; 8208600,21.85; 8209200,21.85; 8209800,21.81;
            8210400,21.82; 8211000,21.81; 8211600,21.81; 8212200,21.93; 8212800,21.98;
            8213400,22.02; 8214000,22.01; 8214600,22.02; 8215200,21.97; 8215800,21.93;
            8216400,21.89; 8217000,21.85; 8217600,21.87; 8218200,21.85; 8218800,21.85;
            8219400,21.87; 8220000,21.89; 8220600,21.89; 8221200,21.85; 8221800,21.85;
            8222400,21.83; 8223000,21.85; 8223600,21.85; 8224200,21.85; 8224800,21.81;
            8225400,21.84; 8226000,21.9; 8226600,21.94; 8227200,21.98; 8227800,22.05;
            8228400,22.05; 8229000,22.18; 8229600,22.3; 8230200,22.42; 8230800,22.5;
            8231400,22.62; 8232000,22.66; 8232600,22.74; 8233200,22.84; 8233800,22.88;
            8234400,22.91; 8235000,22.95; 8235600,22.96; 8236200,22.99; 8236800,23.03;
            8237400,23.11; 8238000,23.19; 8238600,23.23; 8239200,23.23; 8239800,23.27;
            8240400,23.36; 8241000,23.4; 8241600,23.4; 8242200,23.41; 8242800,23.36;
            8243400,23.35; 8244000,23.27; 8244600,23.27; 8245200,23.32; 8245800,23.27;
            8246400,23.27; 8247000,23.39; 8247600,23.4; 8248200,23.44; 8248800,23.48;
            8249400,23.52; 8250000,23.53; 8250600,23.53; 8251200,23.6; 8251800,23.64;
            8252400,23.68; 8253000,23.71; 8253600,23.72; 8254200,23.76; 8254800,23.82;
            8255400,23.84; 8256000,23.8; 8256600,23.88; 8257200,23.84; 8257800,23.8;
            8258400,23.8; 8259000,23.88; 8259600,23.88; 8260200,23.84; 8260800,23.84;
            8261400,23.84; 8262000,23.73; 8262600,23.82; 8263200,23.72; 8263800,23.64;
            8264400,23.56; 8265000,23.52; 8265600,23.52; 8266200,23.45; 8266800,23.44;
            8267400,23.4; 8268000,23.33; 8268600,23.27; 8269200,23.27; 8269800,23.19;
            8270400,23.15; 8271000,23.07; 8271600,23.03; 8272200,23; 8272800,22.98;
            8273400,22.95; 8274000,22.91; 8274600,22.86; 8275200,22.79; 8275800,22.74;
            8276400,22.7; 8277000,22.66; 8277600,22.64; 8278200,22.58; 8278800,22.54;
            8279400,22.5; 8280000,22.47; 8280600,22.42; 8281200,22.38; 8281800,22.34;
            8282400,22.3; 8283000,22.27; 8283600,22.22; 8284200,22.18; 8284800,22.14;
            8285400,22.13; 8286000,22.09; 8286600,22.06; 8287200,22.05; 8287800,22.01;
            8288400,21.97; 8289000,21.97; 8289600,21.93; 8290200,21.93; 8290800,21.89;
            8291400,21.89; 8292000,21.876; 8292600,21.862; 8293200,21.848; 8293800,21.834;
            8294400,21.82; 8295000,21.81; 8295600,21.77; 8296200,21.77; 8296800,21.73;
            8297400,21.73; 8298000,21.73; 8298600,21.73; 8299200,21.69; 8299800,21.69;
            8300400,21.69; 8301000,21.65; 8301600,21.65; 8302200,21.65; 8302800,21.65;
            8303400,21.64; 8304000,21.62; 8304600,21.6; 8305200,21.6; 8305800,21.6;
            8306400,21.6; 8307000,21.6; 8307600,21.56; 8308200,21.6; 8308800,21.6; 8309400,
            21.56; 8310000,21.56; 8310600,21.6; 8311200,21.6; 8311800,21.6; 8312400,
            21.62; 8313000,21.65; 8313600,21.85; 8314200,22.14; 8314800,22.37; 8315400,
            22.44; 8316000,22.54; 8316600,22.69; 8317200,22.78; 8317800,22.91; 8318400,
            22.99; 8319000,23.11; 8319600,23.19; 8320200,23.24; 8320800,23.27; 8321400,
            23.31; 8322000,23.4; 8322600,23.44; 8323200,23.48; 8323800,23.48; 8324400,
            23.48; 8325000,23.6; 8325600,23.67; 8326200,23.72; 8326800,23.76; 8327400,
            23.73; 8328000,23.82; 8328600,23.86; 8329200,23.88; 8329800,23.9; 8330400,
            23.94; 8331000,23.92; 8331600,23.92; 8332200,23.92; 8332800,24.04; 8333400,
            24.12; 8334000,24.13; 8334600,24.14; 8335200,24.13; 8335800,24.13; 8336400,
            24.13; 8337000,24.09; 8337600,24.13; 8338200,24.17; 8338800,24.21; 8339400,
            24.22; 8340000,24.21; 8340600,24.25; 8341200,24.29; 8341800,24.37; 8342400,
            24.33; 8343000,24.41; 8343600,24.41; 8344200,24.5; 8344800,24.46; 8345400,
            24.66; 8346000,24.66; 8346600,24.6; 8347200,24.58; 8347800,24.58; 8348400,
            24.48; 8349000,24.41; 8349600,24.34; 8350200,24.27; 8350800,24.25; 8351400,
            24.17; 8352000,24.13; 8352600,24.09; 8353200,24.01; 8353800,23.92; 8354400,
            23.89; 8355000,23.8; 8355600,23.72; 8356200,23.64; 8356800,23.56; 8357400,
            23.52; 8358000,23.45; 8358600,23.4; 8359200,23.36; 8359800,23.27; 8360400,
            23.19; 8361000,23.15; 8361600,23.07; 8362200,22.99; 8362800,22.95; 8363400,
            22.87; 8364000,22.81; 8364600,22.74; 8365200,22.7; 8365800,22.62; 8366400,
            22.58; 8367000,22.5; 8367600,22.42; 8368200,22.42; 8368800,22.35; 8369400,
            22.3; 8370000,22.26; 8370600,22.23; 8371200,22.19; 8371800,22.17; 8372400,
            22.14; 8373000,22.09; 8373600,22.07; 8374200,22.06; 8374800,21.99; 8375400,
            21.97; 8376000,21.96; 8376600,21.94; 8377200,21.88; 8377800,21.85; 8378400,
            21.82; 8379000,21.8; 8379600,21.78; 8380200,21.76; 8380800,21.74; 8381400,
            21.73; 8382000,21.73; 8382600,21.69; 8383200,21.65; 8383800,21.65; 8384400,
            21.64; 8385000,21.61; 8385600,21.62; 8386200,21.62; 8386800,21.6; 8387400,
            21.56; 8388000,21.56; 8388600,21.52; 8389200,21.52; 8389800,21.52; 8390400,
            21.48; 8391000,21.48; 8391600,21.48; 8392200,21.45; 8392800,21.44; 8393400,
            21.44; 8394000,21.44; 8394600,21.41; 8395200,21.44; 8395800,21.44; 8396400,
            21.44; 8397000,21.44; 8397600,21.41; 8398200,21.44; 8398800,21.44; 8399400,
            21.48; 8400000,21.52; 8400600,21.66; 8401200,21.73; 8401800,21.81; 8402400,
            21.88; 8403000,22.18; 8403600,22.42; 8404200,22.62; 8404800,22.74; 8405400,
            22.74; 8406000,22.78; 8406600,22.87; 8407200,22.91; 8407800,23.01; 8408400,
            23.03; 8409000,23.07; 8409600,23.08; 8410200,23.07; 8410800,23.07; 8411400,
            23.11; 8412000,23.15; 8412600,23.19; 8413200,23.23; 8413800,23.24; 8414400,
            23.27; 8415000,23.27; 8415600,23.27; 8416200,23.27; 8416800,23.27; 8417400,
            23.27; 8418000,23.38; 8418600,23.39; 8419200,23.44; 8419800,23.53; 8420400,
            23.56; 8421000,23.61; 8421600,23.68; 8422200,23.68; 8422800,23.8; 8423400,
            23.84; 8424000,23.88; 8424600,23.89; 8425200,23.96; 8425800,23.96; 8426400,
            24; 8427000,24.01; 8427600,24.02; 8428200,24.04; 8428800,24.11; 8429400,
            24.25; 8430000,24.33; 8430600,24.33; 8431200,24.37; 8431800,24.37; 8432400,
            24.29; 8433000,24.26; 8433600,24.22; 8434200,24.13; 8434800,24.06; 8435400,
            24.05; 8436000,24.01; 8436600,24.01; 8437200,24.06; 8437800,24; 8438400,
            23.92; 8439000,23.88; 8439600,23.84; 8440200,23.8; 8440800,23.76; 8441400,
            23.72; 8442000,23.68; 8442600,23.6; 8443200,23.56; 8443800,23.56; 8444400,
            23.52; 8445000,23.48; 8445600,23.4; 8446200,23.36; 8446800,23.32; 8447400,
            23.27; 8448000,23.2; 8448600,23.15; 8449200,23.1; 8449800,23.03; 8450400,
            22.99; 8451000,22.91; 8451600,22.87; 8452200,22.78; 8452800,22.74; 8453400,
            22.68; 8454000,22.62; 8454600,22.58; 8455200,22.54; 8455800,22.54; 8456400,
            22.5; 8457000,22.46; 8457600,22.42; 8458200,22.38; 8458800,22.38; 8459400,
            22.34; 8460000,22.34; 8460600,22.3; 8461200,22.26; 8461800,22.26; 8462400,
            22.22; 8463000,22.18; 8463600,22.18; 8464200,22.14; 8464800,22.122; 8465400,
            22.104; 8466000,22.086; 8466600,22.068; 8467200,22.05; 8467800,22.02; 8468400,
            21.98; 8469000,21.97; 8469600,21.98; 8470200,21.97; 8470800,21.97; 8471400,
            22.01; 8472000,22.01; 8472600,22.01; 8473200,22.05; 8473800,22.05; 8474400,
            22.03; 8475000,22.04; 8475600,22.05; 8476200,22.05; 8476800,22.05; 8477400,
            22.06; 8478000,22.01; 8478600,22.05; 8479200,22.22; 8479800,22.3; 8480400,
            22.35; 8481000,22.3; 8481600,22.26; 8482200,22.22; 8482800,22.18; 8483400,
            22.09; 8484000,22.09; 8484600,22.18; 8485200,22.26; 8485800,22.34; 8486400,
            22.38; 8487000,22.46; 8487600,22.46; 8488200,22.46; 8488800,22.5; 8489400,
            22.58; 8490000,22.66; 8490600,22.83; 8491200,22.98; 8491800,23.03; 8492400,
            23.03; 8493000,23.06; 8493600,23.11; 8494200,23.11; 8494800,23.3; 8495400,
            23.41; 8496000,23.52; 8496600,23.52; 8497200,23.48; 8497800,23.52; 8498400,
            23.52; 8499000,23.52; 8499600,23.56; 8500200,23.6; 8500800,23.68; 8501400,
            23.64; 8502000,23.6; 8502600,23.56; 8503200,23.48; 8503800,23.4; 8504400,
            23.4; 8505000,23.49; 8505600,23.56; 8506200,23.56; 8506800,23.56; 8507400,
            23.56; 8508000,23.6; 8508600,23.6; 8509200,23.64; 8509800,23.56; 8510400,
            23.64; 8511000,23.72; 8511600,23.72; 8512200,23.74; 8512800,23.72; 8513400,
            23.76; 8514000,23.76; 8514600,23.72; 8515200,23.76; 8515800,23.77; 8516400,
            23.8; 8517000,23.8; 8517600,23.76; 8518200,23.8; 8518800,23.8; 8519400,23.8;
            8520000,23.77; 8520600,23.8; 8521200,23.8; 8521800,23.77; 8522400,23.8;
            8523000,23.76; 8523600,23.73; 8524200,23.6; 8524800,23.56; 8525400,23.48;
            8526000,23.4; 8526600,23.36; 8527200,23.27; 8527800,23.19; 8528400,23.11;
            8529000,23.07; 8529600,23.07; 8530200,23.04; 8530800,23.01; 8531400,22.95;
            8532000,22.91; 8532600,22.82; 8533200,22.74; 8533800,22.74; 8534400,22.71;
            8535000,22.66; 8535600,22.62; 8536200,22.59; 8536800,22.54; 8537400,22.5;
            8538000,22.43; 8538600,22.38; 8539200,22.34; 8539800,22.3; 8540400,22.26;
            8541000,22.22; 8541600,22.18; 8542200,22.18; 8542800,22.14; 8543400,22.14;
            8544000,22.1; 8544600,22.09; 8545200,22.09; 8545800,22.05; 8546400,22.01;
            8547000,22.01; 8547600,21.97; 8548200,21.98; 8548800,21.89; 8549400,21.88;
            8550000,21.86; 8550600,21.85; 8551200,21.81; 8551800,21.78; 8552400,21.75;
            8553000,21.72; 8553600,21.69; 8554200,21.65; 8554800,21.66; 8555400,21.6;
            8556000,21.6; 8556600,21.59; 8557200,21.55; 8557800,21.52; 8558400,21.52;
            8559000,21.48; 8559600,21.47; 8560200,21.44; 8560800,21.44; 8561400,21.44;
            8562000,21.4; 8562600,21.4; 8563200,21.4; 8563800,21.36; 8564400,21.37;
            8565000,21.36; 8565600,21.32; 8566200,21.32; 8566800,21.32; 8567400,21.32;
            8568000,21.32; 8568600,21.28; 8569200,21.24; 8569800,21.24; 8570400,21.24;
            8571000,21.24; 8571600,21.24; 8572200,21.2; 8572800,21.2; 8573400,21.2;
            8574000,21.2; 8574600,21.2; 8575200,21.2; 8575800,21.16; 8576400,21.16;
            8577000,21.2; 8577600,21.2; 8578200,21.2; 8578800,21.2; 8579400,21.2; 8580000,
            21.2; 8580600,21.2; 8581200,21.2; 8581800,21.2; 8582400,21.2; 8583000,21.2;
            8583600,21.23; 8584200,21.24; 8584800,21.24; 8585400,21.26; 8586000,21.27;
            8586600,21.29; 8587200,21.28; 8587800,21.28; 8588400,21.28; 8589000,21.28;
            8589600,21.32; 8590200,21.28; 8590800,21.32; 8591400,21.32; 8592000,21.4;
            8592600,21.48; 8593200,21.52; 8593800,21.52; 8594400,21.52; 8595000,21.57;
            8595600,21.6; 8596200,21.72; 8596800,21.81; 8597400,21.81; 8598000,21.81;
            8598600,21.77; 8599200,21.76; 8599800,21.73; 8600400,21.69; 8601000,21.69;
            8601600,21.65; 8602200,21.64; 8602800,21.6; 8603400,21.6; 8604000,21.56;
            8604600,21.56; 8605200,21.56; 8605800,21.56; 8606400,21.56; 8607000,21.56;
            8607600,21.56; 8608200,21.58; 8608800,21.56; 8609400,21.52; 8610000,21.54;
            8610600,21.54; 8611200,21.52; 8611800,21.52; 8612400,21.5; 8613000,21.48;
            8613600,21.48; 8614200,21.48; 8614800,21.44; 8615400,21.45; 8616000,21.4;
            8616600,21.38; 8617200,21.36; 8617800,21.32; 8618400,21.32; 8619000,21.28;
            8619600,21.24; 8620200,21.21; 8620800,21.2; 8621400,21.16; 8622000,21.12;
            8622600,21.13; 8623200,21.12; 8623800,21.08; 8624400,21.08; 8625000,21.04;
            8625600,21; 8626200,21; 8626800,20.96; 8627400,20.96; 8628000,20.91; 8628600,
            20.91; 8629200,20.87; 8629800,20.85; 8630400,20.83; 8631000,20.83; 8631600,
            20.79; 8632200,20.78; 8632800,20.76; 8633400,20.71; 8634000,20.83; 8634600,
            20.95; 8635200,21; 8635800,21.04; 8636400,21.08; 8637000,21.08; 8637600,
            21.12; 8638200,21.1375; 8638800,21.155; 8639400,21.1725; 8640000,21.19;
            8640600,21.2; 8641200,21.2; 8641800,21.2; 8642400,21.24; 8643000,21.25;
            8643600,21.24; 8644200,21.25; 8644800,21.24; 8645400,21.28; 8646000,21.25;
            8646600,21.28; 8647200,21.28; 8647800,21.28; 8648400,21.28; 8649000,21.24;
            8649600,21.28; 8650200,21.28; 8650800,21.28; 8651400,21.28; 8652000,21.28;
            8652600,21.28; 8653200,21.28; 8653800,21.28; 8654400,21.28; 8655000,21.28;
            8655600,21.28; 8656200,21.28; 8656800,21.28; 8657400,21.28; 8658000,21.32;
            8658600,21.32; 8659200,21.32; 8659800,21.35; 8660400,21.36; 8661000,21.36;
            8661600,21.4; 8662200,21.4; 8662800,21.45; 8663400,21.46; 8664000,21.44;
            8664600,21.48; 8665200,21.45; 8665800,21.48; 8666400,21.52; 8667000,21.52;
            8667600,21.56; 8668200,21.56; 8668800,21.56; 8669400,21.56; 8670000,21.57;
            8670600,21.56; 8671200,21.56; 8671800,21.56; 8672400,21.56; 8673000,21.56;
            8673600,21.56; 8674200,21.56; 8674800,21.58; 8675400,21.6; 8676000,21.6;
            8676600,21.6; 8677200,21.6; 8677800,21.65; 8678400,21.73; 8679000,21.77;
            8679600,21.81; 8680200,22.09; 8680800,21.92; 8681400,21.97; 8682000,22.09;
            8682600,22.18; 8683200,22.26; 8683800,22.34; 8684400,22.42; 8685000,22.46;
            8685600,22.54; 8686200,22.62; 8686800,22.66; 8687400,22.71; 8688000,22.78;
            8688600,22.83; 8689200,22.87; 8689800,22.91; 8690400,22.93; 8691000,22.95;
            8691600,22.99; 8692200,23.03; 8692800,23.07; 8693400,23.11; 8694000,23.15;
            8694600,23.15; 8695200,23.19; 8695800,23.23; 8696400,23.56; 8697000,23.23;
            8697600,23.11; 8698200,23.07; 8698800,23.03; 8699400,22.94; 8700000,22.78;
            8700600,22.74; 8701200,22.7; 8701800,22.66; 8702400,22.58; 8703000,22.5;
            8703600,22.42; 8704200,22.38; 8704800,22.31; 8705400,22.26; 8706000,22.26;
            8706600,22.22; 8707200,22.14; 8707800,22.14; 8708400,22.09; 8709000,22.06;
            8709600,22.05; 8710200,22.01; 8710800,22.01; 8711400,22.01; 8712000,21.97;
            8712600,21.98; 8713200,21.97; 8713800,21.98; 8714400,21.97; 8715000,21.93;
            8715600,21.93; 8716200,21.92; 8716800,21.93; 8717400,21.89; 8718000,21.89;
            8718600,21.89; 8719200,21.85; 8719800,21.85; 8720400,21.85; 8721000,21.85;
            8721600,21.84; 8722200,21.85; 8722800,21.81; 8723400,21.81; 8724000,21.81;
            8724600,21.8; 8725200,21.79; 8725800,21.78; 8726400,21.77; 8727000,21.73;
            8727600,21.73; 8728200,21.74; 8728800,21.73; 8729400,21.73; 8730000,21.7;
            8730600,21.7; 8731200,21.69; 8731800,21.69; 8732400,21.7; 8733000,21.65;
            8733600,21.66; 8734200,21.65; 8734800,21.65; 8735400,21.65; 8736000,21.66;
            8736600,21.65; 8737200,21.62; 8737800,21.64; 8738400,21.81; 8739000,22.01;
            8739600,22.09; 8740200,22.14; 8740800,22.05; 8741400,21.93; 8742000,21.85;
            8742600,21.81; 8743200,21.81; 8743800,21.88; 8744400,21.97; 8745000,22.05;
            8745600,22.05; 8746200,22.09; 8746800,22.14; 8747400,22.22; 8748000,22.26;
            8748600,22.42; 8749200,22.6; 8749800,22.74; 8750400,22.84; 8751000,22.95;
            8751600,23.07; 8752200,23.2; 8752800,23.23; 8753400,23.27; 8754000,23.32;
            8754600,23.37; 8755200,23.4; 8755800,23.48; 8756400,23.52; 8757000,23.52;
            8757600,23.64; 8758200,23.64; 8758800,23.69; 8759400,23.72; 8760000,23.68;
            8760600,23.68; 8761200,23.68; 8761800,23.68; 8762400,23.68; 8763000,23.68;
            8763600,23.65; 8764200,23.57; 8764800,23.56; 8765400,23.48; 8766000,23.44;
            8766600,23.44; 8767200,23.44; 8767800,23.4; 8768400,23.36; 8769000,23.36;
            8769600,23.36; 8770200,23.36; 8770800,23.52; 8771400,23.56; 8772000,23.54;
            8772600,23.68; 8773200,23.76; 8773800,23.8; 8774400,23.94; 8775000,24.04;
            8775600,24.1; 8776200,24.09; 8776800,24.17; 8777400,24.13; 8778000,24.36;
            8778600,24.09; 8779200,24.17; 8779800,24.13; 8780400,24.17; 8781000,24.2;
            8781600,24.22; 8782200,24.33; 8782800,24.49; 8783400,24.21; 8784000,24.13;
            8784600,24.05; 8785200,23.96; 8785800,23.88; 8786400,23.76; 8787000,23.68;
            8787600,23.65; 8788200,23.6; 8788800,23.56; 8789400,23.45; 8790000,23.4;
            8790600,23.32; 8791200,23.27; 8791800,23.19; 8792400,23.12; 8793000,23.07;
            8793600,23.03; 8794200,22.91; 8794800,22.87; 8795400,22.78; 8796000,22.76;
            8796600,22.78; 8797200,22.74; 8797800,22.66; 8798400,22.62; 8799000,22.59;
            8799600,22.5; 8800200,22.46; 8800800,22.42; 8801400,22.39; 8802000,22.33;
            8802600,22.29; 8803200,22.26; 8803800,22.22; 8804400,22.19; 8805000,22.17;
            8805600,22.14; 8806200,22.09; 8806800,22.08; 8807400,22.05; 8808000,22.02;
            8808600,22.01; 8809200,22; 8809800,21.97; 8810400,21.946; 8811000,21.922;
            8811600,21.898; 8812200,21.874; 8812800,21.85; 8813400,21.86; 8814000,21.81;
            8814600,21.81; 8815200,21.81; 8815800,21.82; 8816400,21.81; 8817000,21.81;
            8817600,21.82; 8818200,21.85; 8818800,21.88; 8819400,21.89; 8820000,21.89;
            8820600,21.89; 8821200,21.93; 8821800,21.93; 8822400,21.97; 8823000,21.97;
            8823600,21.97; 8824200,21.97; 8824800,22.05; 8825400,22.26; 8826000,22.42;
            8826600,22.47; 8827200,22.42; 8827800,22.3; 8828400,22.22; 8829000,22.18;
            8829600,22.12; 8830200,22.14; 8830800,22.34; 8831400,22.48; 8832000,22.58;
            8832600,22.62; 8833200,22.67; 8833800,22.62; 8834400,22.68; 8835000,22.74;
            8835600,22.9; 8836200,22.99; 8836800,23.1; 8837400,23.23; 8838000,23.27;
            8838600,23.36; 8839200,23.44; 8839800,23.52; 8840400,23.53; 8841000,23.64;
            8841600,23.72; 8842200,23.76; 8842800,23.89; 8843400,23.89; 8844000,23.88;
            8844600,23.92; 8845200,24.06; 8845800,24.05; 8846400,24.04; 8847000,24.01;
            8847600,24; 8848200,23.95; 8848800,23.92; 8849400,23.92; 8850000,23.97;
            8850600,24.01; 8851200,24.01; 8851800,24; 8852400,24.1; 8853000,24.13; 8853600,
            24.17; 8854200,24.13; 8854800,24.13; 8855400,24.21; 8856000,24.25; 8856600,
            24.25; 8857200,24.25; 8857800,24.26; 8858400,24.25; 8859000,24.29; 8859600,
            24.37; 8860200,24.42; 8860800,24.46; 8861400,24.57; 8862000,24.58; 8862600,
            24.66; 8863200,24.69; 8863800,24.66; 8864400,24.71; 8865000,24.66; 8865600,
            24.58; 8866200,24.5; 8866800,24.41; 8867400,24.37; 8868000,24.25; 8868600,
            24.18; 8869200,24.1; 8869800,24.05; 8870400,23.96; 8871000,23.89; 8871600,
            23.86; 8872200,23.8; 8872800,23.74; 8873400,23.68; 8874000,23.6; 8874600,
            23.56; 8875200,23.52; 8875800,23.48; 8876400,23.44; 8877000,23.41; 8877600,
            23.36; 8878200,23.35; 8878800,23.27; 8879400,23.23; 8880000,23.19; 8880600,
            23.15; 8881200,23.11; 8881800,23.07; 8882400,23.03; 8883000,22.95; 8883600,
            22.91; 8884200,22.87; 8884800,22.8; 8885400,22.78; 8886000,22.74; 8886600,
            22.7; 8887200,22.7; 8887800,22.67; 8888400,22.62; 8889000,22.59; 8889600,
            22.54; 8890200,22.5; 8890800,22.5; 8891400,22.46; 8892000,22.43; 8892600,
            22.42; 8893200,22.38; 8893800,22.36; 8894400,22.34; 8895000,22.3; 8895600,
            22.26; 8896200,22.26; 8896800,22.22; 8897400,22.2; 8898000,22.18; 8898600,
            22.16; 8899200,22.14; 8899800,22.13; 8900400,22.09; 8901000,22.09; 8901600,
            22.05; 8902200,22.06; 8902800,22.01; 8903400,21.97; 8904000,21.97; 8904600,
            21.97; 8905200,21.93; 8905800,21.93; 8906400,21.89; 8907000,21.88; 8907600,
            21.85; 8908200,21.85; 8908800,21.86; 8909400,21.81; 8910000,21.81; 8910600,
            21.81; 8911200,21.81; 8911800,21.78; 8912400,21.77; 8913000,21.77; 8913600,
            21.77; 8914200,22.01; 8914800,21.77; 8915400,21.81; 8916000,21.85; 8916600,
            21.85; 8917200,21.85; 8917800,22.06; 8918400,22.33; 8919000,22.5; 8919600,
            22.62; 8920200,22.79; 8920800,22.9; 8921400,22.97; 8922000,22.99; 8922600,
            23.07; 8923200,23.11; 8923800,23.15; 8924400,23.16; 8925000,23.19; 8925600,
            23.23; 8926200,23.27; 8926800,23.24; 8927400,23.23; 8928000,23.26; 8928600,
            23.27; 8929200,23.28; 8929800,23.32; 8930400,23.36; 8931000,23.36; 8931600,
            23.4; 8932200,23.42; 8932800,23.42; 8933400,23.46; 8934000,23.51; 8934600,
            23.46; 8935200,23.4; 8935800,23.4; 8936400,23.48; 8937000,23.55; 8937600,
            23.6; 8938200,23.6; 8938800,23.6; 8939400,23.61; 8940000,23.6; 8940600,23.72;
            8941200,23.8; 8941800,23.91; 8942400,23.89; 8943000,23.76; 8943600,23.64;
            8944200,23.48; 8944800,23.48; 8945400,23.6; 8946000,23.42; 8946600,23.65;
            8947200,23.79; 8947800,23.8; 8948400,23.8; 8949000,23.88; 8949600,23.88;
            8950200,23.92; 8950800,24.01; 8951400,23.96; 8952000,23.88; 8952600,23.88;
            8953200,23.88; 8953800,23.8; 8954400,23.72; 8955000,23.68; 8955600,23.68;
            8956200,23.6; 8956800,23.56; 8957400,23.48; 8958000,23.44; 8958600,23.36;
            8959200,23.3; 8959800,23.23; 8960400,23.16; 8961000,23.11; 8961600,23.07;
            8962200,22.98; 8962800,22.91; 8963400,22.87; 8964000,22.83; 8964600,22.78;
            8965200,22.76; 8965800,22.66; 8966400,22.62; 8967000,22.55; 8967600,22.5;
            8968200,22.46; 8968800,22.42; 8969400,22.38; 8970000,22.34; 8970600,22.3;
            8971200,22.25; 8971800,22.18; 8972400,22.26; 8973000,22.14; 8973600,22.09;
            8974200,22.09; 8974800,22.05; 8975400,22.02; 8976000,21.98; 8976600,21.97;
            8977200,21.96; 8977800,21.93; 8978400,21.9; 8979000,21.89; 8979600,21.85;
            8980200,21.85; 8980800,21.85; 8981400,21.81; 8982000,21.81; 8982600,21.77;
            8983200,21.754; 8983800,21.738; 8984400,21.722; 8985000,21.706; 8985600,
            21.69; 8986200,21.73; 8986800,21.69; 8987400,21.68; 8988000,21.65; 8988600,
            21.65; 8989200,21.65; 8989800,21.6; 8990400,21.6; 8991000,21.6; 8991600,
            21.6; 8992200,21.6; 8992800,21.6; 8993400,21.56; 8994000,21.56; 8994600,
            21.57; 8995200,21.52; 8995800,21.52; 8996400,21.52; 8997000,21.54; 8997600,
            21.54; 8998200,21.52; 8998800,21.48; 8999400,21.52; 9000000,21.52; 9000600,
            21.52; 9001200,21.52; 9001800,21.52; 9002400,21.52; 9003000,21.56; 9003600,
            21.56; 9004200,21.6; 9004800,21.69; 9005400,21.81; 9006000,21.89; 9006600,
            21.97; 9007200,22.14; 9007800,22.28; 9008400,22.38; 9009000,22.46; 9009600,
            22.55; 9010200,22.62; 9010800,22.66; 9011400,22.98; 9012000,23.19; 9012600,
            23.27; 9013200,23.32; 9013800,23.35; 9014400,23.4; 9015000,23.4; 9015600,
            23.44; 9016200,23.41; 9016800,23.56; 9017400,23.6; 9018000,23.68; 9018600,
            23.68; 9019200,23.68; 9019800,23.68; 9020400,23.68; 9021000,23.68; 9021600,
            23.69; 9022200,23.72; 9022800,23.6; 9023400,23.64; 9024000,23.72; 9024600,
            23.68; 9025200,23.72; 9025800,23.82; 9026400,23.88; 9027000,23.99; 9027600,
            24.01; 9028200,24.1; 9028800,24.09; 9029400,24.17; 9030000,24.21; 9030600,
            24.37; 9031200,24.33; 9031800,24.41; 9032400,24.37; 9033000,24.33; 9033600,
            24.33; 9034200,24.42; 9034800,24.36; 9035400,24.37; 9036000,24.55; 9036600,
            24.37; 9037200,24.37; 9037800,24.37; 9038400,24.41; 9039000,24.37; 9039600,
            24.37; 9040200,24.37; 9040800,24.36; 9041400,24.39; 9042000,24.37; 9042600,
            24.3; 9043200,24.21; 9043800,24.13; 9044400,24.05; 9045000,23.96; 9045600,
            23.92; 9046200,23.9; 9046800,23.84; 9047400,23.76; 9048000,23.68; 9048600,
            23.61; 9049200,23.52; 9049800,23.48; 9050400,23.36; 9051000,23.27; 9051600,
            23.23; 9052200,23.19; 9052800,23.11; 9053400,23.07; 9054000,23.03; 9054600,
            23; 9055200,22.95; 9055800,22.9; 9056400,22.83; 9057000,22.78; 9057600,22.74;
            9058200,22.7; 9058800,22.66; 9059400,22.62; 9060000,22.58; 9060600,22.54;
            9061200,22.5; 9061800,22.5; 9062400,22.47; 9063000,22.42; 9063600,22.38;
            9064200,22.34; 9064800,22.3; 9065400,22.26; 9066000,22.22; 9066600,22.18;
            9067200,22.16; 9067800,22.14; 9068400,22.13; 9069000,22.1; 9069600,22.09;
            9070200,22.06; 9070800,22.03; 9071400,22; 9072000,21.97; 9072600,21.97;
            9073200,21.98; 9073800,21.97; 9074400,21.97; 9075000,21.97; 9075600,21.93;
            9076200,21.89; 9076800,21.89; 9077400,21.89; 9078000,21.85; 9078600,21.85;
            9079200,21.85; 9079800,21.84; 9080400,21.81; 9081000,21.81; 9081600,21.81;
            9082200,21.81; 9082800,21.81; 9083400,21.8; 9084000,21.77; 9084600,21.77;
            9085200,21.77; 9085800,21.77; 9086400,21.77; 9087000,21.77; 9087600,21.77;
            9088200,21.77; 9088800,21.77; 9089400,21.77; 9090000,21.77; 9090600,21.81;
            9091200,21.84; 9091800,21.93; 9092400,21.97; 9093000,22.02; 9093600,22.09;
            9094200,22.14; 9094800,22.22; 9095400,22.34; 9096000,22.41; 9096600,22.49;
            9097200,22.58; 9097800,22.63; 9098400,22.7; 9099000,22.76; 9099600,22.79;
            9100200,22.87; 9100800,22.99; 9101400,23.03; 9102000,23.12; 9102600,23.2;
            9103200,23.23; 9103800,23.36; 9104400,23.4; 9105000,23.41; 9105600,23.44;
            9106200,23.48; 9106800,23.52; 9107400,23.52; 9108000,23.56; 9108600,23.6;
            9109200,23.72; 9109800,23.72; 9110400,23.8; 9111000,23.84; 9111600,23.84;
            9112200,23.84; 9112800,23.88; 9113400,23.88; 9114000,23.88; 9114600,23.88;
            9115200,23.92; 9115800,23.93; 9116400,23.92; 9117000,23.96; 9117600,24.01;
            9118200,24; 9118800,24.05; 9119400,24.12; 9120000,24.18; 9120600,24.12;
            9121200,24.09; 9121800,24.13; 9122400,24.13; 9123000,24.21; 9123600,24.21;
            9124200,24.21; 9124800,24.21; 9125400,24.16; 9126000,24.17; 9126600,24.21;
            9127200,24.22; 9127800,24.3; 9128400,24.33; 9129000,24.32; 9129600,24.24;
            9130200,24.21; 9130800,24.17; 9131400,24.1; 9132000,24.01; 9132600,23.92;
            9133200,23.92; 9133800,23.84; 9134400,23.76; 9135000,23.72; 9135600,23.65;
            9136200,23.58; 9136800,23.52; 9137400,23.44; 9138000,23.4; 9138600,23.36;
            9139200,23.28; 9139800,23.23; 9140400,23.19; 9141000,23.14; 9141600,23.1;
            9142200,23.06; 9142800,23.03; 9143400,22.95; 9144000,22.95; 9144600,22.92;
            9145200,22.88; 9145800,22.83; 9146400,22.78; 9147000,22.76; 9147600,22.7;
            9148200,22.7; 9148800,22.66; 9149400,22.62; 9150000,22.62; 9150600,22.59;
            9151200,22.55; 9151800,22.5; 9152400,22.5; 9153000,22.46; 9153600,22.42;
            9154200,22.42; 9154800,22.38; 9155400,22.38; 9156000,22.35; 9156600,22.32;
            9157200,22.29; 9157800,22.26; 9158400,22.23; 9159000,22.23; 9159600,22.19;
            9160200,22.18; 9160800,22.18; 9161400,22.19; 9162000,22.17; 9162600,22.16;
            9163200,22.14; 9163800,22.14; 9164400,22.14; 9165000,22.14; 9165600,22.14;
            9166200,22.14; 9166800,22.1; 9167400,22.09; 9168000,22.09; 9168600,22.09;
            9169200,22.09; 9169800,22.09; 9170400,22.09; 9171000,22.09; 9171600,22.05;
            9172200,22.05; 9172800,22.05; 9173400,22.05; 9174000,22.05; 9174600,22.01;
            9175200,22.01; 9175800,22.01; 9176400,22.01; 9177000,22.01; 9177600,22.02;
            9178200,21.97; 9178800,21.97; 9179400,21.97; 9180000,21.98; 9180600,21.98;
            9181200,21.97; 9181800,21.97; 9182400,22.01; 9183000,22.01; 9183600,22.05;
            9184200,22.05; 9184800,22.09; 9185400,22.09; 9186000,22.14; 9186600,22.14;
            9187200,22.18; 9187800,22.22; 9188400,22.26; 9189000,22.3; 9189600,22.34;
            9190200,22.42; 9190800,22.48; 9191400,22.5; 9192000,22.56; 9192600,22.58;
            9193200,22.62; 9193800,22.68; 9194400,22.74; 9195000,22.78; 9195600,22.83;
            9196200,22.87; 9196800,22.95; 9197400,23.03; 9198000,23.11; 9198600,23.19;
            9199200,23.27; 9199800,23.36; 9200400,23.48; 9201000,23.56; 9201600,23.68;
            9202200,23.76; 9202800,23.8; 9203400,23.88; 9204000,23.92; 9204600,23.96;
            9205200,24.01; 9205800,24.02; 9206400,24.05; 9207000,24.09; 9207600,24.13;
            9208200,24.14; 9208800,24.13; 9209400,24.13; 9210000,24.21; 9210600,24.21;
            9211200,24.22; 9211800,24.19; 9212400,24.13; 9213000,24.09; 9213600,24.09;
            9214200,24.05; 9214800,24.01; 9215400,23.96; 9216000,23.93; 9216600,23.88;
            9217200,23.84; 9217800,23.8; 9218400,23.75; 9219000,23.72; 9219600,23.68;
            9220200,23.64; 9220800,23.58; 9221400,23.56; 9222000,23.48; 9222600,23.44;
            9223200,23.4; 9223800,23.36; 9224400,23.32; 9225000,23.27; 9225600,23.26;
            9226200,23.23; 9226800,23.19; 9227400,23.15; 9228000,23.11; 9228600,23.1;
            9229200,23.07; 9229800,23.03; 9230400,23.03; 9231000,22.99; 9231600,22.95;
            9232200,22.94; 9232800,22.91; 9233400,22.91; 9234000,22.87; 9234600,22.87;
            9235200,22.83; 9235800,22.82; 9236400,22.78; 9237000,22.76; 9237600,22.74;
            9238200,22.66; 9238800,22.62; 9239400,22.62; 9240000,22.58; 9240600,22.53;
            9241200,22.5; 9241800,22.46; 9242400,22.444; 9243000,22.428; 9243600,22.412;
            9244200,22.396; 9244800,22.38; 9245400,22.37; 9246000,22.34; 9246600,22.3;
            9247200,22.31; 9247800,22.27; 9248400,22.26; 9249000,22.26; 9249600,22.22;
            9250200,22.22; 9250800,22.18; 9251400,22.18; 9252000,22.18; 9252600,22.16;
            9253200,22.14; 9253800,22.11; 9254400,22.09; 9255000,22.09; 9255600,22.06;
            9256200,22.05; 9256800,22.04; 9257400,22.01; 9258000,21.97; 9258600,21.97;
            9259200,21.97; 9259800,21.97; 9260400,21.94; 9261000,21.94; 9261600,21.93;
            9262200,21.93; 9262800,21.93; 9263400,21.93; 9264000,21.89; 9264600,21.89;
            9265200,21.89; 9265800,21.89; 9266400,21.89; 9267000,21.93; 9267600,21.93;
            9268200,21.94; 9268800,21.96; 9269400,21.97; 9270000,21.97; 9270600,22;
            9271200,22.01; 9271800,22.04; 9272400,22.05; 9273000,22.05; 9273600,22.05;
            9274200,22.05; 9274800,22.05; 9275400,22.05; 9276000,22.06; 9276600,22.05;
            9277200,22.06; 9277800,22.09; 9278400,22.09; 9279000,22.09; 9279600,22.1;
            9280200,22.06; 9280800,22.05; 9281400,22.05; 9282000,22.05; 9282600,22.05;
            9283200,22.05; 9283800,22.05; 9284400,22.09; 9285000,22.18; 9285600,22.22;
            9286200,22.22; 9286800,22.26; 9287400,22.26; 9288000,22.3; 9288600,22.34;
            9289200,22.47; 9289800,22.52; 9290400,22.54; 9291000,22.54; 9291600,22.5;
            9292200,22.46; 9292800,22.44; 9293400,22.42; 9294000,22.38; 9294600,22.38;
            9295200,22.38; 9295800,22.37; 9296400,22.38; 9297000,22.34; 9297600,22.3;
            9298200,22.27; 9298800,22.26; 9299400,22.22; 9300000,22.22; 9300600,22.18;
            9301200,22.19; 9301800,22.14; 9302400,22.09; 9303000,22.09; 9303600,22.05;
            9304200,22.04; 9304800,22.01; 9305400,21.97; 9306000,21.97; 9306600,21.93;
            9307200,21.89; 9307800,21.89; 9308400,21.85; 9309000,21.85; 9309600,21.81;
            9310200,21.81; 9310800,21.78; 9311400,21.77; 9312000,21.73; 9312600,21.73;
            9313200,21.73; 9313800,21.69; 9314400,21.68; 9315000,21.65; 9315600,21.65;
            9316200,21.65; 9316800,21.61; 9317400,21.6; 9318000,21.6; 9318600,21.6;
            9319200,21.56; 9319800,21.56; 9320400,21.53; 9321000,21.52; 9321600,21.52;
            9322200,21.52; 9322800,21.48; 9323400,21.48; 9324000,21.48; 9324600,21.49;
            9325200,21.48; 9325800,21.44; 9326400,21.46; 9327000,21.44; 9327600,21.44;
            9328200,21.44; 9328800,21.4; 9329400,21.3925; 9330000,21.385; 9330600,21.3775;
            9331200,21.37; 9331800,21.36; 9332400,21.36; 9333000,21.34; 9333600,21.32;
            9334200,21.32; 9334800,21.32; 9335400,21.32; 9336000,21.32; 9336600,21.28;
            9337200,21.28; 9337800,21.28; 9338400,21.28; 9339000,21.28; 9339600,21.24;
            9340200,21.24; 9340800,21.24; 9341400,21.21; 9342000,21.2; 9342600,21.2;
            9343200,21.2; 9343800,21.2; 9344400,21.17; 9345000,21.16; 9345600,21.16;
            9346200,21.16; 9346800,21.17; 9347400,21.16; 9348000,21.16; 9348600,21.16;
            9349200,21.2; 9349800,21.2; 9350400,21.44; 9351000,21.85; 9351600,22.09;
            9352200,22.29; 9352800,22.54; 9353400,22.78; 9354000,22.97; 9354600,23.12;
            9355200,23.27; 9355800,23.35; 9356400,23.4; 9357000,23.42; 9357600,23.45;
            9358200,23.48; 9358800,23.52; 9359400,23.6; 9360000,23.6; 9360600,23.65;
            9361200,23.68; 9361800,23.7; 9362400,23.72; 9363000,23.72; 9363600,23.8;
            9364200,23.8; 9364800,23.76; 9365400,23.72; 9366000,23.72; 9366600,23.69;
            9367200,23.68; 9367800,23.68; 9368400,23.8; 9369000,23.88; 9369600,23.84;
            9370200,23.96; 9370800,24.09; 9371400,24.21; 9372000,24.25; 9372600,24.29;
            9373200,24.17; 9373800,24.21; 9374400,24.21; 9375000,24.29; 9375600,24.41;
            9376200,24.46; 9376800,24.5; 9377400,24.43; 9378000,24.42; 9378600,24.45;
            9379200,24.37; 9379800,24.33; 9380400,24.37; 9381000,24.41; 9381600,24.37;
            9382200,24.41; 9382800,24.38; 9383400,24.37; 9384000,24.33; 9384600,24.25;
            9385200,24.17; 9385800,24.08; 9386400,24.01; 9387000,23.96; 9387600,23.88;
            9388200,23.88; 9388800,23.84; 9389400,23.8; 9390000,23.76; 9390600,23.72;
            9391200,23.64; 9391800,23.59; 9392400,23.55; 9393000,23.48; 9393600,23.43;
            9394200,23.4; 9394800,23.32; 9395400,23.27; 9396000,23.27; 9396600,23.23;
            9397200,23.15; 9397800,23.11; 9398400,23.04; 9399000,22.99; 9399600,22.94;
            9400200,22.87; 9400800,22.82; 9401400,22.74; 9402000,22.7; 9402600,22.62;
            9403200,22.58; 9403800,22.53; 9404400,22.5; 9405000,22.46; 9405600,22.42;
            9406200,22.38; 9406800,22.37; 9407400,22.34; 9408000,22.3; 9408600,22.26;
            9409200,22.26; 9409800,22.22; 9410400,22.22; 9411000,22.22; 9411600,22.18;
            9412200,22.14; 9412800,22.14; 9413400,22.09; 9414000,22.09; 9414600,22.09;
            9415200,22.09; 9415800,22.08; 9416400,22.07; 9417000,22.06; 9417600,22.05;
            9418200,22.05; 9418800,22.08; 9419400,22.05; 9420000,22.09; 9420600,22.09;
            9421200,22.09; 9421800,22.1; 9422400,22.09; 9423000,22.09; 9423600,22.1;
            9424200,22.08; 9424800,22.09; 9425400,22.1; 9426000,22.09; 9426600,22.09;
            9427200,22.09; 9427800,22.09; 9428400,22.09; 9429000,22.1; 9429600,22.3;
            9430200,22.5; 9430800,22.62; 9431400,22.7; 9432000,22.69; 9432600,22.58;
            9433200,22.5; 9433800,22.46; 9434400,22.42; 9435000,22.46; 9435600,22.5;
            9436200,22.62; 9436800,22.71; 9437400,22.83; 9438000,22.96; 9438600,23.03;
            9439200,23.16; 9439800,23.22; 9440400,23.24; 9441000,23.36; 9441600,23.43;
            9442200,23.44; 9442800,23.44; 9443400,23.52; 9444000,23.55; 9444600,23.57;
            9445200,23.6; 9445800,23.72; 9446400,23.82; 9447000,23.82; 9447600,23.83;
            9448200,23.84; 9448800,23.84; 9449400,23.88; 9450000,23.92; 9450600,23.96;
            9451200,23.92; 9451800,23.92; 9452400,23.9; 9453000,23.92; 9453600,23.92;
            9454200,23.88; 9454800,23.89; 9455400,23.96; 9456000,24.01; 9456600,24;
            9457200,24.09; 9457800,24.13; 9458400,24.22; 9459000,24.29; 9459600,24.21;
            9460200,24.29; 9460800,24.41; 9461400,24.42; 9462000,24.54; 9462600,24.59;
            9463200,24.58; 9463800,24.62; 9464400,24.62; 9465000,24.69; 9465600,24.66;
            9466200,24.66; 9466800,24.68; 9467400,24.68; 9468000,24.75; 9468600,24.71;
            9469200,24.74; 9469800,24.74; 9470400,24.78; 9471000,24.74; 9471600,24.7;
            9472200,24.62; 9472800,24.62; 9473400,24.62; 9474000,24.58; 9474600,24.54;
            9475200,24.41; 9475800,24.33; 9476400,24.25; 9477000,24.17; 9477600,24.08;
            9478200,24.01; 9478800,24.01; 9479400,23.96; 9480000,23.88; 9480600,23.8;
            9481200,23.72; 9481800,23.69; 9482400,23.56; 9483000,23.52; 9483600,23.44;
            9484200,23.39; 9484800,23.35; 9485400,23.27; 9486000,23.21; 9486600,23.15;
            9487200,23.11; 9487800,23.07; 9488400,23.03; 9489000,22.95; 9489600,22.91;
            9490200,22.84; 9490800,22.78; 9491400,22.74; 9492000,22.72; 9492600,22.7;
            9493200,22.66; 9493800,22.62; 9494400,22.58; 9495000,22.54; 9495600,22.54;
            9496200,22.5; 9496800,22.46; 9497400,22.42; 9498000,22.42; 9498600,22.35;
            9499200,22.34; 9499800,22.3; 9500400,22.26; 9501000,22.26; 9501600,22.242;
            9502200,22.224; 9502800,22.206; 9503400,22.188; 9504000,22.17; 9504600,22.14;
            9505200,22.14; 9505800,22.09; 9506400,22.09; 9507000,22.09; 9507600,22.05;
            9508200,22.01; 9508800,21.97; 9509400,21.97; 9510000,21.97; 9510600,21.97;
            9511200,21.97; 9511800,21.93; 9512400,21.93; 9513000,21.89; 9513600,21.89;
            9514200,21.86; 9514800,21.85; 9515400,21.81; 9516000,21.81; 9516600,21.81;
            9517200,21.81; 9517800,21.81; 9518400,21.77; 9519000,21.77; 9519600,21.77;
            9520200,21.77; 9520800,21.77; 9521400,21.78; 9522000,21.81; 9522600,21.84;
            9523200,21.9; 9523800,22.22; 9524400,22.49; 9525000,22.71; 9525600,22.83;
            9526200,22.96; 9526800,22.95; 9527400,23.07; 9528000,23.16; 9528600,23.24;
            9529200,23.24; 9529800,23.28; 9530400,23.32; 9531000,23.32; 9531600,23.36;
            9532200,23.4; 9532800,23.4; 9533400,23.44; 9534000,23.48; 9534600,23.56;
            9535200,23.6; 9535800,23.73; 9536400,23.72; 9537000,23.84; 9537600,23.84;
            9538200,23.85; 9538800,23.88; 9539400,23.92; 9540000,23.92; 9540600,23.88;
            9541200,23.88; 9541800,23.93; 9542400,24.09; 9543000,24.17; 9543600,24.17;
            9544200,24.25; 9544800,24.33; 9545400,24.37; 9546000,24.41; 9546600,24.51;
            9547200,24.54; 9547800,24.5; 9548400,24.54; 9549000,24.62; 9549600,24.63;
            9550200,24.7; 9550800,24.66; 9551400,24.62; 9552000,24.65; 9552600,24.66;
            9553200,24.7; 9553800,24.7; 9554400,24.7; 9555000,24.72; 9555600,24.74;
            9556200,24.74; 9556800,24.71; 9557400,24.7; 9558000,24.66; 9558600,24.66;
            9559200,24.62; 9559800,24.66; 9560400,24.58; 9561000,24.58; 9561600,24.5;
            9562200,24.45; 9562800,24.37; 9563400,24.35; 9564000,24.28; 9564600,24.21;
            9565200,24.14; 9565800,24.05; 9566400,24.01; 9567000,23.92; 9567600,23.84;
            9568200,23.76; 9568800,23.68; 9569400,23.61; 9570000,23.56; 9570600,23.48;
            9571200,23.44; 9571800,23.4; 9572400,23.35; 9573000,23.27; 9573600,23.23;
            9574200,23.19; 9574800,23.12; 9575400,23.11; 9576000,23.03; 9576600,23.02;
            9577200,22.95; 9577800,22.96; 9578400,22.91; 9579000,22.87; 9579600,22.83;
            9580200,22.78; 9580800,22.78; 9581400,22.74; 9582000,22.74; 9582600,22.7;
            9583200,22.68; 9583800,22.62; 9584400,22.62; 9585000,22.58; 9585600,22.54;
            9586200,22.5; 9586800,22.5; 9587400,22.46; 9588000,22.42; 9588600,22.395;
            9589200,22.37; 9589800,22.345; 9590400,22.32; 9591000,22.34; 9591600,22.34;
            9592200,22.31; 9592800,22.26; 9593400,22.26; 9594000,22.26; 9594600,22.26;
            9595200,22.22; 9595800,22.22; 9596400,22.2; 9597000,22.18; 9597600,22.18;
            9598200,22.14; 9598800,22.14; 9599400,22.09; 9600000,22.09; 9600600,22.1;
            9601200,22.09; 9601800,22.08; 9602400,22.06; 9603000,22.05; 9603600,22.05;
            9604200,22.05; 9604800,22.05; 9605400,22.05; 9606000,22.05; 9606600,22.05;
            9607200,22.09; 9607800,22.12; 9608400,22.14; 9609000,22.18; 9609600,22.3;
            9610200,22.34; 9610800,22.5; 9611400,22.62; 9612000,22.76; 9612600,22.83;
            9613200,22.91; 9613800,23.03; 9614400,23.11; 9615000,23.18; 9615600,23.19;
            9616200,23.27; 9616800,23.32; 9617400,23.38; 9618000,23.44; 9618600,23.56;
            9619200,23.52; 9619800,23.56; 9620400,23.6; 9621000,23.68; 9621600,23.68;
            9622200,23.76; 9622800,23.8; 9623400,23.89; 9624000,23.92; 9624600,23.92;
            9625200,23.96; 9625800,23.93; 9626400,23.96; 9627000,24; 9627600,24; 9628200,
            24.05; 9628800,24.13; 9629400,24.12; 9630000,24.13; 9630600,24.12; 9631200,
            24.21; 9631800,24.25; 9632400,24.33; 9633000,24.29; 9633600,24.33; 9634200,
            24.37; 9634800,24.33; 9635400,24.37; 9636000,24.37; 9636600,24.41; 9637200,
            24.57; 9637800,24.58; 9638400,24.7; 9639000,24.74; 9639600,24.75; 9640200,
            24.78; 9640800,24.78; 9641400,24.78; 9642000,24.74; 9642600,24.7; 9643200,
            24.66; 9643800,24.54; 9644400,24.53; 9645000,24.5; 9645600,24.49; 9646200,
            24.45; 9646800,24.41; 9647400,24.38; 9648000,24.33; 9648600,24.29; 9649200,
            24.25; 9649800,24.25; 9650400,24.21; 9651000,24.13; 9651600,24.05; 9652200,
            24.02; 9652800,23.92; 9653400,23.88; 9654000,23.84; 9654600,23.76; 9655200,
            23.72; 9655800,23.66; 9656400,23.6; 9657000,23.56; 9657600,23.48; 9658200,
            23.44; 9658800,23.4; 9659400,23.36; 9660000,23.35; 9660600,23.27; 9661200,
            23.24; 9661800,23.19; 9662400,23.15; 9663000,23.11; 9663600,23.11; 9664200,
            23.07; 9664800,23.03; 9665400,22.99; 9666000,22.98; 9666600,22.95; 9667200,
            22.92; 9667800,22.91; 9668400,22.87; 9669000,22.86; 9669600,22.84; 9670200,
            22.82; 9670800,22.78; 9671400,22.74; 9672000,22.7; 9672600,22.7; 9673200,
            22.69; 9673800,22.66; 9674400,22.646; 9675000,22.632; 9675600,22.618; 9676200,
            22.604; 9676800,22.59; 9677400,22.58; 9678000,22.54; 9678600,22.54; 9679200,
            22.5; 9679800,22.5; 9680400,22.5; 9681000,22.5; 9681600,22.46; 9682200,22.46;
            9682800,22.45; 9683400,22.42; 9684000,22.42; 9684600,22.42; 9685200,22.42;
            9685800,22.39; 9686400,22.38; 9687000,22.38; 9687600,22.38; 9688200,22.34;
            9688800,22.34; 9689400,22.34; 9690000,22.34; 9690600,22.31; 9691200,22.34;
            9691800,22.3; 9692400,22.3; 9693000,22.3; 9693600,22.3; 9694200,22.34; 9694800,
            22.38; 9695400,22.42; 9696000,22.54; 9696600,22.58; 9697200,22.62; 9697800,
            22.74; 9698400,22.87; 9699000,23.03; 9699600,23.07; 9700200,23.11; 9700800,
            23.23; 9701400,23.4; 9702000,23.49; 9702600,23.53; 9703200,23.56; 9703800,
            23.6; 9704400,23.68; 9705000,23.72; 9705600,23.83; 9706200,23.92; 9706800,
            23.96; 9707400,23.96; 9708000,24.06; 9708600,24.09; 9709200,24.13; 9709800,
            24.21; 9710400,24.21; 9711000,24.22; 9711600,24.25; 9712200,24.29; 9712800,
            24.33; 9713400,24.37; 9714000,24.46; 9714600,24.58; 9715200,24.55; 9715800,
            24.58; 9716400,24.61; 9717000,24.55; 9717600,24.66; 9718200,24.7; 9718800,
            24.74; 9719400,24.78; 9720000,24.74; 9720600,24.7; 9721200,24.66; 9721800,
            24.7; 9722400,24.78; 9723000,24.78; 9723600,24.82; 9724200,24.79; 9724800,
            24.84; 9725400,24.78; 9726000,24.79; 9726600,24.81; 9727200,24.82; 9727800,
            24.85; 9728400,24.81; 9729000,24.74; 9729600,24.74; 9730200,24.74; 9730800,
            24.74; 9731400,24.7; 9732000,24.66; 9732600,24.6; 9733200,24.54; 9733800,
            24.48; 9734400,24.45; 9735000,24.41; 9735600,24.37; 9736200,24.33; 9736800,
            24.29; 9737400,24.26; 9738000,24.21; 9738600,24.13; 9739200,24.09; 9739800,
            24.05; 9740400,24.01; 9741000,23.92; 9741600,23.88; 9742200,23.84; 9742800,
            23.79; 9743400,23.72; 9744000,23.68; 9744600,23.65; 9745200,23.56; 9745800,
            23.52; 9746400,23.48; 9747000,23.44; 9747600,23.4; 9748200,23.36; 9748800,
            23.29; 9749400,23.27; 9750000,23.21; 9750600,23.19; 9751200,23.12; 9751800,
            23.11; 9752400,23.07; 9753000,23.03; 9753600,23.03; 9754200,22.99; 9754800,
            22.95; 9755400,22.91; 9756000,22.91; 9756600,22.87; 9757200,22.86; 9757800,
            22.83; 9758400,22.78; 9759000,22.78; 9759600,22.78; 9760200,22.74; 9760800,
            22.716; 9761400,22.692; 9762000,22.668; 9762600,22.644; 9763200,22.62; 9763800,
            22.62; 9764400,22.62; 9765000,22.62; 9765600,22.58; 9766200,22.58; 9766800,
            22.56; 9767400,22.54; 9768000,22.54; 9768600,22.5; 9769200,22.5; 9769800,
            22.5; 9770400,22.5; 9771000,22.46; 9771600,22.47; 9772200,22.46; 9772800,
            22.42; 9773400,22.42; 9774000,22.42; 9774600,22.42; 9775200,22.4; 9775800,
            22.38; 9776400,22.38; 9777000,22.38; 9777600,22.37; 9778200,22.34; 9778800,
            22.34; 9779400,22.34; 9780000,22.33; 9780600,22.34; 9781200,22.3; 9781800,
            22.3; 9782400,22.3; 9783000,22.3; 9783600,22.3; 9784200,22.3; 9784800,22.3;
            9785400,22.3; 9786000,22.33; 9786600,22.3; 9787200,22.3; 9787800,22.34;
            9788400,22.34; 9789000,22.34; 9789600,22.34; 9790200,22.34; 9790800,22.38;
            9791400,22.34; 9792000,22.34; 9792600,22.34; 9793200,22.34; 9793800,22.34;
            9794400,22.34; 9795000,22.38; 9795600,22.38; 9796200,22.38; 9796800,22.38;
            9797400,22.38; 9798000,22.38; 9798600,22.38; 9799200,22.39; 9799800,22.39;
            9800400,22.38; 9801000,22.4; 9801600,22.56; 9802200,22.41; 9802800,22.38;
            9803400,22.38; 9804000,22.38; 9804600,22.38; 9805200,22.37; 9805800,22.38;
            9806400,22.38; 9807000,22.38; 9807600,22.38; 9808200,22.38; 9808800,22.38;
            9809400,22.38; 9810000,22.38; 9810600,22.38; 9811200,22.38; 9811800,22.38;
            9812400,22.37; 9813000,22.37; 9813600,22.34; 9814200,22.34; 9814800,22.31;
            9815400,22.3; 9816000,22.3; 9816600,22.3; 9817200,22.3; 9817800,22.26; 9818400,
            22.26; 9819000,22.26; 9819600,22.25; 9820200,22.22; 9820800,22.22; 9821400,
            22.2; 9822000,22.18; 9822600,22.17; 9823200,22.14; 9823800,22.14; 9824400,
            22.13; 9825000,22.1; 9825600,22.09; 9826200,22.06; 9826800,22.05; 9827400,
            22.05; 9828000,22.01; 9828600,22.01; 9829200,22; 9829800,21.97; 9830400,
            21.97; 9831000,21.97; 9831600,21.97; 9832200,21.93; 9832800,21.93; 9833400,
            21.93; 9834000,21.89; 9834600,21.89; 9835200,21.86; 9835800,21.85; 9836400,
            21.85; 9837000,21.85; 9837600,21.84; 9838200,21.81; 9838800,21.81; 9839400,
            21.81; 9840000,21.77; 9840600,21.77; 9841200,21.77; 9841800,21.75; 9842400,
            21.74; 9843000,21.73; 9843600,21.7; 9844200,21.69; 9844800,21.7; 9845400,
            21.69; 9846000,21.65; 9846600,21.67; 9847200,21.662; 9847800,21.654; 9848400,
            21.646; 9849000,21.638; 9849600,21.63; 9850200,21.6; 9850800,21.6; 9851400,
            21.6; 9852000,21.62; 9852600,21.56; 9853200,21.56; 9853800,21.55; 9854400,
            21.57; 9855000,21.52; 9855600,21.52; 9856200,21.52; 9856800,21.52; 9857400,
            21.52; 9858000,21.48; 9858600,21.48; 9859200,21.48; 9859800,21.48; 9860400,
            21.45; 9861000,21.46; 9861600,21.44; 9862200,21.44; 9862800,21.44; 9863400,
            21.44; 9864000,21.4; 9864600,21.4; 9865200,21.41; 9865800,21.4; 9866400,
            21.36; 9867000,21.39; 9867600,21.36; 9868200,21.36; 9868800,21.36; 9869400,
            21.36; 9870000,21.36; 9870600,21.37; 9871200,21.37; 9871800,21.39; 9872400,
            21.4; 9873000,21.4; 9873600,21.4; 9874200,21.44; 9874800,21.48; 9875400,
            21.48; 9876000,21.52; 9876600,21.52; 9877200,21.56; 9877800,21.56; 9878400,
            21.6; 9879000,21.6; 9879600,21.6; 9880200,21.6; 9880800,21.65; 9881400,21.7;
            9882000,21.78; 9882600,21.81; 9883200,21.81; 9883800,21.81; 9884400,21.81;
            9885000,21.77; 9885600,21.77; 9886200,21.81; 9886800,21.81; 9887400,21.85;
            9888000,21.92; 9888600,21.97; 9889200,22.05; 9889800,22.13; 9890400,22.14;
            9891000,22.09; 9891600,22.05; 9892200,21.97; 9892800,21.97; 9893400,21.97;
            9894000,21.97; 9894600,21.97; 9895200,21.93; 9895800,21.89; 9896400,21.85;
            9897000,21.84; 9897600,21.82; 9898200,21.81; 9898800,21.81; 9899400,21.77;
            9900000,21.77; 9900600,21.78; 9901200,21.8; 9901800,21.81; 9902400,21.81;
            9903000,21.81; 9903600,21.77; 9904200,21.77; 9904800,21.78; 9905400,21.77;
            9906000,21.77; 9906600,21.74; 9907200,21.74; 9907800,21.74; 9908400,21.69;
            9909000,21.68; 9909600,21.65; 9910200,21.62; 9910800,21.62; 9911400,21.59;
            9912000,21.56; 9912600,21.52; 9913200,21.48; 9913800,21.48; 9914400,21.46;
            9915000,21.44; 9915600,21.4; 9916200,21.4; 9916800,21.36; 9917400,21.35;
            9918000,21.32; 9918600,21.32; 9919200,21.33; 9919800,21.32; 9920400,21.28;
            9921000,21.28; 9921600,21.28; 9922200,21.24; 9922800,21.24; 9923400,21.2;
            9924000,21.2; 9924600,21.2; 9925200,21.16; 9925800,21.16; 9926400,21.16;
            9927000,21.17; 9927600,21.13; 9928200,21.13; 9928800,21.12; 9929400,21.12;
            9930000,21.08; 9930600,21.08; 9931200,21.08; 9931800,21.08; 9932400,21.08;
            9933000,21.04; 9933600,21.04; 9934200,21.03; 9934800,21.02; 9935400,21.01;
            9936000,21; 9936600,21; 9937200,20.96; 9937800,20.96; 9938400,20.96; 9939000,
            20.96; 9939600,20.96; 9940200,20.95; 9940800,20.94; 9941400,20.92; 9942000,
            20.91; 9942600,20.91; 9943200,20.91; 9943800,20.87; 9944400,20.87; 9945000,
            20.87; 9945600,20.87; 9946200,20.83; 9946800,20.83; 9947400,20.83; 9948000,
            20.83; 9948600,20.83; 9949200,20.83; 9949800,20.83; 9950400,20.83; 9951000,
            20.83; 9951600,20.84; 9952200,20.83; 9952800,20.83; 9953400,20.83; 9954000,
            20.84; 9954600,20.91; 9955200,21; 9955800,21.12; 9956400,21.25; 9957000,
            21.32; 9957600,21.44; 9958200,21.6; 9958800,21.69; 9959400,21.78; 9960000,
            22.05; 9960600,22.34; 9961200,22.46; 9961800,22.62; 9962400,22.78; 9963000,
            22.91; 9963600,23.04; 9964200,23.15; 9964800,23.17; 9965400,23.2; 9966000,
            23.19; 9966600,23.19; 9967200,23.19; 9967800,23.27; 9968400,23.4; 9969000,
            23.38; 9969600,23.36; 9970200,23.38; 9970800,23.36; 9971400,23.37; 9972000,
            23.36; 9972600,23.43; 9973200,23.44; 9973800,23.52; 9974400,23.58; 9975000,
            23.56; 9975600,23.6; 9976200,23.68; 9976800,23.68; 9977400,23.72; 9978000,
            23.68; 9978600,23.72; 9979200,23.73; 9979800,23.78; 9980400,23.77; 9981000,
            23.68; 9981600,23.6; 9982200,23.56; 9982800,23.56; 9983400,23.56; 9984000,
            23.64; 9984600,23.76; 9985200,23.75; 9985800,23.84; 9986400,23.84; 9987000,
            23.84; 9987600,23.85; 9988200,23.8; 9988800,23.76; 9989400,23.72; 9990000,
            23.68; 9990600,23.64; 9991200,23.56; 9991800,23.6; 9992400,23.68; 9993000,
            23.68; 9993600,23.68; 9994200,23.55; 9994800,23.48; 9995400,23.44; 9996000,
            23.32; 9996600,23.27; 9997200,23.19; 9997800,23.11; 9998400,23.03; 9999000,
            22.95; 9999600,22.83; 10000200,22.78; 10000800,22.78; 10001400,22.74; 10002000,
            22.62; 10002600,22.58; 10003200,22.5; 10003800,22.42; 10004400,22.38; 10005000,
            22.31; 10005600,22.26; 10006200,22.22; 10006800,22.14; 10007400,22.09; 10008000,
            22.06; 10008600,22.02; 10009200,21.97; 10009800,21.88; 10010400,21.85; 10011000,
            21.81; 10011600,21.8; 10012200,21.78; 10012800,21.73; 10013400,21.73; 10014000,
            21.7; 10014600,21.66; 10015200,21.64; 10015800,21.6; 10016400,21.57; 10017000,
            21.56; 10017600,21.52; 10018200,21.52; 10018800,21.5; 10019400,21.48; 10020000,
            21.48; 10020600,21.46; 10021200,21.44; 10021800,21.42; 10022400,21.4; 10023000,
            21.36; 10023600,21.36; 10024200,21.32; 10024800,21.32; 10025400,21.32; 10026000,
            21.32; 10026600,21.32; 10027200,21.32; 10027800,21.33; 10028400,21.28; 10029000,
            21.28; 10029600,21.28; 10030200,21.28; 10030800,21.25; 10031400,21.24; 10032000,
            21.24; 10032600,21.2; 10033200,21.23; 10033800,21.21; 10034400,21.2; 10035000,
            21.21; 10035600,21.23; 10036200,21.24; 10036800,21.28; 10037400,21.32; 10038000,
            21.3; 10038600,21.28; 10039200,21.28; 10039800,21.24; 10040400,21.28; 10041000,
            21.28; 10041600,21.33; 10042200,21.48; 10042800,21.81; 10043400,21.93; 10044000,
            22.09; 10044600,22.22; 10045200,22.29; 10045800,22.39; 10046400,22.42; 10047000,
            22.46; 10047600,22.47; 10048200,22.5; 10048800,22.5; 10049400,22.5; 10050000,
            22.59; 10050600,22.66; 10051200,22.67; 10051800,22.7; 10052400,22.78; 10053000,
            22.83; 10053600,22.94; 10054200,22.96; 10054800,22.95; 10055400,22.96; 10056000,
            23; 10056600,22.99; 10057200,23.03; 10057800,22.95; 10058400,22.95; 10059000,
            22.91; 10059600,22.87; 10060200,22.95; 10060800,23.03; 10061400,23.07; 10062000,
            23.15; 10062600,23.18; 10063200,23.23; 10063800,23.2; 10064400,23.24; 10065000,
            23.27; 10065600,23.34; 10066200,23.36; 10066800,23.36; 10067400,23.4; 10068000,
            23.4; 10068600,23.4; 10069200,23.44; 10069800,23.44; 10070400,23.47; 10071000,
            23.51; 10071600,23.48; 10072200,23.52; 10072800,23.52; 10073400,23.52; 10074000,
            23.6; 10074600,23.56; 10075200,23.56; 10075800,23.56; 10076400,23.56; 10077000,
            23.56; 10077600,23.54; 10078200,23.55; 10078800,23.45; 10079400,23.43; 10080000,
            23.48; 10080600,23.47; 10081200,23.46; 10081800,23.44; 10082400,23.4; 10083000,
            23.36; 10083600,23.24; 10084200,23.16; 10084800,23.1; 10085400,23.03; 10086000,
            22.98; 10086600,22.92; 10087200,22.83; 10087800,22.78; 10088400,22.7; 10089000,
            22.62; 10089600,22.58; 10090200,22.51; 10090800,22.43; 10091400,22.38; 10092000,
            22.3; 10092600,22.27; 10093200,22.18; 10093800,22.14; 10094400,22.09; 10095000,
            22.05; 10095600,21.98; 10096200,21.96; 10096800,21.9; 10097400,21.85; 10098000,
            21.81; 10098600,21.77; 10099200,21.73; 10099800,21.69; 10100400,21.65; 10101000,
            21.6; 10101600,21.6; 10102200,21.55; 10102800,21.52; 10103400,21.48; 10104000,
            21.44; 10104600,21.41; 10105200,21.4; 10105800,21.36; 10106400,21.342; 10107000,
            21.324; 10107600,21.306; 10108200,21.288; 10108800,21.27; 10109400,21.24;
            10110000,21.2; 10110600,21.16; 10111200,21.16; 10111800,21.16; 10112400,
            21.12; 10113000,21.12; 10113600,21.08; 10114200,21.08; 10114800,21.04; 10115400,
            21.04; 10116000,21.02; 10116600,21; 10117200,20.99; 10117800,20.96; 10118400,
            20.96; 10119000,20.95; 10119600,20.91; 10120200,20.91; 10120800,20.91; 10121400,
            20.87; 10122000,20.87; 10122600,20.87; 10123200,20.85; 10123800,20.83; 10124400,
            20.84; 10125000,20.83; 10125600,20.83; 10126200,20.87; 10126800,20.87; 10127400,
            20.96; 10128000,20.99; 10128600,21.12; 10129200,21.52; 10129800,21.85; 10130400,
            22.05; 10131000,22.09; 10131600,22.15; 10132200,22.18; 10132800,22.26; 10133400,
            22.27; 10134000,22.37; 10134600,22.46; 10135200,22.46; 10135800,22.5; 10136400,
            22.54; 10137000,22.62; 10137600,22.62; 10138200,22.7; 10138800,22.79; 10139400,
            22.86; 10140000,22.92; 10140600,22.98; 10141200,23.11; 10141800,23.23; 10142400,
            23.25; 10143000,23.27; 10143600,23.19; 10144200,23.21; 10144800,23.19; 10145400,
            23.19; 10146000,23.23; 10146600,23.24; 10147200,23.36; 10147800,23.4; 10148400,
            23.56; 10149000,23.61; 10149600,23.6; 10150200,23.64; 10150800,23.72; 10151400,
            23.8; 10152000,23.85; 10152600,23.92; 10153200,23.93; 10153800,23.97; 10154400,
            23.92; 10155000,23.96; 10155600,24.05; 10156200,24.06; 10156800,24.01; 10157400,
            23.96; 10158000,24.01; 10158600,24.01; 10159200,24.01; 10159800,24.04; 10160400,
            24.04; 10161000,24.04; 10161600,24.02; 10162200,23.92; 10162800,23.84; 10163400,
            23.92; 10164000,23.88; 10164600,23.94; 10165200,23.92; 10165800,23.96; 10166400,
            23.92; 10167000,23.8; 10167600,23.8; 10168200,23.76; 10168800,23.69; 10169400,
            23.6; 10170000,23.48; 10170600,23.4; 10171200,23.32; 10171800,23.23; 10172400,
            23.15; 10173000,23.07; 10173600,23.03; 10174200,22.95; 10174800,22.91; 10175400,
            22.83; 10176000,22.76; 10176600,22.66; 10177200,22.62; 10177800,22.54; 10178400,
            22.5; 10179000,22.43; 10179600,22.38; 10180200,22.3; 10180800,22.26; 10181400,
            22.22; 10182000,22.17; 10182600,22.14; 10183200,22.09; 10183800,22.05; 10184400,
            22.02; 10185000,21.97; 10185600,21.93; 10186200,21.89; 10186800,21.85; 10187400,
            21.81; 10188000,21.78; 10188600,21.73; 10189200,21.73; 10189800,21.69; 10190400,
            21.65; 10191000,21.61; 10191600,21.62; 10192200,21.56; 10192800,21.536;
            10193400,21.512; 10194000,21.488; 10194600,21.464; 10195200,21.44; 10195800,
            21.44; 10196400,21.44; 10197000,21.41; 10197600,21.4; 10198200,21.37; 10198800,
            21.36; 10199400,21.32; 10200000,21.32; 10200600,21.32; 10201200,21.29; 10201800,
            21.28; 10202400,21.28; 10203000,21.24; 10203600,21.24; 10204200,21.24; 10204800,
            21.2; 10205400,21.2; 10206000,21.2; 10206600,21.16; 10207200,21.16; 10207800,
            21.16; 10208400,21.16; 10209000,21.16; 10209600,21.16; 10210200,21.16; 10210800,
            21.16; 10211400,21.16; 10212000,21.19; 10212600,21.18; 10213200,21.2; 10213800,
            21.24; 10214400,21.32; 10215000,21.65; 10215600,21.93; 10216200,22.14; 10216800,
            22.26; 10217400,22.42; 10218000,22.46; 10218600,22.5; 10219200,22.58; 10219800,
            22.64; 10220400,22.74; 10221000,22.78; 10221600,22.83; 10222200,22.92; 10222800,
            22.91; 10223400,22.94; 10224000,22.98; 10224600,23.03; 10225200,23.11; 10225800,
            23.18; 10226400,23.19; 10227000,23.25; 10227600,23.35; 10228200,23.32; 10228800,
            23.35; 10229400,23.32; 10230000,23.32; 10230600,23.3; 10231200,23.27; 10231800,
            23.27; 10232400,23.31; 10233000,23.36; 10233600,23.43; 10234200,23.44; 10234800,
            23.44; 10235400,23.56; 10236000,23.64; 10236600,23.68; 10237200,23.68; 10237800,
            23.6; 10238400,23.61; 10239000,23.68; 10239600,23.68; 10240200,23.76; 10240800,
            23.72; 10241400,23.8; 10242000,23.82; 10242600,23.8; 10243200,23.84; 10243800,
            23.72; 10244400,23.71; 10245000,23.64; 10245600,23.6; 10246200,23.59; 10246800,
            23.6; 10247400,23.56; 10248000,23.52; 10248600,23.49; 10249200,23.48; 10249800,
            23.56; 10250400,23.56; 10251000,23.6; 10251600,23.6; 10252200,23.57; 10252800,
            23.6; 10253400,23.52; 10254000,23.44; 10254600,23.36; 10255200,23.27; 10255800,
            23.23; 10256400,23.16; 10257000,23.07; 10257600,22.99; 10258200,22.91; 10258800,
            22.84; 10259400,22.74; 10260000,22.66; 10260600,22.58; 10261200,22.5; 10261800,
            22.43; 10262400,22.38; 10263000,22.3; 10263600,22.22; 10264200,22.14; 10264800,
            22.09; 10265400,22.01; 10266000,21.97; 10266600,21.93; 10267200,21.89; 10267800,
            21.81; 10268400,21.77; 10269000,21.69; 10269600,21.65; 10270200,21.6; 10270800,
            21.52; 10271400,21.48; 10272000,21.44; 10272600,21.4; 10273200,21.35; 10273800,
            21.32; 10274400,21.24; 10275000,21.2; 10275600,21.12; 10276200,21.11; 10276800,
            21.04; 10277400,21.01; 10278000,20.99; 10278600,20.96; 10279200,20.91; 10279800,
            20.86; 10280400,20.81; 10281000,20.76; 10281600,20.71; 10282200,20.71; 10282800,
            20.68; 10283400,20.63; 10284000,20.63; 10284600,20.58; 10285200,20.55; 10285800,
            20.51; 10286400,20.51; 10287000,20.46; 10287600,20.42; 10288200,20.44; 10288800,
            20.38; 10289400,20.38; 10290000,20.34; 10290600,20.34; 10291200,20.3; 10291800,
            20.3; 10292400,20.3; 10293000,20.3; 10293600,20.3; 10294200,20.3; 10294800,
            20.26; 10295400,20.22; 10296000,20.18; 10296600,20.22; 10297200,20.3; 10297800,
            20.3; 10298400,20.3; 10299000,20.3; 10299600,20.38; 10300200,20.46; 10300800,
            20.8; 10301400,21.16; 10302000,21.44; 10302600,21.69; 10303200,21.93; 10303800,
            22.09; 10304400,22.29; 10305000,22.44; 10305600,22.62; 10306200,22.84; 10306800,
            22.92; 10307400,22.95; 10308000,23.07; 10308600,23.12; 10309200,23.23; 10309800,
            23.3; 10310400,23.31; 10311000,23.32; 10311600,23.36; 10312200,23.4; 10312800,
            23.4; 10313400,23.45; 10314000,23.57; 10314600,23.52; 10315200,23.56; 10315800,
            23.52; 10316400,23.52; 10317000,23.48; 10317600,23.48; 10318200,23.48; 10318800,
            23.53; 10319400,23.56; 10320000,23.72; 10320600,23.84; 10321200,23.89; 10321800,
            23.92; 10322400,23.96; 10323000,24.01; 10323600,24.06; 10324200,24.09; 10324800,
            24.09; 10325400,24.18; 10326000,24.21; 10326600,24.21; 10327200,24.2; 10327800,
            24.29; 10328400,24.33; 10329000,24.3; 10329600,24.22; 10330200,24.17; 10330800,
            24.13; 10331400,24.12; 10332000,24.09; 10332600,24.1; 10333200,24.05; 10333800,
            24.05; 10334400,24.04; 10335000,24.01; 10335600,24.04; 10336200,24.02; 10336800,
            24.04; 10337400,24.04; 10338000,24.05; 10338600,24.05; 10339200,24.09; 10339800,
            24.01; 10340400,23.96; 10341000,23.92; 10341600,23.84; 10342200,23.8; 10342800,
            23.68; 10343400,23.6; 10344000,23.52; 10344600,23.44; 10345200,23.4; 10345800,
            23.32; 10346400,23.27; 10347000,23.17; 10347600,23.11; 10348200,23.03; 10348800,
            22.99; 10349400,22.91; 10350000,22.87; 10350600,22.79; 10351200,22.74; 10351800,
            22.7; 10352400,22.65; 10353000,22.62; 10353600,22.54; 10354200,22.5; 10354800,
            22.46; 10355400,22.4; 10356000,22.34; 10356600,22.3; 10357200,22.26; 10357800,
            22.22; 10358400,22.2; 10359000,22.18; 10359600,22.13; 10360200,22.1; 10360800,
            22.06; 10361400,22.03; 10362000,21.99; 10362600,21.97; 10363200,21.93; 10363800,
            21.9; 10364400,21.87; 10365000,21.85; 10365600,21.81; 10366200,21.8; 10366800,
            21.77; 10367400,21.73; 10368000,21.73; 10368600,21.69; 10369200,21.65; 10369800,
            21.64; 10370400,21.6; 10371000,21.6; 10371600,21.56; 10372200,21.52; 10372800,
            21.52; 10373400,21.48; 10374000,21.48; 10374600,21.44; 10375200,21.44; 10375800,
            21.4; 10376400,21.4; 10377000,21.36; 10377600,21.36; 10378200,21.32; 10378800,
            21.32; 10379400,21.28; 10380000,21.28; 10380600,21.26; 10381200,21.24; 10381800,
            21.24; 10382400,21.2; 10383000,21.2; 10383600,21.2; 10384200,21.2; 10384800,
            21.2; 10385400,21.2; 10386000,21.2; 10386600,21.2; 10387200,21.2; 10387800,
            21.21; 10388400,21.24; 10389000,21.25; 10389600,21.28; 10390200,21.29; 10390800,
            21.32; 10391400,21.32; 10392000,21.36; 10392600,21.36; 10393200,21.4; 10393800,
            21.4; 10394400,21.44; 10395000,21.47; 10395600,21.72; 10396200,21.48; 10396800,
            21.52; 10397400,21.56; 10398000,21.62; 10398600,21.66; 10399200,21.65; 10399800,
            21.72; 10400400,21.76; 10401000,21.77; 10401600,21.82; 10402200,21.82; 10402800,
            21.81; 10403400,21.81; 10404000,21.81; 10404600,21.82; 10405200,21.81; 10405800,
            21.81; 10406400,21.81; 10407000,21.81; 10407600,21.8; 10408200,21.8; 10408800,
            21.77; 10409400,21.77; 10410000,21.77; 10410600,21.75; 10411200,21.73; 10411800,
            21.72; 10412400,21.69; 10413000,21.69; 10413600,21.68; 10414200,21.65; 10414800,
            21.65; 10415400,21.65; 10416000,21.64; 10416600,21.6; 10417200,21.6; 10417800,
            21.6; 10418400,21.62; 10419000,21.6; 10419600,21.6; 10420200,21.56; 10420800,
            21.56; 10421400,21.52; 10422000,21.52; 10422600,21.49; 10423200,21.48; 10423800,
            21.48; 10424400,21.45; 10425000,21.44; 10425600,21.44; 10426200,21.4; 10426800,
            21.4; 10427400,21.4; 10428000,21.36; 10428600,21.35; 10429200,21.32; 10429800,
            21.32; 10430400,21.32; 10431000,21.28; 10431600,21.28; 10432200,21.24; 10432800,
            21.25; 10433400,21.22; 10434000,21.2; 10434600,21.2; 10435200,21.16; 10435800,
            21.16; 10436400,21.16; 10437000,21.16; 10437600,21.12; 10438200,21.13; 10438800,
            21.12; 10439400,21.1; 10440000,21.08; 10440600,21.08; 10441200,21.08; 10441800,
            21.05; 10442400,21.04; 10443000,21.04; 10443600,21; 10444200,21; 10444800,
            21; 10445400,20.97; 10446000,20.96; 10446600,20.96; 10447200,20.96; 10447800,
            20.96; 10448400,20.91; 10449000,20.93; 10449600,20.91; 10450200,20.91; 10450800,
            20.91; 10451400,20.91; 10452000,20.91; 10452600,20.91; 10453200,20.91; 10453800,
            20.91])
        annotation (Placement(transformation(extent={{-36,60},{-16,80}})));
      Modelica.Blocks.Math.UnitConversions.From_degC from_degC
        annotation (Placement(transformation(extent={{-2,60},{18,80}})));
      Buildings.Fluid.Sources.Boundary_pT AirVolume(
        use_T_in=true,
        nPorts=2,
        redeclare package Medium = MediumA)
        annotation (Placement(transformation(extent={{36,56},{56,76}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                                weaDat(
        filNam="modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
        "Weather data reader"
        annotation (Placement(transformation(extent={{-34,30},{-14,50}})));
      Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium =
            MediumA)
        annotation (Placement(transformation(extent={{68,14},{88,34}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
            MediumA, m_flow_nominal=75*1.225*1/3600) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=180,
            origin={74,64})));
    equation
      connect(TRoom.y, from_degC.u)
        annotation (Line(points={{-15,70},{-4,70}},  color={0,0,127}));
      connect(from_degC.y, doubleFlux_FreeCooling_MechanicalCooling.TRoo[1])
        annotation (Line(points={{19,70},{20,70},{20,42.5714},{19.88,42.5714}},
            color={0,0,127}));
      connect(from_degC.y, AirVolume.T_in)
        annotation (Line(points={{19,70},{19,70},{34,70}}, color={0,0,127}));
      connect(AirVolume.ports[1:1], doubleFlux_FreeCooling_MechanicalCooling.port_a1)
        annotation (Line(points={{56,68},{96,68},{96,16.2857},{62,16.2857}},
            color={0,127,255}));
      connect(weaDat.weaBus, doubleFlux_FreeCooling_MechanicalCooling.weaBus)
        annotation (Line(
          points={{-14,40},{-14,40},{-14,36.8571},{14.68,36.8571}},
          color={255,204,51},
          thickness=0.5));
      connect(doubleFlux_FreeCooling_MechanicalCooling.port_b1[1], senMasFlo.port_a)
        annotation (Line(points={{62,24.2857},{65,24.2857},{65,24},{68,24}}, color={
              0,127,255}));
      connect(senMasFlo.port_b, senTem.port_a) annotation (Line(points={{88,24},{90,
              24},{90,64},{84,64}}, color={0,127,255}));
      connect(senTem.port_b, AirVolume.ports[2])
        annotation (Line(points={{64,64},{56,64}}, color={0,127,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-40,
                -20},{100,100}})),                                   Diagram(
            coordinateSystem(preserveAspectRatio=false, extent={{-40,-20},{100,
                100}})),                                 Documentation(info="<html>
<p>
TSimulation maximum = 10453800
</p>

</html>"));
    end DoubleFlow;

    model VAVReheat
      extends Modelica.Icons.Example;
        replaceable package MediumA =
          Buildings.Media.Air(T_default=293.15);
      package MediumW = Buildings.Media.Water "Medium model for water";
      CTA_VAVReheat_Elec cTA_VAVReheat(
        nReheat=1,
        VRoom={75},
        m_flow_room={75*1.225*1/3600},
        m_flow_nominal=1,
        THeaOn=298.15,
        THeaOff=292.15)
        annotation (Placement(transformation(extent={{-58,-16},{0,18}})));
      Modelica.Blocks.Sources.TimeTable TRoom(table=[0,16.76; 600,16.76; 1200,16.76;
            1800,16.76; 2400,16.76; 3000,16.76; 3600,16.73; 4200,16.74; 4800,16.72;
            5400,16.72; 6000,16.72; 6600,16.72; 7200,16.72; 7800,16.69; 8400,16.69;
            9000,16.68; 9600,16.68; 10200,16.67; 10800,16.65; 11400,16.64; 12000,16.64;
            12600,16.64; 13200,16.64; 13800,16.64; 14400,16.64; 15000,16.64; 15600,16.63;
            16200,16.6; 16800,16.6; 17400,16.6; 18000,16.6; 18600,16.6; 19200,16.6;
            19800,16.6; 20400,16.6; 21000,16.6; 21600,16.6; 22200,16.6; 22800,16.59;
            23400,16.56; 24000,16.56; 24600,16.56; 25200,16.57; 25800,16.56; 26400,16.57;
            27000,16.55; 27600,16.52; 28200,16.53; 28800,16.52; 29400,16.53; 30000,16.52;
            30600,16.53; 31200,16.52; 31800,16.52; 32400,16.52; 33000,16.52; 33600,16.53;
            34200,16.52; 34800,16.58; 35400,16.53; 36000,16.56; 36600,16.56; 37200,16.58;
            37800,16.58; 38400,16.6; 39000,16.6; 39600,16.6; 40200,16.6; 40800,16.64;
            41400,16.64; 42000,16.64; 42600,16.68; 43200,16.72; 43800,16.76; 44400,16.84;
            45000,16.88; 45600,16.96; 46200,17.01; 46800,17.32; 47400,17.05; 48000,17.06;
            48600,17.09; 49200,17.13; 49800,17.17; 50400,17.17; 51000,17.17; 51600,17.2;
            52200,17.21; 52800,17.25; 53400,17.21; 54000,17.21; 54600,17.21; 55200,17.17;
            55800,17.13; 56400,17.09; 57000,17.08; 57600,17.05; 58200,17.01; 58800,16.97;
            59400,16.95; 60000,16.92; 60600,16.92; 61200,16.9; 61800,16.85; 62400,16.86;
            63000,16.8; 63600,16.8; 64200,16.82; 64800,16.76; 65400,16.76; 66000,16.72;
            66600,16.72; 67200,16.71; 67800,16.69; 68400,16.69; 69000,16.64; 69600,16.64;
            70200,16.6; 70800,16.6; 71400,16.6; 72000,16.56; 72600,16.56; 73200,16.52;
            73800,16.52; 74400,16.48; 75000,16.48; 75600,16.44; 76200,16.44; 76800,16.44;
            77400,16.41; 78000,16.44; 78600,16.4; 79200,16.36; 79800,16.36; 80400,16.36;
            81000,16.32; 81600,16.32; 82200,16.32; 82800,16.32; 83400,16.3; 84000,16.28;
            84600,16.28; 85200,16.27; 85800,16.24; 86400,16.24; 87000,16.24; 87600,16.24;
            88200,16.24; 88800,16.22; 89400,16.19; 90000,16.19; 90600,16.19; 91200,16.18;
            91800,16.16; 92400,16.15; 93000,16.15; 93600,16.15; 94200,16.15; 94800,16.11;
            95400,16.11; 96000,16.11; 96600,16.11; 97200,16.11; 97800,16.11; 98400,16.11;
            99000,16.11; 99600,16.11; 100200,16.07; 100800,16.07; 101400,16.07; 102000,
            16.07; 102600,16.07; 103200,16.07; 103800,16.06; 104400,16.03; 105000,16.03;
            105600,16.03; 106200,16.03; 106800,16.03; 107400,16.02; 108000,15.99; 108600,
            15.99; 109200,15.99; 109800,15.99; 110400,15.99; 111000,15.99; 111600,15.99;
            112200,15.95; 112800,15.95; 113400,15.95; 114000,15.95; 114600,15.95; 115200,
            15.95; 115800,15.95; 116400,15.95; 117000,15.95; 117600,15.94; 118200,15.96;
            118800,15.91; 119400,15.92; 120000,15.91; 120600,15.91; 121200,15.91; 121800,
            15.91; 122400,15.91; 123000,16.21; 123600,15.91; 124200,15.91; 124800,15.92;
            125400,15.94; 126000,15.94; 126600,15.95; 127200,15.95; 127800,15.95; 128400,
            15.95; 129000,15.95; 129600,15.95; 130200,15.95; 130800,15.95; 131400,15.95;
            132000,15.95; 132600,15.95; 133200,15.95; 133800,15.96; 134400,15.96; 135000,
            15.96; 135600,15.95; 136200,15.95; 136800,15.94; 137400,15.91; 138000,15.92;
            138600,15.91; 139200,15.91; 139800,15.91; 140400,15.91; 141000,15.91; 141600,
            15.9; 142200,15.87; 142800,15.87; 143400,15.87; 144000,15.87; 144600,15.87;
            145200,15.87; 145800,15.83; 146400,15.83; 147000,15.81; 147600,15.79; 148200,
            15.79; 148800,15.79; 149400,15.79; 150000,15.78; 150600,15.75; 151200,15.75;
            151800,15.76; 152400,15.74; 153000,15.74; 153600,15.74; 154200,15.74; 154800,
            15.74; 155400,15.74; 156000,15.74; 156600,15.74; 157200,15.74; 157800,15.71;
            158400,15.7; 159000,15.72; 159600,15.7; 160200,15.7; 160800,15.72; 161400,
            15.66; 162000,15.66; 162600,15.66; 163200,15.66; 163800,15.66; 164400,15.66;
            165000,15.66; 165600,15.63; 166200,15.62; 166800,15.62; 167400,15.62; 168000,
            15.62; 168600,15.62; 169200,15.62; 169800,15.58; 170400,15.58; 171000,15.58;
            171600,15.58; 172200,15.6; 172800,15.55; 173400,15.58; 174000,15.58; 174600,
            15.54; 175200,15.54; 175800,15.54; 176400,15.54; 177000,15.54; 177600,15.54;
            178200,15.54; 178800,15.53; 179400,15.5; 180000,15.5; 180600,15.51; 181200,
            15.5; 181800,15.46; 182400,15.48; 183000,15.46; 183600,15.46; 184200,15.46;
            184800,15.46; 185400,15.46; 186000,15.46; 186600,15.46; 187200,15.46; 187800,
            15.46; 188400,15.42; 189000,15.42; 189600,15.42; 190200,15.42; 190800,15.42;
            191400,15.42; 192000,15.42; 192600,15.39; 193200,15.39; 193800,15.36; 194400,
            15.34; 195000,15.35; 195600,15.34; 196200,15.3; 196800,15.3; 197400,15.3;
            198000,15.3; 198600,15.3; 199200,15.28; 199800,15.27; 200400,15.29; 201000,
            15.26; 201600,15.26; 202200,15.26; 202800,15.26; 203400,15.25; 204000,15.22;
            204600,15.22; 205200,15.22; 205800,15.26; 206400,15.27; 207000,15.27; 207600,
            15.26; 208200,15.3; 208800,15.3; 209400,15.31; 210000,15.3; 210600,15.35;
            211200,15.34; 211800,15.34; 212400,15.34; 213000,15.36; 213600,15.38; 214200,
            15.42; 214800,15.46; 215400,15.46; 216000,15.46; 216600,15.5; 217200,15.5;
            217800,15.5; 218400,15.52; 219000,15.5; 219600,15.5; 220200,15.5; 220800,
            15.5; 221400,15.5; 222000,15.51; 222600,15.47; 223200,15.47; 223800,15.46;
            224400,15.46; 225000,15.42; 225600,15.42; 226200,15.38; 226800,15.38; 227400,
            15.34; 228000,15.34; 228600,15.34; 229200,15.34; 229800,15.3; 230400,15.3;
            231000,15.3; 231600,15.3; 232200,15.26; 232800,15.26; 233400,15.26; 234000,
            15.22; 234600,15.22; 235200,15.22; 235800,15.22; 236400,15.18; 237000,15.18;
            237600,15.14; 238200,15.14; 238800,15.14; 239400,15.14; 240000,15.13; 240600,
            15.1; 241200,15.1; 241800,15.1; 242400,15.1; 243000,15.1; 243600,15.1; 244200,
            15.1; 244800,15.1; 245400,15.1; 246000,15.09; 246600,15.06; 247200,15.05;
            247800,15.02; 248400,15.02; 249000,15.01; 249600,15.01; 250200,15; 250800,
            14.97; 251400,14.97; 252000,14.97; 252600,14.97; 253200,14.97; 253800,14.94;
            254400,14.93; 255000,14.93; 255600,14.92; 256200,14.89; 256800,14.9; 257400,
            14.85; 258000,14.85; 258600,14.85; 259200,14.82; 259800,14.81; 260400,14.81;
            261000,14.81; 261600,14.81; 262200,14.82; 262800,14.77; 263400,14.77; 264000,
            14.77; 264600,14.77; 265200,14.73; 265800,14.76; 266400,14.73; 267000,14.73;
            267600,14.73; 268200,14.73; 268800,14.73; 269400,14.73; 270000,14.73; 270600,
            14.73; 271200,14.73; 271800,14.73; 272400,14.7; 273000,14.73; 273600,14.72;
            274200,14.73; 274800,14.72; 275400,14.74; 276000,14.72; 276600,14.69; 277200,
            14.7; 277800,14.69; 278400,14.69; 279000,14.69; 279600,14.69; 280200,14.69;
            280800,14.69; 281400,14.66; 282000,14.69; 282600,14.69; 283200,14.65; 283800,
            14.66; 284400,14.77; 285000,14.93; 285600,15.01; 286200,15.11; 286800,15.22;
            287400,15.38; 288000,15.46; 288600,15.58; 289200,15.7; 289800,15.83; 290400,
            15.91; 291000,15.99; 291600,16.12; 292200,16.24; 292800,16.28; 293400,16.31;
            294000,16.41; 294600,16.48; 295200,16.56; 295800,16.6; 296400,16.69; 297000,
            16.72; 297600,16.76; 298200,16.82; 298800,16.88; 299400,16.93; 300000,16.97;
            300600,17.01; 301200,17.01; 301800,17.09; 302400,17.08; 303000,17.09; 303600,
            17.13; 304200,17.17; 304800,17.21; 305400,17.18; 306000,17.22; 306600,17.21;
            307200,17.29; 307800,17.31; 308400,17.38; 309000,17.46; 309600,17.54; 310200,
            17.62; 310800,17.62; 311400,17.67; 312000,17.73; 312600,17.78; 313200,17.74;
            313800,17.9; 314400,17.9; 315000,17.9; 315600,18.02; 316200,18.06; 316800,
            17.94; 317400,17.9; 318000,17.9; 318600,17.98; 319200,17.94; 319800,17.9;
            320400,17.86; 321000,17.86; 321600,17.82; 322200,17.8; 322800,17.78; 323400,
            17.74; 324000,17.74; 324600,17.7; 325200,17.69; 325800,17.66; 326400,17.62;
            327000,17.58; 327600,17.58; 328200,17.62; 328800,17.62; 329400,17.58; 330000,
            17.5; 330600,17.42; 331200,17.37; 331800,17.29; 332400,17.21; 333000,17.17;
            333600,17.18; 334200,17.12; 334800,17.09; 335400,17.09; 336000,17.06; 336600,
            17.06; 337200,17.05; 337800,17.05; 338400,17.01; 339000,17.01; 339600,17.02;
            340200,16.98; 340800,16.97; 341400,16.97; 342000,16.93; 342600,16.92; 343200,
            16.92; 343800,16.9; 344400,16.88; 345000,16.88; 345600,16.88; 346200,16.88;
            346800,16.88; 347400,16.84; 348000,16.84; 348600,16.84; 349200,16.84; 349800,
            16.84; 350400,16.81; 351000,16.81; 351600,16.82; 352200,16.84; 352800,16.84;
            353400,16.84; 354000,16.84; 354600,16.84; 355200,16.86; 355800,16.89; 356400,
            16.88; 357000,16.88; 357600,16.88; 358200,16.88; 358800,16.9; 359400,16.89;
            360000,16.92; 360600,16.92; 361200,16.96; 361800,17.17; 362400,17.42; 363000,
            17.54; 363600,17.71; 364200,17.84; 364800,17.94; 365400,17.98; 366000,18.06;
            366600,18.1; 367200,18.15; 367800,18.19; 368400,18.24; 369000,18.27; 369600,
            18.35; 370200,18.35; 370800,18.35; 371400,18.44; 372000,18.47; 372600,18.58;
            373200,18.67; 373800,18.72; 374400,18.79; 375000,18.84; 375600,18.92; 376200,
            18.96; 376800,19.03; 377400,19.08; 378000,19.16; 378600,19.24; 379200,19.37;
            379800,19.4; 380400,19.47; 381000,19.53; 381600,19.58; 382200,19.61; 382800,
            19.69; 383400,19.81; 384000,19.86; 384600,19.94; 385200,19.98; 385800,20.03;
            386400,20.02; 387000,20.1; 387600,20.14; 388200,20.16; 388800,20.23; 389400,
            20.29; 390000,20.34; 390600,20.34; 391200,20.42; 391800,20.52; 392400,20.54;
            393000,20.54; 393600,20.51; 394200,20.51; 394800,20.5; 395400,20.51; 396000,
            20.51; 396600,20.51; 397200,20.51; 397800,20.47; 398400,20.5; 399000,20.52;
            399600,20.58; 400200,20.59; 400800,20.63; 401400,20.68; 402000,20.67; 402600,
            20.67; 403200,20.67; 403800,20.68; 404400,20.75; 405000,20.8; 405600,20.83;
            406200,20.83; 406800,20.83; 407400,20.87; 408000,20.86; 408600,20.87; 409200,
            20.86; 409800,20.83; 410400,20.83; 411000,20.92; 411600,20.87; 412200,20.88;
            412800,20.87; 413400,20.83; 414000,20.83; 414600,20.8; 415200,20.75; 415800,
            20.62; 416400,20.49; 417000,20.34; 417600,20.26; 418200,20.14; 418800,20.02;
            419400,19.86; 420000,19.78; 420600,19.65; 421200,19.57; 421800,19.53; 422400,
            19.41; 423000,19.34; 423600,19.28; 424200,19.2; 424800,19.16; 425400,19.12;
            426000,19.05; 426600,19; 427200,18.92; 427800,18.88; 428400,18.88; 429000,
            18.84; 429600,18.8; 430200,18.76; 430800,18.72; 431400,18.68; 432000,18.64;
            432600,18.64; 433200,18.6; 433800,18.55; 434400,18.55; 435000,18.51; 435600,
            18.51; 436200,18.47; 436800,18.47; 437400,18.43; 438000,18.4; 438600,18.39;
            439200,18.35; 439800,18.35; 440400,18.35; 441000,18.35; 441600,18.35; 442200,
            18.35; 442800,18.35; 443400,18.36; 444000,18.35; 444600,18.37; 445200,18.35;
            445800,18.35; 446400,18.38; 447000,18.36; 447600,18.4; 448200,18.55; 448800,
            18.84; 449400,19.01; 450000,19.2; 450600,19.37; 451200,19.45; 451800,19.5;
            452400,19.53; 453000,19.57; 453600,19.61; 454200,19.65; 454800,19.69; 455400,
            19.74; 456000,19.78; 456600,19.82; 457200,19.86; 457800,19.9; 458400,19.98;
            459000,20.02; 459600,20.06; 460200,20.06; 460800,20.14; 461400,20.18; 462000,
            20.27; 462600,20.3; 463200,20.34; 463800,20.44; 464400,20.46; 465000,20.47;
            465600,20.76; 466200,20.56; 466800,20.54; 467400,20.54; 468000,20.55; 468600,
            20.59; 469200,20.71; 469800,20.79; 470400,20.91; 471000,20.98; 471600,21;
            472200,21.08; 472800,21.08; 473400,21.16; 474000,21.24; 474600,21.24; 475200,
            21.28; 475800,21.16; 476400,19.86; 477000,20.72; 477600,21.04; 478200,21.15;
            478800,21.2; 479400,21.28; 480000,21.32; 480600,21.32; 481200,21.4; 481800,
            21.44; 482400,21.48; 483000,21.48; 483600,21.52; 484200,21.56; 484800,21.6;
            485400,21.6; 486000,21.66; 486600,21.69; 487200,21.73; 487800,21.74; 488400,
            21.73; 489000,21.77; 489600,21.81; 490200,21.81; 490800,21.81; 491400,21.85;
            492000,21.93; 492600,21.96; 493200,21.97; 493800,21.97; 494400,21.97; 495000,
            21.97; 495600,21.97; 496200,21.97; 496800,21.96; 497400,21.96; 498000,21.93;
            498600,21.93; 499200,21.9; 499800,21.89; 500400,21.93; 501000,21.89; 501600,
            21.81; 502200,21.69; 502800,21.62; 503400,21.49; 504000,21.32; 504600,21.2;
            505200,21.08; 505800,20.99; 506400,20.83; 507000,20.71; 507600,20.63; 508200,
            20.59; 508800,20.51; 509400,20.48; 510000,20.42; 510600,20.34; 511200,20.22;
            511800,20.18; 512400,20.18; 513000,20.11; 513600,20.06; 514200,20.02; 514800,
            19.94; 515400,19.9; 516000,19.82; 516600,19.78; 517200,19.73; 517800,19.69;
            518400,19.65; 519000,19.65; 519600,19.61; 520200,19.57; 520800,19.57; 521400,
            19.53; 522000,19.53; 522600,19.49; 523200,19.49; 523800,19.45; 524400,19.41;
            525000,19.45; 525600,19.57; 526200,19.73; 526800,19.87; 527400,20.02; 528000,
            20.18; 528600,20.26; 529200,20.34; 529800,20.46; 530400,20.55; 531000,20.63;
            531600,20.72; 532200,20.77; 532800,20.83; 533400,20.92; 534000,20.96; 534600,
            20.99; 535200,21; 535800,21.08; 536400,21.12; 537000,21.12; 537600,21.15;
            538200,21.16; 538800,21.19; 539400,21.23; 540000,21.24; 540600,21.28; 541200,
            21.28; 541800,21.32; 542400,21.32; 543000,21.32; 543600,21.31; 544200,21.28;
            544800,21.32; 545400,21.36; 546000,21.4; 546600,21.46; 547200,21.52; 547800,
            21.55; 548400,21.6; 549000,21.6; 549600,21.61; 550200,21.66; 550800,21.73;
            551400,21.77; 552000,21.81; 552600,21.81; 553200,21.93; 553800,21.99; 554400,
            22.05; 555000,22.09; 555600,22.1; 556200,22.17; 556800,22.21; 557400,22.24;
            558000,22.36; 558600,22.38; 559200,22.34; 559800,22.34; 560400,22.38; 561000,
            22.42; 561600,22.42; 562200,22.42; 562800,22.46; 563400,22.46; 564000,22.46;
            564600,22.5; 565200,22.5; 565800,22.5; 566400,22.51; 567000,22.48; 567600,
            22.5; 568200,22.5; 568800,22.5; 569400,22.5; 570000,22.54; 570600,22.58;
            571200,22.58; 571800,22.58; 572400,22.64; 573000,22.74; 573600,22.78; 574200,
            22.85; 574800,22.87; 575400,22.92; 576000,22.95; 576600,23; 577200,23.07;
            577800,23.17; 578400,23.27; 579000,23.27; 579600,23.24; 580200,23.4; 580800,
            23.44; 581400,23.48; 582000,23.48; 582600,23.43; 583200,23.4; 583800,23.37;
            584400,23.32; 585000,23.35; 585600,23.4; 586200,23.4; 586800,23.43; 587400,
            23.4; 588000,23.32; 588600,23.16; 589200,23.07; 589800,22.94; 590400,22.78;
            591000,22.62; 591600,22.46; 592200,22.34; 592800,22.22; 593400,22.18; 594000,
            22.05; 594600,21.93; 595200,21.85; 595800,21.77; 596400,21.65; 597000,21.6;
            597600,21.52; 598200,21.49; 598800,21.4; 599400,21.36; 600000,21.28; 600600,
            21.28; 601200,21.2; 601800,21.16; 602400,21.15; 603000,21.12; 603600,21.08;
            604200,21.04; 604800,21; 605400,20.96; 606000,20.95; 606600,20.9; 607200,
            20.87; 607800,20.83; 608400,20.84; 609000,20.83; 609600,20.78; 610200,20.76;
            610800,20.75; 611400,20.79; 612000,20.91; 612600,21.08; 613200,21.2; 613800,
            21.32; 614400,21.48; 615000,21.6; 615600,21.69; 616200,21.82; 616800,21.89;
            617400,22.01; 618000,22.09; 618600,22.18; 619200,22.25; 619800,22.33; 620400,
            22.38; 621000,22.42; 621600,22.49; 622200,22.5; 622800,22.58; 623400,22.62;
            624000,22.63; 624600,22.7; 625200,22.72; 625800,22.74; 626400,22.78; 627000,
            22.82; 627600,22.83; 628200,22.86; 628800,22.87; 629400,22.91; 630000,22.79;
            630600,22.74; 631200,22.7; 631800,22.7; 632400,22.78; 633000,22.91; 633600,
            23.03; 634200,23.11; 634800,23.18; 635400,23.24; 636000,23.22; 636600,23.15;
            637200,23.06; 637800,22.99; 638400,23; 639000,22.91; 639600,22.91; 640200,
            22.78; 640800,22.87; 641400,23.03; 642000,23.04; 642600,22.99; 643200,23.04;
            643800,23.03; 644400,23.12; 645000,23.11; 645600,23.11; 646200,23.12; 646800,
            23.11; 647400,23.03; 648000,22.88; 648600,22.82; 649200,22.78; 649800,22.83;
            650400,22.96; 651000,22.92; 651600,23.03; 652200,23.08; 652800,23.13; 653400,
            23.11; 654000,23.15; 654600,23.12; 655200,23.15; 655800,23.2; 656400,23.19;
            657000,23.24; 657600,23.27; 658200,23.28; 658800,23.27; 659400,23.28; 660000,
            23.27; 660600,23.27; 661200,23.27; 661800,23.23; 662400,23.27; 663000,23.27;
            663600,23.27; 664200,23.31; 664800,23.32; 665400,23.32; 666000,23.28; 666600,
            23.19; 667200,23.05; 667800,22.95; 668400,22.83; 669000,22.74; 669600,22.7;
            670200,22.62; 670800,22.58; 671400,22.54; 672000,22.48; 672600,22.42; 673200,
            22.34; 673800,22.3; 674400,22.27; 675000,22.22; 675600,22.14; 676200,22.05;
            676800,21.97; 677400,21.93; 678000,21.85; 678600,21.81; 679200,21.77; 679800,
            21.72; 680400,21.65; 681000,21.6; 681600,21.52; 682200,21.48; 682800,21.44;
            683400,21.4; 684000,21.4; 684600,21.32; 685200,21.32; 685800,21.29; 686400,
            21.24; 687000,21.2; 687600,21.16; 688200,21.15; 688800,21.12; 689400,21.08;
            690000,21.07; 690600,21.04; 691200,21; 691800,21; 692400,21; 693000,20.96;
            693600,20.91; 694200,20.92; 694800,20.91; 695400,20.88; 696000,20.88; 696600,
            20.87; 697200,20.83; 697800,20.83; 698400,20.83; 699000,20.79; 699600,20.79;
            700200,20.8; 700800,20.76; 701400,20.78; 702000,20.75; 702600,20.75; 703200,
            20.75; 703800,20.71; 704400,20.71; 705000,20.7; 705600,20.67; 706200,20.67;
            706800,20.67; 707400,20.65; 708000,20.63; 708600,20.63; 709200,20.63; 709800,
            20.63; 710400,20.63; 711000,20.59; 711600,20.59; 712200,20.59; 712800,20.56;
            713400,20.55; 714000,20.55; 714600,20.55; 715200,20.56; 715800,20.51; 716400,
            20.51; 717000,20.52; 717600,20.51; 718200,20.52; 718800,20.51; 719400,20.48;
            720000,20.5; 720600,20.5; 721200,20.48; 721800,20.46; 722400,20.46; 723000,
            20.46; 723600,20.46; 724200,20.48; 724800,20.44; 725400,20.46; 726000,20.42;
            726600,20.42; 727200,20.42; 727800,20.46; 728400,20.42; 729000,20.46; 729600,
            20.42; 730200,20.42; 730800,20.42; 731400,20.42; 732000,20.42; 732600,20.42;
            733200,20.42; 733800,20.42; 734400,20.38; 735000,20.38; 735600,20.4; 736200,
            20.38; 736800,20.38; 737400,20.38; 738000,20.4; 738600,20.38; 739200,20.38;
            739800,20.34; 740400,20.34; 741000,20.34; 741600,20.34; 742200,20.34; 742800,
            20.34; 743400,20.34; 744000,20.34; 744600,20.38; 745200,20.38; 745800,20.38;
            746400,20.38; 747000,20.34; 747600,20.34; 748200,20.36; 748800,20.32; 749400,
            20.3; 750000,20.32; 750600,20.3; 751200,20.3; 751800,20.26; 752400,20.26;
            753000,20.24; 753600,20.26; 754200,20.22; 754800,20.23; 755400,20.22; 756000,
            20.21; 756600,20.18; 757200,20.18; 757800,20.18; 758400,20.18; 759000,20.18;
            759600,20.14; 760200,20.14; 760800,20.14; 761400,20.1; 762000,20.1; 762600,
            20.1; 763200,20.09; 763800,20.06; 764400,20.06; 765000,20.06; 765600,20.04;
            766200,20.02; 766800,20.02; 767400,20.02; 768000,19.98; 768600,19.98; 769200,
            19.98; 769800,19.94; 770400,19.94; 771000,19.94; 771600,19.9; 772200,19.9;
            772800,19.86; 773400,19.86; 774000,19.86; 774600,19.87; 775200,19.82; 775800,
            19.82; 776400,19.81; 777000,19.78; 777600,19.78; 778200,19.75; 778800,19.73;
            779400,19.73; 780000,19.73; 780600,19.73; 781200,19.74; 781800,19.72; 782400,
            19.73; 783000,19.7; 783600,19.69; 784200,19.72; 784800,19.69; 785400,19.69;
            786000,19.65; 786600,19.65; 787200,19.65; 787800,19.65; 788400,19.65; 789000,
            19.65; 789600,19.65; 790200,19.61; 790800,19.62; 791400,19.62; 792000,19.61;
            792600,19.61; 793200,19.62; 793800,19.61; 794400,19.61; 795000,19.61; 795600,
            19.61; 796200,19.57; 796800,19.61; 797400,19.6; 798000,19.59; 798600,19.57;
            799200,19.57; 799800,19.57; 800400,19.57; 801000,19.57; 801600,19.57; 802200,
            19.58; 802800,19.57; 803400,19.57; 804000,19.57; 804600,19.57; 805200,19.57;
            805800,19.57; 806400,19.57; 807000,19.57; 807600,19.57; 808200,19.57; 808800,
            19.57; 809400,19.57; 810000,19.58; 810600,19.61; 811200,19.61; 811800,19.61;
            812400,19.62; 813000,19.61; 813600,19.64; 814200,19.65; 814800,19.65; 815400,
            19.65; 816000,19.66; 816600,19.66; 817200,19.69; 817800,19.7; 818400,19.73;
            819000,19.73; 819600,19.77; 820200,19.78; 820800,19.78; 821400,19.78; 822000,
            19.78; 822600,19.83; 823200,19.82; 823800,19.82; 824400,19.83; 825000,19.82;
            825600,19.84; 826200,19.83; 826800,19.83; 827400,19.82; 828000,19.82; 828600,
            19.82; 829200,19.82; 829800,19.82; 830400,19.82; 831000,19.82; 831600,19.82;
            832200,19.82; 832800,19.82; 833400,19.82; 834000,19.82; 834600,19.82; 835200,
            19.82; 835800,19.82; 836400,19.82; 837000,19.82; 837600,19.8; 838200,19.78;
            838800,19.78; 839400,19.78; 840000,19.78; 840600,19.78; 841200,19.78; 841800,
            19.78; 842400,19.77; 843000,19.73; 843600,19.76; 844200,19.73; 844800,19.73;
            845400,19.73; 846000,19.73; 846600,19.73; 847200,19.74; 847800,19.73; 848400,
            19.73; 849000,19.69; 849600,19.69; 850200,19.69; 850800,19.69; 851400,19.7;
            852000,19.73; 852600,19.73; 853200,19.73; 853800,19.73; 854400,19.74; 855000,
            19.73; 855600,19.73; 856200,19.73; 856800,19.73; 857400,19.73; 858000,19.69;
            858600,19.69; 859200,19.7; 859800,19.7; 860400,19.69; 861000,19.69; 861600,
            19.7; 862200,19.69; 862800,19.69; 863400,19.67; 864000,19.66; 864600,19.66;
            865200,19.65; 865800,19.65; 866400,19.66; 867000,19.65; 867600,19.65; 868200,
            19.61; 868800,19.65; 869400,19.65; 870000,19.65; 870600,19.65; 871200,19.65;
            871800,19.66; 872400,19.65; 873000,19.65; 873600,19.65; 874200,19.61; 874800,
            19.61; 875400,19.63; 876000,19.61; 876600,19.61; 877200,19.65; 877800,19.65;
            878400,19.66; 879000,19.65; 879600,19.65; 880200,19.65; 880800,19.66; 881400,
            19.65; 882000,19.65; 882600,19.65; 883200,19.65; 883800,19.65; 884400,19.65;
            885000,19.65; 885600,19.65; 886200,19.68; 886800,19.65; 887400,19.65; 888000,
            19.81; 888600,20.02; 889200,20.18; 889800,20.26; 890400,20.42; 891000,20.58;
            891600,20.71; 892200,20.83; 892800,20.96; 893400,21; 894000,21.08; 894600,
            21.17; 895200,21.2; 895800,21.32; 896400,21.46; 897000,21.56; 897600,21.65;
            898200,21.69; 898800,21.77; 899400,21.84; 900000,21.89; 900600,22.01; 901200,
            22.09; 901800,22.17; 902400,22.26; 903000,22.38; 903600,22.51; 904200,22.59;
            904800,22.62; 905400,22.66; 906000,22.7; 906600,22.7; 907200,22.71; 907800,
            22.76; 908400,22.74; 909000,22.75; 909600,22.83; 910200,22.91; 910800,22.94;
            911400,22.94; 912000,23.03; 912600,23.07; 913200,23.11; 913800,23.11; 914400,
            23.15; 915000,23.27; 915600,23.4; 916200,23.49; 916800,23.56; 917400,23.6;
            918000,23.64; 918600,23.72; 919200,23.76; 919800,23.88; 920400,23.84; 921000,
            23.76; 921600,23.73; 922200,23.72; 922800,23.68; 923400,23.68; 924000,23.68;
            924600,23.67; 925200,23.68; 925800,23.65; 926400,23.64; 927000,23.64; 927600,
            23.68; 928200,23.72; 928800,23.72; 929400,23.72; 930000,23.72; 930600,23.72;
            931200,23.72; 931800,23.72; 932400,23.72; 933000,23.72; 933600,23.65; 934200,
            23.48; 934800,23.36; 935400,23.18; 936000,23.03; 936600,22.91; 937200,22.78;
            937800,22.67; 938400,22.57; 939000,22.46; 939600,22.38; 940200,22.26; 940800,
            22.18; 941400,22.09; 942000,22.02; 942600,21.97; 943200,21.89; 943800,21.85;
            944400,21.81; 945000,21.77; 945600,21.69; 946200,21.65; 946800,21.61; 947400,
            21.6; 948000,21.6; 948600,21.56; 949200,21.56; 949800,21.52; 950400,21.51;
            951000,21.44; 951600,21.44; 952200,21.39; 952800,21.4; 953400,21.36; 954000,
            21.36; 954600,21.32; 955200,21.28; 955800,21.32; 956400,21.32; 957000,21.32;
            957600,21.44; 958200,21.6; 958800,21.73; 959400,21.89; 960000,22.05; 960600,
            22.18; 961200,22.3; 961800,22.44; 962400,22.58; 963000,22.7; 963600,22.78;
            964200,22.83; 964800,22.91; 965400,22.99; 966000,23.03; 966600,23.07; 967200,
            23.11; 967800,23.11; 968400,23.15; 969000,23.23; 969600,23.25; 970200,23.27;
            970800,23.32; 971400,23.35; 972000,23.36; 972600,23.36; 973200,23.4; 973800,
            23.4; 974400,23.4; 975000,23.36; 975600,23.27; 976200,23.32; 976800,23.32;
            977400,23.27; 978000,23.23; 978600,23.19; 979200,23.16; 979800,23.11; 980400,
            23.14; 981000,23.08; 981600,23.07; 982200,23.03; 982800,23.03; 983400,23.03;
            984000,23.03; 984600,22.95; 985200,22.95; 985800,22.95; 986400,22.99; 987000,
            22.99; 987600,22.95; 988200,23.03; 988800,23.07; 989400,23.07; 990000,23.1;
            990600,23.11; 991200,23.11; 991800,23.12; 992400,23.19; 993000,23.19; 993600,
            23.12; 994200,23.11; 994800,23.06; 995400,23.08; 996000,23.15; 996600,23.24;
            997200,23.23; 997800,23.2; 998400,23.2; 999000,23.15; 999600,23.19; 1000200,
            23.19; 1000800,23.23; 1001400,23.24; 1002000,23.27; 1002600,23.32; 1003200,
            23.23; 1003800,23.23; 1004400,23.27; 1005000,23.24; 1005600,23.27; 1006200,
            23.24; 1006800,23.28; 1007400,23.27; 1008000,23.27; 1008600,23.24; 1009200,
            23.2; 1009800,23.19; 1010400,23.07; 1011000,23.03; 1011600,22.92; 1012200,
            22.91; 1012800,22.83; 1013400,22.79; 1014000,22.66; 1014600,22.58; 1015200,
            22.5; 1015800,22.5; 1016400,22.42; 1017000,22.35; 1017600,22.26; 1018200,
            22.26; 1018800,22.21; 1019400,22.14; 1020000,22.09; 1020600,22.04; 1021200,
            22.02; 1021800,21.97; 1022400,21.89; 1023000,21.81; 1023600,21.77; 1024200,
            21.69; 1024800,21.7; 1025400,21.62; 1026000,21.6; 1026600,21.6; 1027200,
            21.56; 1027800,21.48; 1028400,21.48; 1029000,21.4; 1029600,21.4; 1030200,
            21.36; 1030800,21.32; 1031400,21.32; 1032000,21.28; 1032600,21.28; 1033200,
            21.25; 1033800,21.2; 1034400,21.16; 1035000,21.16; 1035600,21.12; 1036200,
            21.12; 1036800,21.08; 1037400,21.08; 1038000,21.07; 1038600,21.04; 1039200,
            21; 1039800,20.99; 1040400,20.96; 1041000,20.96; 1041600,20.96; 1042200,
            20.95; 1042800,20.91; 1043400,20.87; 1044000,20.87; 1044600,20.91; 1045200,
            20.96; 1045800,20.91; 1046400,20.96; 1047000,20.96; 1047600,20.96; 1048200,
            20.96; 1048800,20.96; 1049400,20.96; 1050000,21; 1050600,20.99; 1051200,
            20.96; 1051800,20.96; 1052400,20.96; 1053000,20.96; 1053600,20.96; 1054200,
            20.96; 1054800,20.96; 1055400,20.96; 1056000,20.95; 1056600,20.94; 1057200,
            20.94; 1057800,20.95; 1058400,20.91; 1059000,20.95; 1059600,20.92; 1060200,
            20.96; 1060800,20.96; 1061400,21; 1062000,21.16; 1062600,21.27; 1063200,
            21.36; 1063800,21.44; 1064400,21.48; 1065000,21.56; 1065600,21.68; 1066200,
            21.81; 1066800,21.94; 1067400,22.01; 1068000,22.1; 1068600,22.18; 1069200,
            22.22; 1069800,22.22; 1070400,22.22; 1071000,22.23; 1071600,22.26; 1072200,
            22.3; 1072800,22.38; 1073400,22.42; 1074000,22.47; 1074600,22.54; 1075200,
            22.58; 1075800,22.7; 1076400,22.78; 1077000,22.83; 1077600,22.83; 1078200,
            22.87; 1078800,22.83; 1079400,22.78; 1080000,22.78; 1080600,22.82; 1081200,
            22.91; 1081800,22.88; 1082400,22.87; 1083000,22.87; 1083600,22.96; 1084200,
            22.96; 1084800,22.99; 1085400,23.03; 1086000,23.07; 1086600,23.16; 1087200,
            23.23; 1087800,23.15; 1088400,23.16; 1089000,23.23; 1089600,23.27; 1090200,
            23.28; 1090800,23.31; 1091400,23.32; 1092000,23.36; 1092600,23.36; 1093200,
            23.34; 1093800,23.36; 1094400,23.39; 1095000,23.4; 1095600,23.32; 1096200,
            23.27; 1096800,23.23; 1097400,23.18; 1098000,23.11; 1098600,23.07; 1099200,
            23; 1099800,22.87; 1100400,22.7; 1101000,22.58; 1101600,22.46; 1102200,22.4;
            1102800,22.3; 1103400,22.22; 1104000,22.14; 1104600,22.05; 1105200,21.97;
            1105800,21.89; 1106400,21.81; 1107000,21.77; 1107600,21.69; 1108200,21.6;
            1108800,21.56; 1109400,21.48; 1110000,21.44; 1110600,21.41; 1111200,21.32;
            1111800,21.24; 1112400,21.2; 1113000,21.16; 1113600,21.08; 1114200,21.02;
            1114800,20.99; 1115400,20.93; 1116000,20.87; 1116600,20.83; 1117200,20.79;
            1117800,20.79; 1118400,20.75; 1119000,20.71; 1119600,20.67; 1120200,20.64;
            1120800,20.63; 1121400,20.59; 1122000,20.55; 1122600,20.51; 1123200,20.51;
            1123800,20.46; 1124400,20.44; 1125000,20.42; 1125600,20.38; 1126200,20.36;
            1126800,20.34; 1127400,20.33; 1128000,20.3; 1128600,20.26; 1129200,20.26;
            1129800,20.23; 1130400,20.22; 1131000,20.22; 1131600,20.18; 1132200,20.18;
            1132800,20.18; 1133400,20.14; 1134000,20.14; 1134600,20.1; 1135200,20.1;
            1135800,20.06; 1136400,20.06; 1137000,20.02; 1137600,20.02; 1138200,20.02;
            1138800,20.02; 1139400,20.02; 1140000,19.98; 1140600,19.98; 1141200,19.98;
            1141800,19.99; 1142400,19.98; 1143000,19.98; 1143600,19.98; 1144200,19.97;
            1144800,19.94; 1145400,19.96; 1146000,19.94; 1146600,19.94; 1147200,19.94;
            1147800,20.02; 1148400,20.3; 1149000,20.47; 1149600,20.63; 1150200,20.88;
            1150800,20.87; 1151400,21.04; 1152000,21.11; 1152600,21.16; 1153200,21.26;
            1153800,21.32; 1154400,21.36; 1155000,21.33; 1155600,21.38; 1156200,21.73;
            1156800,22.06; 1157400,22.38; 1158000,22.58; 1158600,22.78; 1159200,22.97;
            1159800,23.11; 1160400,23.27; 1161000,23.44; 1161600,23.56; 1162200,23.73;
            1162800,23.84; 1163400,23.77; 1164000,23.72; 1164600,23.72; 1165200,23.72;
            1165800,23.8; 1166400,23.82; 1167000,23.84; 1167600,23.72; 1168200,23.8;
            1168800,23.8; 1169400,23.88; 1170000,23.93; 1170600,24.01; 1171200,24.09;
            1171800,24.18; 1172400,24.32; 1173000,24.38; 1173600,24.37; 1174200,24.37;
            1174800,24.29; 1175400,24.16; 1176000,23.98; 1176600,23.93; 1177200,23.79;
            1177800,23.68; 1178400,23.58; 1179000,23.44; 1179600,23.43; 1180200,23.47;
            1180800,23.44; 1181400,23.45; 1182000,23.44; 1182600,23.4; 1183200,23.35;
            1183800,23.24; 1184400,23.24; 1185000,23.17; 1185600,23.11; 1186200,23.07;
            1186800,22.99; 1187400,22.87; 1188000,22.74; 1188600,22.62; 1189200,22.5;
            1189800,22.42; 1190400,22.34; 1191000,22.26; 1191600,22.18; 1192200,22.09;
            1192800,22.05; 1193400,21.97; 1194000,21.89; 1194600,21.81; 1195200,21.81;
            1195800,21.77; 1196400,21.69; 1197000,21.65; 1197600,21.6; 1198200,21.48;
            1198800,21.44; 1199400,21.41; 1200000,21.33; 1200600,21.28; 1201200,21.24;
            1201800,21.16; 1202400,21.12; 1203000,21.12; 1203600,21.04; 1204200,21;
            1204800,20.96; 1205400,20.96; 1206000,20.91; 1206600,20.88; 1207200,20.83;
            1207800,20.79; 1208400,20.79; 1209000,20.75; 1209600,20.71; 1210200,20.67;
            1210800,20.67; 1211400,20.63; 1212000,20.63; 1212600,20.59; 1213200,20.55;
            1213800,20.54; 1214400,20.51; 1215000,20.51; 1215600,20.48; 1216200,20.46;
            1216800,20.46; 1217400,20.42; 1218000,20.42; 1218600,20.42; 1219200,20.39;
            1219800,20.38; 1220400,20.38; 1221000,20.34; 1221600,20.34; 1222200,20.34;
            1222800,20.34; 1223400,20.34; 1224000,20.34; 1224600,20.36; 1225200,20.3;
            1225800,20.3; 1226400,20.3; 1227000,20.3; 1227600,20.3; 1228200,20.3; 1228800,
            20.3; 1229400,20.29; 1230000,20.26; 1230600,20.26; 1231200,20.26; 1231800,
            20.26; 1232400,20.24; 1233000,20.22; 1233600,20.22; 1234200,20.34; 1234800,
            20.51; 1235400,20.67; 1236000,20.75; 1236600,20.87; 1237200,20.95; 1237800,
            21.07; 1238400,21.16; 1239000,21.22; 1239600,21.32; 1240200,21.36; 1240800,
            21.44; 1241400,21.5; 1242000,21.6; 1242600,21.65; 1243200,21.69; 1243800,
            21.76; 1244400,21.85; 1245000,21.85; 1245600,21.93; 1246200,21.97; 1246800,
            22.01; 1247400,22.09; 1248000,22.1; 1248600,22.14; 1249200,22.18; 1249800,
            22.27; 1250400,22.29; 1251000,22.42; 1251600,22.46; 1252200,22.5; 1252800,
            22.5; 1253400,22.5; 1254000,22.54; 1254600,22.58; 1255200,22.54; 1255800,
            22.54; 1256400,22.5; 1257000,22.54; 1257600,22.66; 1258200,22.7; 1258800,
            22.7; 1259400,22.74; 1260000,22.75; 1260600,22.83; 1261200,22.89; 1261800,
            22.8; 1262400,22.76; 1263000,22.7; 1263600,22.78; 1264200,22.78; 1264800,
            22.78; 1265400,22.78; 1266000,22.75; 1266600,22.77; 1267200,22.82; 1267800,
            22.83; 1268400,22.78; 1269000,22.74; 1269600,22.7; 1270200,22.66; 1270800,
            22.64; 1271400,22.58; 1272000,22.58; 1272600,22.54; 1273200,22.54; 1273800,
            22.5; 1274400,22.51; 1275000,22.46; 1275600,22.42; 1276200,22.42; 1276800,
            22.35; 1277400,22.23; 1278000,22.15; 1278600,22.05; 1279200,21.97; 1279800,
            21.89; 1280400,21.82; 1281000,21.75; 1281600,21.72; 1282200,21.65; 1282800,
            21.6; 1283400,21.52; 1284000,21.44; 1284600,21.4; 1285200,21.32; 1285800,
            21.28; 1286400,21.2; 1287000,21.16; 1287600,21.12; 1288200,21.04; 1288800,
            21; 1289400,20.97; 1290000,20.88; 1290600,20.83; 1291200,20.79; 1291800,
            20.75; 1292400,20.72; 1293000,20.67; 1293600,20.63; 1294200,20.59; 1294800,
            20.58; 1295400,20.55; 1296000,20.51; 1296600,20.49; 1297200,20.46; 1297800,
            20.42; 1298400,20.38; 1299000,20.38; 1299600,20.34; 1300200,20.33; 1300800,
            20.3; 1301400,20.26; 1302000,20.26; 1302600,20.22; 1303200,20.23; 1303800,
            20.19; 1304400,20.18; 1305000,20.14; 1305600,20.14; 1306200,20.11; 1306800,
            20.1; 1307400,20.1; 1308000,20.06; 1308600,20.05; 1309200,20.02; 1309800,
            20.02; 1310400,19.99; 1311000,19.98; 1311600,19.98; 1312200,19.94; 1312800,
            19.94; 1313400,19.9; 1314000,19.9; 1314600,19.9; 1315200,19.86; 1315800,
            19.86; 1316400,19.85; 1317000,19.82; 1317600,19.82; 1318200,19.82; 1318800,
            19.78; 1319400,19.78; 1320000,19.78; 1320600,19.78; 1321200,19.73; 1321800,
            19.73; 1322400,19.73; 1323000,19.73; 1323600,19.97; 1324200,19.69; 1324800,
            19.69; 1325400,19.69; 1326000,19.69; 1326600,19.65; 1327200,19.65; 1327800,
            19.65; 1328400,19.65; 1329000,19.66; 1329600,19.65; 1330200,19.69; 1330800,
            19.69; 1331400,19.74; 1332000,19.69; 1332600,19.73; 1333200,19.74; 1333800,
            19.73; 1334400,19.76; 1335000,19.78; 1335600,19.78; 1336200,19.78; 1336800,
            19.78; 1337400,19.78; 1338000,19.78; 1338600,19.78; 1339200,19.82; 1339800,
            19.82; 1340400,19.82; 1341000,19.82; 1341600,19.86; 1342200,19.85; 1342800,
            19.86; 1343400,19.9; 1344000,19.9; 1344600,19.98; 1345200,19.98; 1345800,
            20.02; 1346400,20.02; 1347000,19.98; 1347600,19.94; 1348200,19.9; 1348800,
            19.88; 1349400,19.86; 1350000,19.82; 1350600,19.82; 1351200,19.78; 1351800,
            19.78; 1352400,19.73; 1353000,19.73; 1353600,19.7; 1354200,19.69; 1354800,
            19.65; 1355400,19.65; 1356000,19.61; 1356600,19.57; 1357200,19.57; 1357800,
            19.53; 1358400,19.53; 1359000,19.53; 1359600,19.5; 1360200,19.49; 1360800,
            19.47; 1361400,19.45; 1362000,19.45; 1362600,19.42; 1363200,19.41; 1363800,
            19.37; 1364400,19.37; 1365000,19.37; 1365600,19.34; 1366200,19.34; 1366800,
            19.33; 1367400,19.32; 1368000,19.28; 1368600,19.28; 1369200,19.28; 1369800,
            19.24; 1370400,19.24; 1371000,19.24; 1371600,19.24; 1372200,19.2; 1372800,
            19.2; 1373400,19.21; 1374000,19.2; 1374600,19.16; 1375200,19.16; 1375800,
            19.16; 1376400,19.18; 1377000,19.16; 1377600,19.12; 1378200,19.12; 1378800,
            19.08; 1379400,19.08; 1380000,19.06; 1380600,19.04; 1381200,19.04; 1381800,
            19.04; 1382400,19.02; 1383000,19; 1383600,19; 1384200,19; 1384800,19; 1385400,
            18.98; 1386000,18.96; 1386600,18.96; 1387200,18.96; 1387800,18.96; 1388400,
            18.97; 1389000,18.94; 1389600,18.92; 1390200,18.92; 1390800,18.92; 1391400,
            18.92; 1392000,18.92; 1392600,18.91; 1393200,18.91; 1393800,18.88; 1394400,
            18.89; 1395000,18.88; 1395600,18.88; 1396200,18.88; 1396800,18.88; 1397400,
            18.89; 1398000,18.89; 1398600,18.86; 1399200,18.84; 1399800,18.84; 1400400,
            18.84; 1401000,18.85; 1401600,18.84; 1402200,18.84; 1402800,18.81; 1403400,
            18.82; 1404000,18.8; 1404600,18.81; 1405200,18.8; 1405800,18.8; 1406400,
            18.8; 1407000,18.8; 1407600,18.8; 1408200,18.8; 1408800,18.76; 1409400,18.76;
            1410000,18.76; 1410600,18.77; 1411200,18.76; 1411800,18.77; 1412400,18.76;
            1413000,18.76; 1413600,18.76; 1414200,18.78; 1414800,18.8; 1415400,18.8;
            1416000,18.82; 1416600,18.84; 1417200,18.84; 1417800,18.85; 1418400,18.88;
            1419000,18.88; 1419600,18.89; 1420200,18.92; 1420800,18.92; 1421400,18.96;
            1422000,19; 1422600,19; 1423200,19.04; 1423800,19.04; 1424400,19.08; 1425000,
            19.12; 1425600,19.15; 1426200,19.16; 1426800,19.2; 1427400,19.2; 1428000,
            19.24; 1428600,19.24; 1429200,19.29; 1429800,19.33; 1430400,19.37; 1431000,
            19.37; 1431600,19.4; 1432200,19.45; 1432800,19.45; 1433400,19.49; 1434000,
            19.5; 1434600,19.53; 1435200,19.53; 1435800,19.57; 1436400,19.53; 1437000,
            19.53; 1437600,19.52; 1438200,19.49; 1438800,19.45; 1439400,19.41; 1440000,
            19.37; 1440600,19.37; 1441200,19.33; 1441800,19.28; 1442400,19.24; 1443000,
            19.2; 1443600,19.2; 1444200,19.16; 1444800,19.14; 1445400,19.12; 1446000,
            19.08; 1446600,19.08; 1447200,19.04; 1447800,19.04; 1448400,19; 1449000,
            19; 1449600,18.96; 1450200,18.93; 1450800,18.93; 1451400,18.88; 1452000,
            18.88; 1452600,18.88; 1453200,18.85; 1453800,18.84; 1454400,18.84; 1455000,
            18.8; 1455600,18.8; 1456200,18.8; 1456800,18.77; 1457400,18.76; 1458000,
            18.72; 1458600,18.72; 1459200,18.72; 1459800,18.68; 1460400,18.68; 1461000,
            18.68; 1461600,18.65; 1462200,18.64; 1462800,18.64; 1463400,18.61; 1464000,
            18.6; 1464600,18.6; 1465200,18.6; 1465800,18.56; 1466400,18.55; 1467000,
            18.56; 1467600,18.52; 1468200,18.51; 1468800,18.51; 1469400,18.51; 1470000,
            18.51; 1470600,18.51; 1471200,18.47; 1471800,18.47; 1472400,18.47; 1473000,
            18.47; 1473600,18.43; 1474200,18.43; 1474800,18.43; 1475400,18.43; 1476000,
            18.43; 1476600,18.4; 1477200,18.39; 1477800,18.39; 1478400,18.39; 1479000,
            18.39; 1479600,18.35; 1480200,18.35; 1480800,18.35; 1481400,18.36; 1482000,
            18.32; 1482600,18.31; 1483200,18.31; 1483800,18.31; 1484400,18.32; 1485000,
            18.34; 1485600,18.35; 1486200,18.35; 1486800,18.35; 1487400,18.35; 1488000,
            18.35; 1488600,18.35; 1489200,18.35; 1489800,18.35; 1490400,18.35; 1491000,
            18.35; 1491600,18.33; 1492200,18.35; 1492800,18.35; 1493400,18.35; 1494000,
            18.65; 1494600,18.88; 1495200,19.06; 1495800,19.38; 1496400,19.62; 1497000,
            19.69; 1497600,19.73; 1498200,19.83; 1498800,20.22; 1499400,20.59; 1500000,
            20.91; 1500600,21.13; 1501200,21.36; 1501800,21.56; 1502400,21.74; 1503000,
            21.9; 1503600,21.93; 1504200,22.06; 1504800,22.14; 1505400,22.26; 1506000,
            22.38; 1506600,22.42; 1507200,22.46; 1507800,22.54; 1508400,22.66; 1509000,
            22.77; 1509600,22.84; 1510200,22.83; 1510800,22.78; 1511400,22.75; 1512000,
            22.74; 1512600,22.72; 1513200,22.74; 1513800,22.87; 1514400,22.94; 1515000,
            22.91; 1515600,22.86; 1516200,22.78; 1516800,22.78; 1517400,22.72; 1518000,
            22.62; 1518600,22.54; 1519200,22.46; 1519800,22.43; 1520400,22.38; 1521000,
            22.34; 1521600,22.3; 1522200,22.3; 1522800,22.22; 1523400,22.22; 1524000,
            22.18; 1524600,22.18; 1525200,22.14; 1525800,22.18; 1526400,22.14; 1527000,
            22.1; 1527600,22.09; 1528200,22.05; 1528800,22.01; 1529400,21.97; 1530000,
            21.89; 1530600,21.86; 1531200,21.97; 1531800,21.93; 1532400,21.97; 1533000,
            21.85; 1533600,21.73; 1534200,21.6; 1534800,21.52; 1535400,21.44; 1536000,
            21.46; 1536600,21.44; 1537200,21.4; 1537800,21.32; 1538400,21.2; 1539000,
            21.08; 1539600,21.03; 1540200,20.99; 1540800,20.96; 1541400,20.91; 1542000,
            20.87; 1542600,20.83; 1543200,20.83; 1543800,20.82; 1544400,20.75; 1545000,
            20.71; 1545600,20.67; 1546200,20.63; 1546800,20.59; 1547400,20.55; 1548000,
            20.51; 1548600,20.51; 1549200,20.5; 1549800,20.44; 1550400,20.42; 1551000,
            20.38; 1551600,20.38; 1552200,20.34; 1552800,20.34; 1553400,20.3; 1554000,
            20.3; 1554600,20.26; 1555200,20.28; 1555800,20.26; 1556400,20.25; 1557000,
            20.22; 1557600,20.22; 1558200,20.22; 1558800,20.18; 1559400,20.18; 1560000,
            20.19; 1560600,20.18; 1561200,20.18; 1561800,20.18; 1562400,20.14; 1563000,
            20.14; 1563600,20.14; 1564200,20.14; 1564800,20.14; 1565400,20.15; 1566000,
            20.14; 1566600,20.1; 1567200,20.11; 1567800,20.1; 1568400,20.1; 1569000,
            20.1; 1569600,20.1; 1570200,20.1; 1570800,20.13; 1571400,20.14; 1572000,
            20.1; 1572600,20.06; 1573200,20.02; 1573800,20.01; 1574400,20.02; 1575000,
            20.07; 1575600,20.1; 1576200,20.02; 1576800,19.98; 1577400,19.98; 1578000,
            19.98; 1578600,20.02; 1579200,20.11; 1579800,20.06; 1580400,20.02; 1581000,
            20.02; 1581600,20.03; 1582200,20.05; 1582800,20.19; 1583400,20.42; 1584000,
            20.59; 1584600,20.71; 1585200,20.83; 1585800,20.91; 1586400,20.96; 1587000,
            21.07; 1587600,21.12; 1588200,21.24; 1588800,21.32; 1589400,21.36; 1590000,
            21.44; 1590600,21.56; 1591200,21.65; 1591800,21.73; 1592400,21.65; 1593000,
            21.74; 1593600,21.69; 1594200,21.75; 1594800,21.81; 1595400,21.93; 1596000,
            21.93; 1596600,21.9; 1597200,21.89; 1597800,21.85; 1598400,21.85; 1599000,
            21.86; 1599600,21.88; 1600200,22.01; 1600800,22.09; 1601400,22.17; 1602000,
            22.22; 1602600,22.22; 1603200,22.3; 1603800,22.34; 1604400,22.46; 1605000,
            22.46; 1605600,22.44; 1606200,22.46; 1606800,22.5; 1607400,22.56; 1608000,
            22.58; 1608600,22.62; 1609200,22.61; 1609800,22.54; 1610400,22.58; 1611000,
            22.62; 1611600,22.66; 1612200,22.66; 1612800,22.7; 1613400,22.62; 1614000,
            22.58; 1614600,22.54; 1615200,22.62; 1615800,22.51; 1616400,22.27; 1617000,
            22.22; 1617600,22.26; 1618200,22.29; 1618800,22.23; 1619400,22.18; 1620000,
            22.26; 1620600,22.3; 1621200,22.22; 1621800,22.1; 1622400,21.97; 1623000,
            21.89; 1623600,21.86; 1624200,21.81; 1624800,21.81; 1625400,21.81; 1626000,
            21.81; 1626600,21.74; 1627200,21.6; 1627800,21.52; 1628400,21.44; 1629000,
            21.36; 1629600,21.32; 1630200,21.32; 1630800,21.28; 1631400,21.2; 1632000,
            21.16; 1632600,21.12; 1633200,21.08; 1633800,21; 1634400,21; 1635000,20.96;
            1635600,20.87; 1636200,20.83; 1636800,20.83; 1637400,20.79; 1638000,20.75;
            1638600,20.75; 1639200,20.71; 1639800,20.68; 1640400,20.67; 1641000,20.67;
            1641600,20.63; 1642200,20.59; 1642800,20.6; 1643400,20.59; 1644000,20.55;
            1644600,20.52; 1645200,20.52; 1645800,20.51; 1646400,20.51; 1647000,20.46;
            1647600,20.46; 1648200,20.46; 1648800,20.51; 1649400,20.59; 1650000,20.63;
            1650600,20.63; 1651200,20.63; 1651800,20.63; 1652400,20.55; 1653000,20.55;
            1653600,20.51; 1654200,20.51; 1654800,20.46; 1655400,20.44; 1656000,20.38;
            1656600,20.38; 1657200,20.42; 1657800,20.42; 1658400,20.42; 1659000,20.42;
            1659600,20.38; 1660200,20.34; 1660800,20.34; 1661400,20.39; 1662000,20.46;
            1662600,20.52; 1663200,20.48; 1663800,20.42; 1664400,20.42; 1665000,20.38;
            1665600,20.38; 1666200,20.48; 1666800,20.67; 1667400,20.87; 1668000,20.96;
            1668600,21.04; 1669200,21.11; 1669800,21.24; 1670400,21.32; 1671000,21.47;
            1671600,21.56; 1672200,21.56; 1672800,21.7; 1673400,22.01; 1674000,22.26;
            1674600,22.42; 1675200,22.59; 1675800,22.7; 1676400,22.87; 1677000,22.99;
            1677600,23.07; 1678200,23.23; 1678800,23.38; 1679400,23.48; 1680000,23.6;
            1680600,23.68; 1681200,23.8; 1681800,23.84; 1682400,23.88; 1683000,23.92;
            1683600,23.9; 1684200,23.92; 1684800,23.92; 1685400,23.89; 1686000,23.88;
            1686600,23.96; 1687200,24.04; 1687800,24.12; 1688400,24.21; 1689000,24.21;
            1689600,24.18; 1690200,24.12; 1690800,24.09; 1691400,24.09; 1692000,24.09;
            1692600,24.09; 1693200,24.08; 1693800,24.09; 1694400,24.1; 1695000,24.05;
            1695600,23.96; 1696200,23.88; 1696800,23.88; 1697400,23.72; 1698000,23.76;
            1698600,23.8; 1699200,23.72; 1699800,23.62; 1700400,23.6; 1701000,23.52;
            1701600,23.52; 1702200,23.44; 1702800,23.4; 1703400,23.36; 1704000,23.36;
            1704600,23.23; 1705200,23.2; 1705800,23.15; 1706400,23.11; 1707000,23.04;
            1707600,23.03; 1708200,22.99; 1708800,22.94; 1709400,22.91; 1710000,22.84;
            1710600,22.7; 1711200,22.58; 1711800,22.46; 1712400,22.41; 1713000,22.3;
            1713600,22.22; 1714200,22.14; 1714800,22.05; 1715400,21.97; 1716000,21.89;
            1716600,21.81; 1717200,21.73; 1717800,21.68; 1718400,21.6; 1719000,21.53;
            1719600,21.5; 1720200,21.45; 1720800,21.39; 1721400,21.32; 1722000,21.28;
            1722600,21.24; 1723200,21.2; 1723800,21.16; 1724400,21.12; 1725000,21.08;
            1725600,21.04; 1726200,21; 1726800,20.96; 1727400,20.95; 1728000,20.92;
            1728600,20.87; 1729200,20.84; 1729800,20.83; 1730400,20.79; 1731000,20.79;
            1731600,20.75; 1732200,20.71; 1732800,20.72; 1733400,20.7; 1734000,20.67;
            1734600,20.65; 1735200,20.63; 1735800,20.63; 1736400,20.64; 1737000,20.59;
            1737600,20.59; 1738200,20.59; 1738800,20.55; 1739400,20.51; 1740000,20.52;
            1740600,20.51; 1741200,20.51; 1741800,20.46; 1742400,20.46; 1743000,20.46;
            1743600,20.48; 1744200,20.46; 1744800,20.42; 1745400,20.42; 1746000,20.41;
            1746600,20.38; 1747200,20.38; 1747800,20.38; 1748400,20.38; 1749000,20.34;
            1749600,20.34; 1750200,20.36; 1750800,20.34; 1751400,20.32; 1752000,20.3;
            1752600,20.3; 1753200,20.3; 1753800,20.38; 1754400,20.55; 1755000,20.67;
            1755600,20.79; 1756200,20.84; 1756800,20.87; 1757400,20.91; 1758000,21;
            1758600,21.09; 1759200,21.16; 1759800,21.24; 1760400,21.32; 1761000,21.36;
            1761600,21.45; 1762200,21.44; 1762800,21.58; 1763400,21.65; 1764000,21.78;
            1764600,21.74; 1765200,21.72; 1765800,21.81; 1766400,21.81; 1767000,21.97;
            1767600,21.94; 1768200,21.93; 1768800,22.05; 1769400,22.09; 1770000,22.17;
            1770600,22.18; 1771200,22.19; 1771800,22.09; 1772400,22.09; 1773000,22.13;
            1773600,22.14; 1774200,22.14; 1774800,22.13; 1775400,22.17; 1776000,22.14;
            1776600,22.18; 1777200,22.18; 1777800,22.22; 1778400,22.22; 1779000,22.22;
            1779600,22.23; 1780200,22.3; 1780800,22.38; 1781400,22.46; 1782000,22.5;
            1782600,22.47; 1783200,22.54; 1783800,22.58; 1784400,22.5; 1785000,22.54;
            1785600,22.59; 1786200,22.62; 1786800,22.7; 1787400,22.67; 1788000,22.66;
            1788600,22.58; 1789200,22.46; 1789800,22.39; 1790400,22.33; 1791000,22.3;
            1791600,22.22; 1792200,22.19; 1792800,22.1; 1793400,22.05; 1794000,21.98;
            1794600,21.96; 1795200,21.89; 1795800,21.81; 1796400,21.77; 1797000,21.73;
            1797600,21.65; 1798200,21.6; 1798800,21.56; 1799400,21.48; 1800000,21.44;
            1800600,21.4; 1801200,21.35; 1801800,21.28; 1802400,21.24; 1803000,21.2;
            1803600,21.16; 1804200,21.1; 1804800,21.08; 1805400,21; 1806000,21; 1806600,
            20.96; 1807200,20.91; 1807800,20.86; 1808400,20.83; 1809000,20.79; 1809600,
            20.79; 1810200,20.75; 1810800,20.71; 1811400,20.67; 1812000,20.63; 1812600,
            20.63; 1813200,20.59; 1813800,20.55; 1814400,20.51; 1815000,20.51; 1815600,
            20.48; 1816200,20.43; 1816800,20.42; 1817400,20.38; 1818000,20.34; 1818600,
            20.34; 1819200,20.3; 1819800,20.26; 1820400,20.26; 1821000,20.22; 1821600,
            20.21; 1822200,20.18; 1822800,20.15; 1823400,20.14; 1824000,20.14; 1824600,
            20.1; 1825200,20.1; 1825800,20.07; 1826400,20.06; 1827000,20.06; 1827600,
            20.02; 1828200,20.02; 1828800,20.02; 1829400,19.99; 1830000,19.98; 1830600,
            19.98; 1831200,19.97; 1831800,19.94; 1832400,19.94; 1833000,19.94; 1833600,
            19.9; 1834200,19.9; 1834800,19.9; 1835400,19.9; 1836000,19.88; 1836600,19.86;
            1837200,19.86; 1837800,19.86; 1838400,19.86; 1839000,19.82; 1839600,19.78;
            1840200,20; 1840800,20.18; 1841400,20.3; 1842000,20.46; 1842600,20.55; 1843200,
            20.67; 1843800,20.83; 1844400,20.91; 1845000,21.01; 1845600,21.16; 1846200,
            21.28; 1846800,21.4; 1847400,21.48; 1848000,21.57; 1848600,21.65; 1849200,
            21.77; 1849800,21.81; 1850400,21.93; 1851000,21.97; 1851600,22.02; 1852200,
            22.08; 1852800,22.12; 1853400,22.14; 1854000,22.26; 1854600,22.36; 1855200,
            22.42; 1855800,22.26; 1856400,22.18; 1857000,22.14; 1857600,22.1; 1858200,
            22.09; 1858800,22.1; 1859400,21.9; 1860000,22.04; 1860600,22.3; 1861200,
            22.46; 1861800,22.54; 1862400,22.58; 1863000,22.62; 1863600,22.7; 1864200,
            22.78; 1864800,23.05; 1865400,22.87; 1866000,22.91; 1866600,22.99; 1867200,
            22.99; 1867800,23.03; 1868400,23.03; 1869000,23.03; 1869600,22.95; 1870200,
            22.91; 1870800,22.87; 1871400,22.91; 1872000,22.95; 1872600,22.9; 1873200,
            22.83; 1873800,22.83; 1874400,22.78; 1875000,22.78; 1875600,22.74; 1876200,
            22.75; 1876800,22.62; 1877400,22.56; 1878000,22.46; 1878600,22.38; 1879200,
            22.3; 1879800,22.25; 1880400,22.17; 1881000,22.09; 1881600,22.01; 1882200,
            21.94; 1882800,21.85; 1883400,21.81; 1884000,21.73; 1884600,21.7; 1885200,
            21.61; 1885800,21.56; 1886400,21.49; 1887000,21.44; 1887600,21.4; 1888200,
            21.37; 1888800,21.33; 1889400,21.28; 1890000,21.21; 1890600,21.16; 1891200,
            21.12; 1891800,21.08; 1892400,21.07; 1893000,21.01; 1893600,20.97; 1894200,
            20.96; 1894800,20.91; 1895400,20.87; 1896000,20.88; 1896600,20.83; 1897200,
            20.82; 1897800,20.79; 1898400,20.75; 1899000,20.76; 1899600,20.71; 1900200,
            20.68; 1900800,20.68; 1901400,20.67; 1902000,20.65; 1902600,20.63; 1903200,
            20.61; 1903800,20.59; 1904400,20.55; 1905000,20.55; 1905600,20.54; 1906200,
            20.51; 1906800,20.51; 1907400,20.5; 1908000,20.46; 1908600,20.46; 1909200,
            20.46; 1909800,20.44; 1910400,20.42; 1911000,20.42; 1911600,20.38; 1912200,
            20.37; 1912800,20.34; 1913400,20.34; 1914000,20.34; 1914600,20.31; 1915200,
            20.3; 1915800,20.3; 1916400,20.3; 1917000,20.3; 1917600,20.27; 1918200,20.26;
            1918800,20.24; 1919400,20.23; 1920000,20.22; 1920600,20.22; 1921200,20.23;
            1921800,20.22; 1922400,20.22; 1923000,20.22; 1923600,20.23; 1924200,20.18;
            1924800,20.18; 1925400,20.18; 1926000,20.18; 1926600,20.18; 1927200,20.17;
            1927800,20.15; 1928400,20.14; 1929000,20.15; 1929600,20.14; 1930200,20.14;
            1930800,20.14; 1931400,20.15; 1932000,20.14; 1932600,20.18; 1933200,20.16;
            1933800,20.14; 1934400,20.18; 1935000,20.18; 1935600,20.18; 1936200,20.18;
            1936800,20.22; 1937400,20.22; 1938000,20.22; 1938600,20.22; 1939200,20.26;
            1939800,20.26; 1940400,20.3; 1941000,20.3; 1941600,20.34; 1942200,20.36;
            1942800,20.38; 1943400,20.42; 1944000,20.46; 1944600,20.46; 1945200,20.5;
            1945800,20.56; 1946400,20.59; 1947000,20.65; 1947600,20.67; 1948200,20.71;
            1948800,20.75; 1949400,20.81; 1950000,20.87; 1950600,20.97; 1951200,21.04;
            1951800,21.08; 1952400,21.12; 1953000,21.13; 1953600,21.17; 1954200,21.16;
            1954800,21.24; 1955400,21.3; 1956000,21.32; 1956600,21.2; 1957200,21.2;
            1957800,21.2; 1958400,21.16; 1959000,21.12; 1959600,21.08; 1960200,21.07;
            1960800,21; 1961400,20.96; 1962000,20.96; 1962600,20.91; 1963200,20.83;
            1963800,20.83; 1964400,20.8; 1965000,20.75; 1965600,20.71; 1966200,20.67;
            1966800,20.63; 1967400,20.59; 1968000,20.56; 1968600,20.55; 1969200,20.51;
            1969800,20.47; 1970400,20.46; 1971000,20.42; 1971600,20.42; 1972200,20.38;
            1972800,20.36; 1973400,20.33; 1974000,20.3; 1974600,20.26; 1975200,20.26;
            1975800,20.22; 1976400,20.21; 1977000,20.18; 1977600,20.14; 1978200,20.14;
            1978800,20.1; 1979400,20.06; 1980000,20.06; 1980600,20.02; 1981200,19.98;
            1981800,19.98; 1982400,19.94; 1983000,19.91; 1983600,19.9; 1984200,19.86;
            1984800,19.86; 1985400,19.82; 1986000,19.82; 1986600,19.78; 1987200,19.78;
            1987800,19.73; 1988400,19.7; 1989000,19.69; 1989600,19.69; 1990200,19.65;
            1990800,19.65; 1991400,19.65; 1992000,19.62; 1992600,19.59; 1993200,19.57;
            1993800,19.57; 1994400,19.57; 1995000,19.56; 1995600,19.53; 1996200,19.53;
            1996800,19.5; 1997400,19.49; 1998000,19.49; 1998600,19.48; 1999200,19.45;
            1999800,19.45; 2000400,19.45; 2001000,19.43; 2001600,19.41; 2002200,19.41;
            2002800,19.37; 2003400,19.37; 2004000,19.37; 2004600,19.37; 2005200,19.37;
            2005800,19.33; 2006400,19.34; 2007000,19.33; 2007600,19.34; 2008200,19.28;
            2008800,19.28; 2009400,19.28; 2010000,19.26; 2010600,19.25; 2011200,19.24;
            2011800,19.24; 2012400,19.22; 2013000,19.24; 2013600,19.22; 2014200,19.2;
            2014800,19.2; 2015400,19.21; 2016000,19.21; 2016600,19.2; 2017200,19.22;
            2017800,19.22; 2018400,19.24; 2019000,19.26; 2019600,19.24; 2020200,19.24;
            2020800,19.24; 2021400,19.28; 2022000,19.28; 2022600,19.34; 2023200,19.33;
            2023800,19.38; 2024400,19.38; 2025000,19.41; 2025600,19.41; 2026200,19.45;
            2026800,19.49; 2027400,19.53; 2028000,19.57; 2028600,19.61; 2029200,19.66;
            2029800,19.69; 2030400,19.73; 2031000,19.78; 2031600,19.82; 2032200,19.9;
            2032800,19.94; 2033400,19.98; 2034000,20.02; 2034600,20.1; 2035200,20.14;
            2035800,20.21; 2036400,20.5; 2037000,20.3; 2037600,20.38; 2038200,20.42;
            2038800,20.43; 2039400,20.49; 2040000,20.56; 2040600,20.63; 2041200,20.71;
            2041800,20.75; 2042400,20.79; 2043000,20.67; 2043600,20.67; 2044200,20.67;
            2044800,20.63; 2045400,20.59; 2046000,20.58; 2046600,20.55; 2047200,20.51;
            2047800,20.46; 2048400,20.42; 2049000,20.4; 2049600,20.34; 2050200,20.3;
            2050800,20.26; 2051400,20.27; 2052000,20.22; 2052600,20.18; 2053200,20.14;
            2053800,20.11; 2054400,20.06; 2055000,20.02; 2055600,20.02; 2056200,19.98;
            2056800,19.94; 2057400,19.9; 2058000,19.86; 2058600,19.86; 2059200,19.82;
            2059800,19.81; 2060400,19.78; 2061000,19.73; 2061600,19.73; 2062200,19.69;
            2062800,19.65; 2063400,19.65; 2064000,19.61; 2064600,19.59; 2065200,19.57;
            2065800,19.56; 2066400,19.53; 2067000,19.53; 2067600,19.52; 2068200,19.49;
            2068800,19.45; 2069400,19.45; 2070000,19.44; 2070600,19.41; 2071200,19.38;
            2071800,19.37; 2072400,19.37; 2073000,19.37; 2073600,19.33; 2074200,19.33;
            2074800,19.33; 2075400,19.28; 2076000,19.28; 2076600,19.26; 2077200,19.24;
            2077800,19.24; 2078400,19.24; 2079000,19.21; 2079600,19.2; 2080200,19.2;
            2080800,19.2; 2081400,19.2; 2082000,19.16; 2082600,19.16; 2083200,19.16;
            2083800,19.16; 2084400,19.12; 2085000,19.12; 2085600,19.12; 2086200,19.1;
            2086800,19.08; 2087400,19.08; 2088000,19.08; 2088600,19.04; 2089200,19.04;
            2089800,19.04; 2090400,19.04; 2091000,19; 2091600,19.04; 2092200,19.04;
            2092800,19.05; 2093400,19.04; 2094000,19.01; 2094600,19; 2095200,19; 2095800,
            19.01; 2096400,18.96; 2097000,18.96; 2097600,18.96; 2098200,18.96; 2098800,
            18.92; 2099400,19.18; 2100000,19.42; 2100600,19.53; 2101200,19.65; 2101800,
            19.81; 2102400,19.9; 2103000,20.02; 2103600,20.1; 2104200,20.22; 2104800,
            20.3; 2105400,20.42; 2106000,20.55; 2106600,20.63; 2107200,20.74; 2107800,
            20.79; 2108400,20.87; 2109000,20.98; 2109600,21.08; 2110200,21.16; 2110800,
            21.24; 2111400,21.32; 2112000,21.32; 2112600,21.44; 2113200,21.54; 2113800,
            21.65; 2114400,21.7; 2115000,21.69; 2115600,21.72; 2116200,21.73; 2116800,
            21.61; 2117400,21.68; 2118000,21.82; 2118600,21.48; 2119200,21.66; 2119800,
            21.69; 2120400,21.72; 2121000,21.76; 2121600,21.81; 2122200,21.81; 2122800,
            21.85; 2123400,21.9; 2124000,21.93; 2124600,21.97; 2125200,22.05; 2125800,
            22.14; 2126400,22.14; 2127000,22.22; 2127600,22.27; 2128200,22.34; 2128800,
            22.26; 2129400,22.38; 2130000,22.46; 2130600,22.45; 2131200,22.38; 2131800,
            22.3; 2132400,22.17; 2133000,22.05; 2133600,21.97; 2134200,21.97; 2134800,
            22.05; 2135400,22.05; 2136000,22.06; 2136600,22.09; 2137200,22.09; 2137800,
            22.13; 2138400,22.14; 2139000,22.09; 2139600,22.05; 2140200,22.09; 2140800,
            22.01; 2141400,21.93; 2142000,21.85; 2142600,21.81; 2143200,21.73; 2143800,
            21.65; 2144400,21.6; 2145000,21.52; 2145600,21.44; 2146200,21.36; 2146800,
            21.32; 2147400,21.23; 2148000,21.16; 2148600,21.08; 2149200,21.04; 2149800,
            20.96; 2150400,20.89; 2151000,20.83; 2151600,20.79; 2152200,20.75; 2152800,
            20.67; 2153400,20.63; 2154000,20.63; 2154600,20.55; 2155200,20.51; 2155800,
            20.46; 2156400,20.46; 2157000,20.38; 2157600,20.34; 2158200,20.34; 2158800,
            20.3; 2159400,20.27; 2160000,20.26; 2160600,20.22; 2161200,20.18; 2161800,
            20.19; 2162400,20.16; 2163000,20.1; 2163600,20.1; 2164200,20.09; 2164800,
            20.02; 2165400,20.02; 2166000,20.02; 2166600,19.98; 2167200,19.98; 2167800,
            19.94; 2168400,19.94; 2169000,19.9; 2169600,19.9; 2170200,19.88; 2170800,
            19.86; 2171400,19.86; 2172000,19.82; 2172600,19.82; 2173200,19.82; 2173800,
            19.82; 2174400,19.78; 2175000,19.78; 2175600,19.77; 2176200,19.74; 2176800,
            19.73; 2177400,19.73; 2178000,19.71; 2178600,19.69; 2179200,19.69; 2179800,
            19.69; 2180400,19.7; 2181000,19.69; 2181600,19.69; 2182200,19.69; 2182800,
            19.7; 2183400,19.66; 2184000,19.69; 2184600,19.69; 2185200,19.78; 2185800,
            20.02; 2186400,20.14; 2187000,20.26; 2187600,20.38; 2188200,20.5; 2188800,
            20.59; 2189400,20.67; 2190000,20.79; 2190600,20.83; 2191200,20.87; 2191800,
            20.95; 2192400,21.04; 2193000,21.16; 2193600,21.16; 2194200,21.24; 2194800,
            21.28; 2195400,21.36; 2196000,21.44; 2196600,21.52; 2197200,21.64; 2197800,
            21.77; 2198400,21.93; 2199000,22.05; 2199600,22.09; 2200200,22.22; 2200800,
            22.34; 2201400,22.3; 2202000,22.26; 2202600,22.26; 2203200,22.33; 2203800,
            22.35; 2204400,22.38; 2205000,22.38; 2205600,22.58; 2206200,22.66; 2206800,
            22.62; 2207400,22.78; 2208000,22.83; 2208600,22.87; 2209200,22.92; 2209800,
            22.84; 2210400,22.95; 2211000,22.91; 2211600,22.85; 2212200,22.78; 2212800,
            22.83; 2213400,22.82; 2214000,22.78; 2214600,22.83; 2215200,22.91; 2215800,
            22.91; 2216400,22.91; 2217000,22.91; 2217600,22.95; 2218200,22.99; 2218800,
            23.03; 2219400,22.99; 2220000,23.03; 2220600,22.99; 2221200,22.94; 2221800,
            22.87; 2222400,22.88; 2223000,22.84; 2223600,22.83; 2224200,22.7; 2224800,
            22.64; 2225400,22.54; 2226000,22.46; 2226600,22.39; 2227200,22.3; 2227800,
            22.22; 2228400,22.14; 2229000,22.05; 2229600,22.01; 2230200,21.93; 2230800,
            21.85; 2231400,21.77; 2232000,21.73; 2232600,21.65; 2233200,21.61; 2233800,
            21.52; 2234400,21.44; 2235000,21.4; 2235600,21.36; 2236200,21.28; 2236800,
            21.24; 2237400,21.2; 2238000,21.12; 2238600,21.08; 2239200,21.04; 2239800,
            21; 2240400,20.96; 2241000,20.91; 2241600,20.87; 2242200,20.83; 2242800,
            20.81; 2243400,20.78; 2244000,20.75; 2244600,20.71; 2245200,20.68; 2245800,
            20.68; 2246400,20.67; 2247000,20.63; 2247600,20.6; 2248200,20.56; 2248800,
            20.55; 2249400,20.51; 2250000,20.51; 2250600,20.46; 2251200,20.46; 2251800,
            20.42; 2252400,20.42; 2253000,20.38; 2253600,20.36; 2254200,20.34; 2254800,
            20.3; 2255400,20.3; 2256000,20.28; 2256600,20.26; 2257200,20.26; 2257800,
            20.26; 2258400,20.26; 2259000,20.22; 2259600,20.23; 2260200,20.22; 2260800,
            20.22; 2261400,20.22; 2262000,20.18; 2262600,20.18; 2263200,20.18; 2263800,
            20.18; 2264400,20.18; 2265000,20.18; 2265600,20.18; 2266200,20.14; 2266800,
            20.14; 2267400,20.14; 2268000,20.14; 2268600,20.14; 2269200,20.14; 2269800,
            20.15; 2270400,20.13; 2271000,20.26; 2271600,20.46; 2272200,20.64; 2272800,
            20.75; 2273400,20.87; 2274000,20.96; 2274600,21.04; 2275200,21.2; 2275800,
            21.37; 2276400,21.44; 2277000,21.6; 2277600,21.73; 2278200,21.85; 2278800,
            21.93; 2279400,21.93; 2280000,21.93; 2280600,22.02; 2281200,22.09; 2281800,
            22.09; 2282400,22.13; 2283000,22.26; 2283600,22.26; 2284200,22.26; 2284800,
            22.31; 2285400,22.41; 2286000,22.45; 2286600,22.62; 2287200,22.59; 2287800,
            22.58; 2288400,22.58; 2289000,22.46; 2289600,22.46; 2290200,22.72; 2290800,
            22.58; 2291400,22.7; 2292000,22.83; 2292600,23; 2293200,23.12; 2293800,23.15;
            2294400,23.19; 2295000,23.32; 2295600,23.28; 2296200,23.39; 2296800,23.44;
            2297400,23.52; 2298000,23.55; 2298600,23.6; 2299200,23.68; 2299800,23.68;
            2300400,23.68; 2301000,23.72; 2301600,23.73; 2302200,23.68; 2302800,23.68;
            2303400,23.68; 2304000,23.72; 2304600,23.72; 2305200,23.68; 2305800,23.57;
            2306400,23.52; 2307000,23.48; 2307600,23.44; 2308200,23.4; 2308800,23.36;
            2309400,23.31; 2310000,23.28; 2310600,23.27; 2311200,23.27; 2311800,23.15;
            2312400,23.11; 2313000,23.03; 2313600,22.95; 2314200,22.87; 2314800,22.78;
            2315400,22.7; 2316000,22.62; 2316600,22.5; 2317200,22.42; 2317800,22.34;
            2318400,22.26; 2319000,22.23; 2319600,22.14; 2320200,22.09; 2320800,22.01;
            2321400,21.93; 2322000,21.89; 2322600,21.81; 2323200,21.73; 2323800,21.69;
            2324400,21.6; 2325000,21.58; 2325600,21.52; 2326200,21.48; 2326800,21.4;
            2327400,21.38; 2328000,21.32; 2328600,21.3; 2329200,21.25; 2329800,21.2;
            2330400,21.16; 2331000,21.12; 2331600,21.12; 2332200,21.09; 2332800,21.04;
            2333400,21; 2334000,21.21; 2334600,20.97; 2335200,20.92; 2335800,20.89;
            2336400,20.87; 2337000,20.83; 2337600,20.83; 2338200,20.79; 2338800,20.79;
            2339400,20.78; 2340000,20.75; 2340600,20.74; 2341200,20.71; 2341800,20.71;
            2342400,20.67; 2343000,20.67; 2343600,20.67; 2344200,20.67; 2344800,20.63;
            2345400,20.63; 2346000,20.63; 2346600,20.61; 2347200,20.59; 2347800,20.59;
            2348400,20.59; 2349000,20.59; 2349600,20.58; 2350200,20.55; 2350800,20.55;
            2351400,20.55; 2352000,20.55; 2352600,20.55; 2353200,20.54; 2353800,20.51;
            2354400,20.51; 2355000,20.51; 2355600,20.51; 2356200,20.51; 2356800,20.51;
            2357400,20.51; 2358000,20.72; 2358600,20.8; 2359200,20.96; 2359800,21.08;
            2360400,21.2; 2361000,21.32; 2361600,21.44; 2362200,21.57; 2362800,21.68;
            2363400,21.69; 2364000,21.72; 2364600,21.77; 2365200,21.85; 2365800,21.89;
            2366400,21.97; 2367000,22.02; 2367600,22.09; 2368200,22.14; 2368800,22.18;
            2369400,22.26; 2370000,22.26; 2370600,22.3; 2371200,22.3; 2371800,22.38;
            2372400,22.47; 2373000,22.56; 2373600,22.57; 2374200,22.66; 2374800,22.68;
            2375400,22.7; 2376000,22.72; 2376600,22.66; 2377200,22.66; 2377800,22.7;
            2378400,22.7; 2379000,22.74; 2379600,22.88; 2380200,23.03; 2380800,23.16;
            2381400,23.15; 2382000,23.17; 2382600,23.32; 2383200,23.44; 2383800,23.28;
            2384400,23.07; 2385000,23.03; 2385600,22.95; 2386200,22.91; 2386800,23;
            2387400,23.07; 2388000,23.03; 2388600,23.03; 2389200,22.99; 2389800,22.99;
            2390400,23.12; 2391000,23.03; 2391600,22.91; 2392200,23.14; 2392800,23.27;
            2393400,23.32; 2394000,23.4; 2394600,23.44; 2395200,23.4; 2395800,23.37;
            2396400,23.27; 2397000,23.19; 2397600,23.11; 2398200,23.03; 2398800,22.95;
            2399400,22.91; 2400000,22.83; 2400600,22.74; 2401200,22.7; 2401800,22.64;
            2402400,22.58; 2403000,22.5; 2403600,22.46; 2404200,22.38; 2404800,22.34;
            2405400,22.26; 2406000,22.22; 2406600,22.19; 2407200,22.11; 2407800,22.09;
            2408400,22.01; 2409000,21.98; 2409600,21.93; 2410200,21.89; 2410800,21.85;
            2411400,21.81; 2412000,21.78; 2412600,21.73; 2413200,21.72; 2413800,21.65;
            2414400,21.65; 2415000,21.6; 2415600,21.6; 2416200,21.56; 2416800,21.53;
            2417400,21.52; 2418000,21.48; 2418600,21.48; 2419200,21.48; 2419800,21.46;
            2420400,21.44; 2421000,21.41; 2421600,21.4; 2422200,21.36; 2422800,21.36;
            2423400,21.36; 2424000,21.32; 2424600,21.32; 2425200,21.32; 2425800,21.32;
            2426400,21.32; 2427000,21.28; 2427600,21.24; 2428200,21.24; 2428800,21.24;
            2429400,21.23; 2430000,21.22; 2430600,21.23; 2431200,21.24; 2431800,21.2;
            2432400,21.2; 2433000,21.21; 2433600,21.2; 2434200,21.2; 2434800,21.2; 2435400,
            21.2; 2436000,21.2; 2436600,21.2; 2437200,21.16; 2437800,21.16; 2438400,
            21.18; 2439000,21.16; 2439600,21.16; 2440200,21.16; 2440800,21.16; 2441400,
            21.16; 2442000,21.16; 2442600,21.16; 2443200,21.15; 2443800,21.19; 2444400,
            21.32; 2445000,21.44; 2445600,21.52; 2446200,21.64; 2446800,21.73; 2447400,
            21.81; 2448000,21.93; 2448600,22.09; 2449200,22.18; 2449800,22.29; 2450400,
            22.38; 2451000,22.46; 2451600,22.5; 2452200,22.58; 2452800,22.7; 2453400,
            22.74; 2454000,22.78; 2454600,22.87; 2455200,22.95; 2455800,23.02; 2456400,
            23.07; 2457000,23.11; 2457600,23.19; 2458200,23.19; 2458800,23.2; 2459400,
            23.26; 2460000,23.27; 2460600,23.27; 2461200,23.27; 2461800,23.23; 2462400,
            23.31; 2463000,23.32; 2463600,23.32; 2464200,23.32; 2464800,23.32; 2465400,
            23.32; 2466000,23.4; 2466600,23.36; 2467200,23.35; 2467800,23.47; 2468400,
            23.48; 2469000,23.44; 2469600,23.48; 2470200,23.56; 2470800,23.6; 2471400,
            23.65; 2472000,23.64; 2472600,23.64; 2473200,23.74; 2473800,23.77; 2474400,
            23.64; 2475000,23.64; 2475600,23.68; 2476200,23.73; 2476800,23.68; 2477400,
            23.68; 2478000,23.68; 2478600,23.68; 2479200,23.71; 2479800,23.6; 2480400,
            23.61; 2481000,23.72; 2481600,23.74; 2482200,23.73; 2482800,23.71; 2483400,
            23.56; 2484000,23.48; 2484600,23.4; 2485200,23.32; 2485800,23.27; 2486400,
            23.15; 2487000,23.11; 2487600,23.03; 2488200,22.95; 2488800,22.9; 2489400,
            22.8; 2490000,22.74; 2490600,22.7; 2491200,22.62; 2491800,22.54; 2492400,
            22.47; 2493000,22.43; 2493600,22.34; 2494200,22.27; 2494800,22.22; 2495400,
            22.18; 2496000,22.09; 2496600,22.05; 2497200,22.01; 2497800,21.97; 2498400,
            21.89; 2499000,21.85; 2499600,21.81; 2500200,21.8; 2500800,21.76; 2501400,
            21.73; 2502000,21.69; 2502600,21.65; 2503200,21.64; 2503800,21.6; 2504400,
            21.58; 2505000,21.53; 2505600,21.52; 2506200,21.51; 2506800,21.48; 2507400,
            21.48; 2508000,21.44; 2508600,21.42; 2509200,21.4; 2509800,21.4; 2510400,
            21.37; 2511000,21.36; 2511600,21.32; 2512200,21.32; 2512800,21.32; 2513400,
            21.28; 2514000,21.28; 2514600,21.24; 2515200,21.24; 2515800,21.21; 2516400,
            21.2; 2517000,21.19; 2517600,21.16; 2518200,21.15; 2518800,21.12; 2519400,
            21.12; 2520000,21.12; 2520600,21.08; 2521200,21.08; 2521800,21.07; 2522400,
            21.04; 2523000,21.05; 2523600,21.01; 2524200,21; 2524800,20.98; 2525400,
            20.96; 2526000,20.96; 2526600,20.92; 2527200,20.91; 2527800,20.92; 2528400,
            20.88; 2529000,20.87; 2529600,20.87; 2530200,20.83; 2530800,20.83; 2531400,
            20.83; 2532000,20.83; 2532600,20.83; 2533200,20.8; 2533800,20.8; 2534400,
            20.79; 2535000,20.79; 2535600,20.79; 2536200,20.79; 2536800,20.8; 2537400,
            20.8; 2538000,20.79; 2538600,20.84; 2539200,20.83; 2539800,20.83; 2540400,
            20.83; 2541000,20.87; 2541600,20.87; 2542200,20.91; 2542800,20.96; 2543400,
            21; 2544000,21; 2544600,21; 2545200,21.04; 2545800,21.04; 2546400,21.08;
            2547000,21.12; 2547600,21.12; 2548200,21.12; 2548800,21.12; 2549400,21.16;
            2550000,21.16; 2550600,21.2; 2551200,21.28; 2551800,21.32; 2552400,21.4;
            2553000,21.44; 2553600,21.49; 2554200,21.52; 2554800,21.56; 2555400,21.63;
            2556000,21.64; 2556600,21.65; 2557200,21.74; 2557800,21.71; 2558400,21.69;
            2559000,21.69; 2559600,21.68; 2560200,21.65; 2560800,21.6; 2561400,21.56;
            2562000,21.56; 2562600,21.52; 2563200,21.5; 2563800,21.48; 2564400,21.44;
            2565000,21.44; 2565600,21.43; 2566200,21.4; 2566800,21.38; 2567400,21.32;
            2568000,21.32; 2568600,21.32; 2569200,21.31; 2569800,21.28; 2570400,21.28;
            2571000,21.27; 2571600,21.24; 2572200,21.2; 2572800,21.2; 2573400,21.2;
            2574000,21.16; 2574600,21.16; 2575200,21.12; 2575800,21.12; 2576400,21.08;
            2577000,21.07; 2577600,21.04; 2578200,21.03; 2578800,21; 2579400,21; 2580000,
            20.96; 2580600,20.96; 2581200,20.96; 2581800,20.91; 2582400,20.91; 2583000,
            20.91; 2583600,20.87; 2584200,20.87; 2584800,20.83; 2585400,20.83; 2586000,
            20.83; 2586600,20.83; 2587200,20.79; 2587800,20.79; 2588400,20.79; 2589000,
            20.76; 2589600,20.75; 2590200,20.76; 2590800,20.71; 2591400,20.71; 2592000,
            20.68; 2592600,20.67; 2593200,20.67; 2593800,20.67; 2594400,20.63; 2595000,
            20.63; 2595600,20.63; 2596200,20.63; 2596800,20.63; 2597400,20.64; 2598000,
            20.6; 2598600,20.59; 2599200,20.59; 2599800,20.57; 2600400,20.55; 2601000,
            20.55; 2601600,20.55; 2602200,20.55; 2602800,20.55; 2603400,20.54; 2604000,
            20.51; 2604600,20.51; 2605200,20.51; 2605800,20.51; 2606400,20.51; 2607000,
            20.51; 2607600,20.5; 2608200,20.52; 2608800,20.5; 2609400,20.49; 2610000,
            20.46; 2610600,20.46; 2611200,20.48; 2611800,20.46; 2612400,20.46; 2613000,
            20.46; 2613600,20.46; 2614200,20.46; 2614800,20.46; 2615400,20.46; 2616000,
            20.43; 2616600,20.42; 2617200,20.44; 2617800,20.44; 2618400,20.44; 2619000,
            20.42; 2619600,20.42; 2620200,20.42; 2620800,20.44; 2621400,20.42; 2622000,
            20.42; 2622600,20.42; 2623200,20.42; 2623800,20.42; 2624400,20.42; 2625000,
            20.42; 2625600,20.45; 2626200,20.46; 2626800,20.46; 2627400,20.46; 2628000,
            20.46; 2628600,20.46; 2629200,20.46; 2629800,20.46; 2630400,20.46; 2631000,
            20.5; 2631600,20.5; 2632200,20.46; 2632800,20.46; 2633400,20.46; 2634000,
            20.46; 2634600,20.46; 2635200,20.46; 2635800,20.46; 2636400,20.46; 2637000,
            20.43; 2637600,20.46; 2638200,20.46; 2638800,20.46; 2639400,20.46; 2640000,
            20.46; 2640600,20.46; 2641200,20.46; 2641800,20.46; 2642400,20.46; 2643000,
            20.46; 2643600,20.46; 2644200,20.46; 2644800,20.46; 2645400,20.45; 2646000,
            20.42; 2646600,20.42; 2647200,20.42; 2647800,20.42; 2648400,20.42; 2649000,
            20.42; 2649600,20.42; 2650200,20.41; 2650800,20.42; 2651400,20.38; 2652000,
            20.38; 2652600,20.38; 2653200,20.4; 2653800,20.34; 2654400,20.34; 2655000,
            20.34; 2655600,20.34; 2656200,20.34; 2656800,20.34; 2657400,20.35; 2658000,
            20.32; 2658600,20.33; 2659200,20.3; 2659800,20.31; 2660400,20.32; 2661000,
            20.32; 2661600,20.3; 2662200,20.3; 2662800,20.3; 2663400,20.3; 2664000,20.3;
            2664600,20.3; 2665200,20.3; 2665800,20.3; 2666400,20.3; 2667000,20.3; 2667600,
            20.3; 2668200,20.3; 2668800,20.3; 2669400,20.32; 2670000,20.3; 2670600,20.3;
            2671200,20.31; 2671800,20.3; 2672400,20.3; 2673000,20.3; 2673600,20.3; 2674200,
            20.33; 2674800,20.32; 2675400,20.34; 2676000,20.34; 2676600,20.33; 2677200,
            20.36; 2677800,20.34; 2678400,20.34; 2679000,20.34; 2679600,20.34; 2680200,
            20.34; 2680800,20.34; 2681400,20.34; 2682000,20.34; 2682600,20.34; 2683200,
            20.34; 2683800,20.34; 2684400,20.34; 2685000,20.34; 2685600,20.34; 2686200,
            20.34; 2686800,20.34; 2687400,20.34; 2688000,20.34; 2688600,20.34; 2689200,
            20.34; 2689800,20.34; 2690400,20.34; 2691000,20.34; 2691600,20.34; 2692200,
            20.34; 2692800,20.34; 2693400,20.34; 2694000,20.34; 2694600,20.34; 2695200,
            20.34; 2695800,20.35; 2696400,20.38; 2697000,20.38; 2697600,20.38; 2698200,
            20.38; 2698800,20.39; 2699400,20.42; 2700000,20.42; 2700600,20.42; 2701200,
            20.42; 2701800,20.42; 2702400,20.39; 2703000,20.55; 2703600,20.75; 2704200,
            20.87; 2704800,21.01; 2705400,21.12; 2706000,21.2; 2706600,21.32; 2707200,
            21.43; 2707800,21.58; 2708400,21.64; 2709000,21.6; 2709600,21.65; 2710200,
            21.69; 2710800,21.78; 2711400,21.93; 2712000,22.01; 2712600,22.05; 2713200,
            22.14; 2713800,22.17; 2714400,22.29; 2715000,22.34; 2715600,22.41; 2716200,
            22.47; 2716800,22.7; 2717400,22.82; 2718000,22.91; 2718600,22.95; 2719200,
            22.99; 2719800,23; 2720400,22.91; 2721000,22.83; 2721600,22.62; 2722200,
            21.8; 2722800,22.16; 2723400,22.14; 2724000,22.42; 2724600,22.54; 2725200,
            22.58; 2725800,22.72; 2726400,22.8; 2727000,22.79; 2727600,22.91; 2728200,
            23.02; 2728800,23.15; 2729400,23.23; 2730000,23.27; 2730600,23.32; 2731200,
            23.27; 2731800,23.39; 2732400,23.44; 2733000,23.49; 2733600,23.52; 2734200,
            23.57; 2734800,23.6; 2735400,23.6; 2736000,23.6; 2736600,23.52; 2737200,
            23.52; 2737800,23.4; 2738400,23.27; 2739000,23.24; 2739600,23.27; 2740200,
            23.24; 2740800,23.23; 2741400,23.19; 2742000,23.16; 2742600,23.15; 2743200,
            23.12; 2743800,23.11; 2744400,23.07; 2745000,22.99; 2745600,22.91; 2746200,
            22.8; 2746800,22.74; 2747400,22.66; 2748000,22.58; 2748600,22.5; 2749200,
            22.42; 2749800,22.38; 2750400,22.3; 2751000,22.26; 2751600,22.18; 2752200,
            22.09; 2752800,22.05; 2753400,22.02; 2754000,21.93; 2754600,21.89; 2755200,
            21.84; 2755800,21.77; 2756400,21.73; 2757000,21.69; 2757600,21.65; 2758200,
            21.6; 2758800,21.56; 2759400,21.52; 2760000,21.48; 2760600,21.44; 2761200,
            21.4; 2761800,21.36; 2762400,21.32; 2763000,21.32; 2763600,21.28; 2764200,
            21.28; 2764800,21.24; 2765400,21.2; 2766000,21.2; 2766600,21.16; 2767200,
            21.12; 2767800,21.12; 2768400,21.12; 2769000,21.08; 2769600,21.09; 2770200,
            21.07; 2770800,21.04; 2771400,21.04; 2772000,21.01; 2772600,21; 2773200,
            20.99; 2773800,20.96; 2774400,20.96; 2775000,20.96; 2775600,20.96; 2776200,
            20.96; 2776800,20.95; 2777400,20.95; 2778000,20.94; 2778600,20.92; 2779200,
            20.92; 2779800,20.91; 2780400,20.91; 2781000,20.92; 2781600,20.91; 2782200,
            20.91; 2782800,20.91; 2783400,20.9; 2784000,20.89; 2784600,20.87; 2785200,
            20.88; 2785800,20.87; 2786400,20.87; 2787000,20.87; 2787600,20.88; 2788200,
            20.87; 2788800,20.87; 2789400,20.96; 2790000,21.16; 2790600,21.3; 2791200,
            21.36; 2791800,21.44; 2792400,21.49; 2793000,21.65; 2793600,21.81; 2794200,
            21.86; 2794800,21.93; 2795400,21.97; 2796000,22.06; 2796600,22.14; 2797200,
            22.24; 2797800,22.3; 2798400,22.38; 2799000,22.44; 2799600,22.5; 2800200,
            22.55; 2800800,22.62; 2801400,22.66; 2802000,22.64; 2802600,22.78; 2803200,
            22.87; 2803800,22.91; 2804400,22.95; 2805000,23.07; 2805600,23.08; 2806200,
            23.07; 2806800,23.11; 2807400,23.11; 2808000,23.15; 2808600,23.15; 2809200,
            23.31; 2809800,23.4; 2810400,23.55; 2811000,23.6; 2811600,23.68; 2812200,
            23.69; 2812800,23.72; 2813400,23.75; 2814000,23.8; 2814600,23.8; 2815200,
            23.84; 2815800,23.9; 2816400,23.92; 2817000,23.91; 2817600,24.01; 2818200,
            24.01; 2818800,23.94; 2819400,23.88; 2820000,23.84; 2820600,23.88; 2821200,
            23.94; 2821800,24.01; 2822400,24; 2823000,24.02; 2823600,24.02; 2824200,
            23.96; 2824800,24.01; 2825400,23.96; 2826000,23.85; 2826600,23.72; 2827200,
            23.72; 2827800,23.67; 2828400,23.71; 2829000,23.68; 2829600,23.56; 2830200,
            23.44; 2830800,23.4; 2831400,23.32; 2832000,23.23; 2832600,23.15; 2833200,
            23.11; 2833800,23.03; 2834400,22.95; 2835000,22.91; 2835600,22.82; 2836200,
            22.75; 2836800,22.69; 2837400,22.62; 2838000,22.54; 2838600,22.5; 2839200,
            22.42; 2839800,22.39; 2840400,22.3; 2841000,22.26; 2841600,22.22; 2842200,
            22.18; 2842800,22.1; 2843400,22.05; 2844000,22.01; 2844600,21.97; 2845200,
            21.94; 2845800,21.89; 2846400,21.85; 2847000,21.81; 2847600,21.8; 2848200,
            21.73; 2848800,21.74; 2849400,21.69; 2850000,21.65; 2850600,21.64; 2851200,
            21.6; 2851800,21.6; 2852400,21.56; 2853000,21.56; 2853600,21.52; 2854200,
            21.5; 2854800,21.48; 2855400,21.44; 2856000,21.44; 2856600,21.46; 2857200,
            21.42; 2857800,21.4; 2858400,21.4; 2859000,21.38; 2859600,21.36; 2860200,
            21.37; 2860800,21.32; 2861400,21.32; 2862000,21.32; 2862600,21.3; 2863200,
            21.28; 2863800,21.28; 2864400,21.26; 2865000,21.25; 2865600,21.24; 2866200,
            21.24; 2866800,21.22; 2867400,21.2; 2868000,21.2; 2868600,21.2; 2869200,
            21.16; 2869800,21.16; 2870400,21.16; 2871000,21.16; 2871600,21.16; 2872200,
            21.16; 2872800,21.16; 2873400,21.16; 2874000,21.17; 2874600,21.16; 2875200,
            21.16; 2875800,21.16; 2876400,21.16; 2877000,21.36; 2877600,21.52; 2878200,
            21.65; 2878800,21.77; 2879400,21.89; 2880000,21.97; 2880600,22.01; 2881200,
            22.08; 2881800,22.22; 2882400,22.3; 2883000,22.42; 2883600,22.49; 2884200,
            22.5; 2884800,22.54; 2885400,22.62; 2886000,22.62; 2886600,22.74; 2887200,
            22.78; 2887800,22.88; 2888400,22.87; 2889000,22.92; 2889600,22.95; 2890200,
            23; 2890800,23.03; 2891400,23.11; 2892000,23.11; 2892600,23.11; 2893200,
            23.15; 2893800,23.16; 2894400,23.15; 2895000,23.16; 2895600,23.19; 2896200,
            23.19; 2896800,23.15; 2897400,23.11; 2898000,23.11; 2898600,23.03; 2899200,
            23.06; 2899800,23.19; 2900400,23.23; 2901000,23.2; 2901600,23.25; 2902200,
            23.28; 2902800,23.2; 2903400,23.36; 2904000,23.4; 2904600,23.4; 2905200,
            23.44; 2905800,23.44; 2906400,23.48; 2907000,23.47; 2907600,23.48; 2908200,
            23.48; 2908800,23.52; 2909400,23.57; 2910000,23.55; 2910600,23.53; 2911200,
            23.49; 2911800,23.52; 2912400,23.56; 2913000,23.53; 2913600,23.44; 2914200,
            23.36; 2914800,23.27; 2915400,23.23; 2916000,23.23; 2916600,23.19; 2917200,
            23.14; 2917800,23.11; 2918400,23.03; 2919000,22.92; 2919600,22.83; 2920200,
            22.74; 2920800,22.64; 2921400,22.58; 2922000,22.5; 2922600,22.42; 2923200,
            22.38; 2923800,22.3; 2924400,22.22; 2925000,22.14; 2925600,22.08; 2926200,
            22.01; 2926800,21.97; 2927400,21.89; 2928000,21.81; 2928600,21.77; 2929200,
            21.73; 2929800,21.65; 2930400,21.61; 2931000,21.6; 2931600,21.56; 2932200,
            21.48; 2932800,21.46; 2933400,21.4; 2934000,21.4; 2934600,21.36; 2935200,
            21.32; 2935800,21.28; 2936400,21.28; 2937000,21.24; 2937600,21.2; 2938200,
            21.16; 2938800,21.16; 2939400,21.12; 2940000,21.12; 2940600,21.08; 2941200,
            21.08; 2941800,21.07; 2942400,21.04; 2943000,21; 2943600,21; 2944200,20.99;
            2944800,20.96; 2945400,20.96; 2946000,20.94; 2946600,20.91; 2947200,20.91;
            2947800,20.88; 2948400,20.87; 2949000,20.88; 2949600,20.87; 2950200,20.87;
            2950800,20.83; 2951400,20.84; 2952000,20.83; 2952600,20.83; 2953200,20.83;
            2953800,20.83; 2954400,20.83; 2955000,20.8; 2955600,20.79; 2956200,20.82;
            2956800,20.79; 2957400,20.8; 2958000,20.79; 2958600,20.79; 2959200,20.79;
            2959800,20.79; 2960400,20.8; 2961000,20.79; 2961600,20.76; 2962200,20.75;
            2962800,20.75; 2963400,20.99; 2964000,21.16; 2964600,21.44; 2965200,21.52;
            2965800,21.56; 2966400,21.65; 2967000,21.65; 2967600,21.69; 2968200,21.77;
            2968800,21.81; 2969400,21.93; 2970000,22.09; 2970600,22.18; 2971200,22.26;
            2971800,22.39; 2972400,22.5; 2973000,22.58; 2973600,22.62; 2974200,22.7;
            2974800,22.78; 2975400,22.82; 2976000,22.86; 2976600,22.94; 2977200,23.03;
            2977800,23.03; 2978400,23.12; 2979000,23.15; 2979600,23.15; 2980200,23.15;
            2980800,23.11; 2981400,23.07; 2982000,23.03; 2982600,23.03; 2983200,23.08;
            2983800,23.18; 2984400,23.23; 2985000,23.24; 2985600,23.23; 2986200,23.2;
            2986800,23.24; 2987400,23.32; 2988000,23.4; 2988600,23.4; 2989200,23.4;
            2989800,23.48; 2990400,23.56; 2991000,23.55; 2991600,23.56; 2992200,23.56;
            2992800,23.6; 2993400,23.57; 2994000,23.56; 2994600,23.56; 2995200,23.6;
            2995800,23.64; 2996400,23.64; 2997000,23.6; 2997600,23.56; 2998200,23.52;
            2998800,23.48; 2999400,23.4; 3000000,23.27; 3000600,23.16; 3001200,23.11;
            3001800,23.03; 3002400,22.95; 3003000,22.9; 3003600,22.83; 3004200,22.78;
            3004800,22.7; 3005400,22.62; 3006000,22.58; 3006600,22.5; 3007200,22.42;
            3007800,22.38; 3008400,22.3; 3009000,22.26; 3009600,22.22; 3010200,22.14;
            3010800,22.09; 3011400,22.05; 3012000,22.01; 3012600,21.94; 3013200,21.89;
            3013800,21.85; 3014400,21.81; 3015000,21.78; 3015600,21.73; 3016200,21.69;
            3016800,21.65; 3017400,21.6; 3018000,21.56; 3018600,21.52; 3019200,21.52;
            3019800,21.48; 3020400,21.44; 3021000,21.42; 3021600,21.4; 3022200,21.36;
            3022800,21.33; 3023400,21.32; 3024000,21.32; 3024600,21.29; 3025200,21.24;
            3025800,21.24; 3026400,21.2; 3027000,21.2; 3027600,21.2; 3028200,21.16;
            3028800,21.16; 3029400,21.13; 3030000,21.12; 3030600,21.12; 3031200,21.13;
            3031800,21.08; 3032400,21.08; 3033000,21.08; 3033600,21.08; 3034200,21.07;
            3034800,21.04; 3035400,21.04; 3036000,21.05; 3036600,21.04; 3037200,21.04;
            3037800,21.04; 3038400,21.03; 3039000,21.02; 3039600,21; 3040200,21; 3040800,
            21; 3041400,21; 3042000,21; 3042600,20.99; 3043200,21; 3043800,20.98; 3044400,
            20.96; 3045000,20.96; 3045600,20.96; 3046200,20.97; 3046800,20.96; 3047400,
            20.96; 3048000,20.96; 3048600,20.95; 3049200,20.92; 3049800,20.91; 3050400,
            20.96; 3051000,21.12; 3051600,21.28; 3052200,21.4; 3052800,21.56; 3053400,
            21.65; 3054000,21.6; 3054600,21.77; 3055200,21.93; 3055800,22.01; 3056400,
            22.09; 3057000,22.18; 3057600,22.26; 3058200,22.38; 3058800,22.46; 3059400,
            22.58; 3060000,22.62; 3060600,22.72; 3061200,22.78; 3061800,22.83; 3062400,
            22.91; 3063000,22.94; 3063600,23.04; 3064200,23.07; 3064800,23.12; 3065400,
            23.15; 3066000,23.11; 3066600,23.07; 3067200,23.07; 3067800,23.19; 3068400,
            23.27; 3069000,23.32; 3069600,23.36; 3070200,22.91; 3070800,19.82; 3071400,
            20.14; 3072000,19.17; 3072600,21.07; 3073200,22.02; 3073800,22.38; 3074400,
            22.58; 3075000,22.74; 3075600,22.83; 3076200,22.91; 3076800,23; 3077400,
            23.02; 3078000,23.07; 3078600,23.08; 3079200,23.15; 3079800,23.19; 3080400,
            23.19; 3081000,23.19; 3081600,23.15; 3082200,23.12; 3082800,23.19; 3083400,
            23.19; 3084000,23.15; 3084600,23.11; 3085200,23.11; 3085800,23.11; 3086400,
            23.11; 3087000,23.15; 3087600,23.16; 3088200,23.19; 3088800,23.16; 3089400,
            23.08; 3090000,23.03; 3090600,22.95; 3091200,22.91; 3091800,22.83; 3092400,
            22.78; 3093000,22.7; 3093600,22.66; 3094200,22.59; 3094800,22.5; 3095400,
            22.46; 3096000,22.38; 3096600,22.34; 3097200,22.27; 3097800,22.22; 3098400,
            22.14; 3099000,22.09; 3099600,22.05; 3100200,21.97; 3100800,21.93; 3101400,
            21.88; 3102000,21.84; 3102600,21.81; 3103200,21.73; 3103800,21.74; 3104400,
            21.65; 3105000,21.63; 3105600,21.6; 3106200,21.58; 3106800,21.52; 3107400,
            21.5; 3108000,21.44; 3108600,21.4; 3109200,21.36; 3109800,21.36; 3110400,
            21.32; 3111000,21.29; 3111600,21.25; 3112200,21.25; 3112800,21.2; 3113400,
            21.16; 3114000,21.12; 3114600,21.12; 3115200,21.1; 3115800,21.08; 3116400,
            21.08; 3117000,21.04; 3117600,21; 3118200,21; 3118800,20.97; 3119400,20.95;
            3120000,20.91; 3120600,20.88; 3121200,20.87; 3121800,20.86; 3122400,20.83;
            3123000,20.83; 3123600,20.79; 3124200,20.79; 3124800,20.75; 3125400,20.75;
            3126000,20.71; 3126600,20.71; 3127200,20.67; 3127800,20.67; 3128400,20.67;
            3129000,20.63; 3129600,20.63; 3130200,20.63; 3130800,20.59; 3131400,20.58;
            3132000,20.55; 3132600,20.55; 3133200,20.55; 3133800,20.53; 3134400,20.51;
            3135000,20.51; 3135600,20.51; 3136200,20.51; 3136800,20.51; 3137400,20.52;
            3138000,20.46; 3138600,20.46; 3139200,20.46; 3139800,20.5; 3140400,20.46;
            3141000,20.48; 3141600,20.52; 3142200,20.51; 3142800,20.51; 3143400,20.54;
            3144000,20.55; 3144600,20.55; 3145200,20.6; 3145800,20.59; 3146400,20.63;
            3147000,20.63; 3147600,20.67; 3148200,20.67; 3148800,20.71; 3149400,20.74;
            3150000,20.75; 3150600,20.79; 3151200,20.83; 3151800,20.86; 3152400,20.87;
            3153000,20.91; 3153600,20.95; 3154200,20.97; 3154800,21.04; 3155400,21.08;
            3156000,21.12; 3156600,21.16; 3157200,21.2; 3157800,21.28; 3158400,21.33;
            3159000,21.4; 3159600,21.48; 3160200,21.56; 3160800,21.6; 3161400,21.62;
            3162000,21.69; 3162600,21.73; 3163200,21.77; 3163800,21.81; 3164400,21.85;
            3165000,21.89; 3165600,21.93; 3166200,21.92; 3166800,21.89; 3167400,21.88;
            3168000,21.89; 3168600,21.85; 3169200,21.81; 3169800,21.8; 3170400,21.73;
            3171000,21.7; 3171600,21.65; 3172200,21.64; 3172800,21.6; 3173400,21.53;
            3174000,21.5; 3174600,21.48; 3175200,21.46; 3175800,21.4; 3176400,21.36;
            3177000,21.32; 3177600,21.28; 3178200,21.28; 3178800,21.24; 3179400,21.21;
            3180000,21.16; 3180600,21.13; 3181200,21.12; 3181800,21.08; 3182400,21.05;
            3183000,21; 3183600,20.98; 3184200,20.96; 3184800,20.92; 3185400,20.91;
            3186000,20.88; 3186600,20.83; 3187200,20.83; 3187800,20.79; 3188400,20.76;
            3189000,20.76; 3189600,20.73; 3190200,20.71; 3190800,20.67; 3191400,20.67;
            3192000,20.67; 3192600,20.63; 3193200,20.61; 3193800,20.59; 3194400,20.6;
            3195000,20.55; 3195600,20.55; 3196200,20.51; 3196800,20.51; 3197400,20.51;
            3198000,20.5; 3198600,20.5; 3199200,20.47; 3199800,20.46; 3200400,20.46;
            3201000,20.42; 3201600,20.42; 3202200,20.42; 3202800,20.42; 3203400,20.39;
            3204000,20.38; 3204600,20.38; 3205200,20.38; 3205800,20.38; 3206400,20.36;
            3207000,20.34; 3207600,20.34; 3208200,20.34; 3208800,20.36; 3209400,20.31;
            3210000,20.3; 3210600,20.3; 3211200,20.3; 3211800,20.3; 3212400,20.32; 3213000,
            20.3; 3213600,20.31; 3214200,20.29; 3214800,20.31; 3215400,20.3; 3216000,
            20.3; 3216600,20.28; 3217200,20.26; 3217800,20.26; 3218400,20.26; 3219000,
            20.26; 3219600,20.26; 3220200,20.26; 3220800,20.26; 3221400,20.26; 3222000,
            20.26; 3222600,20.26; 3223200,20.26; 3223800,20.26; 3224400,20.26; 3225000,
            20.26; 3225600,20.26; 3226200,20.26; 3226800,20.26; 3227400,20.26; 3228000,
            20.27; 3228600,20.3; 3229200,20.3; 3229800,20.31; 3230400,20.3; 3231000,
            20.3; 3231600,20.3; 3232200,20.3; 3232800,20.32; 3233400,20.3; 3234000,20.3;
            3234600,20.3; 3235200,20.3; 3235800,20.3; 3236400,20.3; 3237000,20.31; 3237600,
            20.31; 3238200,20.26; 3238800,20.26; 3239400,20.26; 3240000,20.26; 3240600,
            20.24; 3241200,20.22; 3241800,20.22; 3242400,20.22; 3243000,20.22; 3243600,
            20.18; 3244200,20.19; 3244800,20.22; 3245400,20.18; 3246000,20.18; 3246600,
            20.18; 3247200,20.18; 3247800,20.18; 3248400,20.18; 3249000,20.18; 3249600,
            20.18; 3250200,20.2; 3250800,20.22; 3251400,20.2; 3252000,20.18; 3252600,
            20.18; 3253200,20.18; 3253800,20.18; 3254400,20.19; 3255000,20.18; 3255600,
            20.18; 3256200,20.14; 3256800,20.14; 3257400,20.14; 3258000,20.15; 3258600,
            20.13; 3259200,20.1; 3259800,20.1; 3260400,20.1; 3261000,20.1; 3261600,20.1;
            3262200,20.06; 3262800,20.06; 3263400,20.05; 3264000,20.06; 3264600,20.03;
            3265200,20.02; 3265800,20.02; 3266400,20.02; 3267000,20.02; 3267600,20;
            3268200,19.99; 3268800,19.98; 3269400,19.98; 3270000,19.98; 3270600,19.98;
            3271200,19.99; 3271800,19.94; 3272400,19.94; 3273000,19.9; 3273600,19.91;
            3274200,19.9; 3274800,19.9; 3275400,19.9; 3276000,19.89; 3276600,19.86;
            3277200,19.86; 3277800,19.86; 3278400,19.86; 3279000,19.82; 3279600,19.82;
            3280200,19.82; 3280800,19.78; 3281400,19.81; 3282000,19.78; 3282600,19.78;
            3283200,19.78; 3283800,19.75; 3284400,19.73; 3285000,19.73; 3285600,19.7;
            3286200,19.69; 3286800,19.69; 3287400,19.7; 3288000,19.67; 3288600,19.66;
            3289200,19.65; 3289800,19.65; 3290400,19.65; 3291000,19.61; 3291600,19.61;
            3292200,19.57; 3292800,19.57; 3293400,19.55; 3294000,19.53; 3294600,19.53;
            3295200,19.53; 3295800,19.53; 3296400,19.53; 3297000,19.53; 3297600,19.5;
            3298200,19.5; 3298800,19.49; 3299400,19.49; 3300000,19.49; 3300600,19.49;
            3301200,19.49; 3301800,19.49; 3302400,19.49; 3303000,19.49; 3303600,19.49;
            3304200,19.5; 3304800,19.49; 3305400,19.49; 3306000,19.49; 3306600,19.49;
            3307200,19.49; 3307800,19.46; 3308400,19.7; 3309000,19.9; 3309600,20.08;
            3310200,20.34; 3310800,20.75; 3311400,20.87; 3312000,21; 3312600,21.08;
            3313200,21.12; 3313800,21.15; 3314400,21.28; 3315000,21.33; 3315600,21.44;
            3316200,21.52; 3316800,21.6; 3317400,21.69; 3318000,21.74; 3318600,21.76;
            3319200,21.84; 3319800,21.93; 3320400,21.97; 3321000,21.97; 3321600,22.01;
            3322200,22.01; 3322800,22.02; 3323400,22.05; 3324000,22.01; 3324600,21.97;
            3325200,21.97; 3325800,21.97; 3326400,21.97; 3327000,21.96; 3327600,21.85;
            3328200,21.77; 3328800,21.85; 3329400,21.85; 3330000,21.9; 3330600,21.97;
            3331200,22.14; 3331800,22.06; 3332400,22.08; 3333000,22.18; 3333600,22.18;
            3334200,22.18; 3334800,22.27; 3335400,22.3; 3336000,22.36; 3336600,22.42;
            3337200,22.46; 3337800,22.46; 3338400,22.5; 3339000,22.5; 3339600,22.54;
            3340200,22.58; 3340800,22.55; 3341400,22.42; 3342000,22.46; 3342600,22.38;
            3343200,22.31; 3343800,22.26; 3344400,22.18; 3345000,22.15; 3345600,22.09;
            3346200,22.09; 3346800,22.05; 3347400,22; 3348000,21.97; 3348600,21.93;
            3349200,21.89; 3349800,21.85; 3350400,21.81; 3351000,21.81; 3351600,21.77;
            3352200,21.69; 3352800,21.65; 3353400,21.65; 3354000,21.6; 3354600,21.6;
            3355200,21.53; 3355800,21.48; 3356400,21.44; 3357000,21.4; 3357600,21.36;
            3358200,21.32; 3358800,21.28; 3359400,21.24; 3360000,21.2; 3360600,21.16;
            3361200,21.12; 3361800,21.08; 3362400,21.08; 3363000,21; 3363600,20.98;
            3364200,20.96; 3364800,20.96; 3365400,20.91; 3366000,20.91; 3366600,20.87;
            3367200,20.83; 3367800,20.83; 3368400,20.83; 3369000,20.79; 3369600,20.79;
            3370200,20.75; 3370800,20.75; 3371400,20.75; 3372000,20.71; 3372600,20.71;
            3373200,20.67; 3373800,20.67; 3374400,20.67; 3375000,20.67; 3375600,20.66;
            3376200,20.63; 3376800,20.63; 3377400,20.59; 3378000,20.59; 3378600,20.59;
            3379200,20.55; 3379800,20.55; 3380400,20.51; 3381000,20.53; 3381600,20.53;
            3382200,20.51; 3382800,20.5; 3383400,20.5; 3384000,20.46; 3384600,20.46;
            3385200,20.46; 3385800,20.42; 3386400,20.42; 3387000,20.42; 3387600,20.42;
            3388200,20.42; 3388800,20.42; 3389400,20.42; 3390000,20.42; 3390600,20.44;
            3391200,20.42; 3391800,20.42; 3392400,20.39; 3393000,20.42; 3393600,20.42;
            3394200,20.42; 3394800,20.55; 3395400,20.79; 3396000,20.91; 3396600,21.04;
            3397200,21.16; 3397800,21.25; 3398400,21.36; 3399000,21.4; 3399600,21.45;
            3400200,21.46; 3400800,21.44; 3401400,21.52; 3402000,21.58; 3402600,21.6;
            3403200,21.6; 3403800,21.64; 3404400,21.69; 3405000,21.69; 3405600,21.77;
            3406200,21.77; 3406800,21.81; 3407400,21.85; 3408000,21.81; 3408600,21.85;
            3409200,21.85; 3409800,21.85; 3410400,21.89; 3411000,21.9; 3411600,21.9;
            3412200,21.93; 3412800,21.93; 3413400,21.94; 3414000,21.93; 3414600,21.98;
            3415200,22.05; 3415800,22.22; 3416400,22.18; 3417000,22.38; 3417600,22.46;
            3418200,22.54; 3418800,22.62; 3419400,22.66; 3420000,22.74; 3420600,22.78;
            3421200,22.78; 3421800,22.78; 3422400,22.91; 3423000,22.99; 3423600,22.95;
            3424200,22.99; 3424800,23.03; 3425400,23.15; 3426000,23.11; 3426600,23.06;
            3427200,23.07; 3427800,23.11; 3428400,23.11; 3429000,23.15; 3429600,23.16;
            3430200,23.19; 3430800,23.19; 3431400,23.15; 3432000,23.2; 3432600,23.23;
            3433200,23.15; 3433800,23.15; 3434400,23.2; 3435000,23.17; 3435600,23.11;
            3436200,23.03; 3436800,22.91; 3437400,22.8; 3438000,22.67; 3438600,22.58;
            3439200,22.5; 3439800,22.42; 3440400,22.34; 3441000,22.26; 3441600,22.22;
            3442200,22.13; 3442800,22.06; 3443400,21.97; 3444000,21.93; 3444600,21.85;
            3445200,21.81; 3445800,21.76; 3446400,21.69; 3447000,21.64; 3447600,21.59;
            3448200,21.54; 3448800,21.48; 3449400,21.44; 3450000,21.4; 3450600,21.36;
            3451200,21.32; 3451800,21.28; 3452400,21.22; 3453000,21.2; 3453600,21.2;
            3454200,21.16; 3454800,21.12; 3455400,21.11; 3456000,21.08; 3456600,21.04;
            3457200,21.04; 3457800,21.01; 3458400,20.98; 3459000,20.96; 3459600,20.96;
            3460200,20.91; 3460800,20.9; 3461400,20.87; 3462000,20.87; 3462600,20.84;
            3463200,20.83; 3463800,20.82; 3464400,20.79; 3465000,20.79; 3465600,20.78;
            3466200,20.75; 3466800,20.75; 3467400,20.76; 3468000,20.75; 3468600,20.71;
            3469200,20.67; 3469800,20.68; 3470400,20.69; 3471000,20.67; 3471600,20.67;
            3472200,20.67; 3472800,20.68; 3473400,20.67; 3474000,20.63; 3474600,20.64;
            3475200,20.63; 3475800,20.63; 3476400,20.63; 3477000,20.63; 3477600,20.63;
            3478200,20.63; 3478800,20.63; 3479400,20.62; 3480000,20.62; 3480600,20.63;
            3481200,20.58; 3481800,20.83; 3482400,20.96; 3483000,21.12; 3483600,21.2;
            3484200,21.28; 3484800,21.36; 3485400,21.4; 3486000,21.48; 3486600,21.52;
            3487200,21.6; 3487800,21.65; 3488400,21.72; 3489000,21.76; 3489600,21.81;
            3490200,21.85; 3490800,21.85; 3491400,21.94; 3492000,22; 3492600,21.97;
            3493200,21.98; 3493800,22.05; 3494400,22.05; 3495000,22.09; 3495600,22.14;
            3496200,22.22; 3496800,22.22; 3497400,22.18; 3498000,22.22; 3498600,22.3;
            3499200,22.34; 3499800,22.34; 3500400,22.26; 3501000,22.34; 3501600,22.38;
            3502200,22.42; 3502800,22.46; 3503400,22.46; 3504000,22.42; 3504600,22.47;
            3505200,22.47; 3505800,22.49; 3506400,22.5; 3507000,22.54; 3507600,22.58;
            3508200,22.58; 3508800,22.63; 3509400,22.58; 3510000,22.58; 3510600,22.55;
            3511200,22.54; 3511800,22.6; 3512400,22.54; 3513000,22.62; 3513600,22.74;
            3514200,22.74; 3514800,22.83; 3515400,22.78; 3516000,22.87; 3516600,22.84;
            3517200,22.78; 3517800,22.78; 3518400,22.74; 3519000,22.78; 3519600,22.74;
            3520200,22.74; 3520800,22.74; 3521400,22.74; 3522000,22.7; 3522600,22.65;
            3523200,22.64; 3523800,22.52; 3524400,22.42; 3525000,22.34; 3525600,22.26;
            3526200,22.22; 3526800,22.14; 3527400,22.05; 3528000,21.97; 3528600,21.93;
            3529200,21.85; 3529800,21.81; 3530400,21.73; 3531000,21.66; 3531600,21.64;
            3532200,21.56; 3532800,21.48; 3533400,21.44; 3534000,21.4; 3534600,21.36;
            3535200,21.32; 3535800,21.28; 3536400,21.24; 3537000,21.2; 3537600,21.16;
            3538200,21.13; 3538800,21.12; 3539400,21.04; 3540000,21.02; 3540600,20.99;
            3541200,20.96; 3541800,20.92; 3542400,20.89; 3543000,20.87; 3543600,20.83;
            3544200,20.83; 3544800,20.79; 3545400,20.78; 3546000,20.75; 3546600,20.76;
            3547200,20.71; 3547800,20.67; 3548400,20.67; 3549000,20.67; 3549600,20.63;
            3550200,20.63; 3550800,20.59; 3551400,20.59; 3552000,20.59; 3552600,20.55;
            3553200,20.55; 3553800,20.55; 3554400,20.55; 3555000,20.51; 3555600,20.51;
            3556200,20.51; 3556800,20.51; 3557400,20.5; 3558000,20.49; 3558600,20.46;
            3559200,20.46; 3559800,20.46; 3560400,20.43; 3561000,20.44; 3561600,20.42;
            3562200,20.42; 3562800,20.42; 3563400,20.42; 3564000,20.42; 3564600,20.38;
            3565200,20.4; 3565800,20.38; 3566400,20.38; 3567000,20.38; 3567600,20.51;
            3568200,21.07; 3568800,21.36; 3569400,21.6; 3570000,21.73; 3570600,21.86;
            3571200,21.89; 3571800,21.81; 3572400,21.85; 3573000,21.89; 3573600,21.96;
            3574200,22.1; 3574800,22.18; 3575400,22.26; 3576000,22.34; 3576600,22.3;
            3577200,22.26; 3577800,22.34; 3578400,22.43; 3579000,22.54; 3579600,22.5;
            3580200,22.61; 3580800,22.7; 3581400,22.78; 3582000,22.82; 3582600,22.87;
            3583200,22.83; 3583800,22.74; 3584400,22.75; 3585000,22.83; 3585600,22.83;
            3586200,22.83; 3586800,22.87; 3587400,23.03; 3588000,23.11; 3588600,23.22;
            3589200,23.27; 3589800,23.2; 3590400,23.27; 3591000,23.27; 3591600,23.31;
            3592200,23.32; 3592800,23.27; 3593400,23.27; 3594000,23.27; 3594600,23.23;
            3595200,23.23; 3595800,23.23; 3596400,23.23; 3597000,23.24; 3597600,23.27;
            3598200,23.27; 3598800,23.44; 3599400,23.61; 3600000,23.76; 3600600,23.88;
            3601200,23.92; 3601800,23.96; 3602400,23.94; 3603000,23.92; 3603600,23.84;
            3604200,23.76; 3604800,23.76; 3605400,23.76; 3606000,23.76; 3606600,23.76;
            3607200,23.7; 3607800,23.72; 3608400,23.72; 3609000,23.68; 3609600,23.68;
            3610200,23.6; 3610800,23.6; 3611400,23.56; 3612000,23.48; 3612600,23.4;
            3613200,23.36; 3613800,23.27; 3614400,23.11; 3615000,23.03; 3615600,22.95;
            3616200,22.95; 3616800,22.88; 3617400,22.83; 3618000,22.76; 3618600,22.7;
            3619200,22.62; 3619800,22.54; 3620400,22.51; 3621000,22.42; 3621600,22.34;
            3622200,22.3; 3622800,22.26; 3623400,22.22; 3624000,22.19; 3624600,22.14;
            3625200,22.09; 3625800,22.31; 3626400,22.05; 3627000,22.01; 3627600,21.97;
            3628200,21.97; 3628800,21.93; 3629400,21.93; 3630000,21.9; 3630600,21.85;
            3631200,21.85; 3631800,21.81; 3632400,21.81; 3633000,21.77; 3633600,21.74;
            3634200,21.77; 3634800,21.73; 3635400,21.74; 3636000,21.89; 3636600,22.05;
            3637200,22.18; 3637800,22.26; 3638400,22.38; 3639000,22.46; 3639600,22.54;
            3640200,22.63; 3640800,22.62; 3641400,22.66; 3642000,22.66; 3642600,22.62;
            3643200,22.62; 3643800,22.64; 3644400,22.62; 3645000,22.62; 3645600,22.62;
            3646200,22.62; 3646800,22.66; 3647400,22.66; 3648000,22.66; 3648600,22.67;
            3649200,22.62; 3649800,22.64; 3650400,22.62; 3651000,22.62; 3651600,22.66;
            3652200,22.74; 3652800,22.78; 3653400,22.75; 3654000,22.74; 3654600,22.84;
            3655200,22.96; 3655800,23.03; 3656400,23.11; 3657000,23.15; 3657600,23.16;
            3658200,23.32; 3658800,23.35; 3659400,23.4; 3660000,23.48; 3660600,23.56;
            3661200,23.68; 3661800,23.72; 3662400,23.8; 3663000,23.81; 3663600,23.84;
            3664200,23.89; 3664800,23.84; 3665400,23.88; 3666000,23.88; 3666600,23.92;
            3667200,23.94; 3667800,23.92; 3668400,23.96; 3669000,23.88; 3669600,23.8;
            3670200,23.77; 3670800,23.77; 3671400,23.76; 3672000,23.73; 3672600,23.72;
            3673200,23.74; 3673800,23.76; 3674400,23.78; 3675000,23.8; 3675600,23.86;
            3676200,23.84; 3676800,23.88; 3677400,23.86; 3678000,23.88; 3678600,23.88;
            3679200,23.92; 3679800,23.92; 3680400,23.96; 3681000,23.93; 3681600,24.01;
            3682200,24.01; 3682800,24.1; 3683400,24.09; 3684000,24.21; 3684600,24.33;
            3685200,24.41; 3685800,24.41; 3686400,24.45; 3687000,24.46; 3687600,24.45;
            3688200,24.37; 3688800,24.37; 3689400,24.37; 3690000,24.41; 3690600,24.49;
            3691200,24.5; 3691800,24.54; 3692400,24.57; 3693000,24.58; 3693600,24.58;
            3694200,24.58; 3694800,24.58; 3695400,24.58; 3696000,24.58; 3696600,24.58;
            3697200,24.58; 3697800,24.58; 3698400,24.54; 3699000,24.45; 3699600,24.37;
            3700200,24.21; 3700800,24.05; 3701400,23.92; 3702000,23.8; 3702600,23.72;
            3703200,23.66; 3703800,23.56; 3704400,23.48; 3705000,23.4; 3705600,23.3;
            3706200,23.19; 3706800,23.11; 3707400,23.07; 3708000,23.03; 3708600,22.96;
            3709200,22.91; 3709800,22.83; 3710400,22.78; 3711000,22.74; 3711600,22.7;
            3712200,22.66; 3712800,22.58; 3713400,22.5; 3714000,22.46; 3714600,22.42;
            3715200,22.38; 3715800,22.34; 3716400,22.3; 3717000,22.26; 3717600,22.22;
            3718200,22.18; 3718800,22.14; 3719400,22.1; 3720000,22.09; 3720600,22.03;
            3721200,22.01; 3721800,21.97; 3722400,21.95; 3723000,21.94; 3723600,21.89;
            3724200,21.85; 3724800,21.81; 3725400,21.81; 3726000,21.79; 3726600,21.77;
            3727200,21.73; 3727800,21.73; 3728400,21.69; 3729000,21.69; 3729600,21.65;
            3730200,21.66; 3730800,21.6; 3731400,21.6; 3732000,21.6; 3732600,21.56;
            3733200,21.56; 3733800,21.52; 3734400,21.48; 3735000,21.48; 3735600,21.44;
            3736200,21.45; 3736800,21.4; 3737400,21.4; 3738000,21.4; 3738600,21.36;
            3739200,21.32; 3739800,21.28; 3740400,21.24; 3741000,21.2; 3741600,21.2;
            3742200,21.16; 3742800,21.17; 3743400,21.12; 3744000,21.13; 3744600,21.12;
            3745200,21.12; 3745800,21.08; 3746400,21.08; 3747000,21.04; 3747600,21.05;
            3748200,21.04; 3748800,21; 3749400,21; 3750000,21; 3750600,20.99; 3751200,
            20.96; 3751800,20.91; 3752400,20.91; 3753000,20.91; 3753600,20.88; 3754200,
            20.91; 3754800,20.91; 3755400,20.96; 3756000,20.99; 3756600,21; 3757200,
            21; 3757800,21.04; 3758400,21.04; 3759000,21.08; 3759600,21.12; 3760200,
            21.16; 3760800,21.25; 3761400,21.28; 3762000,21.28; 3762600,21.24; 3763200,
            21.24; 3763800,21.28; 3764400,21.28; 3765000,21.25; 3765600,21.2; 3766200,
            21.21; 3766800,21.2; 3767400,21.21; 3768000,21.2; 3768600,21.2; 3769200,
            21.2; 3769800,21.2; 3770400,21.2; 3771000,21.2; 3771600,21.2; 3772200,21.16;
            3772800,21.12; 3773400,21.12; 3774000,21.12; 3774600,21.08; 3775200,21.08;
            3775800,21.08; 3776400,21.07; 3777000,21.09; 3777600,21.04; 3778200,21.04;
            3778800,21.03; 3779400,21.02; 3780000,21.04; 3780600,21; 3781200,21; 3781800,
            21; 3782400,21; 3783000,20.99; 3783600,20.96; 3784200,20.98; 3784800,20.97;
            3785400,20.98; 3786000,20.96; 3786600,20.96; 3787200,20.96; 3787800,20.96;
            3788400,20.96; 3789000,20.97; 3789600,20.96; 3790200,20.96; 3790800,20.96;
            3791400,20.93; 3792000,20.92; 3792600,20.92; 3793200,20.91; 3793800,20.91;
            3794400,20.92; 3795000,20.91; 3795600,20.91; 3796200,20.91; 3796800,20.87;
            3797400,20.87; 3798000,20.87; 3798600,20.87; 3799200,20.88; 3799800,20.94;
            3800400,20.98; 3801000,21; 3801600,21.04; 3802200,21.05; 3802800,21.08;
            3803400,21.12; 3804000,21.12; 3804600,21.15; 3805200,21.16; 3805800,21.16;
            3806400,21.21; 3807000,21.2; 3807600,21.2; 3808200,21.2; 3808800,21.24;
            3809400,21.24; 3810000,21.28; 3810600,21.32; 3811200,21.36; 3811800,21.36;
            3812400,21.36; 3813000,21.4; 3813600,21.4; 3814200,21.4; 3814800,21.44;
            3815400,21.45; 3816000,21.44; 3816600,21.44; 3817200,21.44; 3817800,21.47;
            3818400,21.48; 3819000,21.49; 3819600,21.48; 3820200,21.48; 3820800,21.48;
            3821400,21.48; 3822000,21.48; 3822600,21.52; 3823200,21.52; 3823800,21.52;
            3824400,21.54; 3825000,21.49; 3825600,21.52; 3826200,21.52; 3826800,21.52;
            3827400,21.52; 3828000,21.52; 3828600,21.52; 3829200,21.52; 3829800,21.52;
            3830400,21.52; 3831000,21.48; 3831600,21.44; 3832200,21.44; 3832800,21.4;
            3833400,21.44; 3834000,21.44; 3834600,21.44; 3835200,21.44; 3835800,21.44;
            3836400,21.48; 3837000,21.48; 3837600,21.54; 3838200,21.56; 3838800,21.6;
            3839400,21.6; 3840000,21.64; 3840600,21.65; 3841200,21.67; 3841800,21.73;
            3842400,21.77; 3843000,21.82; 3843600,21.85; 3844200,21.89; 3844800,21.97;
            3845400,22.06; 3846000,22.05; 3846600,22.05; 3847200,22.05; 3847800,22.06;
            3848400,22.1; 3849000,22.14; 3849600,22.13; 3850200,22.14; 3850800,22.14;
            3851400,22.17; 3852000,22.18; 3852600,22.18; 3853200,22.21; 3853800,22.32;
            3854400,22.36; 3855000,22.42; 3855600,22.46; 3856200,22.46; 3856800,22.42;
            3857400,22.42; 3858000,22.42; 3858600,22.42; 3859200,22.42; 3859800,22.42;
            3860400,22.41; 3861000,22.38; 3861600,22.34; 3862200,22.34; 3862800,22.3;
            3863400,22.26; 3864000,22.26; 3864600,22.24; 3865200,22.22; 3865800,22.18;
            3866400,22.18; 3867000,22.14; 3867600,22.14; 3868200,22.13; 3868800,22.09;
            3869400,22.09; 3870000,22.09; 3870600,22.09; 3871200,22.05; 3871800,22.05;
            3872400,22.03; 3873000,22.01; 3873600,22.01; 3874200,21.97; 3874800,21.97;
            3875400,21.97; 3876000,21.97; 3876600,21.93; 3877200,21.93; 3877800,21.93;
            3878400,21.93; 3879000,21.93; 3879600,21.89; 3880200,21.89; 3880800,21.88;
            3881400,21.85; 3882000,21.85; 3882600,21.85; 3883200,21.85; 3883800,21.84;
            3884400,21.81; 3885000,21.81; 3885600,21.81; 3886200,21.81; 3886800,21.81;
            3887400,21.81; 3888000,21.81; 3888600,21.77; 3889200,21.79; 3889800,21.77;
            3890400,21.78; 3891000,21.77; 3891600,21.77; 3892200,21.77; 3892800,21.77;
            3893400,21.77; 3894000,21.73; 3894600,21.77; 3895200,21.75; 3895800,21.76;
            3896400,21.77; 3897000,21.77; 3897600,21.76; 3898200,21.78; 3898800,21.76;
            3899400,21.73; 3900000,21.73; 3900600,21.77; 3901200,21.76; 3901800,21.77;
            3902400,21.73; 3903000,21.73; 3903600,21.89; 3904200,22.18; 3904800,22.3;
            3905400,22.46; 3906000,22.66; 3906600,22.88; 3907200,22.95; 3907800,22.99;
            3908400,23; 3909000,23.03; 3909600,23.07; 3910200,23.11; 3910800,23.11;
            3911400,23.15; 3912000,23.19; 3912600,23.23; 3913200,23.28; 3913800,23.31;
            3914400,23.27; 3915000,23.23; 3915600,23.28; 3916200,23.33; 3916800,23.4;
            3917400,23.44; 3918000,23.48; 3918600,23.56; 3919200,23.6; 3919800,23.69;
            3920400,23.72; 3921000,23.76; 3921600,23.8; 3922200,23.84; 3922800,23.92;
            3923400,23.94; 3924000,23.96; 3924600,24.01; 3925200,24.05; 3925800,24.09;
            3926400,24.09; 3927000,24.09; 3927600,24.09; 3928200,24.05; 3928800,24.1;
            3929400,24.09; 3930000,24.09; 3930600,24.09; 3931200,24.09; 3931800,24.13;
            3932400,24.13; 3933000,24.17; 3933600,24.17; 3934200,24.17; 3934800,24.2;
            3935400,24.21; 3936000,24.22; 3936600,24.21; 3937200,24.22; 3937800,24.25;
            3938400,24.3; 3939000,24.21; 3939600,24.25; 3940200,24.21; 3940800,24.21;
            3941400,24.17; 3942000,24.17; 3942600,24.17; 3943200,24.16; 3943800,24.14;
            3944400,24.13; 3945000,24.13; 3945600,24.13; 3946200,24.13; 3946800,24.06;
            3947400,24.01; 3948000,23.96; 3948600,23.96; 3949200,23.96; 3949800,23.92;
            3950400,23.88; 3951000,23.88; 3951600,23.88; 3952200,23.84; 3952800,23.8;
            3953400,23.8; 3954000,23.76; 3954600,23.72; 3955200,23.68; 3955800,23.68;
            3956400,23.65; 3957000,23.64; 3957600,23.54; 3958200,23.4; 3958800,23.28;
            3959400,23.18; 3960000,23.07; 3960600,22.99; 3961200,22.91; 3961800,22.79;
            3962400,22.7; 3963000,22.58; 3963600,22.52; 3964200,22.46; 3964800,22.42;
            3965400,22.34; 3966000,22.3; 3966600,22.22; 3967200,22.19; 3967800,22.14;
            3968400,22.09; 3969000,22.05; 3969600,22.01; 3970200,21.97; 3970800,21.94;
            3971400,21.89; 3972000,21.89; 3972600,21.85; 3973200,21.81; 3973800,21.81;
            3974400,21.79; 3975000,21.77; 3975600,21.77; 3976200,21.73; 3976800,21.69;
            3977400,21.69; 3978000,21.69; 3978600,21.65; 3979200,21.65; 3979800,21.63;
            3980400,21.65; 3981000,21.65; 3981600,21.65; 3982200,21.65; 3982800,21.65;
            3983400,21.65; 3984000,21.65; 3984600,21.65; 3985200,21.65; 3985800,21.65;
            3986400,21.65; 3987000,21.65; 3987600,21.66; 3988200,21.65; 3988800,21.65;
            3989400,21.65; 3990000,21.72; 3990600,21.94; 3991200,22.09; 3991800,22.26;
            3992400,22.42; 3993000,22.54; 3993600,22.6; 3994200,22.58; 3994800,22.6;
            3995400,22.58; 3996000,22.62; 3996600,22.62; 3997200,22.62; 3997800,22.64;
            3998400,22.66; 3999000,22.74; 3999600,22.77; 4000200,22.78; 4000800,22.83;
            4001400,22.91; 4002000,23.03; 4002600,23.15; 4003200,23.2; 4003800,23.27;
            4004400,23.4; 4005000,23.48; 4005600,23.56; 4006200,23.64; 4006800,23.72;
            4007400,23.72; 4008000,23.8; 4008600,23.88; 4009200,23.95; 4009800,24.01;
            4010400,24.05; 4011000,24.1; 4011600,24.17; 4012200,24.21; 4012800,24.21;
            4013400,24.25; 4014000,24.25; 4014600,24.25; 4015200,24.25; 4015800,24.21;
            4016400,24.21; 4017000,24.21; 4017600,24.21; 4018200,24.22; 4018800,24.21;
            4019400,24.21; 4020000,24.21; 4020600,24.13; 4021200,24.21; 4021800,24.25;
            4022400,24.25; 4023000,24.3; 4023600,24.34; 4024200,24.34; 4024800,24.41;
            4025400,24.62; 4026000,24.74; 4026600,24.86; 4027200,24.9; 4027800,24.9;
            4028400,24.9; 4029000,24.9; 4029600,24.86; 4030200,24.86; 4030800,24.85;
            4031400,24.89; 4032000,24.86; 4032600,24.87; 4033200,24.78; 4033800,24.71;
            4034400,24.66; 4035000,24.58; 4035600,24.57; 4036200,24.54; 4036800,24.5;
            4037400,24.45; 4038000,24.45; 4038600,24.44; 4039200,24.41; 4039800,24.41;
            4040400,24.41; 4041000,24.38; 4041600,24.37; 4042200,24.33; 4042800,24.33;
            4043400,24.3; 4044000,24.22; 4044600,24.14; 4045200,24.05; 4045800,23.92;
            4046400,23.84; 4047000,23.72; 4047600,23.64; 4048200,23.52; 4048800,23.44;
            4049400,23.36; 4050000,23.36; 4050600,23.27; 4051200,23.25; 4051800,23.19;
            4052400,23.15; 4053000,23.11; 4053600,23.07; 4054200,23.03; 4054800,23.03;
            4055400,22.95; 4056000,22.95; 4056600,22.91; 4057200,22.92; 4057800,22.91;
            4058400,22.87; 4059000,22.86; 4059600,22.83; 4060200,22.83; 4060800,22.78;
            4061400,22.74; 4062000,22.74; 4062600,22.74; 4063200,22.74; 4063800,22.72;
            4064400,22.7; 4065000,22.7; 4065600,22.7; 4066200,22.7; 4066800,22.66; 4067400,
            22.66; 4068000,22.66; 4068600,22.68; 4069200,22.66; 4069800,22.66; 4070400,
            22.62; 4071000,22.64; 4071600,22.62; 4072200,22.62; 4072800,22.62; 4073400,
            22.62; 4074000,22.62; 4074600,22.62; 4075200,22.62; 4075800,22.62; 4076400,
            22.66; 4077000,22.91; 4077600,23.12; 4078200,23.36; 4078800,23.56; 4079400,
            23.69; 4080000,23.68; 4080600,23.6; 4081200,23.56; 4081800,23.44; 4082400,
            23.4; 4083000,23.39; 4083600,23.4; 4084200,23.48; 4084800,23.51; 4085400,
            23.56; 4086000,23.56; 4086600,23.58; 4087200,23.56; 4087800,23.48; 4088400,
            23.4; 4089000,23.36; 4089600,23.36; 4090200,23.4; 4090800,23.48; 4091400,
            23.58; 4092000,23.64; 4092600,23.68; 4093200,23.7; 4093800,23.68; 4094400,
            23.68; 4095000,23.68; 4095600,23.72; 4096200,23.72; 4096800,23.72; 4097400,
            23.72; 4098000,23.77; 4098600,23.8; 4099200,23.8; 4099800,23.84; 4100400,
            23.88; 4101000,23.92; 4101600,23.9; 4102200,23.84; 4102800,23.8; 4103400,
            23.85; 4104000,23.96; 4104600,23.96; 4105200,23.96; 4105800,23.94; 4106400,
            24; 4107000,24.01; 4107600,24.01; 4108200,24.05; 4108800,24.05; 4109400,
            24.06; 4110000,24.05; 4110600,24.06; 4111200,24.08; 4111800,24.01; 4112400,
            24.04; 4113000,24.05; 4113600,24.09; 4114200,24.12; 4114800,24.12; 4115400,
            24.08; 4116000,24.09; 4116600,24.09; 4117200,24.13; 4117800,24.13; 4118400,
            24.13; 4119000,24.12; 4119600,24.09; 4120200,24.01; 4120800,23.96; 4121400,
            23.92; 4122000,23.92; 4122600,23.84; 4123200,23.81; 4123800,23.76; 4124400,
            23.76; 4125000,23.68; 4125600,23.68; 4126200,23.6; 4126800,23.6; 4127400,
            23.56; 4128000,23.57; 4128600,23.52; 4129200,23.49; 4129800,23.48; 4130400,
            23.44; 4131000,23.41; 4131600,23.36; 4132200,23.35; 4132800,23.32; 4133400,
            23.27; 4134000,23.16; 4134600,23.07; 4135200,23.03; 4135800,22.99; 4136400,
            22.94; 4137000,22.94; 4137600,22.91; 4138200,22.88; 4138800,22.83; 4139400,
            22.78; 4140000,22.78; 4140600,22.74; 4141200,22.74; 4141800,22.7; 4142400,
            22.66; 4143000,22.63; 4143600,22.64; 4144200,22.62; 4144800,22.58; 4145400,
            22.58; 4146000,22.54; 4146600,22.5; 4147200,22.5; 4147800,22.46; 4148400,
            22.46; 4149000,22.46; 4149600,22.42; 4150200,22.42; 4150800,22.42; 4151400,
            22.42; 4152000,22.38; 4152600,22.38; 4153200,22.42; 4153800,22.42; 4154400,
            22.42; 4155000,22.46; 4155600,22.46; 4156200,22.46; 4156800,22.46; 4157400,
            22.48; 4158000,22.5; 4158600,22.5; 4159200,22.5; 4159800,22.5; 4160400,22.5;
            4161000,22.5; 4161600,22.5; 4162200,22.5; 4162800,22.67; 4163400,22.91;
            4164000,23.15; 4164600,23.23; 4165200,23.27; 4165800,23.23; 4166400,23.15;
            4167000,23.03; 4167600,22.96; 4168200,22.87; 4168800,22.87; 4169400,22.95;
            4170000,23.03; 4170600,23.12; 4171200,23.11; 4171800,23.11; 4172400,23.1;
            4173000,23.07; 4173600,23.11; 4174200,23.11; 4174800,23.15; 4175400,23.15;
            4176000,23.2; 4176600,23.28; 4177200,23.4; 4177800,23.48; 4178400,23.52;
            4179000,23.56; 4179600,23.68; 4180200,23.68; 4180800,23.76; 4181400,23.8;
            4182000,23.92; 4182600,23.92; 4183200,23.96; 4183800,24.04; 4184400,24.01;
            4185000,24.12; 4185600,24.13; 4186200,24.13; 4186800,24.09; 4187400,24.17;
            4188000,24.2; 4188600,24.17; 4189200,24.17; 4189800,24.13; 4190400,24.13;
            4191000,24.05; 4191600,24.09; 4192200,24.13; 4192800,24.17; 4193400,24.26;
            4194000,24.25; 4194600,24.31; 4195200,24.41; 4195800,24.41; 4196400,24.5;
            4197000,24.5; 4197600,24.5; 4198200,24.45; 4198800,24.49; 4199400,24.46;
            4200000,24.5; 4200600,24.54; 4201200,24.54; 4201800,24.54; 4202400,24.58;
            4203000,24.54; 4203600,24.54; 4204200,24.62; 4204800,24.58; 4205400,24.58;
            4206000,24.61; 4206600,24.61; 4207200,24.54; 4207800,24.58; 4208400,24.5;
            4209000,24.45; 4209600,24.41; 4210200,24.37; 4210800,24.37; 4211400,24.29;
            4212000,24.25; 4212600,24.17; 4213200,24.13; 4213800,24.08; 4214400,24.05;
            4215000,24.21; 4215600,24.01; 4216200,24.01; 4216800,23.92; 4217400,23.8;
            4218000,23.76; 4218600,23.68; 4219200,23.6; 4219800,23.56; 4220400,23.48;
            4221000,23.49; 4221600,23.44; 4222200,23.4; 4222800,23.32; 4223400,23.27;
            4224000,23.23; 4224600,23.15; 4225200,23.11; 4225800,23.11; 4226400,22.99;
            4227000,22.95; 4227600,22.91; 4228200,22.91; 4228800,22.87; 4229400,22.87;
            4230000,22.83; 4230600,22.78; 4231200,22.78; 4231800,22.77; 4232400,22.75;
            4233000,22.74; 4233600,22.7; 4234200,22.66; 4234800,22.66; 4235400,22.62;
            4236000,22.62; 4236600,22.58; 4237200,22.58; 4237800,22.55; 4238400,22.54;
            4239000,22.55; 4239600,22.59; 4240200,22.55; 4240800,22.58; 4241400,22.58;
            4242000,22.58; 4242600,22.62; 4243200,22.59; 4243800,22.62; 4244400,22.62;
            4245000,22.63; 4245600,22.62; 4246200,22.62; 4246800,22.64; 4247400,22.62;
            4248000,22.62; 4248600,22.62; 4249200,22.62; 4249800,22.89; 4250400,23.07;
            4251000,23.23; 4251600,23.37; 4252200,23.37; 4252800,23.35; 4253400,23.24;
            4254000,23.15; 4254600,23.07; 4255200,22.99; 4255800,23.03; 4256400,23.11;
            4257000,23.19; 4257600,23.23; 4258200,23.24; 4258800,23.24; 4259400,23.19;
            4260000,23.15; 4260600,23.11; 4261200,23.11; 4261800,23.11; 4262400,23.19;
            4263000,23.21; 4263600,23.36; 4264200,23.45; 4264800,23.52; 4265400,23.59;
            4266000,23.56; 4266600,23.6; 4267200,23.64; 4267800,23.64; 4268400,23.69;
            4269000,23.72; 4269600,23.73; 4270200,23.77; 4270800,23.77; 4271400,23.84;
            4272000,23.92; 4272600,23.93; 4273200,23.88; 4273800,23.92; 4274400,23.93;
            4275000,23.92; 4275600,23.92; 4276200,23.92; 4276800,23.92; 4277400,23.92;
            4278000,23.92; 4278600,23.92; 4279200,23.97; 4279800,24; 4280400,24.09;
            4281000,24.17; 4281600,24.22; 4282200,24.29; 4282800,24.34; 4283400,24.41;
            4284000,24.4; 4284600,24.37; 4285200,24.41; 4285800,24.42; 4286400,24.41;
            4287000,24.41; 4287600,24.41; 4288200,24.37; 4288800,24.29; 4289400,24.21;
            4290000,24.22; 4290600,24.17; 4291200,24.17; 4291800,24.25; 4292400,24.17;
            4293000,24.17; 4293600,24.21; 4294200,24.13; 4294800,24.2; 4295400,24.21;
            4296000,24.17; 4296600,24.17; 4297200,24.13; 4297800,24.09; 4298400,24.05;
            4299000,24.01; 4299600,23.98; 4300200,23.93; 4300800,23.9; 4301400,23.84;
            4302000,23.8; 4302600,23.76; 4303200,23.72; 4303800,23.68; 4304400,23.6;
            4305000,23.56; 4305600,23.48; 4306200,23.4; 4306800,23.32; 4307400,23.24;
            4308000,23.15; 4308600,23.08; 4309200,23.03; 4309800,22.91; 4310400,22.95;
            4311000,22.9; 4311600,22.84; 4312200,22.8; 4312800,22.78; 4313400,22.74;
            4314000,22.71; 4314600,22.66; 4315200,22.62; 4315800,22.58; 4316400,22.54;
            4317000,22.5; 4317600,22.46; 4318200,22.42; 4318800,22.34; 4319400,22.3;
            4320000,22.26; 4320600,22.26; 4321200,22.18; 4321800,22.13; 4322400,22.05;
            4323000,22; 4323600,21.95; 4324200,21.89; 4324800,21.85; 4325400,21.81;
            4326000,21.75; 4326600,21.72; 4327200,21.7; 4327800,21.64; 4328400,21.6;
            4329000,21.54; 4329600,21.52; 4330200,21.48; 4330800,21.44; 4331400,21.4;
            4332000,21.36; 4332600,21.32; 4333200,21.32; 4333800,21.28; 4334400,21.24;
            4335000,21.2; 4335600,21.2; 4336200,21.16; 4336800,21.13; 4337400,21.12;
            4338000,21.08; 4338600,21.08; 4339200,21.04; 4339800,21; 4340400,21; 4341000,
            20.97; 4341600,20.96; 4342200,20.91; 4342800,20.92; 4343400,20.87; 4344000,
            20.87; 4344600,20.87; 4345200,20.84; 4345800,20.83; 4346400,20.83; 4347000,
            20.83; 4347600,20.83; 4348200,20.83; 4348800,20.79; 4349400,20.79; 4350000,
            20.79; 4350600,20.82; 4351200,20.8; 4351800,20.79; 4352400,20.79; 4353000,
            20.79; 4353600,20.79; 4354200,20.8; 4354800,20.83; 4355400,20.83; 4356000,
            20.87; 4356600,20.87; 4357200,20.91; 4357800,20.92; 4358400,20.91; 4359000,
            20.96; 4359600,20.97; 4360200,20.97; 4360800,20.96; 4361400,21; 4362000,
            21.01; 4362600,21.04; 4363200,21.08; 4363800,21.11; 4364400,21.12; 4365000,
            21.16; 4365600,21.16; 4366200,21.2; 4366800,21.2; 4367400,21.24; 4368000,
            21.24; 4368600,21.28; 4369200,21.32; 4369800,21.32; 4370400,21.32; 4371000,
            21.3; 4371600,21.28; 4372200,21.28; 4372800,21.24; 4373400,21.24; 4374000,
            21.22; 4374600,21.2; 4375200,21.2; 4375800,21.16; 4376400,21.16; 4377000,
            21.16; 4377600,21.16; 4378200,21.12; 4378800,21.12; 4379400,21.11; 4380000,
            21.08; 4380600,21.05; 4381200,21.04; 4381800,21.03; 4382400,21; 4383000,
            20.98; 4383600,20.96; 4384200,20.95; 4384800,20.92; 4385400,20.91; 4386000,
            20.91; 4386600,20.87; 4387200,20.87; 4387800,20.87; 4388400,20.84; 4389000,
            20.83; 4389600,20.83; 4390200,20.83; 4390800,20.83; 4391400,20.83; 4392000,
            20.8; 4392600,20.8; 4393200,20.79; 4393800,20.79; 4394400,20.79; 4395000,
            20.79; 4395600,20.79; 4396200,20.79; 4396800,20.75; 4397400,20.75; 4398000,
            20.75; 4398600,20.75; 4399200,20.75; 4399800,20.71; 4400400,20.72; 4401000,
            20.71; 4401600,20.71; 4402200,20.69; 4402800,20.67; 4403400,20.67; 4404000,
            20.75; 4404600,20.85; 4405200,20.96; 4405800,21.04; 4406400,21.08; 4407000,
            21.15; 4407600,21.18; 4408200,21.23; 4408800,21.28; 4409400,21.32; 4410000,
            21.32; 4410600,21.36; 4411200,21.41; 4411800,21.44; 4412400,21.44; 4413000,
            21.48; 4413600,21.48; 4414200,21.52; 4414800,21.52; 4415400,21.52; 4416000,
            21.58; 4416600,21.56; 4417200,21.56; 4417800,21.6; 4418400,21.6; 4419000,
            21.6; 4419600,21.6; 4420200,21.6; 4420800,21.64; 4421400,21.65; 4422000,
            21.63; 4422600,21.64; 4423200,21.65; 4423800,21.65; 4424400,21.66; 4425000,
            21.65; 4425600,21.65; 4426200,21.65; 4426800,21.65; 4427400,21.66; 4428000,
            21.68; 4428600,21.69; 4429200,21.69; 4429800,21.69; 4430400,21.69; 4431000,
            21.69; 4431600,21.69; 4432200,21.7; 4432800,21.72; 4433400,21.73; 4434000,
            21.76; 4434600,21.77; 4435200,21.78; 4435800,21.77; 4436400,21.81; 4437000,
            21.81; 4437600,21.81; 4438200,21.84; 4438800,21.81; 4439400,21.85; 4440000,
            21.85; 4440600,21.89; 4441200,21.89; 4441800,21.93; 4442400,21.93; 4443000,
            21.89; 4443600,21.88; 4444200,21.85; 4444800,21.85; 4445400,21.84; 4446000,
            21.81; 4446600,21.81; 4447200,21.85; 4447800,21.85; 4448400,21.86; 4449000,
            21.89; 4449600,21.89; 4450200,21.9; 4450800,21.98; 4451400,22.02; 4452000,
            22.09; 4452600,22.12; 4453200,22.18; 4453800,22.22; 4454400,22.26; 4455000,
            22.3; 4455600,22.34; 4456200,22.38; 4456800,22.42; 4457400,22.45; 4458000,
            22.46; 4458600,22.51; 4459200,22.58; 4459800,22.58; 4460400,22.62; 4461000,
            22.66; 4461600,22.74; 4462200,22.78; 4462800,22.91; 4463400,22.87; 4464000,
            22.8; 4464600,22.7; 4465200,22.65; 4465800,22.62; 4466400,22.57; 4467000,
            22.5; 4467600,22.46; 4468200,22.42; 4468800,22.34; 4469400,22.3; 4470000,
            22.27; 4470600,22.25; 4471200,22.14; 4471800,22.09; 4472400,22.09; 4473000,
            22.14; 4473600,22.17; 4474200,22.17; 4474800,22.14; 4475400,22.14; 4476000,
            22.17; 4476600,22.17; 4477200,22.14; 4477800,22.17; 4478400,22.14; 4479000,
            22.14; 4479600,22.14; 4480200,22.14; 4480800,22.14; 4481400,22.1; 4482000,
            22.09; 4482600,22.1; 4483200,22.1; 4483800,22.09; 4484400,22.09; 4485000,
            22.09; 4485600,22.09; 4486200,22.09; 4486800,22.09; 4487400,22.09; 4488000,
            22.1; 4488600,22.06; 4489200,22.08; 4489800,22.08; 4490400,22.05; 4491000,
            22.05; 4491600,22.05; 4492200,22.05; 4492800,22.05; 4493400,22.05; 4494000,
            22.05; 4494600,22.03; 4495200,22.05; 4495800,22.05; 4496400,22.01; 4497000,
            22.05; 4497600,22.01; 4498200,22.01; 4498800,22.01; 4499400,22.01; 4500000,
            22.01; 4500600,22.01; 4501200,22.01; 4501800,22.01; 4502400,22.01; 4503000,
            22.01; 4503600,22.01; 4504200,22.01; 4504800,22; 4505400,21.97; 4506000,
            22.02; 4506600,21.97; 4507200,21.97; 4507800,21.97; 4508400,22.06; 4509000,
            22.34; 4509600,22.58; 4510200,22.78; 4510800,22.95; 4511400,23.06; 4512000,
            23.03; 4512600,22.96; 4513200,22.83; 4513800,22.7; 4514400,22.62; 4515000,
            22.7; 4515600,22.78; 4516200,22.87; 4516800,22.91; 4517400,22.97; 4518000,
            23; 4518600,22.91; 4519200,22.87; 4519800,22.86; 4520400,22.87; 4521000,
            22.94; 4521600,22.99; 4522200,22.95; 4522800,23.03; 4523400,23.08; 4524000,
            23.15; 4524600,23.15; 4525200,23.19; 4525800,23.27; 4526400,23.28; 4527000,
            23.38; 4527600,23.44; 4528200,23.51; 4528800,23.56; 4529400,23.6; 4530000,
            23.64; 4530600,23.68; 4531200,23.72; 4531800,23.73; 4532400,23.76; 4533000,
            23.72; 4533600,23.69; 4534200,23.68; 4534800,23.64; 4535400,23.64; 4536000,
            23.61; 4536600,23.68; 4537200,23.72; 4537800,23.76; 4538400,23.8; 4539000,
            23.8; 4539600,23.86; 4540200,23.84; 4540800,23.85; 4541400,23.84; 4542000,
            23.88; 4542600,23.92; 4543200,23.96; 4543800,23.97; 4544400,23.96; 4545000,
            23.94; 4545600,23.96; 4546200,23.94; 4546800,23.92; 4547400,23.92; 4548000,
            23.92; 4548600,23.96; 4549200,24.01; 4549800,23.96; 4550400,23.98; 4551000,
            23.96; 4551600,23.95; 4552200,23.88; 4552800,23.84; 4553400,23.8; 4554000,
            23.8; 4554600,23.76; 4555200,23.77; 4555800,23.81; 4556400,23.8; 4557000,
            23.86; 4557600,23.84; 4558200,23.8; 4558800,23.8; 4559400,23.8; 4560000,
            23.76; 4560600,23.72; 4561200,23.68; 4561800,23.68; 4562400,23.6; 4563000,
            23.56; 4563600,23.48; 4564200,23.43; 4564800,23.36; 4565400,23.27; 4566000,
            23.19; 4566600,23.11; 4567200,23.03; 4567800,22.99; 4568400,22.95; 4569000,
            22.91; 4569600,22.87; 4570200,22.83; 4570800,22.78; 4571400,22.74; 4572000,
            22.7; 4572600,22.66; 4573200,22.66; 4573800,22.62; 4574400,22.62; 4575000,
            22.58; 4575600,22.54; 4576200,22.53; 4576800,22.5; 4577400,22.5; 4578000,
            22.46; 4578600,22.47; 4579200,22.42; 4579800,22.42; 4580400,22.42; 4581000,
            22.38; 4581600,22.39; 4582200,22.38; 4582800,22.37; 4583400,22.34; 4584000,
            22.33; 4584600,22.3; 4585200,22.3; 4585800,22.3; 4586400,22.27; 4587000,
            22.26; 4587600,22.26; 4588200,22.26; 4588800,22.26; 4589400,22.26; 4590000,
            22.26; 4590600,22.26; 4591200,22.22; 4591800,22.23; 4592400,22.22; 4593000,
            22.22; 4593600,22.22; 4594200,22.18; 4594800,22.22; 4595400,22.38; 4596000,
            22.55; 4596600,22.62; 4597200,22.71; 4597800,22.82; 4598400,22.83; 4599000,
            22.83; 4599600,22.83; 4600200,22.83; 4600800,22.83; 4601400,22.78; 4602000,
            22.81; 4602600,22.83; 4603200,22.87; 4603800,22.89; 4604400,22.95; 4605000,
            23.08; 4605600,23.24; 4606200,23.27; 4606800,23.4; 4607400,23.48; 4608000,
            23.56; 4608600,23.6; 4609200,23.72; 4609800,23.76; 4610400,23.8; 4611000,
            23.84; 4611600,23.88; 4612200,23.84; 4612800,23.88; 4613400,23.88; 4614000,
            23.92; 4614600,23.96; 4615200,24.02; 4615800,24.05; 4616400,24.1; 4617000,
            24.12; 4617600,24.09; 4618200,24.09; 4618800,24.09; 4619400,24.13; 4620000,
            24.05; 4620600,24.03; 4621200,24.01; 4621800,24; 4622400,23.96; 4623000,
            24.01; 4623600,24.04; 4624200,24.05; 4624800,24.09; 4625400,24.17; 4626000,
            24.21; 4626600,24.25; 4627200,24.29; 4627800,24.33; 4628400,24.32; 4629000,
            24.33; 4629600,24.33; 4630200,24.37; 4630800,24.37; 4631400,24.37; 4632000,
            24.33; 4632600,24.3; 4633200,24.29; 4633800,24.29; 4634400,24.29; 4635000,
            24.25; 4635600,24.31; 4636200,24.29; 4636800,24.25; 4637400,24.33; 4638000,
            24.3; 4638600,24.29; 4639200,24.3; 4639800,24.27; 4640400,24.21; 4641000,
            24.17; 4641600,24.18; 4642200,24.21; 4642800,24.21; 4643400,24.21; 4644000,
            24.21; 4644600,24.21; 4645200,24.17; 4645800,24.21; 4646400,24.17; 4647000,
            24.17; 4647600,24.17; 4648200,24.13; 4648800,24.09; 4649400,24.05; 4650000,
            24.01; 4650600,23.92; 4651200,23.8; 4651800,23.77; 4652400,23.68; 4653000,
            23.6; 4653600,23.52; 4654200,23.46; 4654800,23.4; 4655400,23.36; 4656000,
            23.35; 4656600,23.32; 4657200,23.27; 4657800,23.23; 4658400,23.2; 4659000,
            23.19; 4659600,23.15; 4660200,23.15; 4660800,23.11; 4661400,23.11; 4662000,
            23.07; 4662600,23.07; 4663200,23.03; 4663800,23.02; 4664400,23.02; 4665000,
            22.99; 4665600,22.95; 4666200,22.95; 4666800,22.95; 4667400,22.94; 4668000,
            22.91; 4668600,22.91; 4669200,22.91; 4669800,22.91; 4670400,22.87; 4671000,
            22.87; 4671600,22.86; 4672200,22.83; 4672800,22.83; 4673400,22.83; 4674000,
            22.82; 4674600,22.8; 4675200,22.78; 4675800,22.78; 4676400,22.78; 4677000,
            22.78; 4677600,22.78; 4678200,22.74; 4678800,22.74; 4679400,22.75; 4680000,
            22.74; 4680600,22.74; 4681200,22.78; 4681800,23.03; 4682400,23.23; 4683000,
            23.4; 4683600,23.52; 4684200,23.48; 4684800,23.43; 4685400,23.36; 4686000,
            23.27; 4686600,23.15; 4687200,23.11; 4687800,23.17; 4688400,23.27; 4689000,
            23.32; 4689600,23.36; 4690200,23.39; 4690800,23.4; 4691400,23.27; 4692000,
            23.24; 4692600,23.23; 4693200,23.23; 4693800,23.32; 4694400,23.4; 4695000,
            23.48; 4695600,23.48; 4696200,23.49; 4696800,23.57; 4697400,23.55; 4698000,
            23.44; 4698600,23.48; 4699200,23.59; 4699800,23.64; 4700400,23.64; 4701000,
            23.68; 4701600,23.76; 4702200,23.72; 4702800,23.73; 4703400,23.76; 4704000,
            23.8; 4704600,23.84; 4705200,23.84; 4705800,23.88; 4706400,23.89; 4707000,
            23.92; 4707600,23.88; 4708200,23.91; 4708800,23.92; 4709400,23.92; 4710000,
            23.93; 4710600,23.88; 4711200,23.92; 4711800,23.96; 4712400,24.05; 4713000,
            24.09; 4713600,24.08; 4714200,24.13; 4714800,24.13; 4715400,24.17; 4716000,
            24.17; 4716600,24.21; 4717200,24.21; 4717800,24.17; 4718400,24.09; 4719000,
            24.08; 4719600,24.13; 4720200,24.25; 4720800,24.25; 4721400,24.26; 4722000,
            24.29; 4722600,24.25; 4723200,24.21; 4723800,24.14; 4724400,24.13; 4725000,
            24.09; 4725600,24.1; 4726200,24.13; 4726800,24.09; 4727400,24.05; 4728000,
            24.01; 4728600,23.96; 4729200,23.99; 4729800,23.96; 4730400,23.92; 4731000,
            23.92; 4731600,23.88; 4732200,23.83; 4732800,23.8; 4733400,23.76; 4734000,
            23.73; 4734600,23.68; 4735200,23.64; 4735800,23.64; 4736400,23.56; 4737000,
            23.48; 4737600,23.4; 4738200,23.32; 4738800,23.28; 4739400,23.15; 4740000,
            23.09; 4740600,23.03; 4741200,22.96; 4741800,22.91; 4742400,22.87; 4743000,
            22.84; 4743600,22.78; 4744200,22.74; 4744800,22.7; 4745400,22.66; 4746000,
            22.62; 4746600,22.58; 4747200,22.58; 4747800,22.54; 4748400,22.5; 4749000,
            22.46; 4749600,22.46; 4750200,22.42; 4750800,22.42; 4751400,22.41; 4752000,
            22.38; 4752600,22.38; 4753200,22.35; 4753800,22.34; 4754400,22.34; 4755000,
            22.3; 4755600,22.27; 4756200,22.29; 4756800,22.26; 4757400,22.26; 4758000,
            22.26; 4758600,22.26; 4759200,22.42; 4759800,22.54; 4760400,22.66; 4761000,
            22.78; 4761600,22.9; 4762200,22.99; 4762800,23.03; 4763400,23.06; 4764000,
            23.03; 4764600,22.99; 4765200,22.91; 4765800,22.84; 4766400,22.83; 4767000,
            22.88; 4767600,22.92; 4768200,22.91; 4768800,22.91; 4769400,22.92; 4770000,
            22.91; 4770600,22.91; 4771200,22.91; 4771800,22.91; 4772400,22.91; 4773000,
            22.92; 4773600,22.91; 4774200,22.91; 4774800,22.91; 4775400,22.91; 4776000,
            22.91; 4776600,22.91; 4777200,22.91; 4777800,22.95; 4778400,23.03; 4779000,
            23.07; 4779600,23.16; 4780200,23.26; 4780800,23.35; 4781400,23.44; 4782000,
            23.53; 4782600,23.6; 4783200,23.64; 4783800,23.68; 4784400,23.68; 4785000,
            23.72; 4785600,23.72; 4786200,23.72; 4786800,23.76; 4787400,23.8; 4788000,
            23.84; 4788600,23.88; 4789200,23.92; 4789800,23.92; 4790400,23.94; 4791000,
            23.98; 4791600,24.05; 4792200,24.1; 4792800,24.09; 4793400,24.05; 4794000,
            24.01; 4794600,23.96; 4795200,23.93; 4795800,24; 4796400,24.01; 4797000,
            24.09; 4797600,24.21; 4798200,24.3; 4798800,24.34; 4799400,24.33; 4800000,
            24.37; 4800600,24.34; 4801200,24.34; 4801800,24.29; 4802400,24.27; 4803000,
            24.29; 4803600,24.29; 4804200,24.33; 4804800,24.33; 4805400,24.33; 4806000,
            24.33; 4806600,24.33; 4807200,24.29; 4807800,24.29; 4808400,24.33; 4809000,
            24.33; 4809600,24.38; 4810200,24.33; 4810800,24.29; 4811400,24.25; 4812000,
            24.22; 4812600,24.12; 4813200,24.17; 4813800,24.13; 4814400,24.05; 4815000,
            24.01; 4815600,23.92; 4816200,23.92; 4816800,23.88; 4817400,23.84; 4818000,
            23.88; 4818600,23.88; 4819200,23.88; 4819800,23.88; 4820400,23.88; 4821000,
            23.84; 4821600,23.84; 4822200,23.76; 4822800,23.73; 4823400,23.68; 4824000,
            23.6; 4824600,23.56; 4825200,23.44; 4825800,23.4; 4826400,23.36; 4827000,
            23.31; 4827600,23.24; 4828200,23.15; 4828800,23.11; 4829400,23.03; 4830000,
            23; 4830600,22.95; 4831200,22.88; 4831800,22.83; 4832400,22.78; 4833000,
            22.8; 4833600,22.74; 4834200,22.7; 4834800,22.66; 4835400,22.62; 4836000,
            22.62; 4836600,22.58; 4837200,22.55; 4837800,22.52; 4838400,22.5; 4839000,
            22.46; 4839600,22.47; 4840200,22.42; 4840800,22.42; 4841400,22.38; 4842000,
            22.38; 4842600,22.38; 4843200,22.35; 4843800,22.34; 4844400,22.38; 4845000,
            22.38; 4845600,22.39; 4846200,22.42; 4846800,22.42; 4847400,22.45; 4848000,
            22.46; 4848600,22.47; 4849200,22.46; 4849800,22.46; 4850400,22.46; 4851000,
            22.46; 4851600,22.46; 4852200,22.46; 4852800,22.46; 4853400,22.46; 4854000,
            22.62; 4854600,22.87; 4855200,23.07; 4855800,23.24; 4856400,23.32; 4857000,
            23.27; 4857600,23.19; 4858200,23.11; 4858800,23; 4859400,22.9; 4860000,22.83;
            4860600,22.93; 4861200,23.07; 4861800,23.15; 4862400,23.19; 4863000,23.2;
            4863600,23.16; 4864200,23.32; 4864800,23.03; 4865400,22.99; 4866000,23.03;
            4866600,23.07; 4867200,23.11; 4867800,23.23; 4868400,23.36; 4869000,23.4;
            4869600,23.44; 4870200,23.46; 4870800,23.48; 4871400,23.44; 4872000,23.45;
            4872600,23.52; 4873200,23.56; 4873800,23.56; 4874400,23.6; 4875000,23.68;
            4875600,23.72; 4876200,23.72; 4876800,23.77; 4877400,23.85; 4878000,23.97;
            4878600,23.92; 4879200,23.92; 4879800,23.88; 4880400,23.88; 4881000,23.88;
            4881600,23.92; 4882200,23.92; 4882800,24.05; 4883400,24.21; 4884000,24.25;
            4884600,24.25; 4885200,24.29; 4885800,24.33; 4886400,24.25; 4887000,24.25;
            4887600,24.29; 4888200,24.34; 4888800,24.38; 4889400,24.41; 4890000,24.41;
            4890600,24.38; 4891200,24.11; 4891800,24.21; 4892400,24.29; 4893000,24.31;
            4893600,24.33; 4894200,24.33; 4894800,24.37; 4895400,24.37; 4896000,24.25;
            4896600,24.33; 4897200,24.21; 4897800,24.13; 4898400,24.13; 4899000,24.12;
            4899600,24.1; 4900200,24.13; 4900800,24.05; 4901400,23.92; 4902000,23.88;
            4902600,23.84; 4903200,23.8; 4903800,23.76; 4904400,23.72; 4905000,23.65;
            4905600,23.59; 4906200,23.52; 4906800,23.48; 4907400,23.44; 4908000,23.4;
            4908600,23.4; 4909200,23.32; 4909800,23.24; 4910400,23.15; 4911000,23.08;
            4911600,23.02; 4912200,22.98; 4912800,22.91; 4913400,22.83; 4914000,22.8;
            4914600,22.74; 4915200,22.7; 4915800,22.64; 4916400,22.58; 4917000,22.54;
            4917600,22.46; 4918200,22.44; 4918800,22.38; 4919400,22.38; 4920000,22.34;
            4920600,22.3; 4921200,22.26; 4921800,22.22; 4922400,22.18; 4923000,22.14;
            4923600,22.09; 4924200,22.05; 4924800,21.97; 4925400,21.97; 4926000,21.89;
            4926600,21.85; 4927200,21.85; 4927800,21.81; 4928400,21.77; 4929000,21.72;
            4929600,21.66; 4930200,21.61; 4930800,21.6; 4931400,21.56; 4932000,21.52;
            4932600,21.52; 4933200,21.48; 4933800,21.44; 4934400,21.45; 4935000,21.4;
            4935600,21.36; 4936200,21.37; 4936800,21.32; 4937400,21.32; 4938000,21.32;
            4938600,21.28; 4939200,21.27; 4939800,21.24; 4940400,21.23; 4941000,21.2;
            4941600,21.2; 4942200,21.16; 4942800,21.16; 4943400,21.12; 4944000,21.12;
            4944600,21.12; 4945200,21.09; 4945800,21.08; 4946400,21.04; 4947000,21.04;
            4947600,21.04; 4948200,21; 4948800,21; 4949400,21; 4950000,20.99; 4950600,
            20.96; 4951200,20.96; 4951800,20.96; 4952400,20.96; 4953000,20.96; 4953600,
            20.96; 4954200,20.96; 4954800,20.96; 4955400,20.96; 4956000,20.99; 4956600,
            21; 4957200,21; 4957800,21; 4958400,21.02; 4959000,21.04; 4959600,21.04;
            4960200,21.04; 4960800,21.08; 4961400,21.08; 4962000,21.12; 4962600,21.12;
            4963200,21.16; 4963800,21.16; 4964400,21.17; 4965000,21.2; 4965600,21.2;
            4966200,21.24; 4966800,21.24; 4967400,21.24; 4968000,21.26; 4968600,21.26;
            4969200,21.27; 4969800,21.24; 4970400,21.24; 4971000,21.24; 4971600,21.21;
            4972200,21.2; 4972800,21.2; 4973400,21.2; 4974000,21.16; 4974600,21.16;
            4975200,21.17; 4975800,21.16; 4976400,21.16; 4977000,21.17; 4977600,21.12;
            4978200,21.12; 4978800,21.12; 4979400,21.12; 4980000,21.12; 4980600,21.12;
            4981200,21.12; 4981800,21.1; 4982400,21.08; 4983000,21.08; 4983600,21.08;
            4984200,21.08; 4984800,21.04; 4985400,21.04; 4986000,21.02; 4986600,21;
            4987200,20.99; 4987800,20.96; 4988400,20.96; 4989000,20.96; 4989600,20.95;
            4990200,20.91; 4990800,20.91; 4991400,20.88; 4992000,20.87; 4992600,20.87;
            4993200,20.87; 4993800,20.85; 4994400,20.83; 4995000,20.83; 4995600,20.83;
            4996200,20.79; 4996800,20.79; 4997400,20.79; 4998000,20.8; 4998600,20.79;
            4999200,20.75; 4999800,20.75; 5000400,20.75; 5001000,20.71; 5001600,20.71;
            5002200,20.71; 5002800,20.67; 5003400,20.67; 5004000,20.67; 5004600,20.67;
            5005200,20.66; 5005800,20.63; 5006400,20.63; 5007000,20.63; 5007600,20.63;
            5008200,20.59; 5008800,20.7; 5009400,20.79; 5010000,20.87; 5010600,20.91;
            5011200,20.96; 5011800,21; 5012400,21.04; 5013000,21.06; 5013600,21.08;
            5014200,21.12; 5014800,21.12; 5015400,21.16; 5016000,21.16; 5016600,21.2;
            5017200,21.2; 5017800,21.2; 5018400,21.24; 5019000,21.24; 5019600,21.27;
            5020200,21.28; 5020800,21.28; 5021400,21.28; 5022000,21.28; 5022600,21.28;
            5023200,21.33; 5023800,21.32; 5024400,21.32; 5025000,21.32; 5025600,21.32;
            5026200,21.36; 5026800,21.36; 5027400,21.36; 5028000,21.36; 5028600,21.36;
            5029200,21.36; 5029800,21.36; 5030400,21.36; 5031000,21.36; 5031600,21.37;
            5032200,21.4; 5032800,21.4; 5033400,21.4; 5034000,21.36; 5034600,21.36;
            5035200,21.4; 5035800,21.4; 5036400,21.4; 5037000,21.4; 5037600,21.4; 5038200,
            21.4; 5038800,21.4; 5039400,21.45; 5040000,21.44; 5040600,21.44; 5041200,
            21.44; 5041800,21.44; 5042400,21.44; 5043000,21.48; 5043600,21.48; 5044200,
            21.48; 5044800,21.52; 5045400,21.56; 5046000,21.56; 5046600,21.64; 5047200,
            21.69; 5047800,21.73; 5048400,21.76; 5049000,21.77; 5049600,21.77; 5050200,
            21.77; 5050800,21.77; 5051400,21.81; 5052000,21.85; 5052600,21.85; 5053200,
            21.89; 5053800,21.93; 5054400,21.93; 5055000,21.96; 5055600,21.97; 5056200,
            21.97; 5056800,21.97; 5057400,21.97; 5058000,21.97; 5058600,21.97; 5059200,
            22.01; 5059800,22.09; 5060400,22.09; 5061000,22.05; 5061600,22.05; 5062200,
            22.05; 5062800,22.05; 5063400,22.01; 5064000,21.97; 5064600,21.97; 5065200,
            21.97; 5065800,21.97; 5066400,21.93; 5067000,21.93; 5067600,21.93; 5068200,
            21.89; 5068800,21.87; 5069400,21.85; 5070000,21.85; 5070600,21.85; 5071200,
            21.81; 5071800,21.81; 5072400,21.78; 5073000,21.77; 5073600,21.78; 5074200,
            21.73; 5074800,21.74; 5075400,21.73; 5076000,21.73; 5076600,21.69; 5077200,
            21.69; 5077800,21.69; 5078400,21.65; 5079000,21.65; 5079600,21.65; 5080200,
            21.65; 5080800,21.66; 5081400,21.64; 5082000,21.62; 5082600,21.6; 5083200,
            21.6; 5083800,21.6; 5084400,21.59; 5085000,21.57; 5085600,21.56; 5086200,
            21.58; 5086800,21.56; 5087400,21.56; 5088000,21.56; 5088600,21.52; 5089200,
            21.54; 5089800,21.54; 5090400,21.52; 5091000,21.48; 5091600,21.48; 5092200,
            21.48; 5092800,21.48; 5093400,21.48; 5094000,21.44; 5094600,21.44; 5095200,
            21.44; 5095800,21.44; 5096400,21.44; 5097000,21.44; 5097600,21.44; 5098200,
            21.42; 5098800,21.4; 5099400,21.4; 5100000,21.4; 5100600,21.4; 5101200,21.4;
            5101800,21.4; 5102400,21.37; 5103000,21.36; 5103600,21.36; 5104200,21.36;
            5104800,21.32; 5105400,21.36; 5106000,21.32; 5106600,21.32; 5107200,21.32;
            5107800,21.33; 5108400,21.32; 5109000,21.32; 5109600,21.32; 5110200,21.32;
            5110800,21.33; 5111400,21.32; 5112000,21.32; 5112600,21.32; 5113200,21.41;
            5113800,21.62; 5114400,21.73; 5115000,21.81; 5115600,21.93; 5116200,22.05;
            5116800,22.05; 5117400,22.05; 5118000,22.05; 5118600,22.05; 5119200,22.01;
            5119800,22.03; 5120400,22.05; 5121000,22.09; 5121600,22.18; 5122200,22.18;
            5122800,22.18; 5123400,22.23; 5124000,22.27; 5124600,22.3; 5125200,22.38;
            5125800,22.5; 5126400,22.62; 5127000,22.7; 5127600,22.7; 5128200,22.82;
            5128800,22.91; 5129400,22.88; 5130000,22.9; 5130600,22.95; 5131200,23; 5131800,
            23.08; 5132400,23.04; 5133000,22.99; 5133600,23.03; 5134200,23.03; 5134800,
            23.03; 5135400,23.08; 5136000,23.15; 5136600,23.2; 5137200,23.23; 5137800,
            23.2; 5138400,23.18; 5139000,23.15; 5139600,23.15; 5140200,23.17; 5140800,
            23.19; 5141400,23.24; 5142000,23.35; 5142600,23.32; 5143200,23.32; 5143800,
            23.35; 5144400,23.36; 5145000,23.4; 5145600,23.4; 5146200,23.35; 5146800,
            23.36; 5147400,23.36; 5148000,23.4; 5148600,23.4; 5149200,23.44; 5149800,
            23.52; 5150400,23.44; 5151000,23.53; 5151600,23.49; 5152200,23.52; 5152800,
            23.56; 5153400,23.56; 5154000,23.64; 5154600,23.64; 5155200,23.65; 5155800,
            23.68; 5156400,23.68; 5157000,23.63; 5157600,23.6; 5158200,23.6; 5158800,
            23.6; 5159400,23.58; 5160000,23.52; 5160600,23.44; 5161200,23.44; 5161800,
            23.43; 5162400,23.42; 5163000,23.4; 5163600,23.4; 5164200,23.39; 5164800,
            23.36; 5165400,23.35; 5166000,23.27; 5166600,23.23; 5167200,23.15; 5167800,
            23.07; 5168400,22.99; 5169000,22.91; 5169600,22.8; 5170200,22.7; 5170800,
            22.64; 5171400,22.54; 5172000,22.42; 5172600,22.37; 5173200,22.3; 5173800,
            22.25; 5174400,22.18; 5175000,22.14; 5175600,22.09; 5176200,22.05; 5176800,
            21.97; 5177400,21.93; 5178000,21.9; 5178600,21.85; 5179200,21.81; 5179800,
            21.8; 5180400,21.77; 5181000,21.73; 5181600,21.7; 5182200,21.65; 5182800,
            21.65; 5183400,21.66; 5184000,21.6; 5184600,21.6; 5185200,21.57; 5185800,
            21.56; 5186400,21.52; 5187000,21.53; 5187600,21.55; 5188200,21.53; 5188800,
            21.52; 5189400,21.48; 5190000,21.48; 5190600,21.48; 5191200,21.65; 5191800,
            21.81; 5192400,21.98; 5193000,22.09; 5193600,22.14; 5194200,22.22; 5194800,
            22.3; 5195400,22.34; 5196000,22.38; 5196600,22.38; 5197200,22.34; 5197800,
            22.31; 5198400,22.3; 5199000,22.3; 5199600,22.3; 5200200,22.3; 5200800,22.34;
            5201400,22.34; 5202000,22.34; 5202600,22.36; 5203200,22.34; 5203800,22.35;
            5204400,22.34; 5205000,22.34; 5205600,22.38; 5206200,22.38; 5206800,22.38;
            5207400,22.38; 5208000,22.38; 5208600,22.38; 5209200,22.38; 5209800,22.42;
            5210400,22.46; 5211000,22.54; 5211600,22.62; 5212200,22.72; 5212800,22.74;
            5213400,22.83; 5214000,22.99; 5214600,23.07; 5215200,23.16; 5215800,23.27;
            5216400,23.29; 5217000,23.35; 5217600,23.39; 5218200,23.42; 5218800,23.48;
            5219400,23.49; 5220000,23.56; 5220600,23.56; 5221200,23.68; 5221800,23.72;
            5222400,23.76; 5223000,23.76; 5223600,23.84; 5224200,23.76; 5224800,23.76;
            5225400,23.76; 5226000,23.8; 5226600,23.84; 5227200,23.88; 5227800,24.01;
            5228400,24.16; 5229000,24.25; 5229600,24.4; 5230200,24.46; 5230800,24.54;
            5231400,24.58; 5232000,24.58; 5232600,24.5; 5233200,24.59; 5233800,24.58;
            5234400,24.62; 5235000,24.66; 5235600,24.66; 5236200,24.66; 5236800,24.66;
            5237400,24.69; 5238000,24.62; 5238600,24.62; 5239200,24.58; 5239800,24.53;
            5240400,24.49; 5241000,24.45; 5241600,24.45; 5242200,24.37; 5242800,24.38;
            5243400,24.38; 5244000,24.29; 5244600,24.21; 5245200,24.13; 5245800,24.21;
            5246400,24.22; 5247000,24.13; 5247600,24.13; 5248200,24.17; 5248800,24.13;
            5249400,24.09; 5250000,24.05; 5250600,24.01; 5251200,23.92; 5251800,23.88;
            5252400,23.8; 5253000,23.76; 5253600,23.72; 5254200,23.65; 5254800,23.6;
            5255400,23.56; 5256000,23.56; 5256600,23.47; 5257200,23.63; 5257800,23.32;
            5258400,23.27; 5259000,23.19; 5259600,23.19; 5260200,23.1; 5260800,23.07;
            5261400,22.99; 5262000,22.91; 5262600,22.87; 5263200,22.83; 5263800,22.78;
            5264400,22.74; 5265000,22.7; 5265600,22.66; 5266200,22.62; 5266800,22.58;
            5267400,22.58; 5268000,22.5; 5268600,22.5; 5269200,22.46; 5269800,22.42;
            5270400,22.42; 5271000,22.38; 5271600,22.35; 5272200,22.35; 5272800,22.31;
            5273400,22.26; 5274000,22.26; 5274600,22.22; 5275200,22.22; 5275800,22.19;
            5276400,22.18; 5277000,22.22; 5277600,22.22; 5278200,22.22; 5278800,22.25;
            5279400,22.26; 5280000,22.26; 5280600,22.26; 5281200,22.26; 5281800,22.26;
            5282400,22.3; 5283000,22.3; 5283600,22.3; 5284200,22.3; 5284800,22.3; 5285400,
            22.3; 5286000,22.42; 5286600,22.77; 5287200,22.99; 5287800,23.11; 5288400,
            23.11; 5289000,23.07; 5289600,22.95; 5290200,22.83; 5290800,22.74; 5291400,
            22.64; 5292000,22.66; 5292600,22.78; 5293200,22.9; 5293800,22.96; 5294400,
            22.95; 5295000,22.95; 5295600,22.91; 5296200,22.83; 5296800,22.74; 5297400,
            22.67; 5298000,22.66; 5298600,22.71; 5299200,22.82; 5299800,22.88; 5300400,
            22.91; 5301000,22.95; 5301600,23.03; 5302200,23.08; 5302800,23.07; 5303400,
            23.12; 5304000,23.11; 5304600,23.11; 5305200,23.23; 5305800,23.27; 5306400,
            23.3; 5307000,23.4; 5307600,23.44; 5308200,23.52; 5308800,23.56; 5309400,
            23.56; 5310000,23.56; 5310600,23.52; 5311200,23.48; 5311800,23.4; 5312400,
            23.36; 5313000,23.4; 5313600,23.4; 5314200,23.4; 5314800,23.49; 5315400,
            23.48; 5316000,23.57; 5316600,23.64; 5317200,23.72; 5317800,23.77; 5318400,
            23.84; 5319000,23.89; 5319600,23.8; 5320200,23.84; 5320800,23.88; 5321400,
            23.92; 5322000,23.92; 5322600,23.92; 5323200,23.92; 5323800,23.96; 5324400,
            24; 5325000,24; 5325600,24.01; 5326200,24.09; 5326800,24.04; 5327400,24.08;
            5328000,24.05; 5328600,24.09; 5329200,24.07; 5329800,24.06; 5330400,24.05;
            5331000,24.05; 5331600,24.05; 5332200,24.04; 5332800,24.01; 5333400,24.02;
            5334000,24.01; 5334600,24.01; 5335200,23.97; 5335800,23.96; 5336400,23.94;
            5337000,23.92; 5337600,23.88; 5338200,23.8; 5338800,23.76; 5339400,23.72;
            5340000,23.68; 5340600,23.61; 5341200,23.52; 5341800,23.58; 5342400,23.43;
            5343000,23.43; 5343600,23.4; 5344200,23.32; 5344800,23.22; 5345400,23.11;
            5346000,23.07; 5346600,23.03; 5347200,22.99; 5347800,22.91; 5348400,22.88;
            5349000,22.82; 5349600,22.74; 5350200,22.7; 5350800,22.64; 5351400,22.58;
            5352000,22.56; 5352600,22.5; 5353200,22.47; 5353800,22.42; 5354400,22.42;
            5355000,22.38; 5355600,22.34; 5356200,22.3; 5356800,22.26; 5357400,22.26;
            5358000,22.22; 5358600,22.22; 5359200,22.18; 5359800,22.14; 5360400,22.12;
            5361000,22.09; 5361600,22.09; 5362200,22.09; 5362800,22.09; 5363400,22.09;
            5364000,22.26; 5364600,22.46; 5365200,22.62; 5365800,22.78; 5366400,22.95;
            5367000,23.03; 5367600,22.99; 5368200,22.91; 5368800,22.78; 5369400,22.71;
            5370000,22.66; 5370600,22.74; 5371200,22.83; 5371800,22.91; 5372400,22.94;
            5373000,22.91; 5373600,22.83; 5374200,22.78; 5374800,22.74; 5375400,22.74;
            5376000,22.74; 5376600,22.74; 5377200,22.74; 5377800,22.74; 5378400,22.74;
            5379000,22.74; 5379600,22.74; 5380200,22.74; 5380800,22.75; 5381400,22.74;
            5382000,22.72; 5382600,22.7; 5383200,22.7; 5383800,22.7; 5384400,22.67;
            5385000,22.7; 5385600,22.74; 5386200,22.83; 5386800,23.01; 5387400,23.07;
            5388000,23.23; 5388600,23.28; 5389200,23.36; 5389800,23.4; 5390400,23.48;
            5391000,23.48; 5391600,23.57; 5392200,23.6; 5392800,23.64; 5393400,23.68;
            5394000,23.64; 5394600,23.64; 5395200,23.64; 5395800,23.68; 5396400,23.64;
            5397000,23.6; 5397600,23.64; 5398200,23.64; 5398800,23.68; 5399400,23.73;
            5400000,23.72; 5400600,23.77; 5401200,23.72; 5401800,23.76; 5402400,23.8;
            5403000,23.88; 5403600,23.88; 5404200,23.86; 5404800,23.84; 5405400,23.82;
            5406000,23.8; 5406600,23.76; 5407200,23.68; 5407800,23.68; 5408400,23.68;
            5409000,23.72; 5409600,23.76; 5410200,23.77; 5410800,23.72; 5411400,23.76;
            5412000,23.79; 5412600,23.81; 5413200,23.85; 5413800,23.84; 5414400,23.84;
            5415000,23.88; 5415600,23.88; 5416200,21.85; 5416800,23.24; 5417400,23.6;
            5418000,23.84; 5418600,23.84; 5419200,23.88; 5419800,23.88; 5420400,23.84;
            5421000,23.72; 5421600,23.64; 5422200,23.6; 5422800,23.56; 5423400,23.56;
            5424000,23.56; 5424600,23.52; 5425200,23.5; 5425800,23.44; 5426400,23.4;
            5427000,23.36; 5427600,23.27; 5428200,23.19; 5428800,23.13; 5429400,23.06;
            5430000,22.99; 5430600,22.87; 5431200,22.83; 5431800,22.74; 5432400,22.7;
            5433000,22.62; 5433600,22.58; 5434200,22.5; 5434800,22.46; 5435400,22.42;
            5436000,22.38; 5436600,22.34; 5437200,22.3; 5437800,22.26; 5438400,22.22;
            5439000,22.18; 5439600,22.13; 5440200,22.09; 5440800,22.05; 5441400,22.01;
            5442000,22.02; 5442600,21.97; 5443200,21.97; 5443800,21.93; 5444400,21.89;
            5445000,21.85; 5445600,21.82; 5446200,21.81; 5446800,21.81; 5447400,21.77;
            5448000,21.76; 5448600,21.77; 5449200,21.77; 5449800,21.77; 5450400,21.73;
            5451000,21.73; 5451600,21.77; 5452200,21.77; 5452800,21.73; 5453400,21.73;
            5454000,21.73; 5454600,21.73; 5455200,21.73; 5455800,21.73; 5456400,21.73;
            5457000,21.73; 5457600,21.73; 5458200,21.78; 5458800,21.89; 5459400,22.01;
            5460000,22.14; 5460600,22.26; 5461200,22.38; 5461800,22.45; 5462400,22.42;
            5463000,22.3; 5463600,22.22; 5464200,22.18; 5464800,22.14; 5465400,22.18;
            5466000,22.18; 5466600,22.22; 5467200,22.22; 5467800,22.25; 5468400,22.26;
            5469000,22.3; 5469600,22.37; 5470200,22.47; 5470800,22.46; 5471400,22.51;
            5472000,22.62; 5472600,22.74; 5473200,22.83; 5473800,22.96; 5474400,23;
            5475000,23.07; 5475600,23.16; 5476200,23.23; 5476800,23.27; 5477400,23.4;
            5478000,23.35; 5478600,23.35; 5479200,23.36; 5479800,23.43; 5480400,23.48;
            5481000,23.56; 5481600,23.56; 5482200,23.54; 5482800,23.56; 5483400,23.52;
            5484000,23.44; 5484600,23.36; 5485200,23.28; 5485800,23.27; 5486400,23.23;
            5487000,23.19; 5487600,23.2; 5488200,23.25; 5488800,23.36; 5489400,23.32;
            5490000,23.3; 5490600,23.36; 5491200,23.32; 5491800,23.31; 5492400,23.32;
            5493000,23.36; 5493600,23.36; 5494200,23.48; 5494800,23.52; 5495400,23.57;
            5496000,23.56; 5496600,23.56; 5497200,23.6; 5497800,23.6; 5498400,23.72;
            5499000,23.64; 5499600,23.64; 5500200,23.64; 5500800,23.66; 5501400,23.64;
            5502000,23.63; 5502600,23.64; 5503200,23.65; 5503800,23.6; 5504400,23.64;
            5505000,23.56; 5505600,23.4; 5506200,23.36; 5506800,23.32; 5507400,23.27;
            5508000,23.23; 5508600,23.19; 5509200,23.19; 5509800,23.15; 5510400,23.11;
            5511000,23.11; 5511600,23.07; 5512200,23.05; 5512800,23.03; 5513400,22.95;
            5514000,22.92; 5514600,22.81; 5515200,22.74; 5515800,22.66; 5516400,22.62;
            5517000,22.54; 5517600,22.46; 5518200,22.43; 5518800,22.42; 5519400,22.3;
            5520000,22.26; 5520600,22.22; 5521200,22.18; 5521800,22.14; 5522400,22.1;
            5523000,22.09; 5523600,22.05; 5524200,22.06; 5524800,22.01; 5525400,22.01;
            5526000,21.97; 5526600,21.97; 5527200,21.93; 5527800,21.89; 5528400,21.85;
            5529000,21.81; 5529600,21.81; 5530200,21.77; 5530800,21.73; 5531400,21.69;
            5532000,21.65; 5532600,21.65; 5533200,21.6; 5533800,21.6; 5534400,21.58;
            5535000,21.52; 5535600,21.52; 5536200,21.48; 5536800,21.44; 5537400,21.41;
            5538000,21.4; 5538600,21.36; 5539200,21.32; 5539800,21.32; 5540400,21.28;
            5541000,21.26; 5541600,21.24; 5542200,21.18; 5542800,21.16; 5543400,21.12;
            5544000,21.13; 5544600,21.08; 5545200,21.08; 5545800,21.04; 5546400,21;
            5547000,20.99; 5547600,20.97; 5548200,20.94; 5548800,20.91; 5549400,20.87;
            5550000,20.83; 5550600,20.81; 5551200,20.79; 5551800,20.75; 5552400,20.71;
            5553000,20.71; 5553600,20.67; 5554200,20.67; 5554800,20.63; 5555400,20.63;
            5556000,20.63; 5556600,20.62; 5557200,20.55; 5557800,20.55; 5558400,20.54;
            5559000,20.52; 5559600,20.51; 5560200,20.51; 5560800,20.51; 5561400,20.51;
            5562000,20.51; 5562600,20.51; 5563200,20.51; 5563800,20.51; 5564400,20.51;
            5565000,20.49; 5565600,20.46; 5566200,20.46; 5566800,20.46; 5567400,20.5;
            5568000,20.54; 5568600,20.55; 5569200,20.59; 5569800,20.63; 5570400,20.63;
            5571000,20.71; 5571600,20.74; 5572200,20.76; 5572800,20.83; 5573400,20.92;
            5574000,21; 5574600,21.08; 5575200,21.2; 5575800,21.32; 5576400,21.4; 5577000,
            21.48; 5577600,21.52; 5578200,21.45; 5578800,21.4; 5579400,21.32; 5580000,
            21.32; 5580600,21.31; 5581200,21.28; 5581800,21.28; 5582400,21.24; 5583000,
            21.2; 5583600,21.2; 5584200,21.2; 5584800,21.16; 5585400,21.16; 5586000,
            21.12; 5586600,21.07; 5587200,21.03; 5587800,21; 5588400,20.94; 5589000,
            20.9; 5589600,20.87; 5590200,20.75; 5590800,20.75; 5591400,20.67; 5592000,
            20.67; 5592600,20.67; 5593200,20.67; 5593800,20.63; 5594400,20.59; 5595000,
            20.55; 5595600,20.55; 5596200,20.51; 5596800,20.46; 5597400,20.42; 5598000,
            20.42; 5598600,20.38; 5599200,20.37; 5599800,20.34; 5600400,20.34; 5601000,
            20.34; 5601600,20.3; 5602200,20.32; 5602800,20.3; 5603400,20.3; 5604000,
            20.26; 5604600,20.26; 5605200,20.26; 5605800,20.22; 5606400,20.22; 5607000,
            20.2; 5607600,20.21; 5608200,20.18; 5608800,20.19; 5609400,20.19; 5610000,
            20.18; 5610600,20.18; 5611200,20.16; 5611800,20.14; 5612400,20.14; 5613000,
            20.1; 5613600,20.18; 5614200,20.3; 5614800,20.38; 5615400,20.42; 5616000,
            20.46; 5616600,20.51; 5617200,20.55; 5617800,20.59; 5618400,20.63; 5619000,
            20.64; 5619600,20.67; 5620200,20.7; 5620800,20.71; 5621400,20.72; 5622000,
            20.75; 5622600,20.78; 5623200,20.79; 5623800,20.79; 5624400,20.8; 5625000,
            20.83; 5625600,20.83; 5626200,20.83; 5626800,20.88; 5627400,20.87; 5628000,
            20.87; 5628600,20.88; 5629200,20.88; 5629800,20.92; 5630400,20.91; 5631000,
            20.9; 5631600,20.91; 5632200,20.91; 5632800,20.87; 5633400,20.9; 5634000,
            20.92; 5634600,20.92; 5635200,20.9; 5635800,20.91; 5636400,20.87; 5637000,
            20.88; 5637600,20.91; 5638200,20.88; 5638800,20.87; 5639400,20.91; 5640000,
            20.91; 5640600,20.91; 5641200,20.91; 5641800,20.91; 5642400,20.91; 5643000,
            20.92; 5643600,20.93; 5644200,20.97; 5644800,20.96; 5645400,20.96; 5646000,
            20.99; 5646600,21; 5647200,21.04; 5647800,21.04; 5648400,21.06; 5649000,
            21.08; 5649600,21.08; 5650200,21.12; 5650800,21.12; 5651400,21.16; 5652000,
            21.2; 5652600,21.28; 5653200,21.32; 5653800,21.39; 5654400,21.4; 5655000,
            21.4; 5655600,21.41; 5656200,21.44; 5656800,21.44; 5657400,21.44; 5658000,
            21.48; 5658600,21.48; 5659200,21.5; 5659800,21.48; 5660400,21.47; 5661000,
            21.44; 5661600,21.44; 5662200,21.44; 5662800,21.44; 5663400,21.48; 5664000,
            21.52; 5664600,21.54; 5665200,21.52; 5665800,21.52; 5666400,21.52; 5667000,
            21.57; 5667600,21.58; 5668200,21.56; 5668800,21.57; 5669400,21.62; 5670000,
            21.7; 5670600,21.73; 5671200,21.81; 5671800,21.82; 5672400,21.77; 5673000,
            21.7; 5673600,21.68; 5674200,21.65; 5674800,21.64; 5675400,21.62; 5676000,
            21.56; 5676600,21.56; 5677200,21.52; 5677800,21.52; 5678400,21.48; 5679000,
            21.44; 5679600,21.44; 5680200,21.4; 5680800,21.4; 5681400,21.36; 5682000,
            21.36; 5682600,21.32; 5683200,21.32; 5683800,21.32; 5684400,21.32; 5685000,
            21.28; 5685600,21.28; 5686200,21.25; 5686800,21.24; 5687400,21.21; 5688000,
            21.2; 5688600,21.2; 5689200,21.2; 5689800,21.18; 5690400,21.16; 5691000,
            21.16; 5691600,21.16; 5692200,21.16; 5692800,21.12; 5693400,21.12; 5694000,
            21.12; 5694600,21.12; 5695200,21.12; 5695800,21.13; 5696400,21.08; 5697000,
            21.08; 5697600,21.08; 5698200,21.08; 5698800,21.08; 5699400,21.07; 5700000,
            21.04; 5700600,21.04; 5701200,21.04; 5701800,21.04; 5702400,21.04; 5703000,
            21.04; 5703600,21; 5704200,21; 5704800,21; 5705400,21; 5706000,21; 5706600,
            21; 5707200,20.96; 5707800,20.96; 5708400,20.96; 5709000,20.96; 5709600,
            20.96; 5710200,20.96; 5710800,20.96; 5711400,20.96; 5712000,20.96; 5712600,
            20.92; 5713200,20.95; 5713800,20.94; 5714400,20.91; 5715000,20.94; 5715600,
            20.95; 5716200,20.91; 5716800,20.91; 5717400,20.98; 5718000,21.12; 5718600,
            21.2; 5719200,21.32; 5719800,21.46; 5720400,21.6; 5721000,21.65; 5721600,
            21.69; 5722200,21.66; 5722800,21.65; 5723400,21.64; 5724000,21.61; 5724600,
            21.6; 5725200,21.65; 5725800,21.65; 5726400,21.69; 5727000,21.73; 5727600,
            21.77; 5728200,21.85; 5728800,21.96; 5729400,22.01; 5730000,22.06; 5730600,
            22.14; 5731200,22.3; 5731800,22.42; 5732400,22.5; 5733000,22.55; 5733600,
            22.62; 5734200,22.7; 5734800,22.76; 5735400,22.83; 5736000,22.91; 5736600,
            22.96; 5737200,23.03; 5737800,23.07; 5738400,23.15; 5739000,23.19; 5739600,
            23.23; 5740200,23.27; 5740800,23.35; 5741400,23.4; 5742000,23.4; 5742600,
            23.4; 5743200,23.44; 5743800,23.4; 5744400,23.43; 5745000,23.48; 5745600,
            23.48; 5746200,23.52; 5746800,23.6; 5747400,23.72; 5748000,23.86; 5748600,
            24.02; 5749200,24.04; 5749800,24.01; 5750400,24.01; 5751000,23.99; 5751600,
            24.02; 5752200,24.05; 5752800,24.05; 5753400,24.04; 5754000,24.13; 5754600,
            24.13; 5755200,24.06; 5755800,24.01; 5756400,23.96; 5757000,23.95; 5757600,
            23.92; 5758200,23.88; 5758800,23.87; 5759400,23.88; 5760000,23.92; 5760600,
            24.01; 5761200,24; 5761800,23.97; 5762400,23.97; 5763000,23.89; 5763600,
            23.89; 5764200,23.92; 5764800,23.88; 5765400,23.8; 5766000,23.72; 5766600,
            23.68; 5767200,23.64; 5767800,23.56; 5768400,23.48; 5769000,23.44; 5769600,
            23.39; 5770200,23.32; 5770800,23.28; 5771400,23.23; 5772000,23.19; 5772600,
            23.11; 5773200,23.03; 5773800,22.95; 5774400,22.87; 5775000,22.74; 5775600,
            22.66; 5776200,22.62; 5776800,22.54; 5777400,22.46; 5778000,22.42; 5778600,
            22.34; 5779200,22.26; 5779800,22.22; 5780400,22.18; 5781000,22.09; 5781600,
            22.05; 5782200,22.01; 5782800,21.97; 5783400,21.93; 5784000,21.89; 5784600,
            21.82; 5785200,21.81; 5785800,21.77; 5786400,21.73; 5787000,21.73; 5787600,
            21.68; 5788200,21.65; 5788800,21.65; 5789400,21.6; 5790000,21.6; 5790600,
            21.6; 5791200,21.56; 5791800,21.52; 5792400,21.52; 5793000,21.48; 5793600,
            21.48; 5794200,21.51; 5794800,21.52; 5795400,21.52; 5796000,21.56; 5796600,
            21.59; 5797200,21.57; 5797800,21.6; 5798400,21.62; 5799000,21.6; 5799600,
            21.6; 5800200,21.6; 5800800,21.6; 5801400,21.64; 5802000,21.61; 5802600,
            21.63; 5803200,21.64; 5803800,21.65; 5804400,21.94; 5805000,22.09; 5805600,
            22.33; 5806200,22.5; 5806800,22.5; 5807400,22.46; 5808000,22.38; 5808600,
            22.26; 5809200,22.14; 5809800,22.09; 5810400,22.18; 5811000,22.3; 5811600,
            22.38; 5812200,22.42; 5812800,22.46; 5813400,22.42; 5814000,22.38; 5814600,
            22.34; 5815200,22.3; 5815800,22.3; 5816400,22.37; 5817000,22.41; 5817600,
            22.5; 5818200,22.62; 5818800,22.7; 5819400,22.76; 5820000,22.78; 5820600,
            22.87; 5821200,22.91; 5821800,22.95; 5822400,22.96; 5823000,23.04; 5823600,
            23.11; 5824200,23.16; 5824800,23.23; 5825400,23.28; 5826000,23.36; 5826600,
            23.36; 5827200,23.37; 5827800,23.4; 5828400,23.44; 5829000,23.44; 5829600,
            23.45; 5830200,23.44; 5830800,23.4; 5831400,23.39; 5832000,23.44; 5832600,
            23.49; 5833200,23.64; 5833800,23.68; 5834400,23.72; 5835000,23.84; 5835600,
            23.84; 5836200,23.8; 5836800,23.76; 5837400,23.74; 5838000,23.72; 5838600,
            23.76; 5839200,23.8; 5839800,23.8; 5840400,23.82; 5841000,23.8; 5841600,
            23.76; 5842200,23.76; 5842800,23.72; 5843400,23.72; 5844000,23.68; 5844600,
            23.65; 5845200,23.76; 5845800,23.78; 5846400,23.76; 5847000,23.76; 5847600,
            23.88; 5848200,23.86; 5848800,23.82; 5849400,23.86; 5850000,23.76; 5850600,
            23.92; 5851200,23.92; 5851800,23.91; 5852400,23.84; 5853000,23.8; 5853600,
            23.72; 5854200,23.68; 5854800,23.64; 5855400,23.59; 5856000,23.53; 5856600,
            23.52; 5857200,23.48; 5857800,23.44; 5858400,23.4; 5859000,23.32; 5859600,
            23.23; 5860200,23.11; 5860800,23.03; 5861400,22.95; 5862000,22.79; 5862600,
            22.74; 5863200,22.63; 5863800,22.62; 5864400,22.54; 5865000,22.46; 5865600,
            22.41; 5866200,22.39; 5866800,22.3; 5867400,22.26; 5868000,22.22; 5868600,
            22.18; 5869200,22.14; 5869800,22.09; 5870400,22.05; 5871000,22.04; 5871600,
            21.98; 5872200,21.97; 5872800,21.93; 5873400,21.86; 5874000,21.85; 5874600,
            21.81; 5875200,21.77; 5875800,21.77; 5876400,21.73; 5877000,21.73; 5877600,
            21.69; 5878200,21.7; 5878800,21.65; 5879400,21.62; 5880000,21.6; 5880600,
            21.56; 5881200,21.56; 5881800,21.56; 5882400,21.77; 5883000,21.98; 5883600,
            22.1; 5884200,22.27; 5884800,22.39; 5885400,22.46; 5886000,22.54; 5886600,
            22.58; 5887200,22.54; 5887800,22.5; 5888400,22.42; 5889000,22.38; 5889600,
            22.42; 5890200,22.42; 5890800,22.46; 5891400,22.46; 5892000,22.46; 5892600,
            22.46; 5893200,22.46; 5893800,22.51; 5894400,22.5; 5895000,22.5; 5895600,
            22.5; 5896200,22.53; 5896800,22.54; 5897400,22.58; 5898000,22.58; 5898600,
            22.58; 5899200,22.58; 5899800,22.58; 5900400,22.58; 5901000,22.62; 5901600,
            22.62; 5902200,22.64; 5902800,22.62; 5903400,22.62; 5904000,22.62; 5904600,
            22.62; 5905200,22.69; 5905800,22.7; 5906400,22.8; 5907000,22.92; 5907600,
            23.03; 5908200,23.11; 5908800,23.18; 5909400,23.15; 5910000,23.24; 5910600,
            23.32; 5911200,23.4; 5911800,23.39; 5912400,23.3; 5913000,23.26; 5913600,
            23.27; 5914200,23.23; 5914800,23.19; 5915400,23.23; 5916000,23.27; 5916600,
            23.27; 5917200,23.32; 5917800,23.33; 5918400,23.36; 5919000,23.37; 5919600,
            23.4; 5920200,23.47; 5920800,23.56; 5921400,23.71; 5922000,23.76; 5922600,
            23.8; 5923200,23.82; 5923800,23.8; 5924400,23.84; 5925000,23.85; 5925600,
            23.82; 5926200,23.85; 5926800,23.84; 5927400,23.8; 5928000,23.76; 5928600,
            23.76; 5929200,23.74; 5929800,23.72; 5930400,23.75; 5931000,23.72; 5931600,
            23.72; 5932200,23.77; 5932800,23.77; 5933400,23.76; 5934000,23.79; 5934600,
            23.8; 5935200,23.8; 5935800,23.81; 5936400,23.8; 5937000,23.8; 5937600,23.84;
            5938200,23.84; 5938800,23.8; 5939400,23.8; 5940000,23.84; 5940600,23.77;
            5941200,23.72; 5941800,23.69; 5942400,23.64; 5943000,23.56; 5943600,23.52;
            5944200,23.44; 5944800,23.4; 5945400,23.6; 5946000,23.32; 5946600,23.24;
            5947200,23.15; 5947800,23.07; 5948400,22.95; 5949000,22.82; 5949600,22.74;
            5950200,22.62; 5950800,22.54; 5951400,22.46; 5952000,22.46; 5952600,22.38;
            5953200,22.34; 5953800,22.26; 5954400,22.18; 5955000,22.14; 5955600,22.09;
            5956200,22.05; 5956800,22.01; 5957400,21.97; 5958000,21.93; 5958600,21.89;
            5959200,21.9; 5959800,21.82; 5960400,21.81; 5961000,21.77; 5961600,21.75;
            5962200,21.74; 5962800,21.69; 5963400,21.68; 5964000,21.65; 5964600,21.65;
            5965200,21.88; 5965800,21.6; 5966400,21.6; 5967000,21.6; 5967600,21.56;
            5968200,21.6; 5968800,21.81; 5969400,21.93; 5970000,22.01; 5970600,22.13;
            5971200,22.22; 5971800,22.3; 5972400,22.3; 5973000,22.27; 5973600,22.18;
            5974200,22.14; 5974800,22.09; 5975400,22.14; 5976000,22.18; 5976600,22.22;
            5977200,22.23; 5977800,22.26; 5978400,22.22; 5979000,22.21; 5979600,22.18;
            5980200,22.18; 5980800,22.17; 5981400,22.14; 5982000,22.16; 5982600,22.16;
            5983200,22.18; 5983800,22.18; 5984400,22.18; 5985000,22.18; 5985600,22.18;
            5986200,22.18; 5986800,22.25; 5987400,22.26; 5988000,22.3; 5988600,22.39;
            5989200,22.42; 5989800,22.5; 5990400,22.5; 5991000,22.58; 5991600,22.63;
            5992200,22.71; 5992800,22.78; 5993400,22.91; 5994000,23.04; 5994600,23.08;
            5995200,23.12; 5995800,23.11; 5996400,23.11; 5997000,23.12; 5997600,23.15;
            5998200,23.23; 5998800,23.27; 5999400,23.32; 6000000,23.36; 6000600,23.44;
            6001200,23.52; 6001800,23.56; 6002400,23.56; 6003000,23.6; 6003600,23.64;
            6004200,23.64; 6004800,23.64; 6005400,23.76; 6006000,23.88; 6006600,23.92;
            6007200,24.13; 6007800,24.3; 6008400,24.42; 6009000,24.54; 6009600,24.62;
            6010200,24.72; 6010800,24.78; 6011400,24.82; 6012000,24.85; 6012600,24.86;
            6013200,24.86; 6013800,24.9; 6014400,24.9; 6015000,24.94; 6015600,24.86;
            6016200,24.66; 6016800,24.62; 6017400,24.63; 6018000,24.65; 6018600,24.78;
            6019200,25.12; 6019800,24.9; 6020400,24.7; 6021000,24.62; 6021600,24.56;
            6022200,24.45; 6022800,24.37; 6023400,24.25; 6024000,24.21; 6024600,24.13;
            6025200,24.05; 6025800,24.01; 6026400,23.92; 6027000,23.9; 6027600,23.81;
            6028200,23.76; 6028800,23.73; 6029400,23.69; 6030000,23.6; 6030600,23.56;
            6031200,23.52; 6031800,23.48; 6032400,23.44; 6033000,23.36; 6033600,23.27;
            6034200,23.16; 6034800,23.07; 6035400,22.99; 6036000,22.88; 6036600,22.82;
            6037200,22.7; 6037800,22.62; 6038400,22.62; 6039000,22.54; 6039600,22.5;
            6040200,22.46; 6040800,22.41; 6041400,22.38; 6042000,22.34; 6042600,22.31;
            6043200,22.26; 6043800,22.22; 6044400,22.18; 6045000,22.14; 6045600,22.09;
            6046200,22.06; 6046800,22.04; 6047400,21.97; 6048000,21.97; 6048600,21.93;
            6049200,21.89; 6049800,21.87; 6050400,21.82; 6051000,21.81; 6051600,21.81;
            6052200,21.77; 6052800,21.73; 6053400,21.69; 6054000,21.65; 6054600,21.65;
            6055200,21.73; 6055800,21.78; 6056400,21.77; 6057000,21.77; 6057600,21.73;
            6058200,21.74; 6058800,21.73; 6059400,21.73; 6060000,21.69; 6060600,21.65;
            6061200,21.65; 6061800,21.64; 6062400,21.6; 6063000,21.56; 6063600,21.54;
            6064200,21.48; 6064800,21.47; 6065400,21.44; 6066000,21.43; 6066600,21.4;
            6067200,21.4; 6067800,21.36; 6068400,21.32; 6069000,21.32; 6069600,21.28;
            6070200,21.24; 6070800,21.2; 6071400,21.16; 6072000,21.16; 6072600,21.13;
            6073200,21.16; 6073800,21.13; 6074400,21.16; 6075000,21.16; 6075600,21.2;
            6076200,21.24; 6076800,21.28; 6077400,21.4; 6078000,21.48; 6078600,21.56;
            6079200,21.6; 6079800,21.69; 6080400,21.81; 6081000,21.85; 6081600,21.93;
            6082200,22.09; 6082800,22.3; 6083400,22.58; 6084000,22.78; 6084600,23.03;
            6085200,23.2; 6085800,23.32; 6086400,23.4; 6087000,23.48; 6087600,23.6;
            6088200,23.68; 6088800,23.68; 6089400,23.73; 6090000,23.76; 6090600,23.84;
            6091200,23.86; 6091800,23.92; 6092400,24.05; 6093000,24.14; 6093600,24.26;
            6094200,24.33; 6094800,24.41; 6095400,24.5; 6096000,24.57; 6096600,24.58;
            6097200,24.62; 6097800,24.66; 6098400,24.62; 6099000,24.66; 6099600,24.75;
            6100200,24.7; 6100800,24.7; 6101400,24.66; 6102000,24.62; 6102600,24.66;
            6103200,24.71; 6103800,24.66; 6104400,24.62; 6105000,24.58; 6105600,24.71;
            6106200,24.59; 6106800,24.57; 6107400,24.59; 6108000,24.58; 6108600,24.54;
            6109200,24.46; 6109800,24.45; 6110400,24.4; 6111000,24.35; 6111600,24.3;
            6112200,24.25; 6112800,24.25; 6113400,24.21; 6114000,24.17; 6114600,24.13;
            6115200,24.1; 6115800,24.08; 6116400,24.05; 6117000,24; 6117600,23.88; 6118200,
            23.76; 6118800,23.68; 6119400,23.61; 6120000,23.52; 6120600,23.44; 6121200,
            23.36; 6121800,23.24; 6122400,23.12; 6123000,23.07; 6123600,22.95; 6124200,
            22.91; 6124800,22.83; 6125400,22.78; 6126000,22.72; 6126600,22.62; 6127200,
            22.58; 6127800,22.51; 6128400,22.46; 6129000,22.42; 6129600,22.42; 6130200,
            22.34; 6130800,22.3; 6131400,22.26; 6132000,22.26; 6132600,22.18; 6133200,
            22.14; 6133800,22.05; 6134400,22.01; 6135000,21.98; 6135600,21.93; 6136200,
            21.89; 6136800,21.85; 6137400,21.82; 6138000,21.77; 6138600,21.73; 6139200,
            21.69; 6139800,21.65; 6140400,21.6; 6141000,21.6; 6141600,21.56; 6142200,
            21.53; 6142800,21.48; 6143400,21.44; 6144000,21.44; 6144600,21.4; 6145200,
            21.41; 6145800,21.37; 6146400,21.36; 6147000,21.36; 6147600,21.32; 6148200,
            21.32; 6148800,21.28; 6149400,21.28; 6150000,21.24; 6150600,21.24; 6151200,
            21.24; 6151800,21.2; 6152400,21.2; 6153000,21.16; 6153600,21.16; 6154200,
            21.16; 6154800,21.13; 6155400,21.12; 6156000,21.12; 6156600,21.08; 6157200,
            21.08; 6157800,21.08; 6158400,21.05; 6159000,21.04; 6159600,21.04; 6160200,
            21.02; 6160800,21.04; 6161400,21.04; 6162000,21.04; 6162600,21.04; 6163200,
            21.04; 6163800,21.06; 6164400,21.08; 6165000,21.08; 6165600,21.12; 6166200,
            21.12; 6166800,21.15; 6167400,21.16; 6168000,21.2; 6168600,21.2; 6169200,
            21.24; 6169800,21.28; 6170400,21.28; 6171000,21.32; 6171600,21.33; 6172200,
            21.36; 6172800,21.4; 6173400,21.44; 6174000,21.49; 6174600,21.5; 6175200,
            21.54; 6175800,21.6; 6176400,21.6; 6177000,21.69; 6177600,21.73; 6178200,
            21.81; 6178800,21.85; 6179400,21.89; 6180000,21.98; 6180600,21.98; 6181200,
            21.97; 6181800,21.97; 6182400,21.94; 6183000,21.93; 6183600,21.93; 6184200,
            21.88; 6184800,21.9; 6185400,21.85; 6186000,21.83; 6186600,21.81; 6187200,
            21.81; 6187800,21.81; 6188400,21.77; 6189000,21.77; 6189600,21.73; 6190200,
            21.73; 6190800,21.66; 6191400,21.64; 6192000,21.6; 6192600,21.6; 6193200,
            21.58; 6193800,21.56; 6194400,21.52; 6195000,21.52; 6195600,21.48; 6196200,
            21.44; 6196800,21.44; 6197400,21.4; 6198000,21.36; 6198600,21.32; 6199200,
            21.31; 6199800,21.28; 6200400,21.24; 6201000,21.2; 6201600,21.16; 6202200,
            21.15; 6202800,21.12; 6203400,21.08; 6204000,21.08; 6204600,21.08; 6205200,
            21.05; 6205800,21.02; 6206400,21; 6207000,20.98; 6207600,20.96; 6208200,
            20.96; 6208800,20.91; 6209400,20.92; 6210000,20.88; 6210600,20.87; 6211200,
            20.87; 6211800,20.83; 6212400,20.83; 6213000,20.79; 6213600,20.79; 6214200,
            20.79; 6214800,20.75; 6215400,20.75; 6216000,20.75; 6216600,20.74; 6217200,
            20.71; 6217800,20.7; 6218400,20.79; 6219000,20.87; 6219600,20.96; 6220200,
            20.96; 6220800,21.01; 6221400,21.04; 6222000,21.08; 6222600,21.08; 6223200,
            21.12; 6223800,21.12; 6224400,21.16; 6225000,21.16; 6225600,21.16; 6226200,
            21.16; 6226800,21.21; 6227400,21.2; 6228000,21.21; 6228600,21.24; 6229200,
            21.24; 6229800,21.24; 6230400,21.27; 6231000,21.28; 6231600,21.28; 6232200,
            21.28; 6232800,21.28; 6233400,21.28; 6234000,21.28; 6234600,21.32; 6235200,
            21.32; 6235800,21.32; 6236400,21.32; 6237000,21.33; 6237600,21.32; 6238200,
            21.32; 6238800,21.32; 6239400,21.33; 6240000,21.32; 6240600,21.33; 6241200,
            21.32; 6241800,21.32; 6242400,21.34; 6243000,21.36; 6243600,21.36; 6244200,
            21.36; 6244800,21.36; 6245400,21.36; 6246000,21.36; 6246600,21.36; 6247200,
            21.36; 6247800,21.36; 6248400,21.4; 6249000,21.4; 6249600,21.4; 6250200,
            21.4; 6250800,21.44; 6251400,21.44; 6252000,21.44; 6252600,21.44; 6253200,
            21.5; 6253800,21.48; 6254400,21.54; 6255000,21.52; 6255600,21.6; 6256200,
            21.61; 6256800,21.65; 6257400,21.69; 6258000,21.77; 6258600,21.81; 6259200,
            21.82; 6259800,21.86; 6260400,21.89; 6261000,21.97; 6261600,21.97; 6262200,
            21.97; 6262800,21.97; 6263400,22.01; 6264000,22.05; 6264600,22.14; 6265200,
            22.18; 6265800,22.22; 6266400,22.26; 6267000,22.26; 6267600,22.22; 6268200,
            22.27; 6268800,22.26; 6269400,22.26; 6270000,22.26; 6270600,22.26; 6271200,
            22.29; 6271800,22.31; 6272400,22.32; 6273000,22.32; 6273600,22.33; 6274200,
            22.3; 6274800,22.34; 6275400,22.3; 6276000,22.34; 6276600,22.34; 6277200,
            22.33; 6277800,22.3; 6278400,22.3; 6279000,22.26; 6279600,22.3; 6280200,
            22.29; 6280800,22.26; 6281400,22.22; 6282000,22.22; 6282600,22.17; 6283200,
            22.13; 6283800,22.09; 6284400,22.05; 6285000,22.01; 6285600,21.97; 6286200,
            21.93; 6286800,21.92; 6287400,21.9; 6288000,21.85; 6288600,21.85; 6289200,
            21.81; 6289800,21.81; 6290400,21.81; 6291000,21.78; 6291600,21.77; 6292200,
            21.73; 6292800,21.73; 6293400,21.74; 6294000,21.69; 6294600,21.69; 6295200,
            21.66; 6295800,21.66; 6296400,21.66; 6297000,21.65; 6297600,21.61; 6298200,
            21.6; 6298800,21.6; 6299400,21.6; 6300000,21.6; 6300600,21.56; 6301200,21.56;
            6301800,21.56; 6302400,21.56; 6303000,21.56; 6303600,21.52; 6304200,21.52;
            6304800,21.52; 6305400,21.52; 6306000,21.52; 6306600,21.52; 6307200,21.48;
            6307800,21.48; 6308400,21.48; 6309000,21.48; 6309600,21.47; 6310200,21.44;
            6310800,21.46; 6311400,21.48; 6312000,21.48; 6312600,21.44; 6313200,21.44;
            6313800,21.44; 6314400,21.44; 6315000,21.44; 6315600,21.44; 6316200,21.44;
            6316800,21.44; 6317400,21.44; 6318000,21.44; 6318600,21.41; 6319200,21.4;
            6319800,21.4; 6320400,21.4; 6321000,21.4; 6321600,21.4; 6322200,21.4; 6322800,
            21.6; 6323400,21.77; 6324000,21.85; 6324600,21.97; 6325200,22.09; 6325800,
            22.14; 6326400,22.18; 6327000,22.17; 6327600,22.15; 6328200,22.14; 6328800,
            22.14; 6329400,22.18; 6330000,22.18; 6330600,22.18; 6331200,22.22; 6331800,
            22.26; 6332400,22.35; 6333000,22.38; 6333600,22.31; 6334200,22.42; 6334800,
            22.5; 6335400,22.57; 6336000,22.7; 6336600,22.83; 6337200,22.96; 6337800,
            23.08; 6338400,23.15; 6339000,23.27; 6339600,23.31; 6340200,23.36; 6340800,
            23.43; 6341400,23.52; 6342000,23.59; 6342600,23.64; 6343200,23.68; 6343800,
            23.73; 6344400,23.77; 6345000,23.84; 6345600,23.88; 6346200,23.92; 6346800,
            23.93; 6347400,24; 6348000,23.97; 6348600,24.13; 6349200,24.29; 6349800,
            24.37; 6350400,24.21; 6351000,24.09; 6351600,22.3; 6352200,22.01; 6352800,
            22.84; 6353400,22.01; 6354000,23.11; 6354600,23.64; 6355200,23.84; 6355800,
            24.01; 6356400,24.06; 6357000,24.13; 6357600,24.13; 6358200,24.18; 6358800,
            24.21; 6359400,24.22; 6360000,24.21; 6360600,24.26; 6361200,24.3; 6361800,
            24.33; 6362400,24.37; 6363000,24.4; 6363600,24.41; 6364200,24.42; 6364800,
            24.53; 6365400,24.5; 6366000,24.38; 6366600,24.29; 6367200,24.25; 6367800,
            24.21; 6368400,24.13; 6369000,24.14; 6369600,24.09; 6370200,24.05; 6370800,
            24.09; 6371400,24.06; 6372000,24.04; 6372600,24.06; 6373200,24.02; 6373800,
            23.96; 6374400,23.88; 6375000,23.8; 6375600,23.72; 6376200,23.72; 6376800,
            23.64; 6377400,23.56; 6378000,23.46; 6378600,23.4; 6379200,23.32; 6379800,
            23.19; 6380400,23.12; 6381000,23.03; 6381600,22.91; 6382200,22.84; 6382800,
            22.71; 6383400,22.66; 6384000,22.61; 6384600,22.54; 6385200,22.48; 6385800,
            22.43; 6386400,22.38; 6387000,22.34; 6387600,22.26; 6388200,22.25; 6388800,
            22.19; 6389400,22.15; 6390000,22.09; 6390600,22.09; 6391200,22.05; 6391800,
            22.01; 6392400,21.97; 6393000,21.94; 6393600,21.93; 6394200,21.89; 6394800,
            21.86; 6395400,21.85; 6396000,21.81; 6396600,21.81; 6397200,21.81; 6397800,
            21.77; 6398400,21.75; 6399000,21.77; 6399600,21.77; 6400200,21.77; 6400800,
            21.94; 6401400,22.06; 6402000,22.17; 6402600,22.26; 6403200,22.41; 6403800,
            22.47; 6404400,22.5; 6405000,22.54; 6405600,22.58; 6406200,22.54; 6406800,
            22.55; 6407400,22.5; 6408000,22.5; 6408600,22.5; 6409200,22.5; 6409800,22.5;
            6410400,22.5; 6411000,22.5; 6411600,22.5; 6412200,22.5; 6412800,22.5; 6413400,
            22.5; 6414000,22.46; 6414600,22.49; 6415200,22.46; 6415800,22.46; 6416400,
            22.46; 6417000,22.46; 6417600,22.5; 6418200,22.5; 6418800,22.54; 6419400,
            22.76; 6420000,22.69; 6420600,22.83; 6421200,22.82; 6421800,22.84; 6422400,
            22.83; 6423000,22.9; 6423600,22.95; 6424200,22.99; 6424800,22.99; 6425400,
            23.07; 6426000,23.15; 6426600,23.23; 6427200,23.28; 6427800,23.4; 6428400,
            23.44; 6429000,23.48; 6429600,23.52; 6430200,23.6; 6430800,23.66; 6431400,
            23.66; 6432000,23.68; 6432600,23.72; 6433200,23.72; 6433800,23.74; 6434400,
            23.82; 6435000,23.8; 6435600,23.8; 6436200,23.84; 6436800,23.86; 6437400,
            23.88; 6438000,23.96; 6438600,24.04; 6439200,24.13; 6439800,24.25; 6440400,
            24.34; 6441000,24.41; 6441600,24.45; 6442200,24.49; 6442800,24.5; 6443400,
            24.5; 6444000,24.45; 6444600,24.45; 6445200,24.43; 6445800,24.45; 6446400,
            24.41; 6447000,24.33; 6447600,24.25; 6448200,24.22; 6448800,24.21; 6449400,
            24.13; 6450000,24.21; 6450600,24.24; 6451200,24.21; 6451800,24.25; 6452400,
            24.17; 6453000,24.21; 6453600,24.21; 6454200,24.21; 6454800,24.21; 6455400,
            24.17; 6456000,24.21; 6456600,24.29; 6457200,24.29; 6457800,24.25; 6458400,
            24.26; 6459000,24.21; 6459600,24.21; 6460200,24.17; 6460800,24.17; 6461400,
            24.1; 6462000,24.15; 6462600,24.13; 6463200,24.09; 6463800,24.05; 6464400,
            23.93; 6465000,23.88; 6465600,23.77; 6466200,23.73; 6466800,23.64; 6467400,
            23.64; 6468000,23.56; 6468600,23.44; 6469200,23.32; 6469800,23.27; 6470400,
            23.2; 6471000,23.16; 6471600,23.1; 6472200,23.07; 6472800,23.03; 6473400,
            22.95; 6474000,22.91; 6474600,22.88; 6475200,22.83; 6475800,22.78; 6476400,
            22.78; 6477000,22.74; 6477600,22.7; 6478200,22.66; 6478800,22.62; 6479400,
            22.62; 6480000,22.58; 6480600,22.54; 6481200,22.55; 6481800,22.51; 6482400,
            22.47; 6483000,22.46; 6483600,22.68; 6484200,22.42; 6484800,22.42; 6485400,
            22.38; 6486000,22.34; 6486600,22.34; 6487200,22.46; 6487800,22.63; 6488400,
            22.74; 6489000,22.87; 6489600,22.95; 6490200,23.04; 6490800,23.08; 6491400,
            23.03; 6492000,22.95; 6492600,22.88; 6493200,22.82; 6493800,22.81; 6494400,
            22.87; 6495000,22.91; 6495600,22.95; 6496200,22.96; 6496800,22.91; 6497400,
            22.91; 6498000,22.92; 6498600,22.87; 6499200,22.84; 6499800,22.83; 6500400,
            22.82; 6501000,22.82; 6501600,22.82; 6502200,22.8; 6502800,22.78; 6503400,
            22.78; 6504000,22.78; 6504600,22.78; 6505200,22.74; 6505800,22.74; 6506400,
            22.66; 6507000,22.62; 6507600,22.7; 6508200,22.7; 6508800,22.79; 6509400,
            22.83; 6510000,22.94; 6510600,23.03; 6511200,23.08; 6511800,23.11; 6512400,
            23.19; 6513000,23.24; 6513600,23.18; 6514200,23.23; 6514800,23.32; 6515400,
            23.33; 6516000,23.36; 6516600,23.4; 6517200,23.48; 6517800,23.52; 6518400,
            23.56; 6519000,23.68; 6519600,23.8; 6520200,23.76; 6520800,23.8; 6521400,
            23.76; 6522000,23.76; 6522600,23.72; 6523200,23.67; 6523800,23.64; 6524400,
            23.65; 6525000,23.68; 6525600,23.81; 6526200,23.88; 6526800,23.86; 6527400,
            23.89; 6528000,23.83; 6528600,23.8; 6529200,23.92; 6529800,24; 6530400,24;
            6531000,24.05; 6531600,24.05; 6532200,24.06; 6532800,24.09; 6533400,24.08;
            6534000,24.09; 6534600,24.13; 6535200,24.09; 6535800,24.08; 6536400,24.13;
            6537000,24.17; 6537600,24.17; 6538200,24.25; 6538800,24.29; 6539400,24.31;
            6540000,24.28; 6540600,24.29; 6541200,24.29; 6541800,24.28; 6542400,24.25;
            6543000,24.2; 6543600,24.21; 6544200,24.21; 6544800,24.17; 6545400,24.21;
            6546000,24.21; 6546600,24.21; 6547200,24.13; 6547800,24.09; 6548400,24.01;
            6549000,23.92; 6549600,23.84; 6550200,23.76; 6550800,23.68; 6551400,23.57;
            6552000,23.6; 6552600,23.56; 6553200,23.56; 6553800,23.52; 6554400,23.44;
            6555000,23.32; 6555600,23.23; 6556200,23.19; 6556800,23.19; 6557400,23.15;
            6558000,23.11; 6558600,23.08; 6559200,23.07; 6559800,23.03; 6560400,22.99;
            6561000,22.95; 6561600,22.95; 6562200,22.91; 6562800,22.91; 6563400,22.87;
            6564000,22.83; 6564600,22.83; 6565200,22.79; 6565800,22.78; 6566400,22.74;
            6567000,22.74; 6567600,22.7; 6568200,22.7; 6568800,22.69; 6569400,22.65;
            6570000,22.62; 6570600,22.62; 6571200,22.58; 6571800,22.58; 6572400,22.58;
            6573000,22.54; 6573600,22.55; 6574200,22.5; 6574800,22.5; 6575400,22.5;
            6576000,22.5; 6576600,22.5; 6577200,22.46; 6577800,22.46; 6578400,22.46;
            6579000,22.43; 6579600,22.43; 6580200,22.42; 6580800,22.42; 6581400,22.42;
            6582000,22.51; 6582600,22.7; 6583200,22.78; 6583800,22.87; 6584400,22.91;
            6585000,22.9; 6585600,22.82; 6586200,22.74; 6586800,22.67; 6587400,22.63;
            6588000,22.62; 6588600,22.7; 6589200,22.74; 6589800,22.79; 6590400,22.83;
            6591000,22.83; 6591600,22.79; 6592200,22.8; 6592800,22.79; 6593400,22.91;
            6594000,22.87; 6594600,22.58; 6595200,22.5; 6595800,22.78; 6596400,22.95;
            6597000,23.02; 6597600,23.11; 6598200,23.19; 6598800,23.33; 6599400,23.4;
            6600000,23.44; 6600600,23.48; 6601200,23.56; 6601800,23.64; 6602400,23.68;
            6603000,23.72; 6603600,23.76; 6604200,23.8; 6604800,23.92; 6605400,24.06;
            6606000,24.12; 6606600,24.13; 6607200,24.13; 6607800,24.17; 6608400,24.18;
            6609000,24.22; 6609600,24.25; 6610200,24.34; 6610800,24.45; 6611400,24.53;
            6612000,24.58; 6612600,24.7; 6613200,24.74; 6613800,24.79; 6614400,24.78;
            6615000,24.78; 6615600,24.82; 6616200,24.86; 6616800,24.94; 6617400,24.87;
            6618000,24.86; 6618600,24.9; 6619200,24.91; 6619800,24.78; 6620400,24.82;
            6621000,24.88; 6621600,24.86; 6622200,24.91; 6622800,24.94; 6623400,24.98;
            6624000,25.06; 6624600,25.1; 6625200,25.06; 6625800,25.02; 6626400,24.9;
            6627000,24.85; 6627600,24.83; 6628200,24.78; 6628800,24.66; 6629400,24.58;
            6630000,24.5; 6630600,24.45; 6631200,24.42; 6631800,24.44; 6632400,24.41;
            6633000,24.41; 6633600,24.38; 6634200,24.33; 6634800,24.26; 6635400,24.21;
            6636000,24.17; 6636600,24.13; 6637200,24.05; 6637800,23.96; 6638400,23.88;
            6639000,23.76; 6639600,23.68; 6640200,23.6; 6640800,23.52; 6641400,23.49;
            6642000,23.4; 6642600,23.36; 6643200,23.28; 6643800,23.24; 6644400,23.19;
            6645000,23.15; 6645600,23.1; 6646200,23.07; 6646800,23.02; 6647400,22.95;
            6648000,22.93; 6648600,22.91; 6649200,22.87; 6649800,22.83; 6650400,22.78;
            6651000,22.78; 6651600,22.74; 6652200,22.74; 6652800,22.7; 6653400,22.66;
            6654000,22.68; 6654600,22.62; 6655200,22.61; 6655800,22.58; 6656400,22.58;
            6657000,22.54; 6657600,22.55; 6658200,22.5; 6658800,22.5; 6659400,22.5;
            6660000,22.46; 6660600,22.45; 6661200,22.42; 6661800,22.42; 6662400,22.4;
            6663000,22.38; 6663600,22.38; 6664200,22.39; 6664800,22.34; 6665400,22.34;
            6666000,22.3; 6666600,22.3; 6667200,22.3; 6667800,22.3; 6668400,22.42; 6669000,
            22.5; 6669600,22.62; 6670200,22.76; 6670800,22.84; 6671400,22.91; 6672000,
            22.91; 6672600,22.87; 6673200,22.78; 6673800,22.77; 6674400,22.74; 6675000,
            22.74; 6675600,22.75; 6676200,22.76; 6676800,22.78; 6677400,22.83; 6678000,
            22.87; 6678600,22.92; 6679200,22.91; 6679800,23.15; 6680400,22.95; 6681000,
            22.99; 6681600,23.12; 6682200,23.19; 6682800,23.26; 6683400,23.4; 6684000,
            23.48; 6684600,23.6; 6685200,23.67; 6685800,23.72; 6686400,23.8; 6687000,
            23.88; 6687600,23.96; 6688200,24.01; 6688800,24.09; 6689400,24.13; 6690000,
            24.21; 6690600,24.25; 6691200,24.29; 6691800,24.34; 6692400,24.62; 6693000,
            24.41; 6693600,24.41; 6694200,24.41; 6694800,24.41; 6695400,24.41; 6696000,
            24.41; 6696600,24.41; 6697200,24.41; 6697800,24.41; 6698400,24.41; 6699000,
            24.53; 6699600,24.54; 6700200,24.58; 6700800,24.65; 6701400,24.74; 6702000,
            24.7; 6702600,24.78; 6703200,24.78; 6703800,24.79; 6704400,24.82; 6705000,
            24.86; 6705600,24.82; 6706200,24.78; 6706800,24.82; 6707400,24.78; 6708000,
            24.73; 6708600,24.74; 6709200,24.61; 6709800,24.62; 6710400,24.63; 6711000,
            24.66; 6711600,24.66; 6712200,24.62; 6712800,24.54; 6713400,24.54; 6714000,
            24.54; 6714600,24.54; 6715200,24.54; 6715800,24.51; 6716400,24.58; 6717000,
            24.58; 6717600,24.59; 6718200,24.53; 6718800,24.45; 6719400,24.41; 6720000,
            24.37; 6720600,24.33; 6721200,24.29; 6721800,24.25; 6722400,24.22; 6723000,
            24.14; 6723600,24.05; 6724200,24; 6724800,23.88; 6725400,23.8; 6726000,23.72;
            6726600,23.6; 6727200,23.52; 6727800,23.4; 6728400,23.37; 6729000,23.27;
            6729600,23.19; 6730200,23.15; 6730800,23.11; 6731400,23.07; 6732000,23.03;
            6732600,22.95; 6733200,22.91; 6733800,22.87; 6734400,22.83; 6735000,22.78;
            6735600,22.74; 6736200,22.7; 6736800,22.66; 6737400,22.62; 6738000,22.58;
            6738600,22.5; 6739200,22.42; 6739800,22.42; 6740400,22.38; 6741000,22.34;
            6741600,22.3; 6742200,22.26; 6742800,22.22; 6743400,22.18; 6744000,22.14;
            6744600,22.1; 6745200,22.06; 6745800,22.05; 6746400,22.01; 6747000,22.01;
            6747600,21.97; 6748200,21.97; 6748800,21.93; 6749400,21.89; 6750000,21.85;
            6750600,21.85; 6751200,21.81; 6751800,21.8; 6752400,21.77; 6753000,21.73;
            6753600,21.74; 6754200,21.69; 6754800,21.69; 6755400,21.65; 6756000,21.65;
            6756600,21.63; 6757200,21.56; 6757800,21.56; 6758400,21.52; 6759000,21.52;
            6759600,21.48; 6760200,21.46; 6760800,21.44; 6761400,21.44; 6762000,21.44;
            6762600,21.43; 6763200,21.4; 6763800,21.4; 6764400,21.4; 6765000,21.41;
            6765600,21.4; 6766200,21.4; 6766800,21.4; 6767400,21.4; 6768000,21.43; 6768600,
            21.44; 6769200,21.44; 6769800,21.44; 6770400,21.46; 6771000,21.48; 6771600,
            21.52; 6772200,21.52; 6772800,21.56; 6773400,21.6; 6774000,21.6; 6774600,
            21.65; 6775200,21.65; 6775800,21.67; 6776400,21.73; 6777000,21.73; 6777600,
            21.81; 6778200,21.82; 6778800,21.87; 6779400,21.93; 6780000,21.97; 6780600,
            22.02; 6781200,22.09; 6781800,22.14; 6782400,22.18; 6783000,22.26; 6783600,
            22.33; 6784200,22.42; 6784800,22.5; 6785400,22.6; 6786000,22.7; 6786600,
            22.81; 6787200,22.91; 6787800,22.98; 6788400,23.03; 6789000,23.07; 6789600,
            23.16; 6790200,23.15; 6790800,23.19; 6791400,23.23; 6792000,23.44; 6792600,
            23.23; 6793200,23.27; 6793800,23.32; 6794400,23.36; 6795000,23.4; 6795600,
            23.44; 6796200,23.51; 6796800,23.48; 6797400,23.52; 6798000,23.48; 6798600,
            23.4; 6799200,23.36; 6799800,23.27; 6800400,23.19; 6801000,23.15; 6801600,
            23.11; 6802200,23.03; 6802800,22.98; 6803400,22.91; 6804000,22.83; 6804600,
            22.78; 6805200,22.78; 6805800,22.7; 6806400,22.66; 6807000,22.62; 6807600,
            22.58; 6808200,22.54; 6808800,22.47; 6809400,22.42; 6810000,22.38; 6810600,
            22.34; 6811200,22.3; 6811800,22.22; 6812400,22.15; 6813000,22.09; 6813600,
            22.08; 6814200,22.05; 6814800,22.01; 6815400,21.97; 6816000,21.97; 6816600,
            21.93; 6817200,21.89; 6817800,21.85; 6818400,21.81; 6819000,21.81; 6819600,
            21.77; 6820200,21.73; 6820800,21.69; 6821400,21.66; 6822000,21.65; 6822600,
            21.6; 6823200,21.65; 6823800,21.73; 6824400,21.78; 6825000,21.81; 6825600,
            21.81; 6826200,21.85; 6826800,21.85; 6827400,21.85; 6828000,21.86; 6828600,
            21.86; 6829200,21.9; 6829800,21.89; 6830400,21.89; 6831000,21.89; 6831600,
            21.89; 6832200,21.89; 6832800,21.89; 6833400,21.89; 6834000,21.89; 6834600,
            21.89; 6835200,21.9; 6835800,21.89; 6836400,21.89; 6837000,21.9; 6837600,
            21.85; 6838200,21.85; 6838800,21.85; 6839400,21.85; 6840000,21.85; 6840600,
            21.86; 6841200,21.85; 6841800,21.85; 6842400,21.85; 6843000,21.84; 6843600,
            21.85; 6844200,21.81; 6844800,21.81; 6845400,21.8; 6846000,21.81; 6846600,
            21.81; 6847200,21.81; 6847800,21.81; 6848400,21.81; 6849000,21.81; 6849600,
            21.81; 6850200,21.84; 6850800,21.81; 6851400,21.85; 6852000,21.85; 6852600,
            21.85; 6853200,21.85; 6853800,21.89; 6854400,21.89; 6855000,21.93; 6855600,
            21.98; 6856200,21.97; 6856800,21.97; 6857400,22.01; 6858000,22.03; 6858600,
            22.06; 6859200,22.06; 6859800,22.09; 6860400,22.14; 6861000,22.16; 6861600,
            22.18; 6862200,22.22; 6862800,22.22; 6863400,22.26; 6864000,22.26; 6864600,
            22.29; 6865200,22.3; 6865800,22.34; 6866400,22.38; 6867000,22.47; 6867600,
            22.5; 6868200,22.54; 6868800,22.55; 6869400,22.54; 6870000,22.54; 6870600,
            22.5; 6871200,22.51; 6871800,22.54; 6872400,22.58; 6873000,22.58; 6873600,
            22.58; 6874200,22.58; 6874800,22.58; 6875400,22.58; 6876000,22.58; 6876600,
            22.58; 6877200,22.54; 6877800,22.54; 6878400,22.58; 6879000,22.58; 6879600,
            22.61; 6880200,22.58; 6880800,22.54; 6881400,22.54; 6882000,22.58; 6882600,
            22.58; 6883200,22.58; 6883800,22.58; 6884400,22.54; 6885000,22.5; 6885600,
            22.46; 6886200,22.42; 6886800,22.42; 6887400,22.38; 6888000,22.38; 6888600,
            22.33; 6889200,22.3; 6889800,22.26; 6890400,22.26; 6891000,22.22; 6891600,
            22.18; 6892200,22.19; 6892800,22.15; 6893400,22.14; 6894000,22.14; 6894600,
            22.1; 6895200,22.1; 6895800,22.09; 6896400,22.06; 6897000,22.09; 6897600,
            22.09; 6898200,22.09; 6898800,22.09; 6899400,22.09; 6900000,22.09; 6900600,
            22.09; 6901200,22.09; 6901800,22.09; 6902400,22.09; 6903000,22.09; 6903600,
            22.09; 6904200,22.09; 6904800,22.09; 6905400,22.09; 6906000,22.09; 6906600,
            22.09; 6907200,22.09; 6907800,22.09; 6908400,22.09; 6909000,22.06; 6909600,
            22.06; 6910200,22.05; 6910800,22.05; 6911400,22.05; 6912000,22.05; 6912600,
            22.05; 6913200,22.05; 6913800,22.05; 6914400,22.02; 6915000,22.06; 6915600,
            22.04; 6916200,22.01; 6916800,22.01; 6917400,22.01; 6918000,22.02; 6918600,
            22.01; 6919200,22.01; 6919800,22.01; 6920400,22.01; 6921000,22.01; 6921600,
            22.01; 6922200,21.98; 6922800,21.97; 6923400,22.01; 6924000,21.98; 6924600,
            21.97; 6925200,21.97; 6925800,21.97; 6926400,21.97; 6927000,21.97; 6927600,
            22.01; 6928200,22.22; 6928800,22.34; 6929400,22.46; 6930000,22.62; 6930600,
            22.7; 6931200,22.7; 6931800,22.66; 6932400,22.62; 6933000,22.62; 6933600,
            22.6; 6934200,22.64; 6934800,22.6; 6935400,22.62; 6936000,22.66; 6936600,
            22.74; 6937200,22.74; 6937800,22.78; 6938400,22.87; 6939000,22.96; 6939600,
            23.03; 6940200,23.15; 6940800,23.27; 6941400,23.4; 6942000,23.42; 6942600,
            23.64; 6943200,23.76; 6943800,23.8; 6944400,23.9; 6945000,23.92; 6945600,
            23.92; 6946200,24.01; 6946800,24.1; 6947400,24.13; 6948000,24.17; 6948600,
            24.21; 6949200,24.24; 6949800,24.3; 6950400,24.33; 6951000,24.29; 6951600,
            24.33; 6952200,24.45; 6952800,24.37; 6953400,24.38; 6954000,24.41; 6954600,
            24.41; 6955200,24.41; 6955800,24.41; 6956400,24.46; 6957000,24.66; 6957600,
            24.74; 6958200,24.78; 6958800,24.95; 6959400,25.02; 6960000,25.1; 6960600,
            25.14; 6961200,25.14; 6961800,25.14; 6962400,25.19; 6963000,25.22; 6963600,
            25.19; 6964200,25.19; 6964800,25.14; 6965400,25.19; 6966000,25.23; 6966600,
            25.23; 6967200,25.23; 6967800,25.26; 6968400,25.23; 6969000,25.24; 6969600,
            25.22; 6970200,25.11; 6970800,25.07; 6971400,24.98; 6972000,24.94; 6972600,
            24.83; 6973200,24.78; 6973800,24.7; 6974400,24.66; 6975000,24.66; 6975600,
            24.64; 6976200,24.65; 6976800,24.6; 6977400,24.58; 6978000,24.5; 6978600,
            24.46; 6979200,24.41; 6979800,24.37; 6980400,24.29; 6981000,24.25; 6981600,
            24.17; 6982200,24.09; 6982800,24.01; 6983400,23.92; 6984000,23.8; 6984600,
            23.72; 6985200,23.64; 6985800,23.52; 6986400,23.43; 6987000,23.36; 6987600,
            23.33; 6988200,23.27; 6988800,23.19; 6989400,23.14; 6990000,23.12; 6990600,
            23.03; 6991200,22.99; 6991800,22.95; 6992400,22.91; 6993000,22.9; 6993600,
            22.86; 6994200,22.83; 6994800,22.78; 6995400,22.78; 6996000,22.74; 6996600,
            22.74; 6997200,22.7; 6997800,22.7; 6998400,22.68; 6999000,22.64; 6999600,
            22.62; 7000200,22.62; 7000800,22.62; 7001400,22.59; 7002000,22.58; 7002600,
            22.58; 7003200,22.55; 7003800,22.54; 7004400,22.54; 7005000,22.52; 7005600,
            22.5; 7006200,22.5; 7006800,22.46; 7007400,22.46; 7008000,22.44; 7008600,
            22.42; 7009200,22.42; 7009800,22.42; 7010400,22.42; 7011000,22.38; 7011600,
            22.38; 7012200,22.38; 7012800,22.38; 7013400,22.36; 7014000,22.42; 7014600,
            22.62; 7015200,22.78; 7015800,22.88; 7016400,22.99; 7017000,23.08; 7017600,
            23.07; 7018200,22.95; 7018800,22.87; 7019400,22.83; 7020000,22.78; 7020600,
            22.79; 7021200,22.84; 7021800,22.87; 7022400,22.91; 7023000,22.94; 7023600,
            22.95; 7024200,23.03; 7024800,23.07; 7025400,23.15; 7026000,23.15; 7026600,
            23.33; 7027200,23.44; 7027800,23.52; 7028400,23.6; 7029000,23.68; 7029600,
            23.71; 7030200,23.76; 7030800,23.81; 7031400,23.84; 7032000,23.96; 7032600,
            24.13; 7033200,24.14; 7033800,24.23; 7034400,24.25; 7035000,24.3; 7035600,
            24.37; 7036200,24.38; 7036800,24.53; 7037400,24.66; 7038000,24.72; 7038600,
            24.78; 7039200,24.7; 7039800,24.7; 7040400,24.66; 7041000,24.62; 7041600,
            24.66; 7042200,24.66; 7042800,24.79; 7043400,24.82; 7044000,24.98; 7044600,
            25.07; 7045200,25.14; 7045800,25.19; 7046400,25.27; 7047000,25.31; 7047600,
            25.31; 7048200,25.32; 7048800,25.31; 7049400,25.27; 7050000,25.3; 7050600,
            25.18; 7051200,25.23; 7051800,25.27; 7052400,25.31; 7053000,25.32; 7053600,
            25.27; 7054200,25.31; 7054800,25.39; 7055400,25.31; 7056000,25.32; 7056600,
            25.27; 7057200,25.19; 7057800,25.1; 7058400,25.1; 7059000,25.06; 7059600,
            25.02; 7060200,24.98; 7060800,24.94; 7061400,24.86; 7062000,24.82; 7062600,
            24.78; 7063200,24.7; 7063800,24.66; 7064400,24.58; 7065000,24.5; 7065600,
            24.41; 7066200,24.37; 7066800,24.33; 7067400,24.25; 7068000,24.21; 7068600,
            24.13; 7069200,24.05; 7069800,24; 7070400,23.92; 7071000,23.88; 7071600,
            23.8; 7072200,23.72; 7072800,23.64; 7073400,23.54; 7074000,23.44; 7074600,
            23.4; 7075200,23.32; 7075800,23.27; 7076400,23.2; 7077000,23.15; 7077600,
            23.11; 7078200,23.07; 7078800,23.03; 7079400,22.96; 7080000,22.96; 7080600,
            22.91; 7081200,22.87; 7081800,22.83; 7082400,22.78; 7083000,22.75; 7083600,
            22.74; 7084200,22.7; 7084800,22.66; 7085400,22.62; 7086000,22.63; 7086600,
            22.58; 7087200,22.54; 7087800,22.54; 7088400,22.5; 7089000,22.49; 7089600,
            22.46; 7090200,22.47; 7090800,22.43; 7091400,22.42; 7092000,22.54; 7092600,
            22.62; 7093200,22.78; 7093800,22.87; 7094400,22.91; 7095000,23.02; 7095600,
            23.03; 7096200,23.03; 7096800,23.03; 7097400,22.99; 7098000,22.95; 7098600,
            22.95; 7099200,22.91; 7099800,22.91; 7100400,22.93; 7101000,22.95; 7101600,
            22.95; 7102200,22.91; 7102800,22.92; 7103400,22.91; 7104000,22.91; 7104600,
            22.91; 7105200,22.91; 7105800,22.91; 7106400,22.95; 7107000,22.95; 7107600,
            22.95; 7108200,22.95; 7108800,22.99; 7109400,23.03; 7110000,23.03; 7110600,
            23.11; 7111200,23.15; 7111800,23.19; 7112400,23.27; 7113000,23.32; 7113600,
            23.44; 7114200,23.57; 7114800,23.68; 7115400,23.76; 7116000,23.84; 7116600,
            23.89; 7117200,23.92; 7117800,23.96; 7118400,24.01; 7119000,24.09; 7119600,
            24.09; 7120200,24.21; 7120800,24.25; 7121400,24.27; 7122000,24.33; 7122600,
            24.29; 7123200,24.34; 7123800,24.41; 7124400,24.5; 7125000,24.58; 7125600,
            24.55; 7126200,24.48; 7126800,24.45; 7127400,24.45; 7128000,24.45; 7128600,
            24.5; 7129200,24.53; 7129800,24.57; 7130400,24.7; 7131000,24.82; 7131600,
            24.78; 7132200,24.82; 7132800,24.82; 7133400,24.9; 7134000,24.9; 7134600,
            24.94; 7135200,24.98; 7135800,24.94; 7136400,24.94; 7137000,24.92; 7137600,
            24.82; 7138200,24.86; 7138800,24.83; 7139400,24.86; 7140000,24.9; 7140600,
            24.86; 7141200,24.94; 7141800,24.88; 7142400,24.98; 7143000,24.98; 7143600,
            25.04; 7144200,24.94; 7144800,24.9; 7145400,24.86; 7146000,24.86; 7146600,
            24.87; 7147200,24.78; 7147800,24.7; 7148400,24.62; 7149000,24.54; 7149600,
            24.45; 7150200,24.41; 7150800,24.33; 7151400,24.25; 7152000,24.21; 7152600,
            24.18; 7153200,24.13; 7153800,24.05; 7154400,24.04; 7155000,23.96; 7155600,
            23.92; 7156200,23.84; 7156800,23.8; 7157400,23.76; 7158000,23.68; 7158600,
            23.61; 7159200,23.55; 7159800,23.48; 7160400,23.42; 7161000,23.36; 7161600,
            23.29; 7162200,23.24; 7162800,23.19; 7163400,23.15; 7164000,23.12; 7164600,
            23.07; 7165200,23.04; 7165800,22.99; 7166400,22.98; 7167000,22.96; 7167600,
            22.91; 7168200,22.87; 7168800,22.87; 7169400,22.83; 7170000,22.8; 7170600,
            22.78; 7171200,22.78; 7171800,22.74; 7172400,22.71; 7173000,22.7; 7173600,
            22.68; 7174200,22.66; 7174800,22.62; 7175400,22.62; 7176000,22.62; 7176600,
            22.58; 7177200,22.58; 7177800,22.54; 7178400,22.7; 7179000,22.8; 7179600,
            22.91; 7180200,22.99; 7180800,23.07; 7181400,23.12; 7182000,23.15; 7182600,
            23.15; 7183200,23.11; 7183800,23.11; 7184400,23.07; 7185000,23.07; 7185600,
            23.07; 7186200,23.03; 7186800,23.03; 7187400,23.07; 7188000,23.11; 7188600,
            23.11; 7189200,23.11; 7189800,23.11; 7190400,23.1; 7191000,23.07; 7191600,
            23.03; 7192200,23.03; 7192800,23.04; 7193400,23.05; 7194000,23.07; 7194600,
            23.07; 7195200,23.09; 7195800,23.11; 7196400,23.15; 7197000,23.19; 7197600,
            23.27; 7198200,23.37; 7198800,23.4; 7199400,23.48; 7200000,23.56; 7200600,
            23.68; 7201200,23.8; 7201800,23.89; 7202400,23.92; 7203000,23.96; 7203600,
            24; 7204200,24.13; 7204800,24.17; 7205400,24.14; 7206000,24.13; 7206600,
            24.21; 7207200,24.3; 7207800,24.37; 7208400,24.45; 7209000,24.5; 7209600,
            24.54; 7210200,24.58; 7210800,24.69; 7211400,24.66; 7212000,24.6; 7212600,
            24.56; 7213200,24.54; 7213800,24.58; 7214400,24.66; 7215000,24.62; 7215600,
            24.74; 7216200,24.86; 7216800,24.94; 7217400,24.9; 7218000,24.97; 7218600,
            24.99; 7219200,25.04; 7219800,25.06; 7220400,25.06; 7221000,25.06; 7221600,
            25.14; 7222200,25.14; 7222800,25.15; 7223400,25.14; 7224000,25.08; 7224600,
            25.1; 7225200,25.16; 7225800,25.12; 7226400,25.16; 7227000,25.23; 7227600,
            25.2; 7228200,25.14; 7228800,25.1; 7229400,25.08; 7230000,25.1; 7230600,
            24.98; 7231200,24.74; 7231800,24.67; 7232400,24.58; 7233000,24.53; 7233600,
            24.45; 7234200,24.41; 7234800,24.37; 7235400,24.25; 7236000,24.17; 7236600,
            24.09; 7237200,24.05; 7237800,24.01; 7238400,23.92; 7239000,23.88; 7239600,
            23.8; 7240200,23.73; 7240800,23.68; 7241400,23.64; 7242000,23.56; 7242600,
            23.5; 7243200,23.44; 7243800,23.4; 7244400,23.32; 7245000,23.27; 7245600,
            23.19; 7246200,23.11; 7246800,23.07; 7247400,23.03; 7248000,22.99; 7248600,
            22.91; 7249200,22.87; 7249800,22.83; 7250400,22.78; 7251000,22.7; 7251600,
            22.66; 7252200,22.62; 7252800,22.58; 7253400,22.54; 7254000,22.5; 7254600,
            22.46; 7255200,22.43; 7255800,22.42; 7256400,22.38; 7257000,22.34; 7257600,
            22.3; 7258200,22.26; 7258800,22.26; 7259400,22.25; 7260000,22.22; 7260600,
            22.18; 7261200,22.18; 7261800,22.14; 7262400,22.13; 7263000,22.09; 7263600,
            22.09; 7264200,22.05; 7264800,22.05; 7265400,22.01; 7266000,22; 7266600,
            21.98; 7267200,21.97; 7267800,21.97; 7268400,21.97; 7269000,21.97; 7269600,
            21.97; 7270200,21.97; 7270800,21.97; 7271400,21.93; 7272000,21.93; 7272600,
            21.93; 7273200,21.93; 7273800,21.93; 7274400,21.92; 7275000,21.93; 7275600,
            21.89; 7276200,21.87; 7276800,21.85; 7277400,21.85; 7278000,21.89; 7278600,
            21.89; 7279200,21.85; 7279800,21.85; 7280400,21.85; 7281000,21.85; 7281600,
            21.85; 7282200,21.85; 7282800,21.85; 7283400,21.86; 7284000,21.86; 7284600,
            21.88; 7285200,21.92; 7285800,21.98; 7286400,22.09; 7287000,22.26; 7287600,
            22.38; 7288200,22.46; 7288800,22.62; 7289400,22.73; 7290000,22.83; 7290600,
            23; 7291200,23.06; 7291800,23.19; 7292400,23.27; 7293000,23.33; 7293600,
            23.36; 7294200,23.4; 7294800,23.48; 7295400,23.57; 7296000,23.68; 7296600,
            23.72; 7297200,23.79; 7297800,23.8; 7298400,23.78; 7299000,23.72; 7299600,
            23.64; 7300200,23.76; 7300800,23.76; 7301400,23.76; 7302000,23.8; 7302600,
            23.86; 7303200,23.92; 7303800,23.96; 7304400,23.96; 7305000,23.97; 7305600,
            24.05; 7306200,24.09; 7306800,24.09; 7307400,24.09; 7308000,24.14; 7308600,
            24.13; 7309200,24.2; 7309800,24.25; 7310400,24.25; 7311000,24.26; 7311600,
            24.28; 7312200,24.25; 7312800,24.21; 7313400,24.21; 7314000,24.17; 7314600,
            24.13; 7315200,24.13; 7315800,24.13; 7316400,24.17; 7317000,24.09; 7317600,
            24.05; 7318200,24.05; 7318800,24.01; 7319400,23.93; 7320000,23.88; 7320600,
            23.83; 7321200,23.76; 7321800,23.72; 7322400,23.64; 7323000,23.56; 7323600,
            23.56; 7324200,23.48; 7324800,23.44; 7325400,23.4; 7326000,23.36; 7326600,
            23.32; 7327200,23.27; 7327800,23.19; 7328400,23.14; 7329000,23.07; 7329600,
            23.03; 7330200,22.95; 7330800,22.93; 7331400,22.9; 7332000,22.83; 7332600,
            22.78; 7333200,22.74; 7333800,22.7; 7334400,22.66; 7335000,22.62; 7335600,
            22.58; 7336200,22.54; 7336800,22.5; 7337400,22.47; 7338000,22.42; 7338600,
            22.38; 7339200,22.36; 7339800,22.34; 7340400,22.3; 7341000,22.3; 7341600,
            22.27; 7342200,22.22; 7342800,22.22; 7343400,22.2; 7344000,22.19; 7344600,
            22.14; 7345200,22.1; 7345800,22.09; 7346400,22.08; 7347000,22.05; 7347600,
            22.05; 7348200,22.01; 7348800,21.98; 7349400,21.97; 7350000,21.97; 7350600,
            21.93; 7351200,21.93; 7351800,21.89; 7352400,21.86; 7353000,21.84; 7353600,
            21.81; 7354200,21.81; 7354800,21.77; 7355400,21.77; 7356000,21.73; 7356600,
            21.73; 7357200,21.69; 7357800,21.65; 7358400,21.65; 7359000,21.61; 7359600,
            21.6; 7360200,21.6; 7360800,21.56; 7361400,21.52; 7362000,21.52; 7362600,
            21.48; 7363200,21.48; 7363800,21.44; 7364400,21.44; 7365000,21.44; 7365600,
            21.41; 7366200,21.41; 7366800,21.4; 7367400,21.36; 7368000,21.36; 7368600,
            21.37; 7369200,21.36; 7369800,21.36; 7370400,21.37; 7371000,21.4; 7371600,
            21.4; 7372200,21.4; 7372800,21.4; 7373400,21.44; 7374000,21.44; 7374600,
            21.45; 7375200,21.48; 7375800,21.49; 7376400,21.52; 7377000,21.52; 7377600,
            21.56; 7378200,21.6; 7378800,21.61; 7379400,21.65; 7380000,21.69; 7380600,
            21.73; 7381200,21.77; 7381800,21.81; 7382400,21.81; 7383000,21.85; 7383600,
            21.89; 7384200,21.96; 7384800,21.97; 7385400,22.06; 7386000,22.09; 7386600,
            22.14; 7387200,22.26; 7387800,22.3; 7388400,22.38; 7389000,22.46; 7389600,
            22.58; 7390200,22.66; 7390800,22.78; 7391400,22.87; 7392000,22.98; 7392600,
            23.06; 7393200,23.12; 7393800,23.19; 7394400,23.27; 7395000,23.36; 7395600,
            23.44; 7396200,23.49; 7396800,23.52; 7397400,23.56; 7398000,23.6; 7398600,
            23.68; 7399200,23.73; 7399800,23.76; 7400400,23.88; 7401000,23.92; 7401600,
            24; 7402200,24.05; 7402800,24.08; 7403400,24.01; 7404000,23.95; 7404600,
            23.88; 7405200,23.84; 7405800,23.76; 7406400,23.72; 7407000,23.62; 7407600,
            23.56; 7408200,23.48; 7408800,23.4; 7409400,23.32; 7410000,23.27; 7410600,
            23.23; 7411200,23.15; 7411800,23.11; 7412400,23.03; 7413000,22.99; 7413600,
            22.94; 7414200,22.87; 7414800,22.84; 7415400,22.74; 7416000,22.7; 7416600,
            22.62; 7417200,22.58; 7417800,22.54; 7418400,22.5; 7419000,22.42; 7419600,
            22.38; 7420200,22.34; 7420800,22.3; 7421400,22.27; 7422000,22.25; 7422600,
            22.22; 7423200,22.14; 7423800,22.12; 7424400,22.09; 7425000,22.05; 7425600,
            22.01; 7426200,21.97; 7426800,21.97; 7427400,21.97; 7428000,21.93; 7428600,
            21.93; 7429200,21.89; 7429800,21.92; 7430400,21.89; 7431000,21.89; 7431600,
            21.89; 7432200,21.88; 7432800,21.85; 7433400,21.86; 7434000,21.77; 7434600,
            21.77; 7435200,21.78; 7435800,21.73; 7436400,21.73; 7437000,21.73; 7437600,
            21.69; 7438200,21.65; 7438800,21.65; 7439400,21.65; 7440000,21.61; 7440600,
            21.6; 7441200,21.6; 7441800,21.6; 7442400,21.56; 7443000,21.56; 7443600,
            21.56; 7444200,21.56; 7444800,21.52; 7445400,21.52; 7446000,21.54; 7446600,
            21.52; 7447200,21.48; 7447800,21.48; 7448400,21.48; 7449000,21.48; 7449600,
            21.48; 7450200,21.48; 7450800,21.48; 7451400,21.48; 7452000,21.48; 7452600,
            21.48; 7453200,21.5; 7453800,21.52; 7454400,21.52; 7455000,21.52; 7455600,
            21.52; 7456200,21.55; 7456800,21.56; 7457400,21.56; 7458000,21.6; 7458600,
            21.6; 7459200,21.6; 7459800,21.6; 7460400,21.62; 7461000,21.6; 7461600,21.62;
            7462200,21.64; 7462800,21.65; 7463400,21.65; 7464000,21.65; 7464600,21.66;
            7465200,21.68; 7465800,21.69; 7466400,21.68; 7467000,21.7; 7467600,21.73;
            7468200,21.73; 7468800,21.77; 7469400,21.77; 7470000,21.77; 7470600,21.77;
            7471200,21.77; 7471800,21.77; 7472400,21.77; 7473000,21.77; 7473600,21.77;
            7474200,21.77; 7474800,21.77; 7475400,21.81; 7476000,21.81; 7476600,21.81;
            7477200,21.81; 7477800,21.78; 7478400,21.77; 7479000,21.76; 7479600,21.77;
            7480200,21.81; 7480800,21.81; 7481400,21.81; 7482000,21.81; 7482600,21.81;
            7483200,21.89; 7483800,21.97; 7484400,22.09; 7485000,22.09; 7485600,22.09;
            7486200,22.05; 7486800,22.05; 7487400,22.02; 7488000,22.01; 7488600,22.01;
            7489200,21.97; 7489800,21.97; 7490400,21.93; 7491000,21.93; 7491600,21.89;
            7492200,21.85; 7492800,21.81; 7493400,21.81; 7494000,21.77; 7494600,21.73;
            7495200,21.73; 7495800,21.69; 7496400,21.66; 7497000,21.64; 7497600,21.6;
            7498200,21.6; 7498800,21.58; 7499400,21.52; 7500000,21.52; 7500600,21.48;
            7501200,21.48; 7501800,21.44; 7502400,21.44; 7503000,21.4; 7503600,21.4;
            7504200,21.36; 7504800,21.36; 7505400,21.33; 7506000,21.32; 7506600,21.28;
            7507200,21.28; 7507800,21.24; 7508400,21.24; 7509000,21.21; 7509600,21.2;
            7510200,21.16; 7510800,21.16; 7511400,21.14; 7512000,21.12; 7512600,21.12;
            7513200,21.12; 7513800,21.12; 7514400,21.08; 7515000,21.07; 7515600,21.06;
            7516200,21.05; 7516800,21.04; 7517400,21.04; 7518000,21.04; 7518600,21.03;
            7519200,21; 7519800,21; 7520400,21; 7521000,21; 7521600,21; 7522200,21;
            7522800,21; 7523400,20.96; 7524000,20.96; 7524600,20.96; 7525200,20.96;
            7525800,20.96; 7526400,20.91; 7527000,20.93; 7527600,20.96; 7528200,20.91;
            7528800,20.91; 7529400,20.91; 7530000,20.91; 7530600,20.96; 7531200,20.99;
            7531800,21; 7532400,20.98; 7533000,20.96; 7533600,20.96; 7534200,20.92;
            7534800,20.91; 7535400,20.91; 7536000,20.91; 7536600,21; 7537200,21.08;
            7537800,21.08; 7538400,21.08; 7539000,21.04; 7539600,21.01; 7540200,21.04;
            7540800,21.04; 7541400,21.06; 7542000,21.16; 7542600,21.2; 7543200,21.24;
            7543800,21.24; 7544400,21.2; 7545000,21.2; 7545600,21.21; 7546200,21.21;
            7546800,21.2; 7547400,21.24; 7548000,21.32; 7548600,21.32; 7549200,21.32;
            7549800,21.32; 7550400,21.28; 7551000,21.28; 7551600,21.24; 7552200,21.23;
            7552800,21.25; 7553400,21.36; 7554000,21.4; 7554600,21.36; 7555200,21.32;
            7555800,21.32; 7556400,21.31; 7557000,21.28; 7557600,21.24; 7558200,21.24;
            7558800,21.28; 7559400,21.36; 7560000,21.4; 7560600,21.45; 7561200,21.4;
            7561800,21.4; 7562400,21.36; 7563000,21.36; 7563600,21.32; 7564200,21.32;
            7564800,21.32; 7565400,21.32; 7566000,21.4; 7566600,21.44; 7567200,21.44;
            7567800,21.44; 7568400,21.44; 7569000,21.4; 7569600,21.4; 7570200,21.4;
            7570800,21.44; 7571400,21.44; 7572000,21.52; 7572600,21.52; 7573200,21.52;
            7573800,21.49; 7574400,21.5; 7575000,21.48; 7575600,21.44; 7576200,21.4;
            7576800,21.4; 7577400,21.38; 7578000,21.4; 7578600,21.4; 7579200,21.36;
            7579800,21.36; 7580400,21.32; 7581000,21.32; 7581600,21.32; 7582200,21.28;
            7582800,21.32; 7583400,21.32; 7584000,21.32; 7584600,21.32; 7585200,21.31;
            7585800,21.32; 7586400,21.32; 7587000,21.28; 7587600,21.25; 7588200,21.24;
            7588800,21.2; 7589400,21.19; 7590000,21.16; 7590600,21.16; 7591200,21.16;
            7591800,21.12; 7592400,21.16; 7593000,21.16; 7593600,21.15; 7594200,21.12;
            7594800,21.12; 7595400,21.12; 7596000,21.12; 7596600,21.12; 7597200,21.12;
            7597800,21.12; 7598400,21.13; 7599000,21.08; 7599600,21.08; 7600200,21.08;
            7600800,21.08; 7601400,21.08; 7602000,21.08; 7602600,21.08; 7603200,21.08;
            7603800,21.08; 7604400,21.07; 7605000,21.05; 7605600,21.04; 7606200,21.04;
            7606800,21.05; 7607400,21.04; 7608000,21.04; 7608600,21.04; 7609200,21.08;
            7609800,21.08; 7610400,21.08; 7611000,21.12; 7611600,21.12; 7612200,21.13;
            7612800,21.12; 7613400,21.12; 7614000,21.12; 7614600,21.12; 7615200,21.12;
            7615800,21.09; 7616400,21.12; 7617000,21.16; 7617600,21.13; 7618200,21.08;
            7618800,21.08; 7619400,21.04; 7620000,21; 7620600,21; 7621200,21.04; 7621800,
            21.08; 7622400,21.17; 7623000,21.5; 7623600,21.7; 7624200,21.85; 7624800,
            22.01; 7625400,22.19; 7626000,22.26; 7626600,22.3; 7627200,22.22; 7627800,
            22.34; 7628400,22.37; 7629000,22.47; 7629600,22.54; 7630200,22.62; 7630800,
            22.66; 7631400,22.74; 7632000,22.78; 7632600,22.87; 7633200,22.95; 7633800,
            23.03; 7634400,23.08; 7635000,23.15; 7635600,23.11; 7636200,23.15; 7636800,
            23.15; 7637400,23.19; 7638000,23.19; 7638600,23.19; 7639200,23.19; 7639800,
            23.23; 7640400,23.23; 7641000,23.23; 7641600,23.19; 7642200,23.19; 7642800,
            23.15; 7643400,23.15; 7644000,23.11; 7644600,23.11; 7645200,23.11; 7645800,
            23.15; 7646400,23.19; 7647000,23.27; 7647600,23.24; 7648200,23.24; 7648800,
            23.27; 7649400,23.28; 7650000,23.32; 7650600,23.35; 7651200,23.32; 7651800,
            23.39; 7652400,23.41; 7653000,23.44; 7653600,23.44; 7654200,23.47; 7654800,
            23.51; 7655400,23.48; 7656000,23.48; 7656600,23.46; 7657200,23.4; 7657800,
            23.36; 7658400,23.27; 7659000,23.23; 7659600,23.23; 7660200,23.23; 7660800,
            23.23; 7661400,23.15; 7662000,23.11; 7662600,23.07; 7663200,23.07; 7663800,
            22.95; 7664400,22.91; 7665000,22.87; 7665600,22.83; 7666200,22.78; 7666800,
            22.74; 7667400,22.7; 7668000,22.66; 7668600,22.64; 7669200,22.58; 7669800,
            22.5; 7670400,22.43; 7671000,22.39; 7671600,22.34; 7672200,22.3; 7672800,
            22.26; 7673400,22.22; 7674000,22.14; 7674600,22.09; 7675200,22.05; 7675800,
            22.05; 7676400,22.01; 7677000,21.97; 7677600,21.93; 7678200,21.9; 7678800,
            21.89; 7679400,21.85; 7680000,21.85; 7680600,21.81; 7681200,21.81; 7681800,
            21.77; 7682400,21.77; 7683000,21.73; 7683600,21.73; 7684200,21.7; 7684800,
            21.69; 7685400,21.65; 7686000,21.65; 7686600,21.65; 7687200,21.61; 7687800,
            21.5975; 7688400,21.585; 7689000,21.5725; 7689600,21.56; 7690200,21.52;
            7690800,21.52; 7691400,21.52; 7692000,21.48; 7692600,21.48; 7693200,21.45;
            7693800,21.44; 7694400,21.44; 7695000,21.44; 7695600,21.45; 7696200,21.4;
            7696800,21.44; 7697400,21.43; 7698000,21.44; 7698600,21.44; 7699200,21.48;
            7699800,21.48; 7700400,21.52; 7701000,21.49; 7701600,21.52; 7702200,21.52;
            7702800,21.6; 7703400,21.66; 7704000,21.66; 7704600,21.6; 7705200,21.56;
            7705800,21.52; 7706400,21.48; 7707000,21.52; 7707600,21.57; 7708200,21.59;
            7708800,21.65; 7709400,21.73; 7710000,21.77; 7710600,21.81; 7711200,21.89;
            7711800,22.02; 7712400,22.17; 7713000,22.26; 7713600,22.42; 7714200,22.46;
            7714800,22.62; 7715400,22.62; 7716000,22.7; 7716600,22.78; 7717200,22.82;
            7717800,22.88; 7718400,22.95; 7719000,23.03; 7719600,23.03; 7720200,23.07;
            7720800,23.18; 7721400,23.32; 7722000,23.4; 7722600,23.47; 7723200,23.52;
            7723800,23.44; 7724400,23.55; 7725000,23.48; 7725600,23.43; 7726200,23.44;
            7726800,23.44; 7727400,23.6; 7728000,23.65; 7728600,23.72; 7729200,23.77;
            7729800,23.84; 7730400,23.92; 7731000,24.01; 7731600,24.05; 7732200,24.17;
            7732800,24.21; 7733400,24.29; 7734000,24.33; 7734600,24.29; 7735200,24.29;
            7735800,24.34; 7736400,24.42; 7737000,24.5; 7737600,24.54; 7738200,24.58;
            7738800,24.52; 7739400,24.58; 7740000,24.54; 7740600,24.5; 7741200,24.62;
            7741800,24.62; 7742400,24.62; 7743000,24.58; 7743600,24.53; 7744200,24.42;
            7744800,24.42; 7745400,24.37; 7746000,24.29; 7746600,24.24; 7747200,24.17;
            7747800,24.12; 7748400,24.05; 7749000,24.05; 7749600,24.01; 7750200,23.92;
            7750800,23.81; 7751400,23.73; 7752000,23.68; 7752600,23.68; 7753200,23.6;
            7753800,23.52; 7754400,23.44; 7755000,23.36; 7755600,23.32; 7756200,23.27;
            7756800,23.15; 7757400,23.11; 7758000,23.08; 7758600,23.03; 7759200,22.95;
            7759800,22.91; 7760400,22.87; 7761000,22.8; 7761600,22.78; 7762200,22.74;
            7762800,22.7; 7763400,22.66; 7764000,22.62; 7764600,22.58; 7765200,22.54;
            7765800,22.5; 7766400,22.47; 7767000,22.46; 7767600,22.42; 7768200,22.42;
            7768800,22.38; 7769400,22.38; 7770000,22.36; 7770600,22.34; 7771200,22.3;
            7771800,22.26; 7772400,22.26; 7773000,22.22; 7773600,22.21; 7774200,22.1825;
            7774800,22.155; 7775400,22.1275; 7776000,22.1; 7776600,22.1; 7777200,22.09;
            7777800,22.09; 7778400,22.05; 7779000,22.02; 7779600,22.01; 7780200,21.97;
            7780800,21.97; 7781400,21.97; 7782000,21.97; 7782600,21.93; 7783200,21.96;
            7783800,21.98; 7784400,21.97; 7785000,21.94; 7785600,21.93; 7786200,21.93;
            7786800,21.93; 7787400,21.89; 7788000,21.89; 7788600,21.89; 7789200,21.85;
            7789800,21.85; 7790400,21.85; 7791000,21.81; 7791600,21.81; 7792200,21.81;
            7792800,21.77; 7793400,21.77; 7794000,21.77; 7794600,21.81; 7795200,21.81;
            7795800,21.85; 7796400,21.85; 7797000,21.94; 7797600,22.01; 7798200,22.02;
            7798800,22.1; 7799400,22.25; 7800000,22.3; 7800600,22.42; 7801200,22.5;
            7801800,22.63; 7802400,22.7; 7803000,22.78; 7803600,22.91; 7804200,23.08;
            7804800,23.07; 7805400,23.15; 7806000,23.27; 7806600,23.4; 7807200,23.44;
            7807800,23.52; 7808400,23.56; 7809000,23.61; 7809600,23.6; 7810200,23.6;
            7810800,23.6; 7811400,23.65; 7812000,23.6; 7812600,23.6; 7813200,23.6; 7813800,
            23.61; 7814400,23.6; 7815000,23.65; 7815600,23.72; 7816200,23.81; 7816800,
            23.92; 7817400,23.97; 7818000,24.05; 7818600,24.08; 7819200,24.04; 7819800,
            24.06; 7820400,24.09; 7821000,24.16; 7821600,24.17; 7822200,24.21; 7822800,
            24.17; 7823400,24.13; 7824000,24.13; 7824600,24.17; 7825200,24.13; 7825800,
            24.18; 7826400,24.13; 7827000,24.14; 7827600,24.13; 7828200,24.05; 7828800,
            24.09; 7829400,24.05; 7830000,24.05; 7830600,24.09; 7831200,24.05; 7831800,
            24.05; 7832400,23.96; 7833000,23.88; 7833600,23.76; 7834200,23.69; 7834800,
            23.64; 7835400,23.6; 7836000,23.54; 7836600,23.48; 7837200,23.44; 7837800,
            23.36; 7838400,23.27; 7839000,23.23; 7839600,23.23; 7840200,23.16; 7840800,
            23.12; 7841400,23.03; 7842000,22.99; 7842600,22.96; 7843200,22.9; 7843800,
            22.87; 7844400,22.83; 7845000,22.78; 7845600,22.74; 7846200,22.7; 7846800,
            22.66; 7847400,22.62; 7848000,22.62; 7848600,22.58; 7849200,22.54; 7849800,
            22.54; 7850400,22.5; 7851000,22.46; 7851600,22.46; 7852200,22.42; 7852800,
            22.42; 7853400,22.42; 7854000,22.42; 7854600,22.38; 7855200,22.38; 7855800,
            22.34; 7856400,22.34; 7857000,22.34; 7857600,22.34; 7858200,22.32; 7858800,
            22.3; 7859400,22.3; 7860000,22.3; 7860600,22.29; 7861200,22.28; 7861800,
            22.27; 7862400,22.26; 7863000,22.26; 7863600,22.22; 7864200,22.22; 7864800,
            22.22; 7865400,22.18; 7866000,22.18; 7866600,22.18; 7867200,22.17; 7867800,
            22.16; 7868400,22.14; 7869000,22.14; 7869600,22.14; 7870200,22.09; 7870800,
            22.09; 7871400,22.09; 7872000,22.09; 7872600,22.05; 7873200,22.09; 7873800,
            22.09; 7874400,22.09; 7875000,22.09; 7875600,22.06; 7876200,22.08; 7876800,
            22.06; 7877400,22.09; 7878000,22.07; 7878600,22.05; 7879200,22.05; 7879800,
            22.02; 7880400,22.01; 7881000,22.01; 7881600,22.01; 7882200,22.09; 7882800,
            22.14; 7883400,22.26; 7884000,22.34; 7884600,22.41; 7885200,22.46; 7885800,
            22.51; 7886400,22.59; 7887000,22.7; 7887600,22.8; 7888200,22.94; 7888800,
            23; 7889400,23.11; 7890000,23.11; 7890600,23.16; 7891200,23.24; 7891800,
            23.36; 7892400,23.44; 7893000,23.52; 7893600,23.58; 7894200,23.62; 7894800,
            23.68; 7895400,23.73; 7896000,23.72; 7896600,23.68; 7897200,23.68; 7897800,
            23.66; 7898400,23.64; 7899000,23.64; 7899600,23.64; 7900200,23.6; 7900800,
            23.72; 7901400,23.8; 7902000,23.8; 7902600,23.84; 7903200,23.84; 7903800,
            23.88; 7904400,23.92; 7905000,23.92; 7905600,24.01; 7906200,24.04; 7906800,
            24.05; 7907400,24.02; 7908000,23.98; 7908600,23.96; 7909200,24.01; 7909800,
            24; 7910400,24.02; 7911000,24.05; 7911600,24.2; 7912200,24.25; 7912800,24.23;
            7913400,24.21; 7914000,24.21; 7914600,24.17; 7915200,24.09; 7915800,24.05;
            7916400,24.01; 7917000,23.96; 7917600,23.92; 7918200,23.88; 7918800,23.84;
            7919400,23.82; 7920000,23.76; 7920600,23.72; 7921200,23.65; 7921800,23.56;
            7922400,23.56; 7923000,23.49; 7923600,23.44; 7924200,23.4; 7924800,23.32;
            7925400,23.23; 7926000,23.18; 7926600,23.12; 7927200,23.06; 7927800,22.96;
            7928400,22.91; 7929000,22.83; 7929600,22.8; 7930200,22.78; 7930800,22.7;
            7931400,22.68; 7932000,22.64; 7932600,22.59; 7933200,22.54; 7933800,22.46;
            7934400,22.42; 7935000,22.38; 7935600,22.34; 7936200,22.3; 7936800,22.3;
            7937400,22.26; 7938000,22.27; 7938600,22.22; 7939200,22.17; 7939800,22.17;
            7940400,22.14; 7941000,22.13; 7941600,22.09; 7942200,22.09; 7942800,22.05;
            7943400,22.05; 7944000,22; 7944600,21.97; 7945200,21.93; 7945800,21.93;
            7946400,21.89; 7947000,21.87; 7947600,21.85; 7948200,21.83; 7948800,21.81;
            7949400,21.8; 7950000,21.77; 7950600,21.73; 7951200,21.73; 7951800,21.72;
            7952400,21.69; 7953000,21.69; 7953600,21.65; 7954200,21.66; 7954800,21.64;
            7955400,21.6; 7956000,21.6; 7956600,21.57; 7957200,21.56; 7957800,21.56;
            7958400,21.52; 7959000,21.52; 7959600,21.48; 7960200,21.48; 7960800,21.48;
            7961400,21.48; 7962000,21.44; 7962600,21.44; 7963200,21.44; 7963800,21.45;
            7964400,21.44; 7965000,21.4; 7965600,21.4; 7966200,21.4; 7966800,21.38;
            7967400,21.36; 7968000,21.36; 7968600,21.37; 7969200,21.36; 7969800,21.36;
            7970400,21.39; 7971000,21.38; 7971600,21.36; 7972200,21.38; 7972800,21.36;
            7973400,21.36; 7974000,21.4; 7974600,21.41; 7975200,21.4; 7975800,21.4;
            7976400,21.4; 7977000,21.42; 7977600,21.44; 7978200,21.48; 7978800,21.48;
            7979400,21.52; 7980000,21.56; 7980600,21.6; 7981200,21.65; 7981800,21.69;
            7982400,21.76; 7983000,21.82; 7983600,21.84; 7984200,21.85; 7984800,21.93;
            7985400,21.97; 7986000,22.05; 7986600,22.07; 7987200,22.14; 7987800,22.18;
            7988400,22.26; 7989000,22.27; 7989600,22.34; 7990200,22.39; 7990800,22.42;
            7991400,22.42; 7992000,22.46; 7992600,22.5; 7993200,22.54; 7993800,22.58;
            7994400,22.6; 7995000,22.61; 7995600,22.62; 7996200,22.66; 7996800,22.7;
            7997400,22.78; 7998000,22.8; 7998600,22.86; 7999200,22.91; 7999800,22.92;
            8000400,23.02; 8001000,23.07; 8001600,23.11; 8002200,23.19; 8002800,23.19;
            8003400,23.19; 8004000,23.15; 8004600,23.15; 8005200,23.15; 8005800,23.12;
            8006400,23.11; 8007000,23.11; 8007600,23.08; 8008200,23.03; 8008800,22.99;
            8009400,22.95; 8010000,22.87; 8010600,22.83; 8011200,22.76; 8011800,22.74;
            8012400,22.7; 8013000,22.66; 8013600,22.62; 8014200,22.59; 8014800,22.55;
            8015400,22.5; 8016000,22.43; 8016600,22.42; 8017200,22.38; 8017800,22.35;
            8018400,22.3; 8019000,22.3; 8019600,22.26; 8020200,22.22; 8020800,22.18;
            8021400,22.17; 8022000,22.14; 8022600,22.09; 8023200,22.07; 8023800,22.05;
            8024400,22.01; 8025000,21.97; 8025600,21.97; 8026200,21.93; 8026800,21.92;
            8027400,21.89; 8028000,21.85; 8028600,21.85; 8029200,21.81; 8029800,21.85;
            8030400,21.85; 8031000,21.89; 8031600,21.89; 8032200,21.89; 8032800,21.92;
            8033400,21.9225; 8034000,21.925; 8034600,21.9275; 8035200,21.93; 8035800,
            21.93; 8036400,21.93; 8037000,21.93; 8037600,21.93; 8038200,21.93; 8038800,
            21.89; 8039400,21.9; 8040000,21.92; 8040600,21.93; 8041200,21.89; 8041800,
            21.89; 8042400,21.89; 8043000,21.89; 8043600,21.9; 8044200,21.9; 8044800,
            21.85; 8045400,21.85; 8046000,21.82; 8046600,21.81; 8047200,21.81; 8047800,
            21.81; 8048400,21.81; 8049000,21.81; 8049600,21.81; 8050200,21.81; 8050800,
            21.81; 8051400,21.81; 8052000,21.81; 8052600,21.82; 8053200,21.81; 8053800,
            21.81; 8054400,21.81; 8055000,21.81; 8055600,21.81; 8056200,21.85; 8056800,
            21.85; 8057400,21.85; 8058000,21.85; 8058600,21.88; 8059200,21.89; 8059800,
            21.89; 8060400,21.89; 8061000,21.92; 8061600,21.93; 8062200,21.93; 8062800,
            21.97; 8063400,21.97; 8064000,21.97; 8064600,21.94; 8065200,21.97; 8065800,
            21.97; 8066400,21.97; 8067000,21.97; 8067600,21.97; 8068200,22; 8068800,
            22.04; 8069400,22.01; 8070000,22.04; 8070600,22.05; 8071200,22.03; 8071800,
            22.01; 8072400,22.01; 8073000,22.01; 8073600,22.05; 8074200,22.06; 8074800,
            22.06; 8075400,22.05; 8076000,22.05; 8076600,22.05; 8077200,22.05; 8077800,
            22.01; 8078400,22.02; 8079000,22.03; 8079600,22.05; 8080200,22.06; 8080800,
            22.05; 8081400,22.05; 8082000,22.01; 8082600,22.01; 8083200,22.01; 8083800,
            22.02; 8084400,21.98; 8085000,21.97; 8085600,21.98; 8086200,21.97; 8086800,
            21.98; 8087400,21.97; 8088000,21.97; 8088600,21.97; 8089200,21.98; 8089800,
            21.97; 8090400,21.97; 8091000,21.97; 8091600,21.96; 8092200,21.95; 8092800,
            21.93; 8093400,21.93; 8094000,21.89; 8094600,21.89; 8095200,21.89; 8095800,
            21.85; 8096400,21.85; 8097000,21.85; 8097600,21.81; 8098200,21.81; 8098800,
            21.78; 8099400,21.77; 8100000,21.77; 8100600,21.73; 8101200,21.73; 8101800,
            21.7; 8102400,21.69; 8103000,21.7; 8103600,21.69; 8104200,21.65; 8104800,
            21.65; 8105400,21.65; 8106000,21.65; 8106600,21.62; 8107200,21.6; 8107800,
            21.6; 8108400,21.63; 8109000,21.6; 8109600,21.6; 8110200,21.62; 8110800,
            21.64; 8111400,21.66; 8112000,21.65; 8112600,21.65; 8113200,21.65; 8113800,
            21.65; 8114400,21.7; 8115000,21.69; 8115600,21.69; 8116200,21.72; 8116800,
            21.7; 8117400,21.69; 8118000,21.69; 8118600,21.71; 8119200,21.72; 8119800,
            21.7125; 8120400,21.705; 8121000,21.6975; 8121600,21.69; 8122200,21.68;
            8122800,21.68; 8123400,21.65; 8124000,21.65; 8124600,21.65; 8125200,21.65;
            8125800,21.65; 8126400,21.64; 8127000,21.64; 8127600,21.64; 8128200,21.64;
            8128800,21.6; 8129400,21.6; 8130000,21.6; 8130600,21.6; 8131200,21.6; 8131800,
            21.6; 8132400,21.6; 8133000,21.6; 8133600,21.6; 8134200,21.6; 8134800,21.65;
            8135400,21.66; 8136000,21.65; 8136600,21.65; 8137200,21.6; 8137800,21.56;
            8138400,21.54; 8139000,21.52; 8139600,21.52; 8140200,21.54; 8140800,21.6;
            8141400,21.69; 8142000,21.75; 8142600,21.85; 8143200,21.93; 8143800,21.97;
            8144400,22.01; 8145000,22.09; 8145600,22.21; 8146200,22.3; 8146800,22.45;
            8147400,22.46; 8148000,22.55; 8148600,22.58; 8149200,22.62; 8149800,22.62;
            8150400,22.67; 8151000,22.74; 8151600,22.83; 8152200,22.86; 8152800,22.92;
            8153400,23; 8154000,22.99; 8154600,22.99; 8155200,23.03; 8155800,23.07;
            8156400,23.11; 8157000,23.11; 8157600,23.15; 8158200,23.17; 8158800,23.2;
            8159400,23.31; 8160000,23.36; 8160600,23.4; 8161200,23.44; 8161800,23.52;
            8162400,23.52; 8163000,23.52; 8163600,23.56; 8164200,23.56; 8164800,23.6;
            8165400,23.64; 8166000,23.64; 8166600,23.69; 8167200,23.75; 8167800,23.72;
            8168400,23.76; 8169000,23.76; 8169600,23.78; 8170200,23.76; 8170800,23.8;
            8171400,23.88; 8172000,23.88; 8172600,23.92; 8173200,23.96; 8173800,24;
            8174400,23.96; 8175000,23.96; 8175600,23.96; 8176200,23.92; 8176800,23.94;
            8177400,23.92; 8178000,23.92; 8178600,23.88; 8179200,23.84; 8179800,23.8;
            8180400,23.72; 8181000,23.69; 8181600,23.68; 8182200,23.56; 8182800,23.56;
            8183400,23.48; 8184000,23.44; 8184600,23.41; 8185200,23.33; 8185800,23.24;
            8186400,23.15; 8187000,23.11; 8187600,23.03; 8188200,22.95; 8188800,22.91;
            8189400,22.86; 8190000,22.78; 8190600,22.74; 8191200,22.7; 8191800,22.62;
            8192400,22.54; 8193000,22.5; 8193600,22.42; 8194200,22.42; 8194800,22.38;
            8195400,22.34; 8196000,22.3; 8196600,22.26; 8197200,22.19; 8197800,22.14;
            8198400,22.14; 8199000,22.09; 8199600,22.09; 8200200,22.05; 8200800,22.05;
            8201400,22.02; 8202000,22.01; 8202600,21.98; 8203200,21.97; 8203800,21.97;
            8204400,21.96; 8205000,21.93; 8205600,21.93; 8206200,21.9125; 8206800,21.895;
            8207400,21.8775; 8208000,21.86; 8208600,21.85; 8209200,21.85; 8209800,21.81;
            8210400,21.82; 8211000,21.81; 8211600,21.81; 8212200,21.93; 8212800,21.98;
            8213400,22.02; 8214000,22.01; 8214600,22.02; 8215200,21.97; 8215800,21.93;
            8216400,21.89; 8217000,21.85; 8217600,21.87; 8218200,21.85; 8218800,21.85;
            8219400,21.87; 8220000,21.89; 8220600,21.89; 8221200,21.85; 8221800,21.85;
            8222400,21.83; 8223000,21.85; 8223600,21.85; 8224200,21.85; 8224800,21.81;
            8225400,21.84; 8226000,21.9; 8226600,21.94; 8227200,21.98; 8227800,22.05;
            8228400,22.05; 8229000,22.18; 8229600,22.3; 8230200,22.42; 8230800,22.5;
            8231400,22.62; 8232000,22.66; 8232600,22.74; 8233200,22.84; 8233800,22.88;
            8234400,22.91; 8235000,22.95; 8235600,22.96; 8236200,22.99; 8236800,23.03;
            8237400,23.11; 8238000,23.19; 8238600,23.23; 8239200,23.23; 8239800,23.27;
            8240400,23.36; 8241000,23.4; 8241600,23.4; 8242200,23.41; 8242800,23.36;
            8243400,23.35; 8244000,23.27; 8244600,23.27; 8245200,23.32; 8245800,23.27;
            8246400,23.27; 8247000,23.39; 8247600,23.4; 8248200,23.44; 8248800,23.48;
            8249400,23.52; 8250000,23.53; 8250600,23.53; 8251200,23.6; 8251800,23.64;
            8252400,23.68; 8253000,23.71; 8253600,23.72; 8254200,23.76; 8254800,23.82;
            8255400,23.84; 8256000,23.8; 8256600,23.88; 8257200,23.84; 8257800,23.8;
            8258400,23.8; 8259000,23.88; 8259600,23.88; 8260200,23.84; 8260800,23.84;
            8261400,23.84; 8262000,23.73; 8262600,23.82; 8263200,23.72; 8263800,23.64;
            8264400,23.56; 8265000,23.52; 8265600,23.52; 8266200,23.45; 8266800,23.44;
            8267400,23.4; 8268000,23.33; 8268600,23.27; 8269200,23.27; 8269800,23.19;
            8270400,23.15; 8271000,23.07; 8271600,23.03; 8272200,23; 8272800,22.98;
            8273400,22.95; 8274000,22.91; 8274600,22.86; 8275200,22.79; 8275800,22.74;
            8276400,22.7; 8277000,22.66; 8277600,22.64; 8278200,22.58; 8278800,22.54;
            8279400,22.5; 8280000,22.47; 8280600,22.42; 8281200,22.38; 8281800,22.34;
            8282400,22.3; 8283000,22.27; 8283600,22.22; 8284200,22.18; 8284800,22.14;
            8285400,22.13; 8286000,22.09; 8286600,22.06; 8287200,22.05; 8287800,22.01;
            8288400,21.97; 8289000,21.97; 8289600,21.93; 8290200,21.93; 8290800,21.89;
            8291400,21.89; 8292000,21.876; 8292600,21.862; 8293200,21.848; 8293800,21.834;
            8294400,21.82; 8295000,21.81; 8295600,21.77; 8296200,21.77; 8296800,21.73;
            8297400,21.73; 8298000,21.73; 8298600,21.73; 8299200,21.69; 8299800,21.69;
            8300400,21.69; 8301000,21.65; 8301600,21.65; 8302200,21.65; 8302800,21.65;
            8303400,21.64; 8304000,21.62; 8304600,21.6; 8305200,21.6; 8305800,21.6;
            8306400,21.6; 8307000,21.6; 8307600,21.56; 8308200,21.6; 8308800,21.6; 8309400,
            21.56; 8310000,21.56; 8310600,21.6; 8311200,21.6; 8311800,21.6; 8312400,
            21.62; 8313000,21.65; 8313600,21.85; 8314200,22.14; 8314800,22.37; 8315400,
            22.44; 8316000,22.54; 8316600,22.69; 8317200,22.78; 8317800,22.91; 8318400,
            22.99; 8319000,23.11; 8319600,23.19; 8320200,23.24; 8320800,23.27; 8321400,
            23.31; 8322000,23.4; 8322600,23.44; 8323200,23.48; 8323800,23.48; 8324400,
            23.48; 8325000,23.6; 8325600,23.67; 8326200,23.72; 8326800,23.76; 8327400,
            23.73; 8328000,23.82; 8328600,23.86; 8329200,23.88; 8329800,23.9; 8330400,
            23.94; 8331000,23.92; 8331600,23.92; 8332200,23.92; 8332800,24.04; 8333400,
            24.12; 8334000,24.13; 8334600,24.14; 8335200,24.13; 8335800,24.13; 8336400,
            24.13; 8337000,24.09; 8337600,24.13; 8338200,24.17; 8338800,24.21; 8339400,
            24.22; 8340000,24.21; 8340600,24.25; 8341200,24.29; 8341800,24.37; 8342400,
            24.33; 8343000,24.41; 8343600,24.41; 8344200,24.5; 8344800,24.46; 8345400,
            24.66; 8346000,24.66; 8346600,24.6; 8347200,24.58; 8347800,24.58; 8348400,
            24.48; 8349000,24.41; 8349600,24.34; 8350200,24.27; 8350800,24.25; 8351400,
            24.17; 8352000,24.13; 8352600,24.09; 8353200,24.01; 8353800,23.92; 8354400,
            23.89; 8355000,23.8; 8355600,23.72; 8356200,23.64; 8356800,23.56; 8357400,
            23.52; 8358000,23.45; 8358600,23.4; 8359200,23.36; 8359800,23.27; 8360400,
            23.19; 8361000,23.15; 8361600,23.07; 8362200,22.99; 8362800,22.95; 8363400,
            22.87; 8364000,22.81; 8364600,22.74; 8365200,22.7; 8365800,22.62; 8366400,
            22.58; 8367000,22.5; 8367600,22.42; 8368200,22.42; 8368800,22.35; 8369400,
            22.3; 8370000,22.26; 8370600,22.23; 8371200,22.19; 8371800,22.17; 8372400,
            22.14; 8373000,22.09; 8373600,22.07; 8374200,22.06; 8374800,21.99; 8375400,
            21.97; 8376000,21.96; 8376600,21.94; 8377200,21.88; 8377800,21.85; 8378400,
            21.82; 8379000,21.8; 8379600,21.78; 8380200,21.76; 8380800,21.74; 8381400,
            21.73; 8382000,21.73; 8382600,21.69; 8383200,21.65; 8383800,21.65; 8384400,
            21.64; 8385000,21.61; 8385600,21.62; 8386200,21.62; 8386800,21.6; 8387400,
            21.56; 8388000,21.56; 8388600,21.52; 8389200,21.52; 8389800,21.52; 8390400,
            21.48; 8391000,21.48; 8391600,21.48; 8392200,21.45; 8392800,21.44; 8393400,
            21.44; 8394000,21.44; 8394600,21.41; 8395200,21.44; 8395800,21.44; 8396400,
            21.44; 8397000,21.44; 8397600,21.41; 8398200,21.44; 8398800,21.44; 8399400,
            21.48; 8400000,21.52; 8400600,21.66; 8401200,21.73; 8401800,21.81; 8402400,
            21.88; 8403000,22.18; 8403600,22.42; 8404200,22.62; 8404800,22.74; 8405400,
            22.74; 8406000,22.78; 8406600,22.87; 8407200,22.91; 8407800,23.01; 8408400,
            23.03; 8409000,23.07; 8409600,23.08; 8410200,23.07; 8410800,23.07; 8411400,
            23.11; 8412000,23.15; 8412600,23.19; 8413200,23.23; 8413800,23.24; 8414400,
            23.27; 8415000,23.27; 8415600,23.27; 8416200,23.27; 8416800,23.27; 8417400,
            23.27; 8418000,23.38; 8418600,23.39; 8419200,23.44; 8419800,23.53; 8420400,
            23.56; 8421000,23.61; 8421600,23.68; 8422200,23.68; 8422800,23.8; 8423400,
            23.84; 8424000,23.88; 8424600,23.89; 8425200,23.96; 8425800,23.96; 8426400,
            24; 8427000,24.01; 8427600,24.02; 8428200,24.04; 8428800,24.11; 8429400,
            24.25; 8430000,24.33; 8430600,24.33; 8431200,24.37; 8431800,24.37; 8432400,
            24.29; 8433000,24.26; 8433600,24.22; 8434200,24.13; 8434800,24.06; 8435400,
            24.05; 8436000,24.01; 8436600,24.01; 8437200,24.06; 8437800,24; 8438400,
            23.92; 8439000,23.88; 8439600,23.84; 8440200,23.8; 8440800,23.76; 8441400,
            23.72; 8442000,23.68; 8442600,23.6; 8443200,23.56; 8443800,23.56; 8444400,
            23.52; 8445000,23.48; 8445600,23.4; 8446200,23.36; 8446800,23.32; 8447400,
            23.27; 8448000,23.2; 8448600,23.15; 8449200,23.1; 8449800,23.03; 8450400,
            22.99; 8451000,22.91; 8451600,22.87; 8452200,22.78; 8452800,22.74; 8453400,
            22.68; 8454000,22.62; 8454600,22.58; 8455200,22.54; 8455800,22.54; 8456400,
            22.5; 8457000,22.46; 8457600,22.42; 8458200,22.38; 8458800,22.38; 8459400,
            22.34; 8460000,22.34; 8460600,22.3; 8461200,22.26; 8461800,22.26; 8462400,
            22.22; 8463000,22.18; 8463600,22.18; 8464200,22.14; 8464800,22.122; 8465400,
            22.104; 8466000,22.086; 8466600,22.068; 8467200,22.05; 8467800,22.02; 8468400,
            21.98; 8469000,21.97; 8469600,21.98; 8470200,21.97; 8470800,21.97; 8471400,
            22.01; 8472000,22.01; 8472600,22.01; 8473200,22.05; 8473800,22.05; 8474400,
            22.03; 8475000,22.04; 8475600,22.05; 8476200,22.05; 8476800,22.05; 8477400,
            22.06; 8478000,22.01; 8478600,22.05; 8479200,22.22; 8479800,22.3; 8480400,
            22.35; 8481000,22.3; 8481600,22.26; 8482200,22.22; 8482800,22.18; 8483400,
            22.09; 8484000,22.09; 8484600,22.18; 8485200,22.26; 8485800,22.34; 8486400,
            22.38; 8487000,22.46; 8487600,22.46; 8488200,22.46; 8488800,22.5; 8489400,
            22.58; 8490000,22.66; 8490600,22.83; 8491200,22.98; 8491800,23.03; 8492400,
            23.03; 8493000,23.06; 8493600,23.11; 8494200,23.11; 8494800,23.3; 8495400,
            23.41; 8496000,23.52; 8496600,23.52; 8497200,23.48; 8497800,23.52; 8498400,
            23.52; 8499000,23.52; 8499600,23.56; 8500200,23.6; 8500800,23.68; 8501400,
            23.64; 8502000,23.6; 8502600,23.56; 8503200,23.48; 8503800,23.4; 8504400,
            23.4; 8505000,23.49; 8505600,23.56; 8506200,23.56; 8506800,23.56; 8507400,
            23.56; 8508000,23.6; 8508600,23.6; 8509200,23.64; 8509800,23.56; 8510400,
            23.64; 8511000,23.72; 8511600,23.72; 8512200,23.74; 8512800,23.72; 8513400,
            23.76; 8514000,23.76; 8514600,23.72; 8515200,23.76; 8515800,23.77; 8516400,
            23.8; 8517000,23.8; 8517600,23.76; 8518200,23.8; 8518800,23.8; 8519400,23.8;
            8520000,23.77; 8520600,23.8; 8521200,23.8; 8521800,23.77; 8522400,23.8;
            8523000,23.76; 8523600,23.73; 8524200,23.6; 8524800,23.56; 8525400,23.48;
            8526000,23.4; 8526600,23.36; 8527200,23.27; 8527800,23.19; 8528400,23.11;
            8529000,23.07; 8529600,23.07; 8530200,23.04; 8530800,23.01; 8531400,22.95;
            8532000,22.91; 8532600,22.82; 8533200,22.74; 8533800,22.74; 8534400,22.71;
            8535000,22.66; 8535600,22.62; 8536200,22.59; 8536800,22.54; 8537400,22.5;
            8538000,22.43; 8538600,22.38; 8539200,22.34; 8539800,22.3; 8540400,22.26;
            8541000,22.22; 8541600,22.18; 8542200,22.18; 8542800,22.14; 8543400,22.14;
            8544000,22.1; 8544600,22.09; 8545200,22.09; 8545800,22.05; 8546400,22.01;
            8547000,22.01; 8547600,21.97; 8548200,21.98; 8548800,21.89; 8549400,21.88;
            8550000,21.86; 8550600,21.85; 8551200,21.81; 8551800,21.78; 8552400,21.75;
            8553000,21.72; 8553600,21.69; 8554200,21.65; 8554800,21.66; 8555400,21.6;
            8556000,21.6; 8556600,21.59; 8557200,21.55; 8557800,21.52; 8558400,21.52;
            8559000,21.48; 8559600,21.47; 8560200,21.44; 8560800,21.44; 8561400,21.44;
            8562000,21.4; 8562600,21.4; 8563200,21.4; 8563800,21.36; 8564400,21.37;
            8565000,21.36; 8565600,21.32; 8566200,21.32; 8566800,21.32; 8567400,21.32;
            8568000,21.32; 8568600,21.28; 8569200,21.24; 8569800,21.24; 8570400,21.24;
            8571000,21.24; 8571600,21.24; 8572200,21.2; 8572800,21.2; 8573400,21.2;
            8574000,21.2; 8574600,21.2; 8575200,21.2; 8575800,21.16; 8576400,21.16;
            8577000,21.2; 8577600,21.2; 8578200,21.2; 8578800,21.2; 8579400,21.2; 8580000,
            21.2; 8580600,21.2; 8581200,21.2; 8581800,21.2; 8582400,21.2; 8583000,21.2;
            8583600,21.23; 8584200,21.24; 8584800,21.24; 8585400,21.26; 8586000,21.27;
            8586600,21.29; 8587200,21.28; 8587800,21.28; 8588400,21.28; 8589000,21.28;
            8589600,21.32; 8590200,21.28; 8590800,21.32; 8591400,21.32; 8592000,21.4;
            8592600,21.48; 8593200,21.52; 8593800,21.52; 8594400,21.52; 8595000,21.57;
            8595600,21.6; 8596200,21.72; 8596800,21.81; 8597400,21.81; 8598000,21.81;
            8598600,21.77; 8599200,21.76; 8599800,21.73; 8600400,21.69; 8601000,21.69;
            8601600,21.65; 8602200,21.64; 8602800,21.6; 8603400,21.6; 8604000,21.56;
            8604600,21.56; 8605200,21.56; 8605800,21.56; 8606400,21.56; 8607000,21.56;
            8607600,21.56; 8608200,21.58; 8608800,21.56; 8609400,21.52; 8610000,21.54;
            8610600,21.54; 8611200,21.52; 8611800,21.52; 8612400,21.5; 8613000,21.48;
            8613600,21.48; 8614200,21.48; 8614800,21.44; 8615400,21.45; 8616000,21.4;
            8616600,21.38; 8617200,21.36; 8617800,21.32; 8618400,21.32; 8619000,21.28;
            8619600,21.24; 8620200,21.21; 8620800,21.2; 8621400,21.16; 8622000,21.12;
            8622600,21.13; 8623200,21.12; 8623800,21.08; 8624400,21.08; 8625000,21.04;
            8625600,21; 8626200,21; 8626800,20.96; 8627400,20.96; 8628000,20.91; 8628600,
            20.91; 8629200,20.87; 8629800,20.85; 8630400,20.83; 8631000,20.83; 8631600,
            20.79; 8632200,20.78; 8632800,20.76; 8633400,20.71; 8634000,20.83; 8634600,
            20.95; 8635200,21; 8635800,21.04; 8636400,21.08; 8637000,21.08; 8637600,
            21.12; 8638200,21.1375; 8638800,21.155; 8639400,21.1725; 8640000,21.19;
            8640600,21.2; 8641200,21.2; 8641800,21.2; 8642400,21.24; 8643000,21.25;
            8643600,21.24; 8644200,21.25; 8644800,21.24; 8645400,21.28; 8646000,21.25;
            8646600,21.28; 8647200,21.28; 8647800,21.28; 8648400,21.28; 8649000,21.24;
            8649600,21.28; 8650200,21.28; 8650800,21.28; 8651400,21.28; 8652000,21.28;
            8652600,21.28; 8653200,21.28; 8653800,21.28; 8654400,21.28; 8655000,21.28;
            8655600,21.28; 8656200,21.28; 8656800,21.28; 8657400,21.28; 8658000,21.32;
            8658600,21.32; 8659200,21.32; 8659800,21.35; 8660400,21.36; 8661000,21.36;
            8661600,21.4; 8662200,21.4; 8662800,21.45; 8663400,21.46; 8664000,21.44;
            8664600,21.48; 8665200,21.45; 8665800,21.48; 8666400,21.52; 8667000,21.52;
            8667600,21.56; 8668200,21.56; 8668800,21.56; 8669400,21.56; 8670000,21.57;
            8670600,21.56; 8671200,21.56; 8671800,21.56; 8672400,21.56; 8673000,21.56;
            8673600,21.56; 8674200,21.56; 8674800,21.58; 8675400,21.6; 8676000,21.6;
            8676600,21.6; 8677200,21.6; 8677800,21.65; 8678400,21.73; 8679000,21.77;
            8679600,21.81; 8680200,22.09; 8680800,21.92; 8681400,21.97; 8682000,22.09;
            8682600,22.18; 8683200,22.26; 8683800,22.34; 8684400,22.42; 8685000,22.46;
            8685600,22.54; 8686200,22.62; 8686800,22.66; 8687400,22.71; 8688000,22.78;
            8688600,22.83; 8689200,22.87; 8689800,22.91; 8690400,22.93; 8691000,22.95;
            8691600,22.99; 8692200,23.03; 8692800,23.07; 8693400,23.11; 8694000,23.15;
            8694600,23.15; 8695200,23.19; 8695800,23.23; 8696400,23.56; 8697000,23.23;
            8697600,23.11; 8698200,23.07; 8698800,23.03; 8699400,22.94; 8700000,22.78;
            8700600,22.74; 8701200,22.7; 8701800,22.66; 8702400,22.58; 8703000,22.5;
            8703600,22.42; 8704200,22.38; 8704800,22.31; 8705400,22.26; 8706000,22.26;
            8706600,22.22; 8707200,22.14; 8707800,22.14; 8708400,22.09; 8709000,22.06;
            8709600,22.05; 8710200,22.01; 8710800,22.01; 8711400,22.01; 8712000,21.97;
            8712600,21.98; 8713200,21.97; 8713800,21.98; 8714400,21.97; 8715000,21.93;
            8715600,21.93; 8716200,21.92; 8716800,21.93; 8717400,21.89; 8718000,21.89;
            8718600,21.89; 8719200,21.85; 8719800,21.85; 8720400,21.85; 8721000,21.85;
            8721600,21.84; 8722200,21.85; 8722800,21.81; 8723400,21.81; 8724000,21.81;
            8724600,21.8; 8725200,21.79; 8725800,21.78; 8726400,21.77; 8727000,21.73;
            8727600,21.73; 8728200,21.74; 8728800,21.73; 8729400,21.73; 8730000,21.7;
            8730600,21.7; 8731200,21.69; 8731800,21.69; 8732400,21.7; 8733000,21.65;
            8733600,21.66; 8734200,21.65; 8734800,21.65; 8735400,21.65; 8736000,21.66;
            8736600,21.65; 8737200,21.62; 8737800,21.64; 8738400,21.81; 8739000,22.01;
            8739600,22.09; 8740200,22.14; 8740800,22.05; 8741400,21.93; 8742000,21.85;
            8742600,21.81; 8743200,21.81; 8743800,21.88; 8744400,21.97; 8745000,22.05;
            8745600,22.05; 8746200,22.09; 8746800,22.14; 8747400,22.22; 8748000,22.26;
            8748600,22.42; 8749200,22.6; 8749800,22.74; 8750400,22.84; 8751000,22.95;
            8751600,23.07; 8752200,23.2; 8752800,23.23; 8753400,23.27; 8754000,23.32;
            8754600,23.37; 8755200,23.4; 8755800,23.48; 8756400,23.52; 8757000,23.52;
            8757600,23.64; 8758200,23.64; 8758800,23.69; 8759400,23.72; 8760000,23.68;
            8760600,23.68; 8761200,23.68; 8761800,23.68; 8762400,23.68; 8763000,23.68;
            8763600,23.65; 8764200,23.57; 8764800,23.56; 8765400,23.48; 8766000,23.44;
            8766600,23.44; 8767200,23.44; 8767800,23.4; 8768400,23.36; 8769000,23.36;
            8769600,23.36; 8770200,23.36; 8770800,23.52; 8771400,23.56; 8772000,23.54;
            8772600,23.68; 8773200,23.76; 8773800,23.8; 8774400,23.94; 8775000,24.04;
            8775600,24.1; 8776200,24.09; 8776800,24.17; 8777400,24.13; 8778000,24.36;
            8778600,24.09; 8779200,24.17; 8779800,24.13; 8780400,24.17; 8781000,24.2;
            8781600,24.22; 8782200,24.33; 8782800,24.49; 8783400,24.21; 8784000,24.13;
            8784600,24.05; 8785200,23.96; 8785800,23.88; 8786400,23.76; 8787000,23.68;
            8787600,23.65; 8788200,23.6; 8788800,23.56; 8789400,23.45; 8790000,23.4;
            8790600,23.32; 8791200,23.27; 8791800,23.19; 8792400,23.12; 8793000,23.07;
            8793600,23.03; 8794200,22.91; 8794800,22.87; 8795400,22.78; 8796000,22.76;
            8796600,22.78; 8797200,22.74; 8797800,22.66; 8798400,22.62; 8799000,22.59;
            8799600,22.5; 8800200,22.46; 8800800,22.42; 8801400,22.39; 8802000,22.33;
            8802600,22.29; 8803200,22.26; 8803800,22.22; 8804400,22.19; 8805000,22.17;
            8805600,22.14; 8806200,22.09; 8806800,22.08; 8807400,22.05; 8808000,22.02;
            8808600,22.01; 8809200,22; 8809800,21.97; 8810400,21.946; 8811000,21.922;
            8811600,21.898; 8812200,21.874; 8812800,21.85; 8813400,21.86; 8814000,21.81;
            8814600,21.81; 8815200,21.81; 8815800,21.82; 8816400,21.81; 8817000,21.81;
            8817600,21.82; 8818200,21.85; 8818800,21.88; 8819400,21.89; 8820000,21.89;
            8820600,21.89; 8821200,21.93; 8821800,21.93; 8822400,21.97; 8823000,21.97;
            8823600,21.97; 8824200,21.97; 8824800,22.05; 8825400,22.26; 8826000,22.42;
            8826600,22.47; 8827200,22.42; 8827800,22.3; 8828400,22.22; 8829000,22.18;
            8829600,22.12; 8830200,22.14; 8830800,22.34; 8831400,22.48; 8832000,22.58;
            8832600,22.62; 8833200,22.67; 8833800,22.62; 8834400,22.68; 8835000,22.74;
            8835600,22.9; 8836200,22.99; 8836800,23.1; 8837400,23.23; 8838000,23.27;
            8838600,23.36; 8839200,23.44; 8839800,23.52; 8840400,23.53; 8841000,23.64;
            8841600,23.72; 8842200,23.76; 8842800,23.89; 8843400,23.89; 8844000,23.88;
            8844600,23.92; 8845200,24.06; 8845800,24.05; 8846400,24.04; 8847000,24.01;
            8847600,24; 8848200,23.95; 8848800,23.92; 8849400,23.92; 8850000,23.97;
            8850600,24.01; 8851200,24.01; 8851800,24; 8852400,24.1; 8853000,24.13; 8853600,
            24.17; 8854200,24.13; 8854800,24.13; 8855400,24.21; 8856000,24.25; 8856600,
            24.25; 8857200,24.25; 8857800,24.26; 8858400,24.25; 8859000,24.29; 8859600,
            24.37; 8860200,24.42; 8860800,24.46; 8861400,24.57; 8862000,24.58; 8862600,
            24.66; 8863200,24.69; 8863800,24.66; 8864400,24.71; 8865000,24.66; 8865600,
            24.58; 8866200,24.5; 8866800,24.41; 8867400,24.37; 8868000,24.25; 8868600,
            24.18; 8869200,24.1; 8869800,24.05; 8870400,23.96; 8871000,23.89; 8871600,
            23.86; 8872200,23.8; 8872800,23.74; 8873400,23.68; 8874000,23.6; 8874600,
            23.56; 8875200,23.52; 8875800,23.48; 8876400,23.44; 8877000,23.41; 8877600,
            23.36; 8878200,23.35; 8878800,23.27; 8879400,23.23; 8880000,23.19; 8880600,
            23.15; 8881200,23.11; 8881800,23.07; 8882400,23.03; 8883000,22.95; 8883600,
            22.91; 8884200,22.87; 8884800,22.8; 8885400,22.78; 8886000,22.74; 8886600,
            22.7; 8887200,22.7; 8887800,22.67; 8888400,22.62; 8889000,22.59; 8889600,
            22.54; 8890200,22.5; 8890800,22.5; 8891400,22.46; 8892000,22.43; 8892600,
            22.42; 8893200,22.38; 8893800,22.36; 8894400,22.34; 8895000,22.3; 8895600,
            22.26; 8896200,22.26; 8896800,22.22; 8897400,22.2; 8898000,22.18; 8898600,
            22.16; 8899200,22.14; 8899800,22.13; 8900400,22.09; 8901000,22.09; 8901600,
            22.05; 8902200,22.06; 8902800,22.01; 8903400,21.97; 8904000,21.97; 8904600,
            21.97; 8905200,21.93; 8905800,21.93; 8906400,21.89; 8907000,21.88; 8907600,
            21.85; 8908200,21.85; 8908800,21.86; 8909400,21.81; 8910000,21.81; 8910600,
            21.81; 8911200,21.81; 8911800,21.78; 8912400,21.77; 8913000,21.77; 8913600,
            21.77; 8914200,22.01; 8914800,21.77; 8915400,21.81; 8916000,21.85; 8916600,
            21.85; 8917200,21.85; 8917800,22.06; 8918400,22.33; 8919000,22.5; 8919600,
            22.62; 8920200,22.79; 8920800,22.9; 8921400,22.97; 8922000,22.99; 8922600,
            23.07; 8923200,23.11; 8923800,23.15; 8924400,23.16; 8925000,23.19; 8925600,
            23.23; 8926200,23.27; 8926800,23.24; 8927400,23.23; 8928000,23.26; 8928600,
            23.27; 8929200,23.28; 8929800,23.32; 8930400,23.36; 8931000,23.36; 8931600,
            23.4; 8932200,23.42; 8932800,23.42; 8933400,23.46; 8934000,23.51; 8934600,
            23.46; 8935200,23.4; 8935800,23.4; 8936400,23.48; 8937000,23.55; 8937600,
            23.6; 8938200,23.6; 8938800,23.6; 8939400,23.61; 8940000,23.6; 8940600,23.72;
            8941200,23.8; 8941800,23.91; 8942400,23.89; 8943000,23.76; 8943600,23.64;
            8944200,23.48; 8944800,23.48; 8945400,23.6; 8946000,23.42; 8946600,23.65;
            8947200,23.79; 8947800,23.8; 8948400,23.8; 8949000,23.88; 8949600,23.88;
            8950200,23.92; 8950800,24.01; 8951400,23.96; 8952000,23.88; 8952600,23.88;
            8953200,23.88; 8953800,23.8; 8954400,23.72; 8955000,23.68; 8955600,23.68;
            8956200,23.6; 8956800,23.56; 8957400,23.48; 8958000,23.44; 8958600,23.36;
            8959200,23.3; 8959800,23.23; 8960400,23.16; 8961000,23.11; 8961600,23.07;
            8962200,22.98; 8962800,22.91; 8963400,22.87; 8964000,22.83; 8964600,22.78;
            8965200,22.76; 8965800,22.66; 8966400,22.62; 8967000,22.55; 8967600,22.5;
            8968200,22.46; 8968800,22.42; 8969400,22.38; 8970000,22.34; 8970600,22.3;
            8971200,22.25; 8971800,22.18; 8972400,22.26; 8973000,22.14; 8973600,22.09;
            8974200,22.09; 8974800,22.05; 8975400,22.02; 8976000,21.98; 8976600,21.97;
            8977200,21.96; 8977800,21.93; 8978400,21.9; 8979000,21.89; 8979600,21.85;
            8980200,21.85; 8980800,21.85; 8981400,21.81; 8982000,21.81; 8982600,21.77;
            8983200,21.754; 8983800,21.738; 8984400,21.722; 8985000,21.706; 8985600,
            21.69; 8986200,21.73; 8986800,21.69; 8987400,21.68; 8988000,21.65; 8988600,
            21.65; 8989200,21.65; 8989800,21.6; 8990400,21.6; 8991000,21.6; 8991600,
            21.6; 8992200,21.6; 8992800,21.6; 8993400,21.56; 8994000,21.56; 8994600,
            21.57; 8995200,21.52; 8995800,21.52; 8996400,21.52; 8997000,21.54; 8997600,
            21.54; 8998200,21.52; 8998800,21.48; 8999400,21.52; 9000000,21.52; 9000600,
            21.52; 9001200,21.52; 9001800,21.52; 9002400,21.52; 9003000,21.56; 9003600,
            21.56; 9004200,21.6; 9004800,21.69; 9005400,21.81; 9006000,21.89; 9006600,
            21.97; 9007200,22.14; 9007800,22.28; 9008400,22.38; 9009000,22.46; 9009600,
            22.55; 9010200,22.62; 9010800,22.66; 9011400,22.98; 9012000,23.19; 9012600,
            23.27; 9013200,23.32; 9013800,23.35; 9014400,23.4; 9015000,23.4; 9015600,
            23.44; 9016200,23.41; 9016800,23.56; 9017400,23.6; 9018000,23.68; 9018600,
            23.68; 9019200,23.68; 9019800,23.68; 9020400,23.68; 9021000,23.68; 9021600,
            23.69; 9022200,23.72; 9022800,23.6; 9023400,23.64; 9024000,23.72; 9024600,
            23.68; 9025200,23.72; 9025800,23.82; 9026400,23.88; 9027000,23.99; 9027600,
            24.01; 9028200,24.1; 9028800,24.09; 9029400,24.17; 9030000,24.21; 9030600,
            24.37; 9031200,24.33; 9031800,24.41; 9032400,24.37; 9033000,24.33; 9033600,
            24.33; 9034200,24.42; 9034800,24.36; 9035400,24.37; 9036000,24.55; 9036600,
            24.37; 9037200,24.37; 9037800,24.37; 9038400,24.41; 9039000,24.37; 9039600,
            24.37; 9040200,24.37; 9040800,24.36; 9041400,24.39; 9042000,24.37; 9042600,
            24.3; 9043200,24.21; 9043800,24.13; 9044400,24.05; 9045000,23.96; 9045600,
            23.92; 9046200,23.9; 9046800,23.84; 9047400,23.76; 9048000,23.68; 9048600,
            23.61; 9049200,23.52; 9049800,23.48; 9050400,23.36; 9051000,23.27; 9051600,
            23.23; 9052200,23.19; 9052800,23.11; 9053400,23.07; 9054000,23.03; 9054600,
            23; 9055200,22.95; 9055800,22.9; 9056400,22.83; 9057000,22.78; 9057600,22.74;
            9058200,22.7; 9058800,22.66; 9059400,22.62; 9060000,22.58; 9060600,22.54;
            9061200,22.5; 9061800,22.5; 9062400,22.47; 9063000,22.42; 9063600,22.38;
            9064200,22.34; 9064800,22.3; 9065400,22.26; 9066000,22.22; 9066600,22.18;
            9067200,22.16; 9067800,22.14; 9068400,22.13; 9069000,22.1; 9069600,22.09;
            9070200,22.06; 9070800,22.03; 9071400,22; 9072000,21.97; 9072600,21.97;
            9073200,21.98; 9073800,21.97; 9074400,21.97; 9075000,21.97; 9075600,21.93;
            9076200,21.89; 9076800,21.89; 9077400,21.89; 9078000,21.85; 9078600,21.85;
            9079200,21.85; 9079800,21.84; 9080400,21.81; 9081000,21.81; 9081600,21.81;
            9082200,21.81; 9082800,21.81; 9083400,21.8; 9084000,21.77; 9084600,21.77;
            9085200,21.77; 9085800,21.77; 9086400,21.77; 9087000,21.77; 9087600,21.77;
            9088200,21.77; 9088800,21.77; 9089400,21.77; 9090000,21.77; 9090600,21.81;
            9091200,21.84; 9091800,21.93; 9092400,21.97; 9093000,22.02; 9093600,22.09;
            9094200,22.14; 9094800,22.22; 9095400,22.34; 9096000,22.41; 9096600,22.49;
            9097200,22.58; 9097800,22.63; 9098400,22.7; 9099000,22.76; 9099600,22.79;
            9100200,22.87; 9100800,22.99; 9101400,23.03; 9102000,23.12; 9102600,23.2;
            9103200,23.23; 9103800,23.36; 9104400,23.4; 9105000,23.41; 9105600,23.44;
            9106200,23.48; 9106800,23.52; 9107400,23.52; 9108000,23.56; 9108600,23.6;
            9109200,23.72; 9109800,23.72; 9110400,23.8; 9111000,23.84; 9111600,23.84;
            9112200,23.84; 9112800,23.88; 9113400,23.88; 9114000,23.88; 9114600,23.88;
            9115200,23.92; 9115800,23.93; 9116400,23.92; 9117000,23.96; 9117600,24.01;
            9118200,24; 9118800,24.05; 9119400,24.12; 9120000,24.18; 9120600,24.12;
            9121200,24.09; 9121800,24.13; 9122400,24.13; 9123000,24.21; 9123600,24.21;
            9124200,24.21; 9124800,24.21; 9125400,24.16; 9126000,24.17; 9126600,24.21;
            9127200,24.22; 9127800,24.3; 9128400,24.33; 9129000,24.32; 9129600,24.24;
            9130200,24.21; 9130800,24.17; 9131400,24.1; 9132000,24.01; 9132600,23.92;
            9133200,23.92; 9133800,23.84; 9134400,23.76; 9135000,23.72; 9135600,23.65;
            9136200,23.58; 9136800,23.52; 9137400,23.44; 9138000,23.4; 9138600,23.36;
            9139200,23.28; 9139800,23.23; 9140400,23.19; 9141000,23.14; 9141600,23.1;
            9142200,23.06; 9142800,23.03; 9143400,22.95; 9144000,22.95; 9144600,22.92;
            9145200,22.88; 9145800,22.83; 9146400,22.78; 9147000,22.76; 9147600,22.7;
            9148200,22.7; 9148800,22.66; 9149400,22.62; 9150000,22.62; 9150600,22.59;
            9151200,22.55; 9151800,22.5; 9152400,22.5; 9153000,22.46; 9153600,22.42;
            9154200,22.42; 9154800,22.38; 9155400,22.38; 9156000,22.35; 9156600,22.32;
            9157200,22.29; 9157800,22.26; 9158400,22.23; 9159000,22.23; 9159600,22.19;
            9160200,22.18; 9160800,22.18; 9161400,22.19; 9162000,22.17; 9162600,22.16;
            9163200,22.14; 9163800,22.14; 9164400,22.14; 9165000,22.14; 9165600,22.14;
            9166200,22.14; 9166800,22.1; 9167400,22.09; 9168000,22.09; 9168600,22.09;
            9169200,22.09; 9169800,22.09; 9170400,22.09; 9171000,22.09; 9171600,22.05;
            9172200,22.05; 9172800,22.05; 9173400,22.05; 9174000,22.05; 9174600,22.01;
            9175200,22.01; 9175800,22.01; 9176400,22.01; 9177000,22.01; 9177600,22.02;
            9178200,21.97; 9178800,21.97; 9179400,21.97; 9180000,21.98; 9180600,21.98;
            9181200,21.97; 9181800,21.97; 9182400,22.01; 9183000,22.01; 9183600,22.05;
            9184200,22.05; 9184800,22.09; 9185400,22.09; 9186000,22.14; 9186600,22.14;
            9187200,22.18; 9187800,22.22; 9188400,22.26; 9189000,22.3; 9189600,22.34;
            9190200,22.42; 9190800,22.48; 9191400,22.5; 9192000,22.56; 9192600,22.58;
            9193200,22.62; 9193800,22.68; 9194400,22.74; 9195000,22.78; 9195600,22.83;
            9196200,22.87; 9196800,22.95; 9197400,23.03; 9198000,23.11; 9198600,23.19;
            9199200,23.27; 9199800,23.36; 9200400,23.48; 9201000,23.56; 9201600,23.68;
            9202200,23.76; 9202800,23.8; 9203400,23.88; 9204000,23.92; 9204600,23.96;
            9205200,24.01; 9205800,24.02; 9206400,24.05; 9207000,24.09; 9207600,24.13;
            9208200,24.14; 9208800,24.13; 9209400,24.13; 9210000,24.21; 9210600,24.21;
            9211200,24.22; 9211800,24.19; 9212400,24.13; 9213000,24.09; 9213600,24.09;
            9214200,24.05; 9214800,24.01; 9215400,23.96; 9216000,23.93; 9216600,23.88;
            9217200,23.84; 9217800,23.8; 9218400,23.75; 9219000,23.72; 9219600,23.68;
            9220200,23.64; 9220800,23.58; 9221400,23.56; 9222000,23.48; 9222600,23.44;
            9223200,23.4; 9223800,23.36; 9224400,23.32; 9225000,23.27; 9225600,23.26;
            9226200,23.23; 9226800,23.19; 9227400,23.15; 9228000,23.11; 9228600,23.1;
            9229200,23.07; 9229800,23.03; 9230400,23.03; 9231000,22.99; 9231600,22.95;
            9232200,22.94; 9232800,22.91; 9233400,22.91; 9234000,22.87; 9234600,22.87;
            9235200,22.83; 9235800,22.82; 9236400,22.78; 9237000,22.76; 9237600,22.74;
            9238200,22.66; 9238800,22.62; 9239400,22.62; 9240000,22.58; 9240600,22.53;
            9241200,22.5; 9241800,22.46; 9242400,22.444; 9243000,22.428; 9243600,22.412;
            9244200,22.396; 9244800,22.38; 9245400,22.37; 9246000,22.34; 9246600,22.3;
            9247200,22.31; 9247800,22.27; 9248400,22.26; 9249000,22.26; 9249600,22.22;
            9250200,22.22; 9250800,22.18; 9251400,22.18; 9252000,22.18; 9252600,22.16;
            9253200,22.14; 9253800,22.11; 9254400,22.09; 9255000,22.09; 9255600,22.06;
            9256200,22.05; 9256800,22.04; 9257400,22.01; 9258000,21.97; 9258600,21.97;
            9259200,21.97; 9259800,21.97; 9260400,21.94; 9261000,21.94; 9261600,21.93;
            9262200,21.93; 9262800,21.93; 9263400,21.93; 9264000,21.89; 9264600,21.89;
            9265200,21.89; 9265800,21.89; 9266400,21.89; 9267000,21.93; 9267600,21.93;
            9268200,21.94; 9268800,21.96; 9269400,21.97; 9270000,21.97; 9270600,22;
            9271200,22.01; 9271800,22.04; 9272400,22.05; 9273000,22.05; 9273600,22.05;
            9274200,22.05; 9274800,22.05; 9275400,22.05; 9276000,22.06; 9276600,22.05;
            9277200,22.06; 9277800,22.09; 9278400,22.09; 9279000,22.09; 9279600,22.1;
            9280200,22.06; 9280800,22.05; 9281400,22.05; 9282000,22.05; 9282600,22.05;
            9283200,22.05; 9283800,22.05; 9284400,22.09; 9285000,22.18; 9285600,22.22;
            9286200,22.22; 9286800,22.26; 9287400,22.26; 9288000,22.3; 9288600,22.34;
            9289200,22.47; 9289800,22.52; 9290400,22.54; 9291000,22.54; 9291600,22.5;
            9292200,22.46; 9292800,22.44; 9293400,22.42; 9294000,22.38; 9294600,22.38;
            9295200,22.38; 9295800,22.37; 9296400,22.38; 9297000,22.34; 9297600,22.3;
            9298200,22.27; 9298800,22.26; 9299400,22.22; 9300000,22.22; 9300600,22.18;
            9301200,22.19; 9301800,22.14; 9302400,22.09; 9303000,22.09; 9303600,22.05;
            9304200,22.04; 9304800,22.01; 9305400,21.97; 9306000,21.97; 9306600,21.93;
            9307200,21.89; 9307800,21.89; 9308400,21.85; 9309000,21.85; 9309600,21.81;
            9310200,21.81; 9310800,21.78; 9311400,21.77; 9312000,21.73; 9312600,21.73;
            9313200,21.73; 9313800,21.69; 9314400,21.68; 9315000,21.65; 9315600,21.65;
            9316200,21.65; 9316800,21.61; 9317400,21.6; 9318000,21.6; 9318600,21.6;
            9319200,21.56; 9319800,21.56; 9320400,21.53; 9321000,21.52; 9321600,21.52;
            9322200,21.52; 9322800,21.48; 9323400,21.48; 9324000,21.48; 9324600,21.49;
            9325200,21.48; 9325800,21.44; 9326400,21.46; 9327000,21.44; 9327600,21.44;
            9328200,21.44; 9328800,21.4; 9329400,21.3925; 9330000,21.385; 9330600,21.3775;
            9331200,21.37; 9331800,21.36; 9332400,21.36; 9333000,21.34; 9333600,21.32;
            9334200,21.32; 9334800,21.32; 9335400,21.32; 9336000,21.32; 9336600,21.28;
            9337200,21.28; 9337800,21.28; 9338400,21.28; 9339000,21.28; 9339600,21.24;
            9340200,21.24; 9340800,21.24; 9341400,21.21; 9342000,21.2; 9342600,21.2;
            9343200,21.2; 9343800,21.2; 9344400,21.17; 9345000,21.16; 9345600,21.16;
            9346200,21.16; 9346800,21.17; 9347400,21.16; 9348000,21.16; 9348600,21.16;
            9349200,21.2; 9349800,21.2; 9350400,21.44; 9351000,21.85; 9351600,22.09;
            9352200,22.29; 9352800,22.54; 9353400,22.78; 9354000,22.97; 9354600,23.12;
            9355200,23.27; 9355800,23.35; 9356400,23.4; 9357000,23.42; 9357600,23.45;
            9358200,23.48; 9358800,23.52; 9359400,23.6; 9360000,23.6; 9360600,23.65;
            9361200,23.68; 9361800,23.7; 9362400,23.72; 9363000,23.72; 9363600,23.8;
            9364200,23.8; 9364800,23.76; 9365400,23.72; 9366000,23.72; 9366600,23.69;
            9367200,23.68; 9367800,23.68; 9368400,23.8; 9369000,23.88; 9369600,23.84;
            9370200,23.96; 9370800,24.09; 9371400,24.21; 9372000,24.25; 9372600,24.29;
            9373200,24.17; 9373800,24.21; 9374400,24.21; 9375000,24.29; 9375600,24.41;
            9376200,24.46; 9376800,24.5; 9377400,24.43; 9378000,24.42; 9378600,24.45;
            9379200,24.37; 9379800,24.33; 9380400,24.37; 9381000,24.41; 9381600,24.37;
            9382200,24.41; 9382800,24.38; 9383400,24.37; 9384000,24.33; 9384600,24.25;
            9385200,24.17; 9385800,24.08; 9386400,24.01; 9387000,23.96; 9387600,23.88;
            9388200,23.88; 9388800,23.84; 9389400,23.8; 9390000,23.76; 9390600,23.72;
            9391200,23.64; 9391800,23.59; 9392400,23.55; 9393000,23.48; 9393600,23.43;
            9394200,23.4; 9394800,23.32; 9395400,23.27; 9396000,23.27; 9396600,23.23;
            9397200,23.15; 9397800,23.11; 9398400,23.04; 9399000,22.99; 9399600,22.94;
            9400200,22.87; 9400800,22.82; 9401400,22.74; 9402000,22.7; 9402600,22.62;
            9403200,22.58; 9403800,22.53; 9404400,22.5; 9405000,22.46; 9405600,22.42;
            9406200,22.38; 9406800,22.37; 9407400,22.34; 9408000,22.3; 9408600,22.26;
            9409200,22.26; 9409800,22.22; 9410400,22.22; 9411000,22.22; 9411600,22.18;
            9412200,22.14; 9412800,22.14; 9413400,22.09; 9414000,22.09; 9414600,22.09;
            9415200,22.09; 9415800,22.08; 9416400,22.07; 9417000,22.06; 9417600,22.05;
            9418200,22.05; 9418800,22.08; 9419400,22.05; 9420000,22.09; 9420600,22.09;
            9421200,22.09; 9421800,22.1; 9422400,22.09; 9423000,22.09; 9423600,22.1;
            9424200,22.08; 9424800,22.09; 9425400,22.1; 9426000,22.09; 9426600,22.09;
            9427200,22.09; 9427800,22.09; 9428400,22.09; 9429000,22.1; 9429600,22.3;
            9430200,22.5; 9430800,22.62; 9431400,22.7; 9432000,22.69; 9432600,22.58;
            9433200,22.5; 9433800,22.46; 9434400,22.42; 9435000,22.46; 9435600,22.5;
            9436200,22.62; 9436800,22.71; 9437400,22.83; 9438000,22.96; 9438600,23.03;
            9439200,23.16; 9439800,23.22; 9440400,23.24; 9441000,23.36; 9441600,23.43;
            9442200,23.44; 9442800,23.44; 9443400,23.52; 9444000,23.55; 9444600,23.57;
            9445200,23.6; 9445800,23.72; 9446400,23.82; 9447000,23.82; 9447600,23.83;
            9448200,23.84; 9448800,23.84; 9449400,23.88; 9450000,23.92; 9450600,23.96;
            9451200,23.92; 9451800,23.92; 9452400,23.9; 9453000,23.92; 9453600,23.92;
            9454200,23.88; 9454800,23.89; 9455400,23.96; 9456000,24.01; 9456600,24;
            9457200,24.09; 9457800,24.13; 9458400,24.22; 9459000,24.29; 9459600,24.21;
            9460200,24.29; 9460800,24.41; 9461400,24.42; 9462000,24.54; 9462600,24.59;
            9463200,24.58; 9463800,24.62; 9464400,24.62; 9465000,24.69; 9465600,24.66;
            9466200,24.66; 9466800,24.68; 9467400,24.68; 9468000,24.75; 9468600,24.71;
            9469200,24.74; 9469800,24.74; 9470400,24.78; 9471000,24.74; 9471600,24.7;
            9472200,24.62; 9472800,24.62; 9473400,24.62; 9474000,24.58; 9474600,24.54;
            9475200,24.41; 9475800,24.33; 9476400,24.25; 9477000,24.17; 9477600,24.08;
            9478200,24.01; 9478800,24.01; 9479400,23.96; 9480000,23.88; 9480600,23.8;
            9481200,23.72; 9481800,23.69; 9482400,23.56; 9483000,23.52; 9483600,23.44;
            9484200,23.39; 9484800,23.35; 9485400,23.27; 9486000,23.21; 9486600,23.15;
            9487200,23.11; 9487800,23.07; 9488400,23.03; 9489000,22.95; 9489600,22.91;
            9490200,22.84; 9490800,22.78; 9491400,22.74; 9492000,22.72; 9492600,22.7;
            9493200,22.66; 9493800,22.62; 9494400,22.58; 9495000,22.54; 9495600,22.54;
            9496200,22.5; 9496800,22.46; 9497400,22.42; 9498000,22.42; 9498600,22.35;
            9499200,22.34; 9499800,22.3; 9500400,22.26; 9501000,22.26; 9501600,22.242;
            9502200,22.224; 9502800,22.206; 9503400,22.188; 9504000,22.17; 9504600,22.14;
            9505200,22.14; 9505800,22.09; 9506400,22.09; 9507000,22.09; 9507600,22.05;
            9508200,22.01; 9508800,21.97; 9509400,21.97; 9510000,21.97; 9510600,21.97;
            9511200,21.97; 9511800,21.93; 9512400,21.93; 9513000,21.89; 9513600,21.89;
            9514200,21.86; 9514800,21.85; 9515400,21.81; 9516000,21.81; 9516600,21.81;
            9517200,21.81; 9517800,21.81; 9518400,21.77; 9519000,21.77; 9519600,21.77;
            9520200,21.77; 9520800,21.77; 9521400,21.78; 9522000,21.81; 9522600,21.84;
            9523200,21.9; 9523800,22.22; 9524400,22.49; 9525000,22.71; 9525600,22.83;
            9526200,22.96; 9526800,22.95; 9527400,23.07; 9528000,23.16; 9528600,23.24;
            9529200,23.24; 9529800,23.28; 9530400,23.32; 9531000,23.32; 9531600,23.36;
            9532200,23.4; 9532800,23.4; 9533400,23.44; 9534000,23.48; 9534600,23.56;
            9535200,23.6; 9535800,23.73; 9536400,23.72; 9537000,23.84; 9537600,23.84;
            9538200,23.85; 9538800,23.88; 9539400,23.92; 9540000,23.92; 9540600,23.88;
            9541200,23.88; 9541800,23.93; 9542400,24.09; 9543000,24.17; 9543600,24.17;
            9544200,24.25; 9544800,24.33; 9545400,24.37; 9546000,24.41; 9546600,24.51;
            9547200,24.54; 9547800,24.5; 9548400,24.54; 9549000,24.62; 9549600,24.63;
            9550200,24.7; 9550800,24.66; 9551400,24.62; 9552000,24.65; 9552600,24.66;
            9553200,24.7; 9553800,24.7; 9554400,24.7; 9555000,24.72; 9555600,24.74;
            9556200,24.74; 9556800,24.71; 9557400,24.7; 9558000,24.66; 9558600,24.66;
            9559200,24.62; 9559800,24.66; 9560400,24.58; 9561000,24.58; 9561600,24.5;
            9562200,24.45; 9562800,24.37; 9563400,24.35; 9564000,24.28; 9564600,24.21;
            9565200,24.14; 9565800,24.05; 9566400,24.01; 9567000,23.92; 9567600,23.84;
            9568200,23.76; 9568800,23.68; 9569400,23.61; 9570000,23.56; 9570600,23.48;
            9571200,23.44; 9571800,23.4; 9572400,23.35; 9573000,23.27; 9573600,23.23;
            9574200,23.19; 9574800,23.12; 9575400,23.11; 9576000,23.03; 9576600,23.02;
            9577200,22.95; 9577800,22.96; 9578400,22.91; 9579000,22.87; 9579600,22.83;
            9580200,22.78; 9580800,22.78; 9581400,22.74; 9582000,22.74; 9582600,22.7;
            9583200,22.68; 9583800,22.62; 9584400,22.62; 9585000,22.58; 9585600,22.54;
            9586200,22.5; 9586800,22.5; 9587400,22.46; 9588000,22.42; 9588600,22.395;
            9589200,22.37; 9589800,22.345; 9590400,22.32; 9591000,22.34; 9591600,22.34;
            9592200,22.31; 9592800,22.26; 9593400,22.26; 9594000,22.26; 9594600,22.26;
            9595200,22.22; 9595800,22.22; 9596400,22.2; 9597000,22.18; 9597600,22.18;
            9598200,22.14; 9598800,22.14; 9599400,22.09; 9600000,22.09; 9600600,22.1;
            9601200,22.09; 9601800,22.08; 9602400,22.06; 9603000,22.05; 9603600,22.05;
            9604200,22.05; 9604800,22.05; 9605400,22.05; 9606000,22.05; 9606600,22.05;
            9607200,22.09; 9607800,22.12; 9608400,22.14; 9609000,22.18; 9609600,22.3;
            9610200,22.34; 9610800,22.5; 9611400,22.62; 9612000,22.76; 9612600,22.83;
            9613200,22.91; 9613800,23.03; 9614400,23.11; 9615000,23.18; 9615600,23.19;
            9616200,23.27; 9616800,23.32; 9617400,23.38; 9618000,23.44; 9618600,23.56;
            9619200,23.52; 9619800,23.56; 9620400,23.6; 9621000,23.68; 9621600,23.68;
            9622200,23.76; 9622800,23.8; 9623400,23.89; 9624000,23.92; 9624600,23.92;
            9625200,23.96; 9625800,23.93; 9626400,23.96; 9627000,24; 9627600,24; 9628200,
            24.05; 9628800,24.13; 9629400,24.12; 9630000,24.13; 9630600,24.12; 9631200,
            24.21; 9631800,24.25; 9632400,24.33; 9633000,24.29; 9633600,24.33; 9634200,
            24.37; 9634800,24.33; 9635400,24.37; 9636000,24.37; 9636600,24.41; 9637200,
            24.57; 9637800,24.58; 9638400,24.7; 9639000,24.74; 9639600,24.75; 9640200,
            24.78; 9640800,24.78; 9641400,24.78; 9642000,24.74; 9642600,24.7; 9643200,
            24.66; 9643800,24.54; 9644400,24.53; 9645000,24.5; 9645600,24.49; 9646200,
            24.45; 9646800,24.41; 9647400,24.38; 9648000,24.33; 9648600,24.29; 9649200,
            24.25; 9649800,24.25; 9650400,24.21; 9651000,24.13; 9651600,24.05; 9652200,
            24.02; 9652800,23.92; 9653400,23.88; 9654000,23.84; 9654600,23.76; 9655200,
            23.72; 9655800,23.66; 9656400,23.6; 9657000,23.56; 9657600,23.48; 9658200,
            23.44; 9658800,23.4; 9659400,23.36; 9660000,23.35; 9660600,23.27; 9661200,
            23.24; 9661800,23.19; 9662400,23.15; 9663000,23.11; 9663600,23.11; 9664200,
            23.07; 9664800,23.03; 9665400,22.99; 9666000,22.98; 9666600,22.95; 9667200,
            22.92; 9667800,22.91; 9668400,22.87; 9669000,22.86; 9669600,22.84; 9670200,
            22.82; 9670800,22.78; 9671400,22.74; 9672000,22.7; 9672600,22.7; 9673200,
            22.69; 9673800,22.66; 9674400,22.646; 9675000,22.632; 9675600,22.618; 9676200,
            22.604; 9676800,22.59; 9677400,22.58; 9678000,22.54; 9678600,22.54; 9679200,
            22.5; 9679800,22.5; 9680400,22.5; 9681000,22.5; 9681600,22.46; 9682200,22.46;
            9682800,22.45; 9683400,22.42; 9684000,22.42; 9684600,22.42; 9685200,22.42;
            9685800,22.39; 9686400,22.38; 9687000,22.38; 9687600,22.38; 9688200,22.34;
            9688800,22.34; 9689400,22.34; 9690000,22.34; 9690600,22.31; 9691200,22.34;
            9691800,22.3; 9692400,22.3; 9693000,22.3; 9693600,22.3; 9694200,22.34; 9694800,
            22.38; 9695400,22.42; 9696000,22.54; 9696600,22.58; 9697200,22.62; 9697800,
            22.74; 9698400,22.87; 9699000,23.03; 9699600,23.07; 9700200,23.11; 9700800,
            23.23; 9701400,23.4; 9702000,23.49; 9702600,23.53; 9703200,23.56; 9703800,
            23.6; 9704400,23.68; 9705000,23.72; 9705600,23.83; 9706200,23.92; 9706800,
            23.96; 9707400,23.96; 9708000,24.06; 9708600,24.09; 9709200,24.13; 9709800,
            24.21; 9710400,24.21; 9711000,24.22; 9711600,24.25; 9712200,24.29; 9712800,
            24.33; 9713400,24.37; 9714000,24.46; 9714600,24.58; 9715200,24.55; 9715800,
            24.58; 9716400,24.61; 9717000,24.55; 9717600,24.66; 9718200,24.7; 9718800,
            24.74; 9719400,24.78; 9720000,24.74; 9720600,24.7; 9721200,24.66; 9721800,
            24.7; 9722400,24.78; 9723000,24.78; 9723600,24.82; 9724200,24.79; 9724800,
            24.84; 9725400,24.78; 9726000,24.79; 9726600,24.81; 9727200,24.82; 9727800,
            24.85; 9728400,24.81; 9729000,24.74; 9729600,24.74; 9730200,24.74; 9730800,
            24.74; 9731400,24.7; 9732000,24.66; 9732600,24.6; 9733200,24.54; 9733800,
            24.48; 9734400,24.45; 9735000,24.41; 9735600,24.37; 9736200,24.33; 9736800,
            24.29; 9737400,24.26; 9738000,24.21; 9738600,24.13; 9739200,24.09; 9739800,
            24.05; 9740400,24.01; 9741000,23.92; 9741600,23.88; 9742200,23.84; 9742800,
            23.79; 9743400,23.72; 9744000,23.68; 9744600,23.65; 9745200,23.56; 9745800,
            23.52; 9746400,23.48; 9747000,23.44; 9747600,23.4; 9748200,23.36; 9748800,
            23.29; 9749400,23.27; 9750000,23.21; 9750600,23.19; 9751200,23.12; 9751800,
            23.11; 9752400,23.07; 9753000,23.03; 9753600,23.03; 9754200,22.99; 9754800,
            22.95; 9755400,22.91; 9756000,22.91; 9756600,22.87; 9757200,22.86; 9757800,
            22.83; 9758400,22.78; 9759000,22.78; 9759600,22.78; 9760200,22.74; 9760800,
            22.716; 9761400,22.692; 9762000,22.668; 9762600,22.644; 9763200,22.62; 9763800,
            22.62; 9764400,22.62; 9765000,22.62; 9765600,22.58; 9766200,22.58; 9766800,
            22.56; 9767400,22.54; 9768000,22.54; 9768600,22.5; 9769200,22.5; 9769800,
            22.5; 9770400,22.5; 9771000,22.46; 9771600,22.47; 9772200,22.46; 9772800,
            22.42; 9773400,22.42; 9774000,22.42; 9774600,22.42; 9775200,22.4; 9775800,
            22.38; 9776400,22.38; 9777000,22.38; 9777600,22.37; 9778200,22.34; 9778800,
            22.34; 9779400,22.34; 9780000,22.33; 9780600,22.34; 9781200,22.3; 9781800,
            22.3; 9782400,22.3; 9783000,22.3; 9783600,22.3; 9784200,22.3; 9784800,22.3;
            9785400,22.3; 9786000,22.33; 9786600,22.3; 9787200,22.3; 9787800,22.34;
            9788400,22.34; 9789000,22.34; 9789600,22.34; 9790200,22.34; 9790800,22.38;
            9791400,22.34; 9792000,22.34; 9792600,22.34; 9793200,22.34; 9793800,22.34;
            9794400,22.34; 9795000,22.38; 9795600,22.38; 9796200,22.38; 9796800,22.38;
            9797400,22.38; 9798000,22.38; 9798600,22.38; 9799200,22.39; 9799800,22.39;
            9800400,22.38; 9801000,22.4; 9801600,22.56; 9802200,22.41; 9802800,22.38;
            9803400,22.38; 9804000,22.38; 9804600,22.38; 9805200,22.37; 9805800,22.38;
            9806400,22.38; 9807000,22.38; 9807600,22.38; 9808200,22.38; 9808800,22.38;
            9809400,22.38; 9810000,22.38; 9810600,22.38; 9811200,22.38; 9811800,22.38;
            9812400,22.37; 9813000,22.37; 9813600,22.34; 9814200,22.34; 9814800,22.31;
            9815400,22.3; 9816000,22.3; 9816600,22.3; 9817200,22.3; 9817800,22.26; 9818400,
            22.26; 9819000,22.26; 9819600,22.25; 9820200,22.22; 9820800,22.22; 9821400,
            22.2; 9822000,22.18; 9822600,22.17; 9823200,22.14; 9823800,22.14; 9824400,
            22.13; 9825000,22.1; 9825600,22.09; 9826200,22.06; 9826800,22.05; 9827400,
            22.05; 9828000,22.01; 9828600,22.01; 9829200,22; 9829800,21.97; 9830400,
            21.97; 9831000,21.97; 9831600,21.97; 9832200,21.93; 9832800,21.93; 9833400,
            21.93; 9834000,21.89; 9834600,21.89; 9835200,21.86; 9835800,21.85; 9836400,
            21.85; 9837000,21.85; 9837600,21.84; 9838200,21.81; 9838800,21.81; 9839400,
            21.81; 9840000,21.77; 9840600,21.77; 9841200,21.77; 9841800,21.75; 9842400,
            21.74; 9843000,21.73; 9843600,21.7; 9844200,21.69; 9844800,21.7; 9845400,
            21.69; 9846000,21.65; 9846600,21.67; 9847200,21.662; 9847800,21.654; 9848400,
            21.646; 9849000,21.638; 9849600,21.63; 9850200,21.6; 9850800,21.6; 9851400,
            21.6; 9852000,21.62; 9852600,21.56; 9853200,21.56; 9853800,21.55; 9854400,
            21.57; 9855000,21.52; 9855600,21.52; 9856200,21.52; 9856800,21.52; 9857400,
            21.52; 9858000,21.48; 9858600,21.48; 9859200,21.48; 9859800,21.48; 9860400,
            21.45; 9861000,21.46; 9861600,21.44; 9862200,21.44; 9862800,21.44; 9863400,
            21.44; 9864000,21.4; 9864600,21.4; 9865200,21.41; 9865800,21.4; 9866400,
            21.36; 9867000,21.39; 9867600,21.36; 9868200,21.36; 9868800,21.36; 9869400,
            21.36; 9870000,21.36; 9870600,21.37; 9871200,21.37; 9871800,21.39; 9872400,
            21.4; 9873000,21.4; 9873600,21.4; 9874200,21.44; 9874800,21.48; 9875400,
            21.48; 9876000,21.52; 9876600,21.52; 9877200,21.56; 9877800,21.56; 9878400,
            21.6; 9879000,21.6; 9879600,21.6; 9880200,21.6; 9880800,21.65; 9881400,21.7;
            9882000,21.78; 9882600,21.81; 9883200,21.81; 9883800,21.81; 9884400,21.81;
            9885000,21.77; 9885600,21.77; 9886200,21.81; 9886800,21.81; 9887400,21.85;
            9888000,21.92; 9888600,21.97; 9889200,22.05; 9889800,22.13; 9890400,22.14;
            9891000,22.09; 9891600,22.05; 9892200,21.97; 9892800,21.97; 9893400,21.97;
            9894000,21.97; 9894600,21.97; 9895200,21.93; 9895800,21.89; 9896400,21.85;
            9897000,21.84; 9897600,21.82; 9898200,21.81; 9898800,21.81; 9899400,21.77;
            9900000,21.77; 9900600,21.78; 9901200,21.8; 9901800,21.81; 9902400,21.81;
            9903000,21.81; 9903600,21.77; 9904200,21.77; 9904800,21.78; 9905400,21.77;
            9906000,21.77; 9906600,21.74; 9907200,21.74; 9907800,21.74; 9908400,21.69;
            9909000,21.68; 9909600,21.65; 9910200,21.62; 9910800,21.62; 9911400,21.59;
            9912000,21.56; 9912600,21.52; 9913200,21.48; 9913800,21.48; 9914400,21.46;
            9915000,21.44; 9915600,21.4; 9916200,21.4; 9916800,21.36; 9917400,21.35;
            9918000,21.32; 9918600,21.32; 9919200,21.33; 9919800,21.32; 9920400,21.28;
            9921000,21.28; 9921600,21.28; 9922200,21.24; 9922800,21.24; 9923400,21.2;
            9924000,21.2; 9924600,21.2; 9925200,21.16; 9925800,21.16; 9926400,21.16;
            9927000,21.17; 9927600,21.13; 9928200,21.13; 9928800,21.12; 9929400,21.12;
            9930000,21.08; 9930600,21.08; 9931200,21.08; 9931800,21.08; 9932400,21.08;
            9933000,21.04; 9933600,21.04; 9934200,21.03; 9934800,21.02; 9935400,21.01;
            9936000,21; 9936600,21; 9937200,20.96; 9937800,20.96; 9938400,20.96; 9939000,
            20.96; 9939600,20.96; 9940200,20.95; 9940800,20.94; 9941400,20.92; 9942000,
            20.91; 9942600,20.91; 9943200,20.91; 9943800,20.87; 9944400,20.87; 9945000,
            20.87; 9945600,20.87; 9946200,20.83; 9946800,20.83; 9947400,20.83; 9948000,
            20.83; 9948600,20.83; 9949200,20.83; 9949800,20.83; 9950400,20.83; 9951000,
            20.83; 9951600,20.84; 9952200,20.83; 9952800,20.83; 9953400,20.83; 9954000,
            20.84; 9954600,20.91; 9955200,21; 9955800,21.12; 9956400,21.25; 9957000,
            21.32; 9957600,21.44; 9958200,21.6; 9958800,21.69; 9959400,21.78; 9960000,
            22.05; 9960600,22.34; 9961200,22.46; 9961800,22.62; 9962400,22.78; 9963000,
            22.91; 9963600,23.04; 9964200,23.15; 9964800,23.17; 9965400,23.2; 9966000,
            23.19; 9966600,23.19; 9967200,23.19; 9967800,23.27; 9968400,23.4; 9969000,
            23.38; 9969600,23.36; 9970200,23.38; 9970800,23.36; 9971400,23.37; 9972000,
            23.36; 9972600,23.43; 9973200,23.44; 9973800,23.52; 9974400,23.58; 9975000,
            23.56; 9975600,23.6; 9976200,23.68; 9976800,23.68; 9977400,23.72; 9978000,
            23.68; 9978600,23.72; 9979200,23.73; 9979800,23.78; 9980400,23.77; 9981000,
            23.68; 9981600,23.6; 9982200,23.56; 9982800,23.56; 9983400,23.56; 9984000,
            23.64; 9984600,23.76; 9985200,23.75; 9985800,23.84; 9986400,23.84; 9987000,
            23.84; 9987600,23.85; 9988200,23.8; 9988800,23.76; 9989400,23.72; 9990000,
            23.68; 9990600,23.64; 9991200,23.56; 9991800,23.6; 9992400,23.68; 9993000,
            23.68; 9993600,23.68; 9994200,23.55; 9994800,23.48; 9995400,23.44; 9996000,
            23.32; 9996600,23.27; 9997200,23.19; 9997800,23.11; 9998400,23.03; 9999000,
            22.95; 9999600,22.83; 10000200,22.78; 10000800,22.78; 10001400,22.74; 10002000,
            22.62; 10002600,22.58; 10003200,22.5; 10003800,22.42; 10004400,22.38; 10005000,
            22.31; 10005600,22.26; 10006200,22.22; 10006800,22.14; 10007400,22.09; 10008000,
            22.06; 10008600,22.02; 10009200,21.97; 10009800,21.88; 10010400,21.85; 10011000,
            21.81; 10011600,21.8; 10012200,21.78; 10012800,21.73; 10013400,21.73; 10014000,
            21.7; 10014600,21.66; 10015200,21.64; 10015800,21.6; 10016400,21.57; 10017000,
            21.56; 10017600,21.52; 10018200,21.52; 10018800,21.5; 10019400,21.48; 10020000,
            21.48; 10020600,21.46; 10021200,21.44; 10021800,21.42; 10022400,21.4; 10023000,
            21.36; 10023600,21.36; 10024200,21.32; 10024800,21.32; 10025400,21.32; 10026000,
            21.32; 10026600,21.32; 10027200,21.32; 10027800,21.33; 10028400,21.28; 10029000,
            21.28; 10029600,21.28; 10030200,21.28; 10030800,21.25; 10031400,21.24; 10032000,
            21.24; 10032600,21.2; 10033200,21.23; 10033800,21.21; 10034400,21.2; 10035000,
            21.21; 10035600,21.23; 10036200,21.24; 10036800,21.28; 10037400,21.32; 10038000,
            21.3; 10038600,21.28; 10039200,21.28; 10039800,21.24; 10040400,21.28; 10041000,
            21.28; 10041600,21.33; 10042200,21.48; 10042800,21.81; 10043400,21.93; 10044000,
            22.09; 10044600,22.22; 10045200,22.29; 10045800,22.39; 10046400,22.42; 10047000,
            22.46; 10047600,22.47; 10048200,22.5; 10048800,22.5; 10049400,22.5; 10050000,
            22.59; 10050600,22.66; 10051200,22.67; 10051800,22.7; 10052400,22.78; 10053000,
            22.83; 10053600,22.94; 10054200,22.96; 10054800,22.95; 10055400,22.96; 10056000,
            23; 10056600,22.99; 10057200,23.03; 10057800,22.95; 10058400,22.95; 10059000,
            22.91; 10059600,22.87; 10060200,22.95; 10060800,23.03; 10061400,23.07; 10062000,
            23.15; 10062600,23.18; 10063200,23.23; 10063800,23.2; 10064400,23.24; 10065000,
            23.27; 10065600,23.34; 10066200,23.36; 10066800,23.36; 10067400,23.4; 10068000,
            23.4; 10068600,23.4; 10069200,23.44; 10069800,23.44; 10070400,23.47; 10071000,
            23.51; 10071600,23.48; 10072200,23.52; 10072800,23.52; 10073400,23.52; 10074000,
            23.6; 10074600,23.56; 10075200,23.56; 10075800,23.56; 10076400,23.56; 10077000,
            23.56; 10077600,23.54; 10078200,23.55; 10078800,23.45; 10079400,23.43; 10080000,
            23.48; 10080600,23.47; 10081200,23.46; 10081800,23.44; 10082400,23.4; 10083000,
            23.36; 10083600,23.24; 10084200,23.16; 10084800,23.1; 10085400,23.03; 10086000,
            22.98; 10086600,22.92; 10087200,22.83; 10087800,22.78; 10088400,22.7; 10089000,
            22.62; 10089600,22.58; 10090200,22.51; 10090800,22.43; 10091400,22.38; 10092000,
            22.3; 10092600,22.27; 10093200,22.18; 10093800,22.14; 10094400,22.09; 10095000,
            22.05; 10095600,21.98; 10096200,21.96; 10096800,21.9; 10097400,21.85; 10098000,
            21.81; 10098600,21.77; 10099200,21.73; 10099800,21.69; 10100400,21.65; 10101000,
            21.6; 10101600,21.6; 10102200,21.55; 10102800,21.52; 10103400,21.48; 10104000,
            21.44; 10104600,21.41; 10105200,21.4; 10105800,21.36; 10106400,21.342; 10107000,
            21.324; 10107600,21.306; 10108200,21.288; 10108800,21.27; 10109400,21.24;
            10110000,21.2; 10110600,21.16; 10111200,21.16; 10111800,21.16; 10112400,
            21.12; 10113000,21.12; 10113600,21.08; 10114200,21.08; 10114800,21.04; 10115400,
            21.04; 10116000,21.02; 10116600,21; 10117200,20.99; 10117800,20.96; 10118400,
            20.96; 10119000,20.95; 10119600,20.91; 10120200,20.91; 10120800,20.91; 10121400,
            20.87; 10122000,20.87; 10122600,20.87; 10123200,20.85; 10123800,20.83; 10124400,
            20.84; 10125000,20.83; 10125600,20.83; 10126200,20.87; 10126800,20.87; 10127400,
            20.96; 10128000,20.99; 10128600,21.12; 10129200,21.52; 10129800,21.85; 10130400,
            22.05; 10131000,22.09; 10131600,22.15; 10132200,22.18; 10132800,22.26; 10133400,
            22.27; 10134000,22.37; 10134600,22.46; 10135200,22.46; 10135800,22.5; 10136400,
            22.54; 10137000,22.62; 10137600,22.62; 10138200,22.7; 10138800,22.79; 10139400,
            22.86; 10140000,22.92; 10140600,22.98; 10141200,23.11; 10141800,23.23; 10142400,
            23.25; 10143000,23.27; 10143600,23.19; 10144200,23.21; 10144800,23.19; 10145400,
            23.19; 10146000,23.23; 10146600,23.24; 10147200,23.36; 10147800,23.4; 10148400,
            23.56; 10149000,23.61; 10149600,23.6; 10150200,23.64; 10150800,23.72; 10151400,
            23.8; 10152000,23.85; 10152600,23.92; 10153200,23.93; 10153800,23.97; 10154400,
            23.92; 10155000,23.96; 10155600,24.05; 10156200,24.06; 10156800,24.01; 10157400,
            23.96; 10158000,24.01; 10158600,24.01; 10159200,24.01; 10159800,24.04; 10160400,
            24.04; 10161000,24.04; 10161600,24.02; 10162200,23.92; 10162800,23.84; 10163400,
            23.92; 10164000,23.88; 10164600,23.94; 10165200,23.92; 10165800,23.96; 10166400,
            23.92; 10167000,23.8; 10167600,23.8; 10168200,23.76; 10168800,23.69; 10169400,
            23.6; 10170000,23.48; 10170600,23.4; 10171200,23.32; 10171800,23.23; 10172400,
            23.15; 10173000,23.07; 10173600,23.03; 10174200,22.95; 10174800,22.91; 10175400,
            22.83; 10176000,22.76; 10176600,22.66; 10177200,22.62; 10177800,22.54; 10178400,
            22.5; 10179000,22.43; 10179600,22.38; 10180200,22.3; 10180800,22.26; 10181400,
            22.22; 10182000,22.17; 10182600,22.14; 10183200,22.09; 10183800,22.05; 10184400,
            22.02; 10185000,21.97; 10185600,21.93; 10186200,21.89; 10186800,21.85; 10187400,
            21.81; 10188000,21.78; 10188600,21.73; 10189200,21.73; 10189800,21.69; 10190400,
            21.65; 10191000,21.61; 10191600,21.62; 10192200,21.56; 10192800,21.536;
            10193400,21.512; 10194000,21.488; 10194600,21.464; 10195200,21.44; 10195800,
            21.44; 10196400,21.44; 10197000,21.41; 10197600,21.4; 10198200,21.37; 10198800,
            21.36; 10199400,21.32; 10200000,21.32; 10200600,21.32; 10201200,21.29; 10201800,
            21.28; 10202400,21.28; 10203000,21.24; 10203600,21.24; 10204200,21.24; 10204800,
            21.2; 10205400,21.2; 10206000,21.2; 10206600,21.16; 10207200,21.16; 10207800,
            21.16; 10208400,21.16; 10209000,21.16; 10209600,21.16; 10210200,21.16; 10210800,
            21.16; 10211400,21.16; 10212000,21.19; 10212600,21.18; 10213200,21.2; 10213800,
            21.24; 10214400,21.32; 10215000,21.65; 10215600,21.93; 10216200,22.14; 10216800,
            22.26; 10217400,22.42; 10218000,22.46; 10218600,22.5; 10219200,22.58; 10219800,
            22.64; 10220400,22.74; 10221000,22.78; 10221600,22.83; 10222200,22.92; 10222800,
            22.91; 10223400,22.94; 10224000,22.98; 10224600,23.03; 10225200,23.11; 10225800,
            23.18; 10226400,23.19; 10227000,23.25; 10227600,23.35; 10228200,23.32; 10228800,
            23.35; 10229400,23.32; 10230000,23.32; 10230600,23.3; 10231200,23.27; 10231800,
            23.27; 10232400,23.31; 10233000,23.36; 10233600,23.43; 10234200,23.44; 10234800,
            23.44; 10235400,23.56; 10236000,23.64; 10236600,23.68; 10237200,23.68; 10237800,
            23.6; 10238400,23.61; 10239000,23.68; 10239600,23.68; 10240200,23.76; 10240800,
            23.72; 10241400,23.8; 10242000,23.82; 10242600,23.8; 10243200,23.84; 10243800,
            23.72; 10244400,23.71; 10245000,23.64; 10245600,23.6; 10246200,23.59; 10246800,
            23.6; 10247400,23.56; 10248000,23.52; 10248600,23.49; 10249200,23.48; 10249800,
            23.56; 10250400,23.56; 10251000,23.6; 10251600,23.6; 10252200,23.57; 10252800,
            23.6; 10253400,23.52; 10254000,23.44; 10254600,23.36; 10255200,23.27; 10255800,
            23.23; 10256400,23.16; 10257000,23.07; 10257600,22.99; 10258200,22.91; 10258800,
            22.84; 10259400,22.74; 10260000,22.66; 10260600,22.58; 10261200,22.5; 10261800,
            22.43; 10262400,22.38; 10263000,22.3; 10263600,22.22; 10264200,22.14; 10264800,
            22.09; 10265400,22.01; 10266000,21.97; 10266600,21.93; 10267200,21.89; 10267800,
            21.81; 10268400,21.77; 10269000,21.69; 10269600,21.65; 10270200,21.6; 10270800,
            21.52; 10271400,21.48; 10272000,21.44; 10272600,21.4; 10273200,21.35; 10273800,
            21.32; 10274400,21.24; 10275000,21.2; 10275600,21.12; 10276200,21.11; 10276800,
            21.04; 10277400,21.01; 10278000,20.99; 10278600,20.96; 10279200,20.91; 10279800,
            20.86; 10280400,20.81; 10281000,20.76; 10281600,20.71; 10282200,20.71; 10282800,
            20.68; 10283400,20.63; 10284000,20.63; 10284600,20.58; 10285200,20.55; 10285800,
            20.51; 10286400,20.51; 10287000,20.46; 10287600,20.42; 10288200,20.44; 10288800,
            20.38; 10289400,20.38; 10290000,20.34; 10290600,20.34; 10291200,20.3; 10291800,
            20.3; 10292400,20.3; 10293000,20.3; 10293600,20.3; 10294200,20.3; 10294800,
            20.26; 10295400,20.22; 10296000,20.18; 10296600,20.22; 10297200,20.3; 10297800,
            20.3; 10298400,20.3; 10299000,20.3; 10299600,20.38; 10300200,20.46; 10300800,
            20.8; 10301400,21.16; 10302000,21.44; 10302600,21.69; 10303200,21.93; 10303800,
            22.09; 10304400,22.29; 10305000,22.44; 10305600,22.62; 10306200,22.84; 10306800,
            22.92; 10307400,22.95; 10308000,23.07; 10308600,23.12; 10309200,23.23; 10309800,
            23.3; 10310400,23.31; 10311000,23.32; 10311600,23.36; 10312200,23.4; 10312800,
            23.4; 10313400,23.45; 10314000,23.57; 10314600,23.52; 10315200,23.56; 10315800,
            23.52; 10316400,23.52; 10317000,23.48; 10317600,23.48; 10318200,23.48; 10318800,
            23.53; 10319400,23.56; 10320000,23.72; 10320600,23.84; 10321200,23.89; 10321800,
            23.92; 10322400,23.96; 10323000,24.01; 10323600,24.06; 10324200,24.09; 10324800,
            24.09; 10325400,24.18; 10326000,24.21; 10326600,24.21; 10327200,24.2; 10327800,
            24.29; 10328400,24.33; 10329000,24.3; 10329600,24.22; 10330200,24.17; 10330800,
            24.13; 10331400,24.12; 10332000,24.09; 10332600,24.1; 10333200,24.05; 10333800,
            24.05; 10334400,24.04; 10335000,24.01; 10335600,24.04; 10336200,24.02; 10336800,
            24.04; 10337400,24.04; 10338000,24.05; 10338600,24.05; 10339200,24.09; 10339800,
            24.01; 10340400,23.96; 10341000,23.92; 10341600,23.84; 10342200,23.8; 10342800,
            23.68; 10343400,23.6; 10344000,23.52; 10344600,23.44; 10345200,23.4; 10345800,
            23.32; 10346400,23.27; 10347000,23.17; 10347600,23.11; 10348200,23.03; 10348800,
            22.99; 10349400,22.91; 10350000,22.87; 10350600,22.79; 10351200,22.74; 10351800,
            22.7; 10352400,22.65; 10353000,22.62; 10353600,22.54; 10354200,22.5; 10354800,
            22.46; 10355400,22.4; 10356000,22.34; 10356600,22.3; 10357200,22.26; 10357800,
            22.22; 10358400,22.2; 10359000,22.18; 10359600,22.13; 10360200,22.1; 10360800,
            22.06; 10361400,22.03; 10362000,21.99; 10362600,21.97; 10363200,21.93; 10363800,
            21.9; 10364400,21.87; 10365000,21.85; 10365600,21.81; 10366200,21.8; 10366800,
            21.77; 10367400,21.73; 10368000,21.73; 10368600,21.69; 10369200,21.65; 10369800,
            21.64; 10370400,21.6; 10371000,21.6; 10371600,21.56; 10372200,21.52; 10372800,
            21.52; 10373400,21.48; 10374000,21.48; 10374600,21.44; 10375200,21.44; 10375800,
            21.4; 10376400,21.4; 10377000,21.36; 10377600,21.36; 10378200,21.32; 10378800,
            21.32; 10379400,21.28; 10380000,21.28; 10380600,21.26; 10381200,21.24; 10381800,
            21.24; 10382400,21.2; 10383000,21.2; 10383600,21.2; 10384200,21.2; 10384800,
            21.2; 10385400,21.2; 10386000,21.2; 10386600,21.2; 10387200,21.2; 10387800,
            21.21; 10388400,21.24; 10389000,21.25; 10389600,21.28; 10390200,21.29; 10390800,
            21.32; 10391400,21.32; 10392000,21.36; 10392600,21.36; 10393200,21.4; 10393800,
            21.4; 10394400,21.44; 10395000,21.47; 10395600,21.72; 10396200,21.48; 10396800,
            21.52; 10397400,21.56; 10398000,21.62; 10398600,21.66; 10399200,21.65; 10399800,
            21.72; 10400400,21.76; 10401000,21.77; 10401600,21.82; 10402200,21.82; 10402800,
            21.81; 10403400,21.81; 10404000,21.81; 10404600,21.82; 10405200,21.81; 10405800,
            21.81; 10406400,21.81; 10407000,21.81; 10407600,21.8; 10408200,21.8; 10408800,
            21.77; 10409400,21.77; 10410000,21.77; 10410600,21.75; 10411200,21.73; 10411800,
            21.72; 10412400,21.69; 10413000,21.69; 10413600,21.68; 10414200,21.65; 10414800,
            21.65; 10415400,21.65; 10416000,21.64; 10416600,21.6; 10417200,21.6; 10417800,
            21.6; 10418400,21.62; 10419000,21.6; 10419600,21.6; 10420200,21.56; 10420800,
            21.56; 10421400,21.52; 10422000,21.52; 10422600,21.49; 10423200,21.48; 10423800,
            21.48; 10424400,21.45; 10425000,21.44; 10425600,21.44; 10426200,21.4; 10426800,
            21.4; 10427400,21.4; 10428000,21.36; 10428600,21.35; 10429200,21.32; 10429800,
            21.32; 10430400,21.32; 10431000,21.28; 10431600,21.28; 10432200,21.24; 10432800,
            21.25; 10433400,21.22; 10434000,21.2; 10434600,21.2; 10435200,21.16; 10435800,
            21.16; 10436400,21.16; 10437000,21.16; 10437600,21.12; 10438200,21.13; 10438800,
            21.12; 10439400,21.1; 10440000,21.08; 10440600,21.08; 10441200,21.08; 10441800,
            21.05; 10442400,21.04; 10443000,21.04; 10443600,21; 10444200,21; 10444800,
            21; 10445400,20.97; 10446000,20.96; 10446600,20.96; 10447200,20.96; 10447800,
            20.96; 10448400,20.91; 10449000,20.93; 10449600,20.91; 10450200,20.91; 10450800,
            20.91; 10451400,20.91; 10452000,20.91; 10452600,20.91; 10453200,20.91; 10453800,
            20.91])
        annotation (Placement(transformation(extent={{-84,62},{-64,82}})));
      Modelica.Blocks.Math.UnitConversions.From_degC from_degC
        annotation (Placement(transformation(extent={{-50,62},{-30,82}})));
      Buildings.Fluid.Sources.Boundary_pT AirVolume(
        use_T_in=true,
        nPorts=2,
        redeclare package Medium = MediumA)
        annotation (Placement(transformation(extent={{26,58},{46,78}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                                weaDat(
        filNam="modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos")
        "Weather data reader"
        annotation (Placement(transformation(extent={{-92,20},{-72,40}})));
      Buildings.Fluid.Sources.FixedBoundary sinHea(
        redeclare package Medium = MediumW,
        nPorts=1,
        T=308.15) "Sink for heating coil" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-32,-50})));
      Buildings.Fluid.Sources.FixedBoundary souHea(
        redeclare package Medium = MediumW,
        nPorts=1,
        p(displayUnit="Pa"),
        T=328.15) "Source for heating coil" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-52,-50})));
      Buildings.Fluid.Sensors.MassFlowRate senMasFlo(redeclare package Medium =
            MediumA)
        annotation (Placement(transformation(extent={{38,-22},{58,-2}})));
      Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package
          Medium =
            MediumA, m_flow_nominal=75*1.225*1/3600) annotation (Placement(
            transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={62,22})));
      Buildings.Fluid.FixedResistances.PressureDrop PressureDrop(
        from_dp=false,
        redeclare package Medium = MediumA,
        m_flow_nominal=75*1.225*1/3600,
        dp_nominal=25) "Simulate pressure drop " annotation (Placement(
            transformation(extent={{-10,-10},{10,10}}, origin={18,-12})));
      Buildings.Fluid.FixedResistances.PressureDrop PressureDrop1(
        from_dp=false,
        redeclare package Medium = MediumA,
        m_flow_nominal=75*1.225*1/3600,
        dp_nominal=25) "Simulate pressure drop " annotation (Placement(
            transformation(
            extent={{10,-10},{-10,10}},
            origin={62,50},
            rotation=90)));
    equation
      connect(TRoom.y,from_degC. u)
        annotation (Line(points={{-63,72},{-52,72}}, color={0,0,127}));
      connect(from_degC.y, AirVolume.T_in)
        annotation (Line(points={{-29,72},{-29,72},{24,72}}, color={0,0,127}));
      connect(from_degC.y, cTA_VAVReheat.TRoo[1])
        annotation (Line(points={{-29,72},{-14,72},{-14,18.2}},
                                                              color={0,0,127}));
      connect(cTA_VAVReheat.port_b1, AirVolume.ports[1:1]) annotation (Line(points={{0,9},{
              68,9},{68,70},{46,70}},         color={0,127,255}));
      connect(weaDat.weaBus, cTA_VAVReheat.weaBus) annotation (Line(
          points={{-72,30},{-56,30},{-56,14.8},{-54.4,14.8}},
          color={255,204,51},
          thickness=0.5));
      connect(cTA_VAVReheat.port_a, souHea.ports[1])
        annotation (Line(points={{-50,-4},{-50,-24},{-50,-40},{-52,-40}},
                                                      color={0,127,255}));
      connect(cTA_VAVReheat.port_b, sinHea.ports[1])
        annotation (Line(points={{-30,-4},{-30,-40},{-32,-40}},
                                                         color={0,127,255}));
      connect(senTem.port_a, senMasFlo.port_b) annotation (Line(points={{62,12},
              {62,-12},{58,-12}}, color={0,127,255}));
      connect(cTA_VAVReheat.port_a2[1], PressureDrop.port_a)
        annotation (Line(points={{0,-12},{8,-12}}, color={0,127,255}));
      connect(PressureDrop.port_b, senMasFlo.port_a) annotation (Line(points={{
              28,-12},{34,-12},{38,-12}}, color={0,127,255}));
      connect(AirVolume.ports[2], PressureDrop1.port_a)
        annotation (Line(points={{46,66},{62,66},{62,60}}, color={0,127,255}));
      connect(PressureDrop1.port_b, senTem.port_b)
        annotation (Line(points={{62,40},{62,32}}, color={0,127,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end VAVReheat;
  end Examples;

  model CTA_VAVReheat_Hydro "VAV Reheat AHU with hydronicVAV heater"
     extends Buildings.Fluid.Interfaces.PartialTwoPortInterface(redeclare
        replaceable package Medium =
          Buildings.Media.Water);
     extends FBM.AHUSystems.BaseClasses.VAVReheatParameter;
    replaceable package MediumA =
        Buildings.Media.Air(T_default=293.15) "Medium model for Air";
    package MediumW = Buildings.Media.Water "Medium model for water";
    Buildings.Fluid.Sources.Outside
                          amb(redeclare package Medium = MediumA, nPorts=2)
      "Ambient conditions"
      annotation (Placement(transformation(extent={{-84,20},{-62,42}})));
    Buildings.Fluid.FixedResistances.PressureDrop fil(
      m_flow_nominal=m_flow_nominal,
      redeclare package Medium = MediumA,
      dp_nominal=200 + 200 + 100,
      from_dp=false,
      linearized=false) "Filter"
      annotation (Placement(transformation(extent={{70,-20},{90,0}})));
    Buildings.Fluid.HeatExchangers.DryEffectivenessNTU heaCoi(
      redeclare package Medium1 = MediumA,
      redeclare package Medium2 = MediumW,
      allowFlowReversal2=false,
      configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
      dp1_nominal=0,
      m1_flow_nominal=m_flow_nominal_Air,
      m2_flow_nominal=m_flow_nominal_Air*1000*(10 - (-20))/4200/10,
      Q_flow_nominal=m_flow_nominal_Air*1006*(16.7 - 8.5),
      dp2_nominal=dpHotDeck,
      T_a1_nominal=281.65,
      T_a2_nominal=328.15) "Heating coil"
      annotation (Placement(transformation(extent={{108,-26},{128,-6}})));
    Buildings.Fluid.FixedResistances.PressureDrop dpSupDuc(
      redeclare package Medium = MediumA,
      dp_nominal=20,
      m_flow_nominal=m_flow_nominal_Air) "Pressure drop for supply duct"
      annotation (Placement(transformation(extent={{330,-20},{350,0}})));
    Buildings.Fluid.FixedResistances.PressureDrop dpRetDuc(
      redeclare package Medium = MediumA,
      dp_nominal=20,
      m_flow_nominal=m_flow_nominal_Air) "Pressure drop for return duct"
      annotation (Placement(transformation(extent={{332,120},{312,140}})));
    Buildings.Fluid.Movers.SpeedControlled_y fanSup(
      redeclare package Medium = MediumA,
      tau=60,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow={0,m_flow_nominal_Air/1.225*2}, dp={850,0})))
                                                                 "Supply air fan"
      annotation (Placement(transformation(extent={{234,-20},{254,0}})));
    Buildings.Fluid.Movers.SpeedControlled_y fanRet(
      redeclare package Medium = MediumA,
      tau=60,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow=m_flow_nominal_Air/1.225*{0,2}, dp=1.5*110*{2,0})))
                                                                 "Return air fan"
      annotation (Placement(transformation(extent={{184,120},{164,140}})));
    Modelica.Blocks.Routing.RealPassThrough TOut(y(
        final quantity="ThermodynamicTemperature",
        final unit="K",
        displayUnit="degC",
        min=0))
      annotation (Placement(transformation(extent={{-150,184},{-130,204}})));
    Modelica.Blocks.Sources.Constant TSupSetHea(k=TSupSetHeat)
                               "Supply air temperature setpoint for heating"
      annotation (Placement(transformation(extent={{-172,120},{-152,140}})));
    Buildings.Controls.Continuous.LimPID heaCoiCon(
      yMax=1,
      yMin=0,
      Td=60,
      initType=Modelica.Blocks.Types.InitPID.InitialState,
      Ti=600,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=0.05) "Controller for heating coil"
      annotation (Placement(transformation(extent={{-22,-38},{-2,-18}})));
    Buildings.Fluid.Sensors.RelativePressure dpRetFan(
        redeclare package Medium = MediumA) "Pressure difference over return fan"
                                              annotation (Placement(
          transformation(
          extent={{-10,10},{10,-10}},
          rotation=90,
          origin={254,16})));
    Buildings.Examples.VAVReheat.Controls.FanVFD
                    conFanSup(xSet_nominal(displayUnit="Pa") = 410, r_N_min=
          r_N_min,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
      "Controller for fan"
      annotation (Placement(transformation(extent={{182,24},{202,44}})));
    Buildings.Fluid.Sensors.VolumeFlowRate senSupFlo(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Sensor for supply fan flow rate"
      annotation (Placement(transformation(extent={{288,-20},{308,0}})));
    Buildings.Controls.SetPoints.OccupancySchedule occSch(occupancy=3600*{OccOn,
          OccOff})
      "Occupancy schedule"
      annotation (Placement(transformation(extent={{346,152},{326,172}})));
    Buildings.Examples.VAVReheat.Controls.ModeSelector
                          modeSelector(
      delTRooOnOff=delTRooOnOff,
      TRooSetHeaOcc=TRooSetHeaOcc,
      TRooSetCooOcc=TRooSetCooOcc,
      TSetHeaCoiOut=TSetHeaCoiOut)
      annotation (Placement(transformation(extent={{-128,162},{-108,182}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TCoiHeaOut(redeclare package
        Medium = MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Heating coil outlet temperature"
      annotation (Placement(transformation(extent={{144,-20},{164,0}})));
    Buildings.Examples.VAVReheat.Controls.Economizer
                        conEco(
      dT=1,
      Ti=600,
      k=0.1,
      VOut_flow_min=0.3*m_flow_nominal_Air/1.2)
             "Controller for economizer"
      annotation (Placement(transformation(extent={{-60,124},{-40,144}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TRet(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Return air temperature sensor"
      annotation (Placement(transformation(extent={{110,120},{90,140}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort TMix(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Mixed air temperature sensor"
      annotation (Placement(transformation(extent={{40,-20},{60,0}})));
    Buildings.Examples.VAVReheat.Controls.RoomTemperatureSetpoint
                                     TSetRoo(
      THeaOn=THeaOn,
      THeaOff=THeaOff,
      TCooOn=TCooOn,
      TCooOff=TCooOff)
      annotation (Placement(transformation(extent={{348,178},{328,198}})));
    Buildings.Fluid.Actuators.Valves.TwoWayLinear valHea(
      redeclare package Medium = MediumW,
      CvData=Buildings.Fluid.Types.CvTypes.OpPoint,
      from_dp=true,
      m_flow_nominal=m_flow_nominal_Air*1000*40/4200/10,
      dpValve_nominal=dpVal_nominal)
                            "Heating coil valve"
                                         annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={140,-34})));
    Buildings.Fluid.Actuators.Dampers.MixingBox eco(
      redeclare package Medium = MediumA,
      dpOut_nominal=10,
      dpRec_nominal=10,
      dpExh_nominal=10,
      mOut_flow_nominal=m_flow_nominal_Air,
      mRec_flow_nominal=m_flow_nominal_Air,
      mExh_flow_nominal=m_flow_nominal_Air)
                        "Economizer"
      annotation (Placement(transformation(extent={{-14,56},{30,12}})));
    Buildings.Fluid.Sensors.VolumeFlowRate VOut1(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
                                                  "Outside air volume flow rate"
      annotation (Placement(transformation(extent={{-46,12},{-24,34}})));
    Buildings.Examples.VAVReheat.Controls.DuctStaticPressureSetpoint
                                        pSetDuc(
      nin=nReheat,
      pMin=pMin,
      pMax=pMax,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
               "Duct static pressure setpoint"
      annotation (Placement(transformation(extent={{128,24},{148,44}})));
    FBM.Components.VAVBranch_hydro[nReheat]            ReHeater(
      redeclare package MediumA = MediumA,
      redeclare package MediumW = MediumW,
      VRoo={VRoom[i] for i in 1:nReheat},
      m_flow_nominal={m_flow_room[i] for i in 1:nReheat}) "Zone of building"
      annotation (Placement(transformation(extent={{316,12},{384,80}})));
    Buildings.Examples.VAVReheat.Controls.FanVFD
                    conFanRet(
                          xSet_nominal(displayUnit="m3/s") = m_flow_nominal_Air/1.2, r_N_min=
          r_N_min,
      controllerType=Modelica.Blocks.Types.SimpleController.P)
                          "Controller for fan"
      annotation (Placement(transformation(extent={{124,150},{144,170}})));
    Buildings.Fluid.Sensors.VolumeFlowRate senRetFlo(redeclare package Medium =
          MediumA, m_flow_nominal=m_flow_nominal_Air)
      "Sensor for return fan flow rate"
      annotation (Placement(transformation(extent={{308,120},{288,140}})));
    Buildings.Examples.VAVReheat.Controls.CoolingCoilTemperatureSetpoint
                                            TSetCoo(TCooOn=TCooOn, TCooOff=
          TCooOff)                                  "Setpoint for cooling coil"
      annotation (Placement(transformation(extent={{-132,138},{-112,158}})));
    Buildings.Examples.VAVReheat.Controls.ControlBus
                        controlBus
      annotation (Placement(transformation(extent={{290,184},{310,204}}),
          iconTransformation(extent={{290,184},{310,204}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus annotation (Placement(
          transformation(extent={{-188,166},{-148,206}}), iconTransformation(
            extent={{-154,178},{-134,198}})));
    Modelica.Blocks.Interfaces.RealInput[nReheat] TRoo(unit="K", displayUnit="degC")
      "Measured room temperature"
      annotation (Placement(transformation(extent={{-20,-20},{20,20}},
          rotation=270,
          origin={260,222})));
    Modelica.Fluid.Interfaces.FluidPort_a[nReheat] port_a2(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{390,-90},{410,-70}})));
    Modelica.Fluid.Interfaces.FluidPort_b[nReheat] port_b1(redeclare package
        Medium = MediumA)
      annotation (Placement(transformation(extent={{390,120},{410,140}})));
    Buildings.Utilities.Math.Min min(nin=nReheat)
                                            "Computes lowest room temperature"
      annotation (Placement(transformation(extent={{266,170},{286,190}})));
    Buildings.Utilities.Math.Average ave(nin=nReheat)
      "Compute average of room temperatures"
      annotation (Placement(transformation(extent={{266,146},{286,166}})));
    Buildings.Fluid.Movers.SpeedControlled_y pumHotDeck(
      redeclare package Medium = MediumW,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      per(pressure(V_flow=(m_flow_nominal_Air*1000*40/4200/10)/1000*{0,2}, dp=
              dp_nominal*{2,0}))) "Pump that serves the radiators" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={74,-64})));
    Buildings.Fluid.Sensors.RelativePressure dpSen(redeclare package Medium =
          MediumW)
      annotation (Placement(transformation(extent={{-10,10},{10,-10}},
          rotation=180,
          origin={74,-80})));
    Modelica.Blocks.Math.Gain gain(k=1/dp_nominal)
      "Gain used to normalize pressure measurement signal"
      annotation (Placement(transformation(extent={{94,-106},{114,-86}})));
    Buildings.Controls.Continuous.PIDHysteresisTimer
                                           conPum(
      yMax=1,
      Td=60,
      yMin=0.05,
      eOn=0.5,
      Ti=15,
      controllerType=Modelica.Blocks.Types.SimpleController.P,
      k=1)   "Controller for pump"
      annotation (Placement(transformation(extent={{146,-92},{126,-72}})));
    Modelica.Blocks.Sources.Constant pumHotDeckOn(k=1) "Pump on signal"
      annotation (Placement(transformation(extent={{212,-68},{192,-48}})));
    Modelica.Blocks.Logical.Hysteresis hysPum(           uLow=0.01, uHigh=0.5)
      "Hysteresis for pump"
      annotation (Placement(transformation(extent={{236,-80},{216,-60}})));
    Modelica.Blocks.Logical.Switch swiPum "Pump switch"
      annotation (Placement(transformation(extent={{178,-92},{158,-72}})));
    Modelica.Blocks.Sources.Constant pumHotDeckOff(k=0) "Pump off signal"
      annotation (Placement(transformation(extent={{212,-100},{192,-80}})));
    Buildings.Fluid.FixedResistances.Junction jun(
      redeclare package Medium = MediumW,
      m_flow_nominal=(m_flow_nominal_Air*1000*40/4200/10)*({1,-1,-1}),
      dp_nominal={300,-300,-300})
      annotation (Placement(transformation(extent={{130,-54},{150,-74}})));
  equation
    connect(fil.port_b,heaCoi. port_a1) annotation (Line(
        points={{90,-10},{108,-10}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TSupSetHea.y,heaCoiCon. u_s) annotation (Line(
        points={{-151,130},{-36,130},{-36,-28},{-24,-28}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(fanRet.port_a,dpRetFan. port_b) annotation (Line(
        points={{184,130},{254,130},{254,26}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dot));
    connect(fanSup.port_b,dpRetFan. port_a) annotation (Line(
        points={{254,-10},{254,6}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dot));
    connect(senSupFlo.port_b,dpSupDuc. port_a) annotation (Line(
        points={{308,-10},{330,-10}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(controlBus,modeSelector. cb) annotation (Line(
        points={{300,194},{-124,194},{-124,178.818},{-124.818,178.818}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(occSch.tNexOcc,controlBus. dTNexOcc) annotation (Line(
        points={{325,168},{320,168},{320,194},{300,194}},
        color={0,0,127},
        pattern=LinePattern.Dash),
                             Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TOut.y,controlBus. TOut) annotation (Line(
        points={{-129,194},{300,194}},
        color={255,213,170},
        smooth=Smooth.None,
        thickness=0.5),      Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(occSch.occupied,controlBus. occupied) annotation (Line(
        points={{325,156},{300,156},{300,194}},
        color={255,0,255},
        pattern=LinePattern.Dash),
                             Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(controlBus,conFanSup. controlBus) annotation (Line(
        points={{300,194},{156,194},{156,42},{185,42}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TRet.T,conEco. TRet) annotation (Line(
        points={{100,141},{100,182},{-72,182},{-72,141.333},{-61.3333,141.333}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(TMix.T,conEco. TMix) annotation (Line(
        points={{50,1},{50,178},{-70,178},{-70,137.333},{-61.3333,137.333}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(conEco.TSupHeaSet,TSupSetHea. y) annotation (Line(
        points={{-61.3333,129.333},{-148,129.333},{-148,130},{-151,130}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(controlBus,conEco. controlBus) annotation (Line(
        points={{300,194},{-86,194},{-86,134.667},{-56,134.667}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(TSetRoo.controlBus,controlBus)  annotation (Line(
        points={{336,194},{300,194}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(fil.port_a,TMix. port_b) annotation (Line(
        points={{70,-10},{60,-10}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(conFanSup.y,fanSup. y) annotation (Line(
        points={{203,34},{243.8,34},{243.8,2}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(TCoiHeaOut.T,heaCoiCon. u_m) annotation (Line(
        points={{154,1},{154,-12},{10,-12},{10,-40},{-12,-40}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(heaCoiCon.y,valHea. y) annotation (Line(
        points={{-1,-28},{16,-28},{16,-34},{128,-34}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(valHea.port_b,heaCoi. port_a2) annotation (Line(
        points={{140,-24},{140,-22},{128,-22}},
        color={0,127,0},
        smooth=Smooth.None,
        thickness=0.5));
    connect(eco.port_Exh,amb. ports[1]) annotation (Line(
        points={{-14,47.2},{-48,47.2},{-48,33.2},{-62,33.2}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(amb.ports[2],VOut1. port_a) annotation (Line(
        points={{-62,28.8},{-62,28},{-50,28},{-50,24},{-46,24},{-46,23}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(VOut1.port_b,eco. port_Out) annotation (Line(
        points={{-24,23},{-24,20.8},{-14,20.8}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(dpRetFan.p_rel,conFanSup. u_m) annotation (Line(
        points={{245,16},{192,16},{192,22}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(eco.port_Sup,TMix. port_a) annotation (Line(
        points={{30,20.8},{36,20.8},{36,-10},{40,-10}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(pSetDuc.y,conFanSup. u) annotation (Line(
        points={{149,34},{180,34}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(heaCoi.port_b1,TCoiHeaOut. port_a) annotation (Line(
        points={{128,-10},{144,-10}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(controlBus,conFanRet. controlBus) annotation (Line(
        points={{300,194},{100,194},{100,168},{127,168}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(senSupFlo.V_flow,conFanRet. u) annotation (Line(
        points={{298,1},{298,86},{110,86},{110,160},{122,160}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(senRetFlo.port_b,fanRet. port_a) annotation (Line(
        points={{288,130},{184,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(senRetFlo.V_flow,conFanRet. u_m) annotation (Line(
        points={{298,141},{298,144},{134,144},{134,148}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(conFanRet.y,fanRet. y) annotation (Line(
        points={{145,160},{174.2,160},{174.2,142}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(dpRetDuc.port_b,senRetFlo. port_a) annotation (Line(
        points={{312,130},{308,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TRet.port_b,eco. port_Ret) annotation (Line(
        points={{90,130},{34,130},{34,47.2},{30,47.2}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TRet.port_a,fanRet. port_b) annotation (Line(
        points={{110,130},{164,130}},
        color={0,127,255},
        smooth=Smooth.None,
        thickness=0.5));
    connect(TSetCoo.TSet,conEco. TSupCooSet) annotation (Line(
        points={{-111,148},{-104,148},{-104,126},{-82,126},{-82,125.333},{
            -61.3333,125.333}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(TSupSetHea.y,TSetCoo. TSetHea) annotation (Line(
        points={{-151,130},{-142,130},{-142,148},{-134,148}},
        color={0,0,0},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(modeSelector.cb,TSetCoo. controlBus) annotation (Line(
        points={{-124.818,178.818},{-124,178.818},{-124,140},{-123.8,140}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(conEco.VOut_flow,VOut1. V_flow) annotation (Line(
        points={{-61.3333,133.333},{-64,133.333},{-64,118},{-35,118},{-35,35.1}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(amb.weaBus, weaBus) annotation (Line(
        points={{-84,31.22},{-168,31.22},{-168,186}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None));
    connect(weaBus.TDryBul,pSetDuc. TOut) annotation (Line(
        points={{-168,186},{116,186},{116,42},{126,42}},
        color={255,204,51},
        thickness=0.5,
        smooth=Smooth.None), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
    connect(conEco.yOA,eco. y) annotation (Line(
        points={{-39.3333,136},{-36,136},{-36,-10},{8,-10},{8,7.6}},
        color={0,0,127},
        smooth=Smooth.None,
        pattern=LinePattern.Dash));
    connect(weaBus.TDryBul, TOut.u) annotation (Line(
        points={{-168,186},{-160,186},{-160,194},{-152,194}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
       connect(ReHeater.yDam, pSetDuc.u) annotation (Line(points={{386.267,
            34.6667},{390,34.6667},{390,34},{396,34},{396,110},{110,110},{110,
            34},{126,34}},
          color={0,0,127},
        pattern=LinePattern.Dash));
            for i in 1:nReheat loop
             connect(controlBus, ReHeater[i].controlBus) annotation (Line(
        points={{300,194},{274,194},{274,28.32},{316,28.32}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-6,3},{-6,3}}));
        connect(dpSupDuc.port_b, ReHeater[i].port_a) annotation (Line(points={{350,-10},
              {350,28.7733}},         color={0,127,255}));
                connect(dpRetDuc.port_a, port_b1[i]) annotation (Line(points={{332,130},{366,130},
            {400,130}}, color={0,127,255}));
              connect(port_b, ReHeater[i].port_b1) annotation (Line(points={{100,0},
              {390,0},{390,40.1067},{384,40.1067}},
                                          color={0,127,255}));
    connect(jun.port_2, ReHeater[i].port_a1) annotation (Line(points={{150,-64},
              {394,-64},{394,47.8133},{384.453,47.8133}},
                                                   color={0,127,255}));
            end for;
    connect(TRoo, ReHeater.TRoo) annotation (Line(points={{260,222},{292,222},{
            292,57.3333},{311.467,57.3333}},
                                         color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ReHeater.port_b, port_a2) annotation (Line(points={{350,80},{350,80},{
            350,88},{392,88},{392,-80},{400,-80}}, color={0,127,255}));
    connect(TRoo, min.u) annotation (Line(
        points={{260,222},{260,222},{260,180},{264,180}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ave.u, TRoo) annotation (Line(
        points={{264,156},{260,156},{260,222}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(ave.y, controlBus.TRooAve) annotation (Line(
        points={{287,156},{300,156},{300,194}},
        color={0,0,127},
        pattern=LinePattern.Dash), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(min.y, controlBus.TRooMin) annotation (Line(
        points={{287,180},{300,180},{300,194}},
        color={0,0,127},
        pattern=LinePattern.Dash), Text(
        string="%second",
        index=1,
        extent={{6,3},{6,3}}));
    connect(TCoiHeaOut.port_b, fanSup.port_a) annotation (Line(
        points={{164,-10},{200,-10},{234,-10}},
        color={0,127,255},
        thickness=0.5));
    connect(fanSup.port_b, senSupFlo.port_a) annotation (Line(points={{254,-10},{272,
            -10},{288,-10}},      color={0,127,255}));
    connect(pumHotDeck.port_b, dpSen.port_a) annotation (Line(
        points={{84,-64},{86,-64},{86,-80},{84,-80}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(dpSen.port_b, pumHotDeck.port_a) annotation (Line(
        points={{64,-80},{62,-80},{62,-64},{64,-64}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(gain.u,dpSen. p_rel) annotation (Line(
        points={{92,-96},{74,-96},{74,-89}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(pumHotDeck.port_a, port_a) annotation (Line(points={{64,-64},{-101,
            -64},{-101,0},{-100,0}}, color={0,127,255}));
    connect(gain.y, conPum.u_m) annotation (Line(
        points={{115,-96},{115,-96},{114,-96},{134,-96},{136,-96},{136,-94}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(hysPum.y,swiPum. u2) annotation (Line(
        points={{215,-70},{192,-70},{192,-82},{180,-82}},
        color={255,0,255},
        pattern=LinePattern.Dash));
    connect(pumHotDeckOn.y, swiPum.u1) annotation (Line(
        points={{191,-58},{186,-58},{186,-58},{184,-58},{184,-74},{180,-74}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(pumHotDeckOff.y, swiPum.u3) annotation (Line(
        points={{191,-90},{184,-90},{180,-90}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(conPum.u_s, swiPum.y) annotation (Line(
        points={{148,-82},{157,-82}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(valHea.y_actual, hysPum.u) annotation (Line(
        points={{133,-29},{252.5,-29},{252.5,-70},{238,-70}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(conPum.y, pumHotDeck.y) annotation (Line(
        points={{125,-82},{112,-82},{112,-52},{73.8,-52}},
        color={0,0,127},
        pattern=LinePattern.Dash));
    connect(heaCoi.port_b2, port_b)
      annotation (Line(points={{108,-22},{100,-22},{100,0}}, color={0,127,255}));
    connect(pumHotDeck.port_b, jun.port_1)
      annotation (Line(points={{84,-64},{130,-64}}, color={0,127,255}));
    connect(valHea.port_a, jun.port_3)
      annotation (Line(points={{140,-44},{140,-54}}, color={0,127,255}));
   annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-180,
              -120},{400,220}}),
                           graphics={
          Rectangle(
            extent={{-180,220},{400,-120}},
            lineColor={0,0,127},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-170,218},{-116,158}},
            lineColor={255,220,220},
            lineThickness=1,
            fillPattern=FillPattern.Sphere,
            fillColor={255,255,0}),
          Line(
            points={{-144,158},{-144,148},{-144,92},{-80,92}},
            color={0,0,127},
            thickness=0.5),
          Rectangle(
            extent={{-80,102},{-50,60}},
            lineColor={0,0,127},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{-78,92},{-52,68}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-52,68},{2,68}},
            color={255,170,85},
            thickness=0.5),
          Rectangle(
            extent={{2,72},{24,48}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{2,50},{22,70},{24,72}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{6,68},{10,68}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{8,66},{8,70}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{24,68},{84,68}},
            color={255,85,85},
            thickness=0.5),
          Ellipse(
            extent={{84,80},{108,58}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{88,78},{88,62},{108,70},{88,78}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{108,68},{272,68}},
            color={255,85,85},
            thickness=0.5),
          Rectangle(
            extent={{272,88},{294,64}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{272,64},{292,84},{294,86}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{276,82},{276,86}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{274,84},{278,84}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{294,70},{324,70},{324,22}},
            color={255,0,0},
            thickness=0.5),
          Rectangle(
            extent={{-11,12},{11,-12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid,
            origin={333,12},
            rotation=90),
          Line(
            points={{324,18},{330,12},{326,8},{332,4}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{334,20},{340,14},{336,10},{342,6}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{324,0},{324,-102},{390,-102}},
            color={255,0,0},
            thickness=0.5),
          Line(
            points={{390,130},{112,130}},
            color={255,0,0},
            thickness=0.5),
          Ellipse(
            extent={{88,140},{112,118}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{108,138},{108,122},{88,130},{108,138}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{88,130},{-24,130},{-50,96}},
            color={255,0,0},
            thickness=0.5),
          Line(
            points={{-50,96},{-80,68}},
            color={255,170,85},
            thickness=0.5),
          Line(
            points={{-80,70},{-156,70},{-156,162}},
            color={255,170,85},
            thickness=0.5),
          Rectangle(
            extent={{260,104},{364,-14}},
            lineColor={0,0,0},
            lineThickness=0.5,
            pattern=LinePattern.Dash),
          Text(
            extent={{260,78},{366,150}},
            lineColor={0,0,0},
            pattern=LinePattern.Dash,
            lineThickness=0.5,
            textString="VAVBranch * nReheater"),
          Line(
            points={{-90,0},{8,0},{8,48},{6,48}},
            color={255,0,0},
            thickness=1),
          Line(
            points={{8,36},{268,36},{276,36},{276,64}},
            color={255,0,0},
            thickness=1),
          Line(
            points={{92,0},{18,0},{18,48}},
            color={0,0,255},
            thickness=1),
          Line(
            points={{18,32},{286,32},{286,64}},
            color={0,0,255},
            thickness=1),
          Rectangle(
            extent={{208,196},{246,156}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{260,202},{260,180},{246,180}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{208,176},{182,176},{182,2},{322,2}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{182,80},{274,80}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Rectangle(
            extent={{-12,-28},{26,-68}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-36,12},{-36,-12},{-20,0},{-36,12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-4,12},{-4,-12},{-20,0},{-4,12}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={175,175,175},
            fillPattern=FillPattern.Solid),
          Line(
            points={{30,68},{30,-40},{26,-40}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{-12,-50},{-24,-50},{-24,-4}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Rectangle(
            extent={{128,114},{154,90}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(
            points={{142,130},{142,114},{142,114}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{142,90},{142,70}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{128,102},{100,102},{100,116}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Line(
            points={{100,80},{100,102}},
            color={0,0,0},
            pattern=LinePattern.Dash),
          Text(
            extent={{266,214},{324,176}},
            lineColor={0,0,0},
            pattern=LinePattern.Dash,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid,
            textString="TRoom"),
          Rectangle(
            extent={{38,72},{60,48}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={135,135,135},
            fillPattern=FillPattern.Solid),
          Line(
            points={{38,48},{58,70},{60,72}},
            color={0,0,0},
            thickness=0.5),
          Line(
            points={{40,68},{44,68}},
            color={0,0,0},
            thickness=0.5),
          Ellipse(
            extent={{-80,12},{-56,-10}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-76,8},{-76,-8},{-56,0},{-76,8}},
            lineColor={0,0,0},
            lineThickness=0.5,
            fillColor={85,170,255},
            fillPattern=FillPattern.Solid)}),
                            Diagram(coordinateSystem(preserveAspectRatio=false,
            extent={{-180,-120},{400,220}})),Documentation(info="<html>
          <p>
          CTA based on the model <a href=\"modelica://Buildings.Examples.VAVReheat.ClosedLoop\">VAVReheat</a> of the Buldings library
          </p>
          <p><h4>Model use</h4></p>
          <p><ol>
          <p>Firstly, give to each zone, wich is deserved by the CTA, a number and keep it in mind for the connection to the multi-input/output </p>
<li>Set the parameter <i>nReheat</i> egal to the number of zone deserved by an individual re-heater.</li>
<li>Connect <i>TRoo</i>.  </li>
<li>Connect <i>port_a</i> to the  hot water source and <i>port_b</i> to the hot water sink. </li>
<li> Connect <i>port_Supply</i> and <i>port_Return</i> to the air volume of each zone
<li> Set other parameters or let there to the default value <b> Except for VRoom and m_flow_room</b> wich need to be fullfill</li>

</ol></p>
</html>"));
  end CTA_VAVReheat_Hydro;
end AHUSystems;
