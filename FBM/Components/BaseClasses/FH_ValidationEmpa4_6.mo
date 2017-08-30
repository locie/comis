within FBM.Components.BaseClasses;
record FH_ValidationEmpa4_6 "According to Koschenz, 2000, par 4.6"
  extends FBM.Components.BaseClasses.RadiantSlabChar(
    S_1=0.1,
    S_2=0.2,
    T=0.20,
    d_a=0.025,
    s_r=0.0025,
    n1=10,
    n2=10,
    lambda_r=0.45);
  // A_Floor should be 120m * 0.2m = 24 m�
end FH_ValidationEmpa4_6;
