float rest_length = -105.8; // equilibrium flex sensor value, converted to [mm]
float df_convert = -0.2; // displacement/flex value conversion, [mm]
float ls_convert = 137.9; // (half) layers/stiffness conversion, [mm/N]
float firstlayer_k = 0.07; // stiffness of layer 1, [N/mm]

// Conversions -------------------------------

// convert flexsensor values to displacements
float flex2disp(int flex) {
  float disp = flex * df_convert;
  disp -= rest_length;
  return disp;
}

// convert displacements to flexsensor values (reverse)
int disp2flex(float disp) {
  int flex = int((disp + rest_length)/df_convert);
  return flex;
}

// convert spring stiffness to number of layers
float stif2layer(float k) {
  float doublelayers = round(k * ls_convert);
  float layers = doublelayers / 2.;
  return layers;
}

// convert number of layers to  spring stiffness (reverse)
float layer2stif(float layers) {
  float k = layers *2. / ls_convert;
  return k;
}