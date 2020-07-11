/*
    Copyright 2020 Google LLC

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        https://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
package AeroToolKit;

/**
 * A Boundary Layer Probe requires several features
 *   1) A Derived Part comprised of an Arbitrary Plane from a Geometry Part
 *   2) A Report of the local wall shear stress 
 *   3) A field function that will provide the analytic law of the wall plot to compare against
 *   4) An XY Plot for the Law of the Wall in a log-log arrangement
 *   5) It needs to relax itself to the surface on a new mesh
 */

//
import java.util.*;

//
import star.common.*;
import star.base.neo.*;

//
import Naming.*;
import Tools.DerivedPartTool;
import Tools.ReportTool;
import Tools.PlotTool;
import Tools.SimTool;

import star.base.report.MaxReport;
import star.base.report.MinReport;

import star.vis.*;

public class BoundaryLayerProbe {
  // Names
  NamingConvention globalNames;
  String probeName;

  // Debugging verbosity.
  int _debug_verbosity = 0;

  // Geometry Parts.
  GeometryPart geometryPart;
  Part probePart;
  Part probePoint;

  // Near-wall reporta
  MinReport distanceFromMeshSurface;
  MaxReport localDensityReport;
  MaxReport lotwTauReport;

  // Law of the Wall info
  UserFieldFunction yPlusFF; // field function that automatically tracks y+
  UserFieldFunction localYFF; // field function that separates out the csys local Y values
  UserFieldFunction uPlusFF; // field function for U+
  UserFieldFunction lotwUPlusFF; // law of the Wall Field Function to show analytic solution for u+
  double refRho=1.0;
  double refNu=1.0E-5;
  double uTau=1.0;
  double yPlusNorm=1.0;
  double uPlusNorm=1.0;
  double kappa=0.41;
  double cPlus=5.0;

  // Plotting
  InternalDataSet solutionDataSet;
  InternalDataSet analyticDataSet;
  InternalDataSet prismLayerDataSet;

  // Coordinate system 
  String bodyCsysName;
  CartesianCoordinateSystem bodyCsys;
  CartesianCoordinateSystem labCsys;
  CartesianCoordinateSystem probeCADCsys;
  CartesianCoordinateSystem probeCsys;
  String probeCsysName;

  Representation proxyRep;

  // Automatic post-processing
  XYPlot lotwPlot;
  String subLayerStatus = "FAIL";
  String bufferLayerStatus = "FAIL";
  String logLayerStatus = "FAIL";
  String meshStatus = "FAIL";

  public String getName(){
    return probeName;
  }

  public Part getProbePart(){
    return probePart;
  }

  public BoundaryLayerProbe(Simulation tmpSim, GeometryPart inputPart,
          CoordinateSystem parentCsys){
    // Initialize Global Names
    this.globalNames = new NamingConvention();
    proxyRep    = tmpSim.getRepresentationManager()
                  .getObject(globalNames.getProxyName());

    // gather Boundary Layer Probe Parts
    this.geometryPart = inputPart;
    this.probeName = inputPart.getPresentationName();

    // Coordinate System Stuff
    // gather Coordinate System w/ the name & make sure coordinate system will
    // rotate w/ part.
    this.labCsys = (CartesianCoordinateSystem) parentCsys;
    this.bodyCsysName  = globalNames.getBodyCsysName();
    this.bodyCsys = (CartesianCoordinateSystem) this.labCsys
                     .getLocalCoordinateSystemManager().getObject(bodyCsysName);
    this.probeCADCsys = SimTool.getNestedCoordinate(this.labCsys, this.probeName);
    this.probeCsys = SimTool.equalLevelCoordToNestedCoord(tmpSim, this.probeCADCsys,
        this.bodyCsys);
    this.probeCsysName = probeCsys.getPresentationName();

    // Make the associated arbitrary plane part.
    try{
        probePart = (ArbitrarySection) tmpSim.getPartManager().getObject(
            this.geometryPart.getPresentationName());
    }catch(NeoException e){
        probePart = DerivedPartTool.getPartLinkedArbitrarySection(tmpSim,
            this.geometryPart);
    }

    // Make the associated arbitrary plane point needed in 14.06.
    try{
        probePoint = (PointPart) tmpSim.getPartManager().getObject(
                probeName + " Point");
    }catch(NeoException e){
        probePoint = DerivedPartTool.makePointPart(tmpSim, probeName + " Point",
            this.probeCsys, new double[] {0.0, 0.0, 0.0});
    }

    // Make sure the correct field functions and parameters are in place.
    PrimitiveFieldFunction pFF = 
      ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("WallShearStress"));
    VectorComponentFieldFunction wallShearInProbeCsysX = 
      ((VectorComponentFieldFunction) pFF.getFunctionInCoordinateSystem(probeCsys).getComponentFunction(0));
    lotwTauReport = ReportTool.maxReport(tmpSim,"Tau-" + probeName, wallShearInProbeCsysX,
          Collections.singleton(this.probePoint), proxyRep);

    // *Must* use the interpolation option in order to get a non-zero value.
    lotwTauReport.setSmooth(true);

    // Make report to allow coordinate system to slide to the wall.
    PrimitiveFieldFunction pFF_1 = 
      ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("Position"));
    VectorComponentFieldFunction cSysDistanceToWall = 
      ((VectorComponentFieldFunction) pFF_1.getFunctionInCoordinateSystem(probeCsys).getComponentFunction(1));
    distanceFromMeshSurface = ReportTool.minReport(tmpSim,"minY"+probeName, cSysDistanceToWall,
          Collections.singleton(probePart), proxyRep);
    distanceFromMeshSurface.setSmooth(true);

    // Make required FF
    updateLocalYFF(tmpSim);
    updateYPlusFF(tmpSim);
    updateLOTWFF(tmpSim);
    updateUPlusFF(tmpSim);

    // Create the XY Plot
    setupLOTWPlot(tmpSim);

  }
  public void setProbeRegions(ArrayList<Region> inputRegions){
    probePart.getInputParts().setObjects(inputRegions);
  }
  public void setProbeBoundaries(ArrayList<Boundary> inputBoundaries){
      this.probePoint.getInputParts().setObjects(inputBoundaries);
  }
  
  public GeometryPart getGeometryPart(){
    return geometryPart;
  }
  public void setBodyCsys(CartesianCoordinateSystem newBodyCsys){
  this.bodyCsys = newBodyCsys;
  }
  public CartesianCoordinateSystem getBodyCsys(){
    return this.bodyCsys;
  }

  // Probe Values of Interest
  public void updateBLPValues(Simulation tmpSim,double newRho,double newNu){
    //flow conditions
    this.refRho=newRho;
    this.refNu=newNu;
    
    //constants
    getKappaParameter(tmpSim);
    getCPlusParameter(tmpSim);
    
    //calculations
    updateUTau(tmpSim);
    updateYPlusNormalization();
    updateUPlusNormalization();
    
    //field functions
    updateUPlusFF(tmpSim);
    
    // check for Volume mesh and make sure prism layer distribution is plotted
    try{
      Representation volMesh = tmpSim.getRepresentationManager().getObject("Volume Mesh");
      updatePrismLOTWData(tmpSim);
    }catch(NeoException e){
      
    }

  }
  private void updateYPlusNormalization(){
    yPlusNorm = uTau / refNu;
  }
  private void updateUPlusNormalization(){
    uPlusNorm = 1. * uTau;
  }
  private void updateUTau(Simulation tmpSim){
    double fakeLowTau=1.0;
    try{
      Representation volMesh = tmpSim.getRepresentationManager().getObject("Volume Mesh");  
      if(proxyRep.usesRep(volMesh)){
        uTau=Math.sqrt(lotwTauReport.getReportMonitorValue() / refRho);
      }else{ //need to fudge it because we are not using volume mesh therefore no data on report
        uTau=Math.sqrt(fakeLowTau / refRho);
      }
    }catch(NeoException e){//not even a volume mesh available
      uTau=Math.sqrt(fakeLowTau / refRho);
    }
  }
  private void updateLocalYFF(Simulation tmpSim){
    localYFF = SimTool.getUserFF(tmpSim,"localY-"+probeName);
    localYFF.setFunctionName("localy-"+probeName);
    localYFF.setDefinition("$$Position(@CoordinateSystem(\""+this.labCsys.getPresentationName()+
            "."+this.bodyCsysName+"."+probeName+"\"))[1]");
  }
  private void updateLOTWFF(Simulation tmpSim){
    String yplusFFName="${"+yPlusFF.getFunctionName()+"}";
    lotwUPlusFF = SimTool.getUserFF(tmpSim,"lotw-"+probeName);
    //abs is here to avoid negative value errors in position since y=0 in the probe Csys at the surface
    String uPlusFar = "1.0 / ("+kappa+" + 1e-20) * log("+yplusFFName+" + 1e-20)+"+cPlus;
    lotwUPlusFF.setDefinition("("+yplusFFName+">=30)?"+uPlusFar+":("+yplusFFName+"<=5) ? "+yplusFFName+":0");
  }
  private void updateUPlusFF(Simulation tmpSim){
    uPlusFF = SimTool.getUserFF(tmpSim,"uplus-"+probeName);
    // no negative sqrts, no zero rhos
    String uTauStr="sqrt(abs(${"+lotwTauReport.getPresentationName()+"Report})/alternateValue("+refRho+",1.0))";
    // no dividing by zero
    uPlusFF.setDefinition("$$Velocity(@CoordinateSystem(\""+this.labCsys.getPresentationName()+
            "."+this.bodyCsysName+"."+probeName+"\"))[0]/("+uTauStr+"+1e-20)");
  }
  private void updateYPlusFF(Simulation tmpSim){
    yPlusFF = SimTool.getUserFF(tmpSim,"yplus-"+probeName);
    yPlusFF.setFunctionName("yplus-"+probeName);
    //abs is here to avoid negative value errors in position since y=0 in the probe Csys at the surface
    String uTauStr="sqrt(abs(${"+lotwTauReport.getPresentationName()+"Report})/alternateValue("+refRho+",1.0))";
    // we only want positive y-values
    String localY="${"+localYFF.getFunctionName()+"}";
    String yPlus = localY+"*"+uTauStr+"/alternateValue("+refNu+",1.0E-5)";
    yPlusFF.setDefinition(""+yPlus+"+1e-20");
  }

  // Probe relaxing to mesh surface
  public void rotateBLPCADPart(Simulation tmpSim, CoordinateSystem thisCsys, double[] thisVector, double byAngle){
    /* rotateCADPart rotates the CAD part as a no-history operation
    Note: This is a blind operation, meaning there is no tracking of the original part in the simulation history!
    This is because the arbitrary section does not operate on anything except the root/CAD representation.
    */
    Units units_0 = SimTool.getUnitsVec(tmpSim);
    if(Math.abs(byAngle)>1e-10){
      tmpSim.get(SimulationPartManager.class)
        .rotateParts(new NeoObjectVector(new Object[] {geometryPart}),
          new DoubleVector(new double[] {thisVector[0], thisVector[1], thisVector[2]}),
          new NeoObjectVector(new Object[] {units_0, units_0, units_0}),
          byAngle*Math.PI/180.0,thisCsys);      
    }
  }
  public void relaxToMeshSurface(Simulation tmpSim){
    // Allows the coordinate system in the Body Csys to adjust itself to the mesh surface
    double relaxCsysDist = distanceFromMeshSurface.getReportMonitorValue();
    if(_debug_verbosity > 0){
      tmpSim.println("BLP " + this.probeName + ", relax to surface in local y: " + relaxCsysDist);
    }
    Units units_0 = ((Units) tmpSim.getUnitsManager().getObject("m")); // always m kg s
    if(Math.abs(relaxCsysDist)>1e-20){
      probeCsys.getLocalCoordinateSystemManager()
      .translateLocalCoordinateSystems(new NeoObjectVector(new Object[] {probeCsys}),
        new DoubleVector(new double[] {0.0, relaxCsysDist, 0.0}),
        new NeoObjectVector(new Object[] {units_0, units_0, units_0}), probeCsys);
    }
  }

  // Law of the Wall Plots
  public XYPlot getLOTWPlot(){
    return lotwPlot;
  }
  private void updatePrismLOTWData(Simulation tmpSim){
    // Bottom axis is for y+ but uses the y-value of the probes
    //   scaling to create y+ is done on the individual data set level
    
    //Check the Y-axis setup
    Collection<AxisType> currentYAxes = lotwPlot.getYAxes().getObjects();
    boolean isPrismLayerLineSetup = false;
    //
    for(AxisType tmpYAxis:currentYAxes){
      FieldFunction dataFF = ((YAxisType) tmpYAxis).getScalarFunction().getFieldFunction();
      String dataFFName = dataFF.getPresentationName();
      if(dataFFName.equals("Prism Layer Cells")){ // this is the analytic solution for u+
        isPrismLayerLineSetup=true;
      }
    }
    // Next Y Type should always be the analytic output
    if(!isPrismLayerLineSetup){
      YAxisType yAxisType_2 = 
        lotwPlot.getYAxes().createAxisType();
      FieldFunctionUnits fieldFunctionUnits_2 = 
        yAxisType_2.getScalarFunction();
      PrimitiveFieldFunction primitiveFieldFunction_0 = 
      ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().getFunction("PrismLayerCells"));
      fieldFunctionUnits_2.setFieldFunction(primitiveFieldFunction_0);
      // the field function is created using a direct function of y-position so you still need to scale the y+ values
      prismLayerDataSet = 
      ((InternalDataSet) yAxisType_2.getDataSetManager().getDataSet(probePart.getPresentationName()));

      // the field function is solved for u+ therefore no scaling is required
      LineStyle lineStyle_0 = 
        prismLayerDataSet.getLineStyle();
      lineStyle_0.getLinePatternOption().setSelected(LinePatternOption.Type.SOLID);
      lineStyle_0.setColor(new DoubleVector(PlotTool.getColor("red")));
      lineStyle_0.setWidth(4);

      // black squares, size 10, legend name to LoTW
      SymbolStyle solutionSymbol = prismLayerDataSet.getSymbolStyle();
      solutionSymbol.setColor(new DoubleVector(PlotTool.getColor("red")));
      solutionSymbol.getSymbolShapeOption().setSelected(SymbolShapeOption.Type.VERTICAL_LINE);
      solutionSymbol.setSize(16);
      solutionSymbol.setStrokeWidth(4.0);
      prismLayerDataSet.getDataSetStyleOption().setSelected(DataSetStyleOption.Type.SYMBOL_LINE);
      prismLayerDataSet.setSeriesNameLocked(true);
      prismLayerDataSet.setSeriesName("Prisms");
      prismLayerDataSet.setNeedsSorting(true);
    }

  }
  private void setupLOTWPlot(Simulation tmpSim){
    double minYPlus = 1.0e-1;
    double maxYPlus = 1.0e3;
    // create
    lotwPlot = PlotTool.getXYPlot(tmpSim,probeName,probeName);
    lotwPlot.setRepresentation(proxyRep);
    
    //assign the arbitrary part probe surface
    lotwPlot.getParts().setObjects(probePart);

    // Bottom axis is for y+ but uses the y-value of the probes
    //   scaling to create y+ is done on the individual data set level
    PlotTool.setXYPlotXVectorScalar(tmpSim, lotwPlot,probeCsys,"Position",1);
    // direct scalar
    AxisType axisType_0 = 
      lotwPlot.getXAxisType();
    FieldFunctionUnits fieldFunctionUnits_X = 
      axisType_0.getScalarFunction();
    fieldFunctionUnits_X.setFieldFunction(yPlusFF);
    
    //Check the Y-axis setup
    Collection<AxisType> currentYAxes = lotwPlot.getYAxes().getObjects();
    boolean isAnalyticSetUp = false;
    boolean isProbeUPlusSetUp = false;
    boolean isPrismLayerLineSetup = false;
    //
    for(AxisType tmpYAxis:currentYAxes){
      FieldFunction dataFF = ((YAxisType) tmpYAxis).getScalarFunction().getFieldFunction();
      String dataFFName = dataFF.getPresentationName();
      if(dataFFName.equals("lotw-"+probeName)){ // this is the analytic solution for u+
        isAnalyticSetUp=true;
        //presume we do not modify
        analyticDataSet = ((InternalDataSet) ((YAxisType) tmpYAxis).getDataSetManager()
          .getDataSet(probePart.getPresentationName()));
        analyticDataSet.setNeedsSorting(true);
      }else if(dataFFName.equals("Prisms")){ // this is the analytic solution for u+
        isPrismLayerLineSetup=true;

      }else if(dataFF instanceof VectorComponentFieldFunction){ //this is likely the u+ velocity in the probe Csys
        isProbeUPlusSetUp = true;
        //presume we do not modify but track data set
        solutionDataSet = ((InternalDataSet) ((YAxisType) tmpYAxis).getDataSetManager()
          .getDataSet(probePart.getPresentationName()));
        solutionDataSet.setNeedsSorting(true);
      }
    }
    // If neither are set up, we must add a Y-Type and set it up
    // Y Type 1 should *always* be the solution field output
    if(!isProbeUPlusSetUp){
      YAxisType yAxisType_0 = 
      ((YAxisType) lotwPlot.getYAxes().getAxisType("Y Type 1"));
      
      //Set Field Function
      FieldFunctionUnits fieldFunctionUnits_0 = 
        yAxisType_0.getScalarFunction();
      fieldFunctionUnits_0.setFieldFunction(uPlusFF);

      // need to scale the y-position to y+ by the y+ normalization
      solutionDataSet = 
      ((InternalDataSet) yAxisType_0.getDataSetManager().getDataSet(probePart.getPresentationName()));
      SymbolStyle solutionSymbol = solutionDataSet.getSymbolStyle();
      solutionSymbol.setColor(new DoubleVector(PlotTool.getColor("blue")));
      solutionSymbol.getSymbolShapeOption().setSelected(SymbolShapeOption.Type.FILLED_CIRCLE);
      solutionSymbol.setSize(12);
      solutionDataSet.setSeriesNameLocked(true);
      solutionDataSet.setSeriesName("u+ Probe");
      solutionDataSet.setNeedsSorting(true);
    }
    
    // Next Y Type should always be the analytic output
    if(!isAnalyticSetUp){
      YAxisType yAxisType_1 = 
        lotwPlot.getYAxes().createAxisType();
      FieldFunctionUnits fieldFunctionUnits_2 = 
        yAxisType_1.getScalarFunction();
      fieldFunctionUnits_2.setFieldFunction(lotwUPlusFF);
      // the field function is created using a direct function of y-position so you still need to scale the y+ values
      analyticDataSet = 
      ((InternalDataSet) yAxisType_1.getDataSetManager().getDataSet(probePart.getPresentationName()));
      //analyticDataSet.setXScale(yPlusNorm);
      // the field function is solved for u+ therefore no scaling is required
      // black squares, size 10, legend name to LoTW
      SymbolStyle solutionSymbol = analyticDataSet.getSymbolStyle();
      solutionSymbol.setColor(new DoubleVector(PlotTool.getColor("black")));
      solutionSymbol.getSymbolShapeOption().setSelected(SymbolShapeOption.Type.FILLED_SQUARE);
      solutionSymbol.setSize(10);
      analyticDataSet.setSeriesNameLocked(true);
      analyticDataSet.setSeriesName("u+ LoTW");
      analyticDataSet.setNeedsSorting(true);
    }
    
    //must sort these to make use of auto-post-processing
    solutionDataSet.setNeedsSorting(true);
    analyticDataSet.setNeedsSorting(true);

    // Adjust axes
    // Bottom Axis
    PlotTool.setPlotXAxis(lotwPlot, "y+", minYPlus, maxYPlus, 1.0, 1);
    PlotTool.setPlotXToLog(lotwPlot,true);
    Cartesian2DAxisManager cartesian2DAxisManager_0 = 
      ((Cartesian2DAxisManager) lotwPlot.getAxisManager());
    Cartesian2DAxis cartesian2DAxis_0 = 
      ((Cartesian2DAxis) cartesian2DAxisManager_0.getAxis("Bottom Axis"));
    AxisLabels axisLabels_0 = 
      cartesian2DAxis_0.getLabels();
    axisLabels_0.setDecimalsPadded(true);
    axisLabels_0.setPrecision(1);
    AxisTicks axisTicks_0 = 
      cartesian2DAxis_0.getTicks();
    axisTicks_0.setCount(9);
    axisTicks_0.setGridWidth(4);
    axisTicks_0.setGridColor(new IntVector(new int[] {211, 211, 211, 255}));

    // Left Axis
    PlotTool.setPlotYAxis(lotwPlot, "u+", minYPlus, 100.0, 1.0, 1);
    PlotTool.setPlotYToLog(lotwPlot,true);
    Cartesian2DAxis cartesian2DAxis_1 = 
      ((Cartesian2DAxis) cartesian2DAxisManager_0.getAxis("Left Axis"));
    AxisLabels axisLabels_1 = 
      cartesian2DAxis_1.getLabels();
    axisLabels_1.setDecimalsPadded(true);
    axisLabels_1.setPrecision(1);
    AxisTicks axisTicks_1 = 
      cartesian2DAxis_1.getTicks();
    axisTicks_1.setGridColor(new IntVector(new int[] {211, 211, 211, 255}));
    axisTicks_1.setGridWidth(4);
    axisTicks_1.setCount(9);

    // Put legend on the outside NE
    MultiColLegend multiColLegend_0 = 
      lotwPlot.getLegend();
    multiColLegend_0.getChartPositionOption().setSelected(ChartPositionOption.Type.NORTH_EAST);
    multiColLegend_0.setFont(new java.awt.Font("Lucida Sans", 0, 24));

    this.lotwPlot.close();
  }
  public String getResolutionStatus(){
    /* Method returns the following:
        If any status returns "FAIL", status = FAIL
        If subLayer and BufferLayer="HIGH Y+" and LogLayer=PASS, status = HIGH Y+
    */
    meshStatus = "FAIL";
    
    if(subLayerStatus.equals("FAIL")||bufferLayerStatus.equals("FAIL")||logLayerStatus.equals("FAIL")){
      meshStatus = "FAIL";
    }else if(subLayerStatus.equals("HIGH Y+")&&bufferLayerStatus.equals("HIGH Y+")&&logLayerStatus.equals("PASS")){
      meshStatus = "HIGH Y+";
    }else if(subLayerStatus.equals("PASS")&&bufferLayerStatus.equals("PASS")&&logLayerStatus.equals("PASS")){
      meshStatus = "PASS";
    }else if(subLayerStatus.equals("MARGINAL")&&bufferLayerStatus.equals("MARGINAL")){
      meshStatus = "MARGINAL";
    }
    return meshStatus;
  }
  public String prismLayerResolutionToCSV(Simulation tmpSim){
    /* Method to automatically calculate how many cells are in each layer
       under the law of the wall assumption. This will track prism layers only.
       String format:
        BLP Name, First y+ value, Highest Prism Y+, NCells in SubLayer, NCells in BufferLayer, NCells in LogLayer
    */
    String retStr="";
    double[] yPlusValues = analyticDataSet.getXDataArray();
    double[] prismValues = prismLayerDataSet.getYDataArray();

    // figure out distribution of mesh in boundary layer
    int nSubLayer=0;
    int nBufferLayer=0;
    int nLogLayer=0;
    
    // get highest Prism Layer y+ value
    int highestPrismIndx = 0;
    for(int i=0;i<prismValues.length;i++){
      if(prismValues[i]>1e-6){
        highestPrismIndx++;
      }
    }
    double highestYPlusPrism=yPlusValues[highestPrismIndx-1];
    
    //assign boundaries to the 
    for(int i=0;i<=highestPrismIndx-1;i++){
      if(yPlusValues[i]<5){
        nSubLayer++;
      }else if(yPlusValues[i]>30){
        nLogLayer++;
      }else if(yPlusValues[i]>=5 && yPlusValues[i]<=30){
        nBufferLayer++;
      }
    }

    //Write out status
    if(nSubLayer==0 && yPlusValues[0]>30){ //a high y+ situation
      subLayerStatus = "HIGH Y+";
    }else if(nSubLayer==0){
      subLayerStatus = "FAIL";
    }else if(nSubLayer<2){
      subLayerStatus = "MARGINAL";
    }else if(nSubLayer>=4){
      subLayerStatus = "PASS";
    }else{
      subLayerStatus = "FAIL";
    }
    
    if(nBufferLayer==0 && yPlusValues[0]>30){ //high y+ situation
      bufferLayerStatus = "HIGH Y+";
    }else if(nBufferLayer<3){
      bufferLayerStatus = "MARGINAL";
    }else if(nBufferLayer>=5){
      bufferLayerStatus = "PASS";
    }else{
      bufferLayerStatus = "FAIL";
    }

    if(nLogLayer>=4){
      logLayerStatus = "PASS";
    }else if(nLogLayer>=2){
      logLayerStatus = "MARGINAL";
    }else{
      logLayerStatus = "FAIL";
    }

    retStr=probeName+","+yPlusValues[0]+","+highestYPlusPrism
      + "," +nSubLayer + "," + subLayerStatus
      + "," +nBufferLayer + "," + bufferLayerStatus
      + "," +nLogLayer + "," + logLayerStatus;
    return retStr;
  }

  // Law of the Wall Constants
  private void getKappaParameter(Simulation tmpSim){
    /* getKappaParameter gets the Von Karman Constant parameter from the
      simulation for use in the Law of the Wall analytic calculations
    */
    ScalarGlobalParameter kappaParam = SimTool.getScalarSimulationParameter(tmpSim,"VonKarmanConstant");
    // check if it is not initialized
    if(Math.abs(kappaParam.getQuantity().getInternalValue())<1e-5){
      tmpSim.println("BLP MSG: " + probeName + " changing zero Von Karman Constant to 0.41");
      kappaParam.getQuantity().setValue(0.41);
    }
    kappa = kappaParam.getQuantity().getInternalValue();
  }
  private void getCPlusParameter(Simulation tmpSim){
    /* getKappaParameter gets the C+ parameter from the simulation
    for use in the Law of the Wall analytic calculations
    */
    ScalarGlobalParameter cPlusParam = SimTool.getScalarSimulationParameter(tmpSim,"lotwConstant");
    // check if it is not initialized
    if(Math.abs(cPlusParam.getQuantity().getInternalValue())<1e-5){
      tmpSim.println("BLP MSG: "+probeName+" changing zero Law of The Wall Constant to 5.0");
      cPlusParam.getQuantity().setValue(5.0);
    }
    cPlus = cPlusParam.getQuantity().getInternalValue();
  }

}
