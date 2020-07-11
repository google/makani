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
package Tools;

import java.util.*;
import star.common.*;
import star.base.neo.*;
import star.base.report.*;

public class PlotTool {

  // Line Style Options:
  //
  //   None, Solid, Dot, Dash, Dash Dot, Dash Dot Dot
  //
  // Symbol Style Options:
  //
  //  None, Filled Square, Empty Square, Filled Circle, Empty Circle,
  //    Filled Triangle, Empty Triangle, Cross, Plus, Star, Filled Diamond,
  //    Horizontal Line, Vertical Line
  // Header

  // COLOR PALLETTES
  static double[] black   = {0.0, 0.0, 0.0};
  // nice colors
  static double[] red     = {1.0,  0.0,  0.0};
  static double[] orange  = {1.0,0.6,  0.0};
  static double[] yellow  = {1.0,1.0,  0.0};
  static double[] darkgrn = {  0.0, 0.392156863,  0.0};
  static double[] blue    = {  0.0,  0.0,1.0};
  static double[] drkred  = {0.69803921568,  0.0,  0.0};
  static double[] cyan    = {  0.0,1.0,1.0};
  static double[] teal    = {0.0, 0.8, 0.8};
  static double[] magenta = {1.0,  0.0,1.0};
  static double[] purple  = {0.8,  0.0, 0.8};
  static double[] brightgreen = {0.0, 1.0, 0.2862745225429535};
  // plot background colors only
  static double[] gainsboro = {0.8626999855041504, 0.8626999855041504, 0.8626999855041504};
  static double[] slategray = {0.4392000138759613, 0.5019999742507935, 0.5647000074386597};

  // Get Plot Colors
  public static double[] getColor(String color){
    switch (color) {
      case "black":
        return black;
      case "red":
        return red;
      case "orange":
        return orange;
      case "yellow":
        return yellow;
      case "darkgrn":
        return darkgrn;
      case "blue":
        return blue;
      case "drkred":
        return drkred;
      case "cyan":
        return cyan;
      case "teal":
        return teal;
      case "magenta":
        return magenta;
      case "purple":
        return purple;
      case "bright green":
        return brightgreen;
      case "gainsboro":
        return gainsboro;
      case "slate gray":
        return slategray;
      default:
          return new double[] {0.,0.,0.};
    }
  }
  public static double[] forLoopColor(int loopNum){
    ArrayList<double[]> loopColorArray = new ArrayList(
      Arrays.asList(red, orange, yellow, darkgrn, blue, drkred,
              cyan, teal, magenta, purple, brightgreen));
    return loopColorArray.get(loopNum%loopColorArray.size());
  }

  // Monitor Plots
  public static MonitorPlot adjustResidualPlot(Simulation tmpSim,boolean is2D){
    MonitorPlot tmpResPlot;
    String tmpName;
    tmpResPlot = getResidualPlot(tmpSim,"Residuals","Absolute Residuals");
    tmpResPlot.getAxisManager().getObject("Left Axis").setLogarithmic(true);
    tmpName = "Continuity";
    setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Solid",4, red);
    setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"None",2,1,1,red);
    tmpName = "X-momentum";
    setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Solid",4, blue);
    setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"None",2,1,1,blue);
    tmpName = "Y-momentum";
    setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Solid",4, orange);
    setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"None",2,1,1,orange);
    if(!is2D){
      tmpName = "Z-momentum";
      setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
              getDataSet(tmpName),"Solid",4, darkgrn);
      setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
              getDataSet(tmpName),"None",2,1,1,darkgrn);
    }

    tmpName = "Energy";
    setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Solid",4, drkred);
    setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"None",2,1,1,drkred);
    tmpName = "Sdr";

    setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Solid",4, cyan);
    setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Filled Square",10,2,2,cyan);
    tmpName = "Tke";
    setDataLines((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"Solid",4, magenta);
    setDataSymbols((MonitorDataSet) tmpResPlot.getDataSetManager().
            getDataSet(tmpName),"None",2,1,1,magenta);
    // Axes text sizes
    Cartesian2DAxisManager cartesian2DAxisManager_0 = 
      ((Cartesian2DAxisManager) tmpResPlot.getAxisManager());
    Cartesian2DAxis cart2DAxis = 
      ((Cartesian2DAxis) cartesian2DAxisManager_0.getAxis("Bottom Axis"));
    AxisTitle axisTitle_0 = 
      cart2DAxis.getTitle();
    axisTitle_0.setFont(new java.awt.Font("Aharoni", 0, 36));
    AxisLabels axisLabels_0 = 
      cart2DAxis.getLabels();
    axisLabels_0.setFont(new java.awt.Font("Lucida Sans", 0, 24));
    // Left vertical axis
    Cartesian2DAxis cartesian2DAxis_1 = 
      ((Cartesian2DAxis) cartesian2DAxisManager_0.getAxis("Left Axis"));
    AxisTitle axisTitle_1 = 
      cartesian2DAxis_1.getTitle();
    axisTitle_1.setFont(new java.awt.Font("Aharoni", 0, 36));
    AxisLabels axisLabels_1 = 
      cartesian2DAxis_1.getLabels();
    axisLabels_1.setFont(new java.awt.Font("Lucida Sans", 0, 24));    
    // Legend position

    MultiColLegend multiColLegend_0 = 
      tmpResPlot.getLegend();
    multiColLegend_0.getChartPositionOption().setSelected(
            ChartPositionOption.Type.NORTH_EAST);
    multiColLegend_0.setFont(new java.awt.Font("Lucida Sans", 0, 24));

    return tmpResPlot;
  }
  public static ResidualPlot getResidualPlot(Simulation tmpSim,
          String plotName, String plotTitle){
      ResidualPlot retPlot;
      retPlot = (ResidualPlot) tmpSim.getPlotManager().getObject(plotName);
      retPlot.setPresentationName(plotName);
      retPlot.setTitle(plotTitle);
      retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 0, 18));
      retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 1, 18));
      MultiColLegend tmpLegend = 
          retPlot.getLegend();
      tmpLegend.getChartPositionOption().setSelected(
              ChartPositionOption.Type.NORTH_EAST);
      return retPlot;
  }
  
  // Monitor Plot Tools
  public static MonitorPlot getMonitorPlot(Simulation tmpSim, String plotName,
          String plotTitle){
      MonitorPlot retPlot;

      try{
          retPlot = (MonitorPlot) tmpSim.getPlotManager().getObject(plotName);
      }catch(NeoException e){
          retPlot = tmpSim.getPlotManager().createMonitorPlot();
      }
      retPlot.setPresentationName(plotName);
      retPlot.setTitle(plotTitle);
      retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 0, 24));
      retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 1, 24));
      MultiColLegend tmpLegend = 
          retPlot.getLegend();
      tmpLegend.getChartPositionOption().setSelected(
              ChartPositionOption.Type.NORTH_EAST);
      tmpLegend.setFont(new java.awt.Font("Lucida Sans", 0, 24));
      return retPlot;
  }

  // Monitor Data for Plots
  public static MonitorDataSet addDataSet(Simulation tmpSim,
          MonitorPlot tmpMonPlot, String monitorName,String dataName){
      MonitorDataSet tempMonData;
      ReportMonitor reportMonitor = 
        ((ReportMonitor) tmpSim.getMonitorManager().getMonitor(monitorName));
      try{
          tempMonData =(MonitorDataSet) tmpMonPlot.getDataSetManager().
                  getDataSet(monitorName);
      }catch(NeoException e){
          tmpMonPlot.getDataSetManager().addDataProviders(
                  new NeoObjectVector(new Object[] {reportMonitor}));
      }
      tempMonData = 
        ((MonitorDataSet) tmpMonPlot.getDataSetManager().
                getDataSet(monitorName));
      tempMonData.setSeriesNameLocked(true);
      tempMonData.setSeriesName(dataName);
      return tempMonData;
  }
  public static LineStyle setDataLines(MonitorDataSet tmpMonData,
          String lineStyle, int lineWidth, double[] newLineColor ){
      LineStyle dataLine = 
        tmpMonData.getLineStyle();
      dataLine.setWidth(lineWidth);
      dataLine.setColor(new DoubleVector(newLineColor));
      switch (lineStyle) {
          case "None":
              dataLine.getLinePatternOption().
                      setSelected(LinePatternOption.Type.NONE);
              break;
          case "Dot":
              dataLine.getLinePatternOption().
                      setSelected(LinePatternOption.Type.DOT);
          case "Dash":
              dataLine.getLinePatternOption().
                      setSelected(LinePatternOption.Type.DASH);
              break;
          case "Dash Dot":
              dataLine.getLinePatternOption().
                      setSelected(LinePatternOption.Type.DASH_DOT);
              break;
          case "Dash Dot Dot":
              dataLine.getLinePatternOption().
                      setSelected(LinePatternOption.Type.DASH_DOT_DOT);
              break;
          default:
              dataLine.getLinePatternOption().
                      setSelected(LinePatternOption.Type.SOLID);
              break;
      }
      return dataLine;
  }
  public static void setDataSymbols(MonitorDataSet tmpMonData, 
          String symbStyle,int symbSize,int symbSpacing, double symbWidth, double[] symbColor){
      SymbolStyle symbolStyle_0 = 
        tmpMonData.getSymbolStyle();
      switch (symbStyle) {
          case "Horizontal Line":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.HORIZONTAL_LINE);
              break;
          case "Vertical Line":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.VERTICAL_LINE);
          case "Cross":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.CROSS);
              break;
          case "Plus":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.PLUS);
              break;
          case "Star":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.STAR);
              break;
          case "Filled Square":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.FILLED_SQUARE);
              break;
          case "Empty Square":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.EMPTY_SQUARE);
              break;
          case "Filled Circle":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.FILLED_CIRCLE);
              break;
          case "Empty Circle":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.EMPTY_CIRCLE);
              break;
          case "Filled Triangle":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.FILLED_TRIANGLE);
              break;
          case "Filled Diamond":
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.FILLED_DIAMOND);
              break;
          default:
              symbolStyle_0.getSymbolShapeOption().setSelected(
                      SymbolShapeOption.Type.NONE);
              break;
      }
      symbolStyle_0.setColor(new DoubleVector(symbColor));
      symbolStyle_0.setSize(symbSize);
      symbolStyle_0.setStrokeWidth(symbWidth);
      symbolStyle_0.setSpacing(symbSpacing);
  }

  // XY Plots
  public static XYPlot getXYPlot(Simulation tmpSim,String plotName,
          String plotTitle){

      XYPlot retPlot;

      try{
          retPlot = (XYPlot) tmpSim.getPlotManager().getObject(plotName);
      }catch(NeoException e){
          retPlot = tmpSim.getPlotManager().createXYPlot(plotName);
      }
      retPlot.setPresentationName(plotName);
      retPlot.setTitle(plotTitle);
      retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 0, 24));
      retPlot.setTitleFont(new java.awt.Font("Lucida Sans", 1, 24));
      MultiColLegend tmpLegend = 
          retPlot.getLegend();
      tmpLegend.getChartPositionOption().setSelected(
              ChartPositionOption.Type.NORTH_EAST);
      tmpLegend.setFont(new java.awt.Font("Lucida Sans", 0, 24));
      return retPlot;
  }
  public static void setXYPlotXVectorScalar(Simulation tmpSim,
          XYPlot xyPlot,CoordinateSystem inCoord,String ffName,
          int vectorComponentDirection){
      AxisType xAxis = xyPlot.getXAxisType();
      xAxis.setMode(AxisTypeMode.SCALAR);

      FieldFunctionUnits ffUnits = 
          xAxis.getScalarFunction();

      PrimitiveFieldFunction pFF = 
          ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().
                  getFunction(ffName));

      VectorComponentFieldFunction vCFF= 
      ((VectorComponentFieldFunction) pFF.
              getFunctionInCoordinateSystem(inCoord).
              getComponentFunction(vectorComponentDirection));

      ffUnits.setFieldFunction(vCFF);

  }
  public static void setXYPlotYPrimitiveScalar(Simulation tmpSim,
          XYPlot xyPlot,String ffName){

      YAxisType yAxis = ((YAxisType) xyPlot.getYAxes().getAxisType("Y Type 1"));
      FieldFunctionUnits ffUnits= 
        yAxis.getScalarFunction();

      PrimitiveFieldFunction pFF = 
        ((PrimitiveFieldFunction) tmpSim.getFieldFunctionManager().
                getFunction(ffName));
      ffUnits.setFieldFunction(pFF);

  }
  public static void setYTypeLine(XYPlot xyPlot,Part objPart,
          int lineWidth,double[] color){
      String partName=objPart.getPresentationName();
      YAxisType yAxisType_1 = 
          ((YAxisType) xyPlot.getYAxes().getAxisType("Y Type 1"));
      InternalDataSet internalDataSet_0 = 
      ((InternalDataSet) yAxisType_1.getDataSetManager().getDataSet(partName));

      LineStyle lineStyle_0 = 
      internalDataSet_0.getLineStyle();
      lineStyle_0.setWidth(lineWidth);
      lineStyle_0.setColor(new DoubleVector(color));
  }
  public static void setYTypeSymbol(XYPlot xyPlot,Part objPart,
          double[] color,int symbSize,double symbWidth, int symbSpacing){
      String partName=objPart.getPresentationName();
      YAxisType yAxisType_1 = 
          ((YAxisType) xyPlot.getYAxes().getAxisType("Y Type 1"));
      InternalDataSet internalDataSet_0 = 
          ((InternalDataSet) yAxisType_1.getDataSetManager().
                  getDataSet(partName));
      SymbolStyle symbolStyle_0 = 
          internalDataSet_0.getSymbolStyle();
      symbolStyle_0.setColor(new DoubleVector(color));
      symbolStyle_0.getSymbolShapeOption().setSelected(
              SymbolShapeOption.Type.FILLED_CIRCLE);
      symbolStyle_0.setSize(symbSize);
      symbolStyle_0.setStrokeWidth(symbWidth);
      symbolStyle_0.setSpacing(symbSpacing);
  }

  // General
  public static Axis setPlotXAxis(StarPlot tmpPlot,String titleName,
          double minVal,double maxVal,double majSpacing,int minorTicks){
      Axis tmpAxis = tmpPlot.getAxisManager().getObject("Bottom Axis");
      // range
      tmpAxis.setMinimum(minVal);
      tmpAxis.setLockMinimum(true);
      tmpAxis.setMaximum(maxVal);
      tmpAxis.setLockMaximum(true);
      // Major Label
      AxisLabels axisLabel = tmpAxis.getLabels();
      axisLabel.setGridWidth(2);
      axisLabel.setNumSpacing(majSpacing);
      axisLabel.setGridColor(new IntVector(new int[] {0, 0, 0, 255}));
      axisLabel.setFont(new java.awt.Font("SansSerif", 0, 24));
      // Minor Ticks
      AxisTicks axisTicks = 
        tmpAxis.getTicks();
      axisTicks.setCount(minorTicks);
      axisTicks.setGridWidth(2);
      axisTicks.setGridColor(new IntVector(new int[] {245, 245, 245, 255}));
      // Title
      AxisTitle axisTitle = 
        tmpAxis.getTitle();
      axisTitle.setFont(new java.awt.Font("SansSerif", 0, 24));
      axisTitle.setFont(new java.awt.Font("SansSerif", 1, 24));
      axisTitle.setText(titleName);

      return tmpAxis;
  }
  public static Axis setPlotXToLog(StarPlot tmpPlot,boolean setLog){
      Axis axis = 
        tmpPlot.getAxisManager().getObject("Bottom Axis");
      axis.setLogarithmic(setLog);
      return axis;
  }
  public static Axis setPlotYAxis(StarPlot tmpPlot,String titleName,
          double minVal,double maxVal,double majSpacing,int minorTicks){
      Axis tmpAxis = tmpPlot.getAxisManager().getObject("Left Axis");
      // range
      tmpAxis.setMinimum(minVal);
      tmpAxis.setLockMinimum(true);
      tmpAxis.setMaximum(maxVal);
      tmpAxis.setLockMaximum(true);
      // Major Label
      AxisLabels axisLabel = tmpAxis.getLabels();
      axisLabel.setGridWidth(2);
      axisLabel.setNumSpacing(majSpacing);
      axisLabel.setGridColor(new IntVector(new int[] {0, 0, 0, 255}));
      axisLabel.setFont(new java.awt.Font("SansSerif", 0, 24));
      // Minor Ticks
      AxisTicks axisTicks = 
        tmpAxis.getTicks();
      axisTicks.setCount(minorTicks);
      axisTicks.setGridWidth(2);
      axisTicks.setGridColor(new IntVector(new int[] {245, 245, 245, 255}));
      // Title
      AxisTitle axisTitle = 
        tmpAxis.getTitle();
      axisTitle.setFont(new java.awt.Font("SansSerif", 0, 24));
      axisTitle.setFont(new java.awt.Font("SansSerif", 1, 24));
      if(! titleName.equals("") ){
        axisTitle.setText(titleName);
      }

      return tmpAxis;
  }
  public static Axis setPlotYToLog(StarPlot tmpPlot,boolean setLog){
      Axis axis = 
        tmpPlot.getAxisManager().getObject("Left Axis");
      axis.setLogarithmic(setLog);
      return axis;
  }
  

  public static StarPlot autoFitYAxisToYData(Simulation simu, 
          StarPlot tmpPlot, double meanVal, double stdDev){
    double minVal=-1.0;
    double maxVal=1.0;
    double majSpacing=0.2;
    if(meanVal<1e10&&stdDev<1e10){
        //percentage based
        //minVal=meanVal-0.2*Math.abs(meanVal);
        //maxVal=meanVal+0.2*Math.abs(meanVal);
        //majSpacing=meanVal-0.02*Math.abs(meanVal);
        //check if small numbers but large stDev
        //if((minVal>meanVal-stDevVal)||(maxVal<meanVal+stDevVal)){
         minVal=meanVal-20.0*stdDev; 
         maxVal=meanVal+20.0*stdDev;
         majSpacing=2.0*stdDev;
        //}
    }
    PlotTool.setPlotYAxis(tmpPlot, "", minVal, maxVal, majSpacing, 2);
    return tmpPlot;
  }

  //=========================================
  // GENERAL UI CLEANUP
  //=========================================
  public static void groupPlots(Simulation tmpSim,String groupName,
          ArrayList<StarPlot> toBeGrouped){
      //Groups
      getPlotGroup(tmpSim,groupName).getGroupsManager().groupObjects(groupName,
        toBeGrouped, true);
  }
  public static ClientServerObjectGroup getPlotGroup(Simulation tmpSim,
          String groupName){
    ClientServerObjectGroup tmp;
    try{
        tmp=((ClientServerObjectGroup) tmpSim.getPlotManager()
                .getGroupsManager().getObject(groupName));
    }catch(NeoException e){
        tmpSim.getPlotManager().getGroupsManager().createGroup(groupName);
        try{
          Thread.sleep(100);
        }catch(InterruptedException f){

        }
        tmp=((ClientServerObjectGroup) tmpSim.getPlotManager()
                .getGroupsManager().getObject(groupName));
    }
    return tmp;
  }
    
}
