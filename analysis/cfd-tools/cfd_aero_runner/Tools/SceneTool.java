/*
    Copyright 2020 Makani Technologies LLC

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

import java.io.File;
import java.util.*;
import star.common.*;
import star.base.neo.*;
import star.vis.*;

public class SceneTool {
    
public static Scene getScene(Simulation simu, String sceneName) {
  Scene retScene;
  try{
    retScene = simu.getSceneManager().getScene(sceneName);
  }catch(NeoException e){
    retScene =simu.getSceneManager().createScene(sceneName);
    retScene.setPresentationName(sceneName);
  }
  retScene.initializeAndWait();
  retScene.setBackgroundColorMode(BackgroundColorMode.SOLID);
  SolidBackgroundColor solidBackgroundColor_0 = 
    retScene.getSolidBackgroundColor();
  solidBackgroundColor_0.setColor(
      new DoubleVector(new double[] {1.0, 1.0, 1.0}) );
  CurrentView tmpView = retScene.getCurrentView();
  tmpView.setProjectionMode(VisProjectionMode.PARALLEL);
  return retScene;
}
public static void deleteScene(Scene thisScene){
  SceneManager tmpExecutioner = thisScene.getSceneManager();
  tmpExecutioner.deleteScene(thisScene);
}

public static boolean doesSceneExist(Simulation tmpSim, String sceneName){
  Scene tmpScene;
  try{
    tmpScene = tmpSim.getSceneManager().getScene(sceneName);
    return true;
  }catch(NeoException e){

    return false;
  }
}
  
public static void wipeDisplayers(Scene theScene){
  for(Displayer tmp:theScene.getDisplayerManager().getObjects()){
    theScene.getDisplayerManager().remove(tmp);
  }
}
  
// Part displayer building methods
public static PartDisplayer getPartDisplayer(Scene tmpScene, String dispName,
    Collection<? extends NamedObject> tmpParts,Representation tmpRep){

  PartDisplayer retDisp;
  boolean haveDisp=false;
  // Does the scene have any displayers?
  for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
    if(tmp instanceof PartDisplayer){
      haveDisp=true;
    }
  }
  if(!haveDisp){
    retDisp = tmpScene.getDisplayerManager().createPartDisplayer(dispName);
  }else{
    boolean haveMyDesiredDisp=false;
    // Loop through part displayers looking for our desired displayer
    for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
      if(tmp instanceof PartDisplayer && tmp.getPresentationName()
              .equals(dispName)){
        haveMyDesiredDisp=true;
      }
    }
    if(haveMyDesiredDisp){
      retDisp = 
          ((PartDisplayer) tmpScene.getDisplayerManager()
              .getDisplayer(dispName));
    }else{
      retDisp = (PartDisplayer) tmpScene.getDisplayerManager()
          .createPartDisplayer(dispName);
    }
  }
  retDisp.setPresentationName(dispName);
  retDisp.getInputParts().setObjects(tmpParts);
  retDisp.setRepresentation(tmpRep);
  return retDisp;
}
public static PartDisplayer getPartDisplayerFromScene(Scene tmpScene,
    String dispName){
  PartDisplayer retDisp;
  boolean haveDisp=false;
  // Does the scene have any displayers?
  for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
      if(tmp instanceof PartDisplayer){
          haveDisp=true;
      }
  }
  if(!haveDisp){
      retDisp = tmpScene.getDisplayerManager().createPartDisplayer(dispName);
  }else{
    boolean haveMyDesiredDisp=false;
    // Loop through part displayers looking for our desired displayer
    for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
      if(tmp instanceof PartDisplayer && tmp.getPresentationName().
              equals(dispName)){
        haveMyDesiredDisp=true;
      }
    }
    if(haveMyDesiredDisp){
      retDisp = 
        ((PartDisplayer) tmpScene.getDisplayerManager().getDisplayer(dispName));
    }else{
      retDisp = (PartDisplayer) tmpScene.getDisplayerManager().
              createPartDisplayer(dispName);
    }
  }
  retDisp.setPresentationName(dispName);
  return retDisp;
}

public static PartDisplayer setPartDisplayerField(PartDisplayer tmpD,
    boolean needOutline,boolean needMesh,
    boolean needFeature,boolean needSurface,int colorMode){
  tmpD.setMesh(needMesh);
  tmpD.setOutline(needOutline);
  tmpD.setFeatures(needFeature);
  tmpD.setSurface(needSurface);
  PartColorMode tmpPartColorMode;
  switch (colorMode) {
    case 1:
      //distinguish parts
      tmpPartColorMode=PartColorMode.DP;
      break;
    case 2:
      //distinguish regions
      tmpPartColorMode=PartColorMode.DR;
      break;
    case 3:
      //region type
      tmpPartColorMode=PartColorMode.RT;
      break;
    case 4:
      //constant
      tmpPartColorMode=PartColorMode.CONSTANT;
      break;              
    case 5:
      //region type
      tmpPartColorMode=PartColorMode.RT;
      break;              
    default:
      tmpPartColorMode=PartColorMode.DEFAULT;
      break;
  }
  tmpD.setColorMode(tmpPartColorMode);
  return tmpD;
}
  public static PartDisplayer setPartDisplayerField(PartDisplayer tmpD,
          boolean needOutline,boolean needMesh,
          boolean needFeature,boolean needSurface,int colorMode,double opacity){
      setPartDisplayerField(
              tmpD,needOutline,needMesh,needFeature,needSurface,colorMode);
      tmpD.setOpacity(opacity);
      return tmpD;
  }

  // scalar displayer building methods
  public static ScalarDisplayer getScalarDisplayer(
          Scene tmpScene,String dispName,
          Collection<? extends NamedObject> tmpParts){
    ScalarDisplayer retDisp;
    boolean haveDisp=false;
    for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
      if(tmp instanceof ScalarDisplayer){
          haveDisp=true;
      }
    }
    if(!haveDisp){
      retDisp = tmpScene.getDisplayerManager().createScalarDisplayer(dispName);
    }else{
      // Loop through part displayers looking for our desired displayer
      for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
        if(tmp instanceof ScalarDisplayer && tmp.getPresentationName().
                equals(dispName)){
            haveDisp=true;
        }
      }
      if(haveDisp){
        retDisp = 
          ((ScalarDisplayer) tmpScene.getDisplayerManager().
                  getDisplayer(dispName));
      }else{
        retDisp = tmpScene.getDisplayerManager().
                createScalarDisplayer(dispName);
      }
    }
    retDisp.getInputParts().setObjects(tmpParts);
    retDisp.setPresentationName(dispName);
    return retDisp;
  }
  public static ScalarDisplayer getScalarDisplayerFromScene(Scene tmpScene,
          String dispName){
      ScalarDisplayer retDisp;
      boolean haveDisp=false;
      for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
          if(tmp instanceof ScalarDisplayer){
              haveDisp=true;
          }
      }
      if(!haveDisp){
          retDisp = tmpScene.getDisplayerManager().
                  createScalarDisplayer(dispName);
      }else{
          //loop through part displayers looking for our desired displayer
          for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
              if(tmp instanceof ScalarDisplayer && tmp.getPresentationName().
                      equals(dispName)){
                  haveDisp=true;
              }
          }
          if(haveDisp){
              retDisp = 
                ((ScalarDisplayer) tmpScene.getDisplayerManager().
                        getDisplayer(dispName));
          }else{
              retDisp = tmpScene.getDisplayerManager().
                      createScalarDisplayer(dispName);
          }
      }
      retDisp.setPresentationName(dispName);
      return retDisp;
  }
  public static ScalarDisplayer setScalarDisplayerField(ScalarDisplayer tmpSD,
          String fieldFunctionName,String autoOff, String clipSet,
          double minVal, double maxVal,Representation tmpRep){
      ScalarDisplayer retDisp=tmpSD;
      FieldFunction scalFF = tmpSD.getScene().getSceneManager().getSimulation()
        .getFieldFunctionManager().getFunction(fieldFunctionName);
      retDisp.getScalarDisplayQuantity().setFieldFunction(scalFF);
      //Clipchoice = {0,1,2} 0 is off, 1 is above max, 2 is below min
      switch (autoOff) {
          case "Min and Max Values":
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.BOTH);
              break;
          case "Min Value":
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.MIN);
              break;
          case "Max Value":
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.MAX);
              break;
          default:
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.NONE);
              break;
      }

      //Clipchoice = {0,1,2} 0 is off, 1 is above max, 2 is below min
      switch (clipSet) {
          case "Below Min, Above Max":
              retDisp.getScalarDisplayQuantity().setClip(ClipMode.BOTH);
              break;
          case "Below Min":
              retDisp.getScalarDisplayQuantity().setClip(ClipMode.MIN);
              break;
          case "Above Max":
              retDisp.getScalarDisplayQuantity().setClip(ClipMode.MAX);
              break;
          default:
              retDisp.getScalarDisplayQuantity().setClip(ClipMode.NONE);
              break;
      }
      retDisp.getScalarDisplayQuantity().setRangeMin(minVal);
      retDisp.getScalarDisplayQuantity().setRangeMax(maxVal);
      retDisp.setRepresentation(tmpRep);
      retDisp.setFillMode(ScalarFillMode.NODE_FILLED);
      return retDisp;
  }
  public static ScalarDisplayer setScalarVectorComponent(ScalarDisplayer tmpSD,
          String vectorFieldFunctionName,CoordinateSystem tmpCoord,int coordDir,
          String autoOff, String clipSet, double minVal, double maxVal,
          Representation tmpRep){
      ScalarDisplayer retDisp=tmpSD;

      PrimitiveFieldFunction vectFF = (PrimitiveFieldFunction) tmpSD.getScene().
              getSceneManager().getSimulation()
        .getFieldFunctionManager().getFunction(vectorFieldFunctionName);

      VectorComponentFieldFunction vCFF = 
       ((VectorComponentFieldFunction) vectFF.
               getFunctionInCoordinateSystem(tmpCoord).
               getComponentFunction(coordDir));

      retDisp.getScalarDisplayQuantity().setFieldFunction(vCFF);

      int autoInt;
      //Clipchoice = {0,1,2} 0 is off, 1 is above max, 2 is below min
      switch (autoOff) {
          case "Min and Max Values":
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.BOTH);
              break;
          case "Min Value":
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.MIN);
              break;
          case "Max Value":
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.MAX);
              break;
          default:
              retDisp.getScalarDisplayQuantity().
                      setAutoRange(AutoRangeMode.NONE);
              break;
      }

      //Clipchoice = {0,1,2} 0 is off, 1 is above max, 2 is below min
      switch (clipSet) {
          case "Below Min, Above Max":
              retDisp.getScalarDisplayQuantity().
                      setClip(ClipMode.BOTH);
              break;
          case "Below Min":
              retDisp.getScalarDisplayQuantity().
                      setClip(ClipMode.MIN);
              break;
          case "Above Max":
              retDisp.getScalarDisplayQuantity().
                      setClip(ClipMode.MAX);
              break;
          default:
              retDisp.getScalarDisplayQuantity().
                      setClip(ClipMode.NONE);
              break;
      }
      retDisp.getScalarDisplayQuantity().setRangeMin(minVal);
      retDisp.getScalarDisplayQuantity().setRangeMax(maxVal);

      return retDisp;
  }
  public static ScalarDisplayer setScalarColormap(ScalarDisplayer tmpSD,
          String colorMapName,int numLvls,int numLbls,boolean isFlipped,
          String legendPosition){
      ScalarDisplayer retDisp=tmpSD;
      Legend tmpLegend=retDisp.getLegend();
      tmpLegend = setScalarLegendPosition(tmpLegend,legendPosition);
      tmpLegend.setLookupTable(tmpSD.getScene().getSceneManager().
              getSimulation()
        .get(LookupTableManager.class).getObject(colorMapName));
      tmpLegend.setReverse(isFlipped);
      tmpLegend.setLevels(numLvls);
      tmpLegend.setNumberOfLabels(numLbls);
      return retDisp;
  }
  public static Legend setScalarLegendPosition(Legend tmpLegend,
          String legendPosition){
      int setOrientation;
      double barWidth;
      double barHeight;
      double barTitleHeight;
      double barLabelHeight;
      double lowerLeftCorner_X;
      double lowerLeftCorner_Y;
      //
      switch (legendPosition) {
          case "Bottom":
              //
              setOrientation=0;
              barWidth=0.45;
              barHeight=0.045;
              barLabelHeight=0.025;
              //
              barTitleHeight=0.0275;
              //
              lowerLeftCorner_X=0.275;
              lowerLeftCorner_Y=0.15;
              break;
          case "Top":
              //
              setOrientation=0;
              barWidth=0.45;
              barHeight=0.045;
              barLabelHeight=0.025;
              //
              barTitleHeight=0.0275;
              //
              lowerLeftCorner_X=0.275;
              lowerLeftCorner_Y=0.85;
              break;
          case "Left Side":
              //
              setOrientation=1;
              barWidth=0.05;
              barHeight=0.45;
              barLabelHeight=0.025;
              //
              barTitleHeight=0.0275;
              //
              lowerLeftCorner_X=0.06;
              lowerLeftCorner_Y=0.275;
              break;
          default:
              setOrientation=0;
              barWidth=0.45;
              barHeight=0.045;
              barLabelHeight=0.025;
              //
              barTitleHeight=0.0275;
              //
              lowerLeftCorner_X=0.275;
              lowerLeftCorner_Y=0.85;
              break;
      }
      if(setOrientation==0){
          tmpLegend
                  .setOrientation(
                          LegendOrientation.LEGEND_ORIENTATION_HORIZONTAL);
      }else{
          tmpLegend
                  .setOrientation(
                          LegendOrientation.LEGEND_ORIENTATION_VERTICAL);
      }
      tmpLegend.setWidth(barWidth);
      tmpLegend.setHeight(barHeight);
      tmpLegend.setTitleHeight(barTitleHeight);
      tmpLegend.setLabelHeight(barLabelHeight);
      tmpLegend.setPositionCoordinate(new DoubleVector(
              new double[] {lowerLeftCorner_X,lowerLeftCorner_Y}));
      return tmpLegend;
  }
  public static CurrentView setSceneView(Scene tmpScene,double[] focalPt,
          double[] camPos,double[] viewUp,CoordinateSystem tmpCsys,
          double pllMag,int projMode){
      /* Method setSceneView controls the view characteristic of Scene tmpScene
          focalPt is the origin of the camera rotation
          camPos is the position of the camera in tmpCsys system
          viewUp controls which direction is "up" in the scene
          pllMag sets the magnification for parallel viewpoints only
          projMod sets parallel (1) or perspective(0)
      */
      CurrentView tmpView = tmpScene.getCurrentView();
      Units units_0 = 
          ((Units) tmpScene.getSceneManager().getSimulation().
                  getUnitsManager().getObject("m"));
      tmpView.setInput(new DoubleVector(
              new double[] {focalPt[0],focalPt[1],focalPt[2]}),
               new DoubleVector(new double[] {camPos[0],camPos[1],camPos[2]}),
                   new DoubleVector(
                           new double[] {viewUp[0],viewUp[1],viewUp[2]}),
                   pllMag, projMode);
      tmpView.setCoordinateSystem(tmpCsys);
      Coordinate focalPtCoord = 
          tmpView.getFocalPointCoordinate();
      focalPtCoord.setCoordinate(units_0, units_0, units_0,
              new DoubleVector(focalPt));
      Coordinate camPosCoord = 
          tmpView.getPositionCoordinate();
      camPosCoord.setCoordinate(units_0, units_0, units_0,
              new DoubleVector(camPos));
      Coordinate viewUpCoord=
          tmpView.getViewUpCoordinate();
      viewUpCoord.setCoordinate(units_0, units_0, units_0,
              new DoubleVector(viewUp));
      return tmpView;
  }
  
  // Streamline displayer
  public static StreamDisplayer getStreamLineDisplayer(Scene tmpScene,
          String dispName,
          Collection<? extends NamedObject> tmpParts){
    StreamDisplayer retDisp;
    boolean haveDisp=false;
    for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
      if(tmp instanceof StreamDisplayer){
        haveDisp=true;
      }
    }
    if(!haveDisp){
      retDisp = tmpScene.getDisplayerManager().createStreamDisplayer(dispName);
    }else{
      //loop through part displayers looking for our desired displayer
      for(Displayer tmp:tmpScene.getDisplayerManager().getObjects()){
        if(tmp instanceof StreamDisplayer && tmp.getPresentationName().
                equals(dispName)){
            haveDisp=true;
        }
      }
      if(haveDisp){
        retDisp = 
          ((StreamDisplayer) tmpScene.getDisplayerManager().
                  getDisplayer(dispName));
      }else{
        retDisp = tmpScene.getDisplayerManager().
                createStreamDisplayer(dispName);
      }
    }
    retDisp.getInputParts().setObjects(tmpParts);
    retDisp.setPresentationName(dispName);
    return retDisp;
  }
  public static StreamDisplayer setStreamScalarDisplayerProperties(
          StreamDisplayer tmpSD, String autoOff,
          String clipSet, double minVal, double maxVal, Representation tmpRep){
    // Note: a streamline is always going to be a vector field integration
    // This is currently best done directly to the streamline displayer part.
    StreamDisplayer retDisp=tmpSD;

    //Clipchoice = {0,1,2} 0 is off, 1 is above max, 2 is below min
    switch (autoOff) {
      case "Min and Max Values":
        retDisp.getScalarDisplayQuantity().setAutoRange(AutoRangeMode.BOTH);
        break;
      case "Min Value":
        retDisp.getScalarDisplayQuantity().setAutoRange(AutoRangeMode.MIN);
        break;
      case "Max Value":
        retDisp.getScalarDisplayQuantity().setAutoRange(AutoRangeMode.MAX);
        break;
      default:
        retDisp.getScalarDisplayQuantity().setAutoRange(AutoRangeMode.NONE);
        break;
    }

    //Clipchoice = {0,1,2} 0 is off, 1 is above max, 2 is below min
    switch (clipSet) {
      case "Below Min, Above Max":
        retDisp.getScalarDisplayQuantity().setClip(ClipMode.BOTH);
        break;
      case "Below Min":
        retDisp.getScalarDisplayQuantity().setClip(ClipMode.MIN);
        break;
      case "Above Max":
        retDisp.getScalarDisplayQuantity().setClip(ClipMode.MAX);
        break;
      default:
        retDisp.getScalarDisplayQuantity().setClip(ClipMode.NONE);
        break;
    }
    retDisp.getScalarDisplayQuantity().setRangeMin(minVal);
    retDisp.getScalarDisplayQuantity().setRangeMax(maxVal);
    retDisp.setRepresentation(tmpRep);
    return retDisp;
  }
  public static StreamDisplayer setStreamColormap(StreamDisplayer tmpSD,
          String colorMapName, int numLvls,
          int numLbls,boolean isFlipped,String legendPosition){
      StreamDisplayer retDisp=tmpSD;
      Legend tmpLegend=retDisp.getLegend();
      tmpLegend = setScalarLegendPosition(tmpLegend,legendPosition);
      tmpLegend.setLookupTable(tmpSD.getScene().getSceneManager().
              getSimulation()
        .get(LookupTableManager.class).getObject(colorMapName));
      tmpLegend.setReverse(isFlipped);
      tmpLegend.setLevels(numLvls);
      tmpLegend.setNumberOfLabels(numLbls);
      return retDisp;
  }
  
  public static VectorDisplayer setVectorDisp(Scene tmpScene,String tmpName,
          Collection<PlaneSection> tmpPS, CoordinateSystem tmpCsys,
          Representation tmpRep){
      VectorDisplayer vectD = tmpScene.getDisplayerManager().
              createVectorDisplayer(tmpName);
      vectD.initialize();
      Legend legend_0 = vectD.getLegend();
      legend_0.setVisible(false);
      vectD.getVisibleParts().addParts(tmpPS);
      vectD.setColorMode(VectorColorMode.CONSTANT);
      vectD.setDisplayMode(VectorDisplayMode.VECTOR_DISPLAY_MODE_LIC);
      vectD.setDisplayerColor(new DoubleVector(new double[] {0.0, 0.0, 0.0}));
      vectD.setRepresentation(tmpRep);
      vectD.setDisplayMode(VectorDisplayMode.VECTOR_DISPLAY_MODE_GLYPH);

      GlyphSettings glyphS = vectD.getGlyphSettings();
      glyphS.setVectorStyle(VectorStyle.FILLED_2D);
      glyphS.setRelativeToModelLength(4.0);
      GeometricGlyphSpacingOption geometricGlyphSpacingOption_0 = 
              glyphS.getGeometricSpacing();
      geometricGlyphSpacingOption_0.setOnRatio(4);
      geometricGlyphSpacingOption_0.setRandom(true);
      glyphS.setTipScale(0.4); 
      
      return vectD;
  }
  
  // Annotations
  public static SimpleAnnotation getTextAnnotation(Simulation simu, 
          String tmpName, String txtString,double defH, 
          double[] newPosition,boolean needBckgrnd){
      SimpleAnnotation tmpAnn;
      try{
          tmpAnn = (SimpleAnnotation) simu.getAnnotationManager().
                  getObject(tmpName);
      }catch(NeoException e){
          tmpAnn = simu.getAnnotationManager().createSimpleAnnotation();
          tmpAnn.setPresentationName(tmpName);
      }
      tmpAnn.setText(txtString);
      tmpAnn.setDefaultHeight(defH);
      tmpAnn.setDefaultPosition(new DoubleVector(newPosition));
      tmpAnn.setBackground(needBckgrnd);

      return tmpAnn;
  }
  public static void removeDefaultLogo(Scene tmpScene){
      Collection<AnnotationProp> allAnnProps = tmpScene.
              getAnnotationPropManager().getObjects();
      boolean doesContain=false;
      for(AnnotationProp tmpProp:allAnnProps){
        if(tmpProp.getPresentationName().equals("Logo")){
          doesContain=true;
        }
      }
      try{
        if(doesContain){
      LogoAnnotation logoAnnotation = 
           ((LogoAnnotation) tmpScene.getSceneManager().getSimulation().
                   getAnnotationManager().getObject("Logo"));
          tmpScene.getAnnotationPropManager().getAnnotationProp("Logo");
          AnnotationProp tmpAP = tmpScene.getAnnotationPropManager()
                    .getPropForAnnotation(logoAnnotation);                
            tmpScene.getAnnotationPropManager().remove(tmpAP);
        }

      }catch(NeoException e){
      }
  }

  public static void addAnnotation(Scene tmpScene,Annotation tmpAnn){

    boolean doesContain=false;
    for(AnnotationProp tmpProp:tmpScene.getAnnotationPropManager().
            getObjects()){
      if(tmpProp.getPresentationName().equals(tmpAnn.getPresentationName())){
        doesContain=true;
      }
    }

    try{
        // Adds any annotation into the scene
        if(!doesContain){
          tmpScene.getAnnotationPropManager().createPropForAnnotation(tmpAnn);
        }
      }catch(NeoException e){
      }
  }
  public static Scene axisLocation(Scene tmpScene, double[] loc,
          Boolean display){
      tmpScene.setAxesViewport(new DoubleVector(loc));
      tmpScene.setAxesVisible(display);
      return tmpScene;
  }
  
  //Utility
  public static void groupScenes(Simulation simu,String groupName,
          ArrayList<Scene> toBeGrouped){
      //Groups
      getSceneGroup(simu,groupName).getGroupsManager().groupObjects(groupName,
        toBeGrouped, true);
  }
  public static ClientServerObjectGroup getSceneGroup(Simulation simu,
          String groupName){
      ClientServerObjectGroup tmp;
      try{
          tmp=((ClientServerObjectGroup) simu.getSceneManager()
                  .getGroupsManager().getObject(groupName));
      }catch(NeoException e){
          simu.getSceneManager().getGroupsManager().createGroup(groupName);
          tmp=((ClientServerObjectGroup) simu.getSceneManager()
                  .getGroupsManager().getObject(groupName));
      }
      return tmp;
  }
  //Views
  public static VisView getView(Simulation simu, String viewName,
          CoordinateSystem viewCoord, double[] focalCoord,double[] posCoord,
          double[] upCoord, boolean isPll){
      VisView myView;
      try{
          myView = simu.getViewManager().getObject(viewName);
      }catch(NeoException e){
          myView = 
        simu.getViewManager().createView();
          myView.setPresentationName(viewName);
      }

      if(isPll){
          myView.setProjectionMode(VisProjectionMode.PARALLEL);
      }else{
          myView.setProjectionMode(VisProjectionMode.PERSPECTIVE);
      }

      Coordinate coordinate_2 = 
        myView.getFocalPointCoordinate();
      Units units_0 = 
        ((Units) simu.getUnitsManager().getObject("m"));
      coordinate_2.setCoordinate(units_0, units_0, units_0,
              new DoubleVector(focalCoord));
      Coordinate coordinate_3 = 
        myView.getPositionCoordinate();
      coordinate_3.setCoordinate(units_0, units_0, units_0,
              new DoubleVector(posCoord));
      Coordinate coordinate_4 = 
        myView.getViewUpCoordinate();
      coordinate_4.setCoordinate(units_0, units_0, units_0,
              new DoubleVector(upCoord));
      myView.setCoordinateSystem(viewCoord);

      return myView;
  }
  //output
  public static void setSceneUnsteadyOutput(Scene tmpScene, String savePath,
          String fileName,int timeStepFreq,int xRes, int yRes){
          SceneUpdate sceneUpdate = 
            tmpScene.getSceneUpdate();
          sceneUpdate.setSaveAnimation(true);
          sceneUpdate.setAnimationFilePath(savePath);
          sceneUpdate.setAnimationFilenameBase(fileName);
          sceneUpdate.getUpdateModeOption()
                  .setSelected(StarUpdateModeOption.Type.TIMESTEP);
          sceneUpdate.getTimeStepUpdateFrequency().setTimeSteps(timeStepFreq);
          //output
          HardcopyProperties hardcopyProps = 
            sceneUpdate.getHardcopyProperties();
          hardcopyProps.setUseCurrentResolution(false);
          hardcopyProps.setOutputHeight(yRes);
          hardcopyProps.setOutputWidth(xRes);
  }

  public static void outputSceneHardcopy(Scene tmpScene,String savePath,
          String fileName,int xRes, int yRes){
      tmpScene.printAndWait(savePath+File.separator+fileName, 1, xRes, yRes, true, false);
  }

  public static void outputSceneFromViews(Scene tmpScene,String savePath,
          String fileName,int[] xyRes,ArrayList<VisView> viewList){
        for(VisView tmpView:viewList){
            String viewName=tmpView.getPresentationName();
            tmpScene.setCurrentView(tmpView);
            tmpScene.printAndWait(savePath+File.separator+fileName+"_"+viewName,
                    1, xyRes[0], xyRes[1], true, false);
        }
    }

  // STAR-VEW TOOLS
  public static void exportSceneToStarView(Simulation tmpSim, String fileDir,
    Scene tmpScene, String sceneDescription, String fileName,
    boolean needToAppend){
    //
    SystemTool.touchDirectoryChain(tmpSim, fileDir);
    //
    String tmpSceneTitle= tmpScene.getPresentationName();
    //
    tmpScene.export3DSceneFileAndWait(fileDir+File.separator+fileName+".sce",
      tmpSceneTitle, sceneDescription, needToAppend, true);
  }

}
