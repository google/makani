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
package MeshTools;

import java.util.*;

import star.common.*;
import star.base.neo.*;
import star.base.query.*;
import star.meshing.*;
import star.prismmesher.*;
import star.resurfacer.*;
import star.trimmer.*;

public class MeshOpTool {
  /* class MeshOpTool
       allows easier access to some of the Geometry and Mesh Operations in the STAR-CCM+ tree
  */

  public static Units getPrefUVec(Simulation simu){
    /* method getPrefUVec returns the preferred units vector required for simulations
       We use meters, kilograms, seconds as our default units
    */
    return simu.getUnitsManager().getPreferredUnits(
      new IntVector(new int[] {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  }

  //Geometry Part Filters
  public static ArrayList<GeometryPart> findPartsByFirstName(
      Collection<GeometryPart> allParts,String identStr){
    /* method findPartsByFirstName returns a Part called by its specific 
       presentationName. If there are multiple parts with this name, 
       it will return a warning, indicating that the caller may not be getting
       GeometryPart anciticpated.
    */
    ArrayList<GeometryPart> retList = new ArrayList();
    for(GeometryPart tmpPart:allParts){
      if((tmpPart instanceof CompositePart)){ // A Composite Part - look through its children instead
        for(GeometryPart tmpChildPart:((CompositePart) tmpPart).getChildParts().getObjects()){
          if( tmpChildPart.getPresentationName().startsWith(identStr)||
              tmpChildPart.getPresentationName().contains("."+identStr) ){
            retList.add(tmpChildPart);
          }
        }
      }else{
        if(tmpPart.getPresentationName().startsWith(identStr)||
                tmpPart.getPresentationName().contains("."+identStr)){
            retList.add(tmpPart);
        }
      }
    }
    return retList;
  }

  public static GeometryPart findGeometryPart(Simulation simu, Collection<GeometryPart> allParts,
                                              ArrayList<String> identList){
    /* method findGeometryPart returns a Part based on its specific Surfaces presentationName.
       If there are multiple parts with this name, it will return a warning, indicating
       that the caller may not be getting GeometryPart anciticpated.
    */
    GeometryPart retPart=null;
    boolean isPart = false;
    int cntr=0;
    for(GeometryPart tmpPart:allParts){
      if((tmpPart instanceof CompositePart)){ // A Composite Part - look through its children instead
        for(GeometryPart tmpChildPart:((CompositePart) tmpPart).getChildParts().getObjects()){
          for(PartSurface tmpSurf:tmpChildPart.getPartSurfaces()){
            String surfName = tmpSurf.getPresentationName();
            boolean isCorrectPreFix=false;
            for(String tmpStr:identList){
                if(surfName.startsWith(tmpStr)||surfName.contains("."+tmpStr)){
                    isCorrectPreFix=true;
                }
            }
            if(isCorrectPreFix){
                isPart=true;
            }
            if(isPart){
              retPart=tmpChildPart;
              isPart=false;
              cntr+=1;
              break;
            }
          } 
        }
      } else{ // Raw Geometery Part
        for(PartSurface tmpSurf:tmpPart.getPartSurfaces()){
          String surfName = tmpSurf.getPresentationName();
          boolean isCorrectPreFix=false;
          for(String tmpStr:identList){
              if(surfName.startsWith(tmpStr)||surfName.contains("."+tmpStr)){
                  isCorrectPreFix=true;
              }
          }
          if(isCorrectPreFix){
              isPart=true;
          }
          if(isPart){
            retPart=tmpPart;
            isPart=false;
            cntr+=1;
            break;
          }
        }
      }
    }
    if(cntr>1) simu.println("WARNING MeshOpTool: findGeometryPart - found multiple Geom Parts!");
    if(retPart==null) simu.println("WARNING MeshOpTool: findGeometryPart - retPart is null. Will likely fail.");
    return retPart;
  }
  
  public static ArrayList<GeometryPart> findGeometryParts(Simulation simu, 
      Collection<GeometryPart> allParts, ArrayList<String> identList){
    /* method findGeometryParts returns an ArrayList of GeometryParts based on
       its specific Surface presentationName starting with or containing String
       values contained in identList. It will expose any children of
       CompositeParts within allParts.
    */
    ArrayList<GeometryPart> relevantParts = new ArrayList(allParts);
    ArrayList<GeometryPart> retParts = new ArrayList();
    // Compsite parts are usually present due to an imported assembly. For
    // example, when importing multiple CAD parts, all the CAD parts
    // will get put into a single Composite Part. So, if it is a Composite
    // Part, it is likely multiple parts imported from CAD. So, look through
    // the children instead.
    for(GeometryPart tmpPart:allParts){
      if(tmpPart instanceof CompositePart){
        relevantParts.addAll(
            ((CompositePart) tmpPart).getChildParts().getObjects()
            );
        relevantParts.remove(tmpPart);
      }
    }

    boolean isPart = false;
    for(GeometryPart tmpPart:relevantParts){
      // Everything cylcled through here should now be pure GeometeryParts
      // comprised of named Surfaces.
      for(PartSurface tmpSurf:tmpPart.getPartSurfaces()){
        String surfName = tmpSurf.getPresentationName();
        boolean nameMatchesID=false;
        for(String tmpStr:identList){
          if(surfName.startsWith(tmpStr) || surfName.contains("."+tmpStr)){
            nameMatchesID=true;
          }
        }
        // This part is a possible candidate based on its Surfaces. No need to
        // continue looping through.
        if(nameMatchesID){
          isPart=true;
          retParts.add(tmpPart);
          isPart=false;
          break;
        }
      }
    }
    if(retParts.isEmpty()){
      simu.println("MeshOpTool WRN: findGeometryParts - retParts is empty!");
    }
    return retParts;
  }
  
  public static ArrayList<GeometryPart> filterOutGeometryParts(Simulation simu, Collection<GeometryPart> allParts,
                                                               String idStr){
    /* Filteres out GeometryPart from given list when surfaces do not all
       start with idStr
    */
    ArrayList<GeometryPart> retParts=new ArrayList();
    for(GeometryPart tmpPart:allParts){
        boolean doIkeepPart=false;
        for(PartSurface tmpSurf:tmpPart.getPartSurfaces()){
            String surfName = tmpSurf.getPresentationName();
            if((surfName.startsWith(idStr)||surfName.contains("."+idStr))){
                doIkeepPart=true;
            }else{
                doIkeepPart=false;
                break;
            }
        }
        if(doIkeepPart) retParts.add(tmpPart);
    }
    if(retParts.isEmpty()) simu.println("MeshOpTool: filterOutGeometryParts is empty.");
    return retParts;
  }
public static Collection<GeometryPart> removeMeshOpParts(
    Collection<GeometryPart> theseGeomParts){
  ArrayList<MeshOperationPart> meshOpParts=new ArrayList();
  for(GeometryPart tmpPart:theseGeomParts){ //remove mesh operation parts
    if(tmpPart instanceof MeshOperationPart){
      meshOpParts.add((MeshOperationPart) tmpPart);
    }
  }
  theseGeomParts.removeAll(meshOpParts);
  return theseGeomParts;
}
    
    //Geometric Part Operations
    //unite
    public static UnitePartsOperation uniteParts(Simulation simu,String opName,boolean performOnCAD,boolean remeshOnIntersection){
        UnitePartsOperation unitePBMO;
        try{
            unitePBMO = ((UnitePartsOperation) simu.get(MeshOperationManager.class).getObject(opName));
        }catch(NeoException e){
            unitePBMO  = (UnitePartsOperation) simu.get(MeshOperationManager.class).createUnitePartsOperation(new NeoObjectVector(new Object[] {}));
            unitePBMO.setPresentationName(opName);
            String bodyName =  unitePBMO.getOutputPartNames();
            MeshOperationPart mOP = 
          ((MeshOperationPart) simu.get(SimulationPartManager.class).getPart(bodyName.substring(1,bodyName.length()-1)));
            mOP.setPresentationName(opName);
        }
        unitePBMO.setPerformCADBoolean(performOnCAD);
        if(remeshOnIntersection){
          RemeshIntersectionCurveOption remeshOption = 
            unitePBMO.getBooleanOperationValuesManager().get(RemeshIntersectionCurveOption.class);
          remeshOption.setRemeshIntersectionCurve(true);
        }
        return unitePBMO;
    }
    public static GeometryPart uniteParts(Simulation simu, String opName,boolean performOnCAD,
            boolean remeshOnIntersection,ArrayList<GeometryPart> allParts){
        UnitePartsOperation unitePBMO;
        MeshOperationPart mOP;
        try{
            unitePBMO = ((UnitePartsOperation) simu.
                    get(MeshOperationManager.class).getObject(opName));
        }catch(NeoException e){
            unitePBMO  = (UnitePartsOperation) simu.
                    get(MeshOperationManager.class).
                    createUnitePartsOperation(
                            new NeoObjectVector(new Object[] {}));
            unitePBMO.setPresentationName(opName);
        }
        String bodyName =  unitePBMO.getOutputPartNames();
        mOP = 
            ((MeshOperationPart) simu.get(SimulationPartManager.class)
                    .getPart(bodyName.substring(1,bodyName.length()-1)));
        mOP.setPresentationName(opName);
        unitePBMO.getInputGeometryObjects().setObjects(allParts);
        unitePBMO.setPerformCADBoolean(performOnCAD);
        if(remeshOnIntersection){
          RemeshIntersectionCurveOption remeshOption = 
            unitePBMO.getBooleanOperationValuesManager().get(RemeshIntersectionCurveOption.class);
          remeshOption.setRemeshIntersectionCurve(true);
        }
        return mOP;
    }
    public static GeometryPart getUniteOpPart(Simulation simu, UnitePartsOperation tmpOp){
        GeometryPart retPart;
        // 1,len-1 indicies are because part name returns in brackets
        retPart = simu.get(SimulationPartManager.class)
                .getPart(tmpOp.getOutputPartNames().substring(1,tmpOp.getOutputPartNames().length()-1));
        return retPart;
    }
    
    //subtract
    public static SubtractPartsOperation subtractPartOp(Simulation simu,String opName,String partName,boolean performOnCAD, boolean remeshOnIntersection){
      SubtractPartsOperation subPBMO;
      try{
        subPBMO = ((SubtractPartsOperation) simu.get(MeshOperationManager.class).getObject(opName));
      }catch(NeoException e){
        subPBMO  = (SubtractPartsOperation) simu.get(MeshOperationManager.class).createSubtractPartsOperation(new NeoObjectVector(new Object[] {}));
        subPBMO.setPresentationName(opName);
        String bodyName =  subPBMO.getOutputPartNames();
        MeshOperationPart mOP = 
          ((MeshOperationPart) simu.get(SimulationPartManager.class)
                  .getPart(bodyName.substring(1,bodyName.length()-1)));
        mOP.setPresentationName(partName);
      }
      subPBMO.setPerformCADBoolean(performOnCAD);
      if(remeshOnIntersection){
        RemeshIntersectionCurveOption remeshOption = 
          subPBMO.getBooleanOperationValuesManager().get(RemeshIntersectionCurveOption.class);
        remeshOption.setRemeshIntersectionCurve(true);
      }
      return subPBMO;
    }
    public static SubtractPartsOperation subtractOp(Simulation simu,String opName,String partName,GeometryPart targetPart,Collection<GeometryPart> allOthers){
      SubtractPartsOperation subMeshOp= MeshOpTool.subtractPartOp(simu,opName,partName, true,false);
      subMeshOp.getInputGeometryObjects().setObjects(allOthers);
      subMeshOp.setTargetPart((MeshPart) targetPart);
      return subMeshOp;
    }
    public static GeometryPart getSubtractOpPart(Simulation simu,SubtractPartsOperation tmpOp){
      GeometryPart retPart;
      // 1,len-1 indicies are because part name returns in brackets
      retPart = simu.get(SimulationPartManager.class)
              .getPart(tmpOp.getOutputPartNames().substring(1,tmpOp.getOutputPartNames().length()-1));
      return retPart;
    }
    
    //transforms
    public static TransformPartsOperation rotationTransform(Simulation simu,String opName,GeometryPart onePart,CoordinateSystem cSys, double[] rotAxis){
        TransformPartsOperation transPartsOp;
        try{
            transPartsOp = 
              (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(opName);
        }catch(NeoException e){
            transPartsOp = 
              (TransformPartsOperation) simu.get(MeshOperationManager.class).createTransformPartsOperation(new NeoObjectVector(new Object[] {}));
            transPartsOp.setPresentationName(opName);
        }
        //Empty and reset
        transPartsOp.getInputGeometryObjects().setQuery(null);
        transPartsOp.getInputGeometryObjects().setObjects(onePart);

        //Rotation
        RotationControl rotControl;
        try{
            rotControl = ((RotationControl) transPartsOp.getTransforms().getObject("Rotate"));
            
        }catch(NeoException e){
            rotControl = 
              transPartsOp.getTransforms().createRotationControl();
        }
        rotControl.setCoordinateSystem(cSys);
        Units prefUVec = getPrefUVec(simu);
        rotControl.getAxisVector().setCoordinate(prefUVec, prefUVec, prefUVec,new DoubleVector(rotAxis));
        rotControl.getAngle().setUnits((Units) simu.getUnitsManager().getObject("deg"));
        rotControl.getAngle().setValue(0.0);
        
        
        return transPartsOp;
    }
    public static TransformPartsOperation rotationTransform(Simulation simu,String opName,Collection<GeometryPart> allParts,CoordinateSystem cSys, double[] rotAxis){
        TransformPartsOperation transPartsOp;

        try{
            transPartsOp = 
              (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(opName);
        }catch(NeoException e){
            transPartsOp = 
              (TransformPartsOperation) simu.get(MeshOperationManager.class).createTransformPartsOperation(new NeoObjectVector(new Object[] {}));
            transPartsOp.setPresentationName(opName);
        }
        //Empty and reset
        transPartsOp.getInputGeometryObjects().setQuery(null);
        transPartsOp.getInputGeometryObjects().setObjects(allParts);
        //Rotation
        RotationControl rotControl;
        try{
            rotControl = ((RotationControl) transPartsOp.getTransforms().getObject("Rotate"));
            
        }catch(NeoException e){
            rotControl = 
              transPartsOp.getTransforms().createRotationControl();
        }
        
        rotControl.setCoordinateSystem(cSys);
        Units prefUVec = getPrefUVec(simu);
        rotControl.getAxisVector().setCoordinate(prefUVec, prefUVec, prefUVec,new DoubleVector(rotAxis));
        rotControl.getAngle().setUnits((Units) simu.getUnitsManager().getObject("deg"));
        rotControl.getAngle().setValue(0.0);
        
        
        return transPartsOp;
    }
    public static void setRotationAngle(Simulation simu,String meshOpName,double newValue){
        TransformPartsOperation tmpOp;
        try{
            tmpOp = (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(meshOpName);
            ((RotationControl) tmpOp.getTransforms().getObject("Rotate")).getAngle().setValue(newValue);
        }catch(NeoException e){
            simu.println("MeshOpTool: There is no rotation Mesh Operation entitled "+meshOpName);
        }
    }
    public static double getRotationAngle(Simulation simu, String meshOpName){
      TransformPartsOperation tmpOp;
      double retVal;
      try{
        tmpOp = (TransformPartsOperation) simu
            .get(MeshOperationManager.class).getObject(meshOpName);
        retVal = ((RotationControl) tmpOp.getTransforms()
            .getObject("Rotate")).getAngle().getRawValue();
      }catch(NeoException e){
        simu.println("MeshOpTool: There is no rotation Mesh Operation entitled "
                +meshOpName);
        retVal = -999.;
      }
      return retVal;
    }
    public static void setRotationAxis(Simulation simu,String meshOpName,double[] newAxis){
        TransformPartsOperation tmpOp;
        try{
          tmpOp = (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(meshOpName);
          Units prefUVec = getPrefUVec(simu);
          ((RotationControl) tmpOp.getTransforms().getObject("Rotate"))
                  .getAxisVector().setCoordinate(prefUVec, prefUVec, prefUVec, new DoubleVector(newAxis));
        }catch(NeoException e){
          simu.println("MeshOpTool: There is no rotation Mesh Operation entitled "+meshOpName);
        }
    }
    public static TransformPartsOperation getRotationOperation(Simulation simu,String meshOpName){
        return (TransformPartsOperation) simu.get(MeshOperationManager.class).getObject(meshOpName);
    }

    //Automated Meshing
    public static AutoMeshOperation surfMeshOp(Simulation simu,String opName, double baseSize, double targetSizePct, double minSizePct,
            double nCircle, double sGRValue,boolean autoRepair){
        /* Method setupSurfPBMO
            A method to create or get a particular surface remesh PBM Operation
              Turns off proximity refinement by default
        */
        String selectRemesher="star.resurfacer.ResurfacerAutoMesher";
        String selectAutoRepair="star.resurfacer.AutomaticSurfaceRepairAutoMesher";
        boolean proxRef = false;
        double minTriQual = 0.2; //default best practice for aero
        AutoMeshOperation pBMO;
        
        try{ 
            pBMO = ((AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName));
        }catch(NeoException e){
            if(autoRepair){
                pBMO =
                    simu.get(MeshOperationManager.class).createAutoMeshOperation(new StringVector(new String[] {selectRemesher,selectAutoRepair}), new NeoObjectVector(new Object[] {}));
            }else{
                pBMO =
                    simu.get(MeshOperationManager.class).createAutoMeshOperation(new StringVector(new String[] {selectRemesher}), new NeoObjectVector(new Object[] {}));
            }
            pBMO.setPresentationName(opName);
        }
        ResurfacerAutoMesher remeshModel = ((ResurfacerAutoMesher) pBMO.getMeshers().getObject("Surface Remesher"));
        //Auto Repair model setup
        if(autoRepair){
            AutomaticSurfaceRepairAutoMesher automaticSurfaceRepairAutoMesher_0 = 
             ((AutomaticSurfaceRepairAutoMesher) pBMO.getMeshers().getObject("Automatic Surface Repair"));
            automaticSurfaceRepairAutoMesher_0.setMinimumFaceQuality(minTriQual);
        }
        // Remesher model setup
        if(!proxRef){
            remeshModel.setDoProximityRefinement(proxRef);
        }else{
            remeshModel.setDoProximityRefinement(!proxRef);
        }
        remeshModel.setMinimumFaceQuality(minTriQual);
        pBMO.getDefaultValues().get(BaseSize.class).setValue(baseSize);
            
        // Target Size
        PartsTargetSurfaceSize targetSS = 
            pBMO.getDefaultValues().get(PartsTargetSurfaceSize.class);
        targetSS.setRelativeSizeValue(targetSizePct);
            
        // Minimum Size
        PartsMinimumSurfaceSize partsMinSS = 
          pBMO.getDefaultValues().get(PartsMinimumSurfaceSize.class);
        partsMinSS.setRelativeSizeValue(minSizePct);

        //Surface curvature
        pBMO.getDefaultValues().get(SurfaceCurvature.class).setNumPointsAroundCircle(nCircle);
            
        //Surface growth rate
        SurfaceGrowthRate sGR = pBMO.getDefaultValues().get(SurfaceGrowthRate.class);
        sGR.setGrowthRate(sGRValue);

        return pBMO;
    }
    public static AutoMeshOperation getAutoMeshOp(Simulation simu, 
        String opName){
        return (AutoMeshOperation) simu.get(MeshOperationManager.class).getObject(opName);
    }
    
    //surface controls
    public static SurfaceCustomMeshControl surfControl(AutoMeshOperation pBMO, String controlName){
        /* Method getCustSurf
            Gets or creates a specified custom surface control
        */
        SurfaceCustomMeshControl custControl; 
        try{
            custControl = (SurfaceCustomMeshControl) pBMO.getCustomMeshControls().getObject(controlName);
        }catch(NeoException e){
            custControl= pBMO.getCustomMeshControls().createSurfaceControl();
            custControl.setPresentationName(controlName);
        }

        //custControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);
        return custControl;
    }
    public static void surfCustTargSize(SurfaceCustomMeshControl custControl, String defRelAbs, double newVal){
        custControl.getCustomConditions().get(PartsTargetSurfaceSizeOption.class).setSelected(PartsTargetSurfaceSizeOption.Type.CUSTOM);
        PartsTargetSurfaceSize pTSS = 
          custControl.getCustomValues().get(PartsTargetSurfaceSize.class);
        Units prefUVec = getPrefUVec(custControl.getSimulation());
        if(defRelAbs.equals("Absolute")){
            pTSS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
            pTSS.setAbsoluteSize(newVal, prefUVec);
            
        }else if(defRelAbs.equals("Relative")){
            pTSS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
            pTSS.setRelativeSizeValue(newVal);
        }
    }
    public static void surfCustMinSize(SurfaceCustomMeshControl custControl, String defRelAbs, double newVal){
      custControl.getCustomConditions().get(PartsMinimumSurfaceSizeOption.class).setSelected(PartsMinimumSurfaceSizeOption.Type.CUSTOM);
      CustomMeshControlValueManager pMSSprinter = custControl.getCustomValues();
        
      PartsMinimumSurfaceSize pMSS = custControl.getCustomValues().get(PartsMinimumSurfaceSize.class);
      Units prefUVec = getPrefUVec(custControl.getSimulation());
      if(defRelAbs.equals("Absolute")){
          pMSS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
          pMSS.setAbsoluteSize(newVal, prefUVec);
      }else if(defRelAbs.equals("Relative")){
          pMSS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
          pMSS.setRelativeSize(newVal);
      }
    }
    public static void surfNCircle(SurfaceCustomMeshControl custControl, double nPts){
        custControl.getCustomConditions().get(PartsSurfaceCurvatureOption.class)
          .setSelected(PartsSurfaceCurvatureOption.Type.CUSTOM_VALUES);
        
      CustomMeshControlValueManager pMSSprinter = custControl.getCustomValues();
        
      SurfaceCurvature sCC = 
          custControl.getCustomValues().get(SurfaceCurvature.class);
      sCC.setNumPointsAroundCircle(nPts);
    }
    public static void surfEdgeProx(SurfaceCustomMeshControl custControl, double nFaces){
        custControl.getCustomConditions().get(PartsEdgeProximityOption.class).setSelected(PartsEdgeProximityOption.Type.CUSTOM_VALUES);
      CustomMeshControlValueManager pMSSprinter = custControl.getCustomValues();
      EdgeProximity edgeProx = custControl.getCustomValues().get(EdgeProximity.class);
      edgeProx.setNumDivisions(nFaces);
    }
    public static void surfSurfProx(SurfaceCustomMeshControl custControl, double nPtsGap, boolean isCeil, double ceilVal){
        custControl.getCustomConditions().get(PartsSurfaceProximityOption.class).setSelected(PartsSurfaceProximityOption.Type.CUSTOM_VALUES);
            SurfaceProximity surfaceProximity_0 = 
        custControl.getCustomValues().get(SurfaceProximity.class);
        if(isCeil){
            surfaceProximity_0.setEnableCeiling(isCeil);
            surfaceProximity_0.getFloor().setValue(ceilVal);
        }else{
            surfaceProximity_0.setEnableCeiling(!isCeil);
            surfaceProximity_0.getFloor().setValue(0.0);
        }
        surfaceProximity_0.setNumPointsInGap(nPtsGap);
    }
    public static void surfGrowthRate(SurfaceCustomMeshControl custControl,double sGRValue){
        custControl.getCustomConditions().get(PartsResurfacerSurfaceGrowthRateOption.class).setSelected(PartsResurfacerSurfaceGrowthRateOption.Type.CUSTOM_VALUES);
        SurfaceGrowthRate sGR = 
            custControl.getCustomValues().get(SurfaceGrowthRate.class);
        sGR.setGrowthRate(sGRValue);
    }
    public static void surfFilter(SurfaceCustomMeshControl custControl,int nDigitsToFilter){
        String preChar="";
        for(int i=0;i<=nDigitsToFilter-1;i++){
            preChar = preChar + custControl.getPresentationName().charAt(i);
        }
        custControl.getGeometryObjects().setQuery(new Query(
            new CompoundPredicate(CompoundOperator.And,
            Arrays.asList(
            new CompoundPredicate(CompoundOperator.Or,
                Arrays.asList(
                new NamePredicate(NameOperator.Contains, "."+preChar),
                new NamePredicate(NameOperator.StartsWith, ""+preChar))),
            new NamePredicate(NameOperator.DoesNotEndWith, " TE"),
            new NamePredicate(NameOperator.DoesNotEndWith, " TIP"))),
          Query.STANDARD_MODIFIERS));
    }
    public static void surfFilter_TE(SurfaceCustomMeshControl custControl,int nDigits){
        String preChar="";
        for(int i=0;i<=nDigits-1;i++){
            preChar = preChar + custControl.getPresentationName().charAt(i);
        }
        String postChar = " TE";
        custControl.getGeometryObjects().setQuery(new Query(
          new CompoundPredicate(CompoundOperator.And,
          Arrays.asList(
              new NamePredicate(NameOperator.EndsWith, postChar), 
          new CompoundPredicate(CompoundOperator.Or, 
            Arrays.asList(
              new NamePredicate(NameOperator.Contains, "."+preChar),
              new NamePredicate(NameOperator.StartsWith, preChar))))),Query.STANDARD_MODIFIERS));
    }
    public static void surf2ndWildFilter(SurfaceCustomMeshControl custControl){
        String firstChar = ""+custControl.getPresentationName().charAt(0);
        String thirdChar = ""+custControl.getPresentationName().charAt(2);

        custControl.getGeometryObjects().setQuery(new Query(
          new CompoundPredicate(CompoundOperator.Or,
            Arrays.asList(
              new NamePredicate(NameOperator.Contains, "."+firstChar+"1"+thirdChar), 
              new NamePredicate(NameOperator.Contains, "."+firstChar+"2"+thirdChar),
              new NamePredicate(NameOperator.Contains, "."+firstChar+"3"+thirdChar),
              new NamePredicate(NameOperator.Contains, "."+firstChar+"4"+thirdChar),
              new NamePredicate(NameOperator.StartsWith, firstChar+"1"+thirdChar),
              new NamePredicate(NameOperator.StartsWith, firstChar+"2"+thirdChar),
              new NamePredicate(NameOperator.StartsWith, firstChar+"3"+thirdChar),
              new NamePredicate(NameOperator.StartsWith, firstChar+"4"+thirdChar))),Query.STANDARD_MODIFIERS));
    }

    public static double getSurfCustTargetSize(SurfaceCustomMeshControl custControl){
        custControl.getCustomConditions().get(PartsTargetSurfaceSizeOption.class).setSelected(PartsTargetSurfaceSizeOption.Type.CUSTOM);
        PartsTargetSurfaceSize pTSS = 
          custControl.getCustomValues().get(PartsTargetSurfaceSize.class);
        double retVal=-1;
        try{
            if(pTSS.getRelativeOrAbsoluteOption().getSelectedInput().toString().equals("Absolute")){
                retVal=pTSS.getAbsoluteSizeValue().getInternalValue();
            }else{
                retVal= pTSS.getRelativeSizeValue();
            }
            return retVal;
        }catch(NeoException e){
            return retVal;
        }
    }
    public static double getSurfCustMinSize(SurfaceCustomMeshControl custControl){
        custControl.getCustomConditions().get(PartsMinimumSurfaceSizeOption.class).setSelected(PartsMinimumSurfaceSizeOption.Type.CUSTOM);
        PartsMinimumSurfaceSize pMSS = 
          custControl.getCustomValues().get(PartsMinimumSurfaceSize.class);
        double retVal=-1.;
        try{
            if(pMSS.getRelativeOrAbsolute().toString().equals("Absolute")){
                retVal= pMSS.getAbsoluteSizeValue().getInternalValue();
            }else{
                retVal= pMSS.getRelativeSizeValue();
            }
            return retVal;
        }catch(NeoException e){
            return retVal;
        }
    }
    public static double getSurfCustNPtsOnCircle(SurfaceCustomMeshControl custControl){
        SurfaceCurvature sCC = 
            custControl.getCustomValues().get(SurfaceCurvature.class);
        double retVal=-1.;    
        try{
            retVal=sCC.getNumPointsAroundCircle();
            return retVal;
        }catch(NeoException e){
            return retVal;
        }
    }
    
    //volume meshing controls
    public static PrismThickness surfPrismThick(SurfaceCustomMeshControl custControl, String relAbs,double totThickVal){
        PartsCustomizePrismMesh custPrismMesh = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);
        custPrismMesh.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);
        PartsCustomizePrismMeshControls custPrismControls = 
          custPrismMesh .getCustomPrismControls();

        //Total Thickness Control
        custPrismControls.setCustomizeTotalThickness(true);
        PrismThickness prismThick=custControl.getCustomValues().get(CustomPrismValuesManager.class).get(PrismThickness.class);

        Units prefUVec = getPrefUVec(custControl.getSimulation());
        if(relAbs.equals("Relative")){
            prismThick.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
            prismThick.setRelativeSize(totThickVal);
        }else if(relAbs.equals("Absolute")){
            prismThick.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
            prismThick.setAbsoluteSize(totThickVal, prefUVec);
        }
        return prismThick;

    }

    public static void surfPrismNearWall(SurfaceCustomMeshControl custControl, double newNearWall){
      //  Allow custom settings on surfaces
      PartsCustomizePrismMesh custPrismMesh = custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);
      custPrismMesh.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);
      
      // Set Custome stretching
      PartsCustomizePrismMeshControls custPrismControls = custPrismMesh.getCustomPrismControls();
      custPrismControls.setCustomizeStretching(true);
      CustomPrismValuesManager cPVM = (CustomPrismValuesManager) custControl.getCustomValues().getObject("Custom Prism Values");
      custControl.getCustomValues().get(CustomPrismValuesManager.class)
        .get(PrismWallThickness.class).setValue(newNearWall);
    }
    
    public static void surfPrismOverride(SurfaceCustomMeshControl custControl, boolean doOverrride){
        PartsCustomizePrismMesh custPrismMesh = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);

        custPrismMesh.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);
        PartsCustomizePrismMeshControls custPrismControls = 
          custPrismMesh .getCustomPrismControls();

        // Override Boundary Defaults
        custPrismControls.setOverrideBoundaryDefault(doOverrride);
        CustomPrismValuesManager cPVM = (CustomPrismValuesManager) custControl.getCustomValues().getObject("Custom Prism Values");
        
        if(doOverrride){
            PartsOverrideBoundaryDefault partsOverride = 
              custControl.getCustomValues().get(CustomPrismValuesManager.class).get(PartsOverrideBoundaryDefault.class);
            partsOverride.setOverride(true);
        }
    }
    public static void surfPrismNumLayers(SurfaceCustomMeshControl custControl,int numLay){
      PartsCustomizePrismMesh custPrismMesh = 
          custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);

      custPrismMesh.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);
      PartsCustomizePrismMeshControls custPrismControls = 
        custPrismMesh .getCustomPrismControls();

      //Number of Layers Control
      custPrismControls.setCustomizeNumLayers(true);
        
      CustomPrismValuesManager cPVM = (CustomPrismValuesManager) custControl.getCustomValues().getObject("Custom Prism Values");
      NumPrismLayers numPrismLayers = 
      custControl.getCustomValues().get(CustomPrismValuesManager.class).get(NumPrismLayers.class);

      numPrismLayers.setNumLayers(numLay);
    }
    public static void surfDisablePrisms(SurfaceCustomMeshControl custControl){
        PartsCustomizePrismMesh custPrismMesh = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);
        custPrismMesh.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);
        PartsCustomizePrismMesh partsCustomizePrismMesh_1 = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);
        partsCustomizePrismMesh_1.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.DISABLE);
    }
    
    public static double getCustomPrismThick(SurfaceCustomMeshControl custControl){
        PartsCustomizePrismMesh custPrismMesh = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);
        PartsCustomizePrismMeshControls custPrismControls = 
          custPrismMesh .getCustomPrismControls();
        double retVal=-1;
        if(custPrismControls.getCustomizeTotalThickness()){
            try{
                PrismThickness prismThick=custControl.getCustomValues().get(CustomPrismValuesManager.class).get(PrismThickness.class);
                return prismThick.getAbsoluteSizeValue().getInternalValue();
                
            }catch(NeoException e){
                try{
                    PrismThickness prismThick=custControl.getCustomValues().get(CustomPrismValuesManager.class).get(PrismThickness.class);
                    retVal= prismThick.getRelativeSizeValue();
                    return retVal;
                }catch(NeoException j){
                    return retVal;
                }
            }
        }
        return retVal;
    }
    public static double getCustomPrismNearWall(SurfaceCustomMeshControl custControl){
        PartsCustomizePrismMesh custPrismMesh = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);
        custPrismMesh.getCustomPrismOptions().setSelected(PartsCustomPrismsOption.Type.CUSTOMIZE);
        PartsCustomizePrismMeshControls custPrismControls = 
          custPrismMesh .getCustomPrismControls();
        double retVal=-1;
        if(custPrismControls.getCustomizeStretching()){
            try{
                PrismWallThickness prismWallThick=custControl.getCustomValues().get(CustomPrismValuesManager.class).get(PrismWallThickness.class);
                retVal = prismWallThick.getSIValue();
                return retVal;
            }catch(NeoException e){
                return retVal;
            }
        }
        return retVal;
    }
    public static int getCustomNumPrismLayers(SurfaceCustomMeshControl custControl){
        PartsCustomizePrismMesh custPrismMesh = 
            custControl.getCustomConditions().get(PartsCustomizePrismMesh.class);

        PartsCustomizePrismMeshControls custPrismControls = 
          custPrismMesh .getCustomPrismControls();
         //Number of Layers Control
        custPrismControls.setCustomizeNumLayers(true);
        
        int retVal=-1;
        if(custPrismControls.getCustomizeNumLayers()){
            try{
                NumPrismLayers numPrismLayers = 
                custControl.getCustomValues().get(CustomPrismValuesManager.class).get(NumPrismLayers.class);
                retVal = numPrismLayers.getNumLayers();
                return retVal;
            }catch(NeoException e){
                return retVal;
            }
        }
        return retVal;
    }
    
    public static void activateWakeVolumeControl(CoordinateSystem localCsys,
      SurfaceCustomMeshControl custControl,double wakeLength,double isoSize,double spreadAngle){
      //
      Simulation tmpSim = custControl.getSimulation();
      PartsTrimmerWakeRefinementOption partsTrimmerWakeRefinementOption_0 = 
        custControl.getCustomConditions().get(PartsTrimmerWakeRefinementOption.class);
      partsTrimmerWakeRefinementOption_0.setPartsWakeRefinementOption(true);
      custControl.getCustomValues().get(PartsWakeRefinementValuesManager.class).setCoordinateSystem(localCsys);
      custControl.getCustomValues()
        .get(PartsWakeRefinementValuesManager.class).getDirection().setComponents(-1.0, 0.0, 0.0);
      custControl.getCustomValues().get(PartsWakeRefinementValuesManager.class).getDistance().setValue(wakeLength);
      Units units_0 = 
        ((Units) tmpSim.getUnitsManager().getObject("deg"));
      custControl.getCustomValues().get(PartsWakeRefinementValuesManager.class).getSpreadAngle().setUnits(units_0);
      custControl.getCustomValues().get(PartsWakeRefinementValuesManager.class).getSpreadAngle().setValue(spreadAngle);
      PartsRelativeWakeRefinementSize partsRelativeWakeRefinementSize_0 = 
        custControl.getCustomValues().get(PartsWakeRefinementValuesManager.class).getRelativeRefSize();
      partsRelativeWakeRefinementSize_0.setPercentage(isoSize);
      PartsTrimmerWakeRefinementSet partsTrimmerWakeRefinementSet_0 = 
        custControl.getCustomValues().get(PartsWakeRefinementValuesManager.class).get(PartsTrimmerWakeRefinementSet.class);
      partsTrimmerWakeRefinementSet_0.getGrowthRateOption().setSelected(PartsGrowthRateOption.Type.VERYSLOW);
      partsTrimmerWakeRefinementSet_0.getSurfaceGrowthRateOption().setSelected(PartsSurfaceGrowthRateOption.Type.VERYSLOW);
    }

    // Overset Addition
    public static void custVCIsoSize(VolumeCustomMeshControl volControl, String defRelAbs, double newVal){
      // Enable Surface Remeshing Volume Control
      VolumeControlTrimmerSizeOption sizeOption = 
        volControl.getCustomConditions().get(VolumeControlTrimmerSizeOption.class);
      sizeOption.setVolumeControlBaseSizeOption(true);
      sizeOption.setTrimmerAnisotropicSizeOption(false);
      Units prefUVec = getPrefUVec(volControl.getSimulation());
      if(defRelAbs.equals("Absolute")){
          VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
          vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.ABSOLUTE);
          vCS.setAbsoluteSizeValue(newVal, prefUVec);
      }else if(defRelAbs.equals("Relative")){
          VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
          vCS.getRelativeOrAbsoluteOption().setSelected(RelativeOrAbsoluteOption.Type.RELATIVE);
          vCS.setRelativeSizeValue(newVal);
      }
      volControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);
     }
    public static double getVCAbsoluteIsoSize(VolumeCustomMeshControl volControl){
      // makes a projected cell count based on the assumption of a cubic volume control
      VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
      return vCS.getAbsoluteSizeValue().getInternalValue(); // m
    }
    public static double getCustVCIsoRelSize(VolumeCustomMeshControl volControl){
        // Enable Surface Remeshing Volume Control
        String relOrAbs=volControl.getCustomValues().get(VolumeControlSize.class).getRelativeOrAbsolute().toString();
        if(relOrAbs.contains("Relative")){
            VolumeControlSize vCS = volControl.getCustomValues().get(VolumeControlSize.class);
            return vCS.getRelativeSizeValue();
        }else{
            return 1e38;
        }
    }
    public static boolean isVC(CustomMeshControl thisControl){
      return thisControl instanceof VolumeCustomMeshControl;
    }
    
    public static VolumeCustomMeshControl getVolumeControl(AutoMeshOperation pBMO,String controlName){
        /* Method getCustSurf
            Gets or creates a specified custom surface control
        */
        VolumeCustomMeshControl volControl;
        try{
            volControl = (VolumeCustomMeshControl) pBMO.getCustomMeshControls().getObject(controlName);
        }catch(NeoException e){
            volControl= pBMO.getCustomMeshControls().createVolumeControl();
            volControl.setPresentationName(controlName);
        }
        volControl.getDisplayMode().setSelected(HideNonCustomizedControlsOption.Type.CUSTOMIZED);
        return volControl;
    }
    public static void setVCNumPrisms(VolumeCustomMeshControl volControl,int newNum){
        VolumeControlPrismsOption custPrismOpt
                = volControl.getCustomConditions()
                        .get(VolumeControlPrismsOption.class);
        custPrismOpt.setCustomizeNumLayers(true);
        NumPrismLayers numPrismLayers = 
            volControl.getCustomValues().get(CustomPrismValuesManager.class)
                    .get(NumPrismLayers.class);
        numPrismLayers.setNumLayers(newNum);
    }
    public static void setVCPrismNearWallThick(VolumeCustomMeshControl volControl,double newVal){
        VolumeControlPrismsOption custPrismOpt
                = volControl.getCustomConditions()
                        .get(VolumeControlPrismsOption.class);
        custPrismOpt.setCustomizeStretching(true);
        volControl.getCustomValues().get(CustomPrismValuesManager.class)
                .get(PrismWallThickness.class).setValue(newVal);
    }
    public static void setVCPrismTotalRelativeThickness(VolumeCustomMeshControl volControl,double newVal){
        VolumeControlPrismsOption custPrismOpt
                = volControl.getCustomConditions()
                        .get(VolumeControlPrismsOption.class);
        custPrismOpt.setCustomizeTotalThickness(true);
        PrismThickness prismThickness = 
             volControl.getCustomValues()
                     .get(CustomPrismValuesManager.class).get(PrismThickness.class);
        prismThickness.setRelativeSizeValue(newVal);

    }

  public static boolean isAutoMeshOpUpToDate(AutoMeshOperation autoOp){
    try{
      boolean retVal=!autoOp.isDirty();
      return retVal;
    }catch(NeoException e){
      return false;
    }
  }
  public static boolean areVolumeMeshOpsUpToDate(Simulation tmpSim,ArrayList<AutoMeshOperation> opsList){
    boolean allMeshOpsUpToDate=true;
    for(AutoMeshOperation tmpOp:opsList){
      String meshOpName = tmpOp.getPresentationName();
      if(!isAutoMeshOpUpToDate(tmpOp)){
        tmpSim.println("    WARNING: "+meshOpName+" DOES NOT APPEAR UP TO DATE");
        allMeshOpsUpToDate=false;
      }
    }
    return allMeshOpsUpToDate;
  }

  
  
}
