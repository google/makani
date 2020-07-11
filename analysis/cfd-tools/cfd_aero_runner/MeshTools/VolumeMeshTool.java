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

import java.io.*;
import java.util.*;
import star.base.neo.*;
import star.common.*;
import star.vis.*;
import star.meshing.*;

import Tools.*;

public class VolumeMeshTool {
  
  // Mesh Count Predictor
  public static int predictVolumeMeshSize(Simulation tmpSim,ArrayList<AutoMeshOperation> volMeshOps){
    int projectedMeshCount = 0;
    String predictedBreakdown = "";
    for(AutoMeshOperation tmpOp:volMeshOps){
      predictedBreakdown += reportMeshOpVCBreakdown(tmpSim, tmpOp);
    }
    tmpSim.println("Predicted Breakdown of Mesh Count: ");
    tmpSim.println(predictedBreakdown);
    return projectedMeshCount;
  }
  public static String reportMeshOpVCBreakdown(Simulation tmpSim, AutoMeshOperation tmpOp){
    // Volume Mesh is Comprised of Prism Layers + Volume Control Operations
    
    // ArrayList opVCs;
    ArrayList<VolumeCustomMeshControl> opVCs = new ArrayList();
    for(CustomMeshControl tmpControl:tmpOp.getCustomMeshControls().getObjects()){
      if(MeshOpTool.isVC(tmpControl)) opVCs.add((VolumeCustomMeshControl) tmpControl);
    }
    // report out the approximate mesh requested from this VC
    Scene tmpScene = SceneTool.getScene(tmpSim, "Surface Repair Temporary Scene");
    String vcBreakdown = "";
    
    for(VolumeCustomMeshControl tmpVC:opVCs){
      String vcName = tmpVC.getPresentationName();
      double cellSizeInM = MeshOpTool.getVCAbsoluteIsoSize(tmpVC);
      double partVolume=0.;
      // get total volume for this volume control
      for(GeometryPart tmpPart:tmpVC.getGeometryObjects().getLeafParts()){
        partVolume += SurfaceRepair.getApproximateGeometryPartVolume(tmpSim, tmpScene, tmpPart);
      }
      int approxCells = (int) ((partVolume + 1e-20)/cellSizeInM);
      // Build the total breakdown
      vcBreakdown += vcName+","+approxCells+"\n";
    }
    
    //Don't need the temporary scene anymore
    SceneTool.deleteScene(tmpScene);
    
    tmpSim.println("Cell Count Breakdown for Volume Mesh Operation: "+tmpOp.getPresentationName());
    tmpSim.println(vcBreakdown);
    
    return vcBreakdown;
  }
  
  // Volume Mesh Information
  public static long getVolumeMeshCount(Simulation tmpSim, Collection<Region> tmpRegs){
    FvRegionManager tmpFvRegManager = ((FvRepresentation) tmpSim.getRepresentationManager()
      .getObject("Volume Mesh")).getFvRegionManager();
    long cellCount = 0;
    for(Region tmpReg:tmpRegs){
      cellCount = cellCount + tmpFvRegManager.getFvRegion(tmpReg).getCellCount();
    }
    return cellCount;
  }

  // Derived Parts for Mesh Quality
  public static ArrayList<Part> makeMeshQualityDerivedParts(Simulation tmpSim,Collection <Region> tmpRegs){
    // hard coded poor quality cell metrics
    double highSkewnessAngle = 95.0;
    double lowCellQuality = 1e-4;
    double lowCellVolume = 1e-6;
    double lowFaceValidity = 0.95;

    ArrayList<Part> checkCells = new ArrayList();
    checkCells.add(DerivedPartTool.highValueCells(tmpSim,tmpRegs,
      "Metric High Skewness Angle","SkewnessAngle",highSkewnessAngle));
    checkCells.add(DerivedPartTool.lowValueCells(tmpSim,tmpRegs,
      "Metric Low Cell Quality","CellQuality",lowCellQuality));
    checkCells.add(DerivedPartTool.lowValueCells(tmpSim,tmpRegs,
      "Metric Low Cell Volume","Volume",lowCellVolume));
    checkCells.add(DerivedPartTool.lowValueCells(tmpSim,tmpRegs,
      "Metric Low Face Validity","FaceValidity",lowFaceValidity));
    
    return checkCells;
  }
  
  // Prism Layer Parts
  public static Part getPrismLayerCellsPart(Simulation tmpSim,Collection <Region> tmpRegs){
    double notPrismLayerValue = 0.0;
    return DerivedPartTool.highValueCells(tmpSim,tmpRegs,"Prism Layer Cells","PrismLayerCells",notPrismLayerValue);
  }
  
}
