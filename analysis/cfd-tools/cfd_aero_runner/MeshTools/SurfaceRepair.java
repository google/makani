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
package MeshTools;


import Tools.SimTool;
import java.util.*;
import star.common.*;
import star.base.neo.*;
import star.meshing.*;
import star.vis.*;


/**
 */
public class SurfaceRepair {

  public static double getApproximateGeometryPartVolume(Simulation tmpSim, Scene tmpScene, GeometryPart tmpPart){
    // This representation will always be called "Geometry"
    PartRepresentation partRepresentation_0 = 
      ((PartRepresentation) tmpSim.getRepresentationManager().getObject("Geometry"));
    
    // It is a requirement of the surface repair tool to use a Scene because it is a widget
    PartSurfaceMeshWidget partSurfaceMeshWidget_1 = 
      partRepresentation_0.startSurfaceMeshWidget(tmpScene);

    // Only care about root
    RootDescriptionSource rootDescriptionSource = 
      ((RootDescriptionSource) tmpSim.get(SimulationMeshPartDescriptionSourceManager.class).getObject("Root"));

    partSurfaceMeshWidget_1.setActiveParts(new NeoObjectVector(new Object[] {tmpPart}), rootDescriptionSource);
    partSurfaceMeshWidget_1.startSurfaceRepairControllers();

    //Open surface repair tool for this part - no faces for speed
    SurfaceMeshWidgetDisplayController surfaceMeshWidgetDisplayController_1 = 
      partSurfaceMeshWidget_1.getControllers().getController(SurfaceMeshWidgetDisplayController.class);
    surfaceMeshWidgetDisplayController_1.hideAllFaces();

    SurfaceMeshWidgetSelectController surfaceMeshWidgetSelectController_0 = 
      partSurfaceMeshWidget_1.getControllers().getController(SurfaceMeshWidgetSelectController.class);

    ArrayList<PartSurface> tmpList = new ArrayList();
    tmpList.addAll(tmpPart.getPartSurfaces());
    surfaceMeshWidgetSelectController_0.selectPartSurfaces(tmpList);

    SurfaceMeshWidgetQueryController surfaceMeshWidgetQueryController_0 = 
      partSurfaceMeshWidget_1.getControllers().getController(SurfaceMeshWidgetQueryController.class);
    
    double approxVol = surfaceMeshWidgetQueryController_0.queryFaceVolume().getDouble("FaceVolume");
    partSurfaceMeshWidget_1.stop();
    return approxVol;
  }
  
  
  
}
