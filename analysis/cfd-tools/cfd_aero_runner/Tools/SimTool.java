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
import java.math.*;
import star.base.neo.*;
import star.common.*;
import star.flow.*;

import Naming.*;
import star.base.report.*;

public class SimTool {
  static final double SMALL_EPS=1.0E-6;
  
  private static LabCoordinateSystem getLabCsys(Simulation tmpSim){
    return tmpSim.getCoordinateSystemManager().getLabCoordinateSystem();
  }
  public static Units getUnitsVec(Simulation tmpSim){
      return tmpSim.getUnitsManager().getPreferredUnits(new IntVector(new int[] 
    {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
          0, 0, 0, 0, 0, 0, 0, 0, 0, 0}));
  }

  //PUBLIC METHODS
  // Parameters
  public static ScalarGlobalParameter getScalarSimulationParameter(
          Simulation tmpSim, String paramName){
    ScalarGlobalParameter scalarParameter;
    try{
      scalarParameter = ((ScalarGlobalParameter) tmpSim.
              get(GlobalParameterManager.class).getObject(paramName));
    }catch(NeoException e){
      tmpSim.get(GlobalParameterManager.class).
              createGlobalParameter(ScalarGlobalParameter.class, paramName);
      scalarParameter = ((ScalarGlobalParameter) tmpSim.
              get(GlobalParameterManager.class).getObject(paramName));
      scalarParameter.getQuantity().setValue( 0.0 );
    }
    return scalarParameter;
  }
  public static VectorGlobalParameter getVectorSimulationParameter(
          Simulation tmpSim, String paramName){
    VectorGlobalParameter vectorParameter;
    try{
      vectorParameter = ((VectorGlobalParameter) tmpSim.
              get(GlobalParameterManager.class).getObject(paramName));
    }catch(NeoException e){
      tmpSim.get(GlobalParameterManager.class).
              createGlobalParameter(VectorGlobalParameter.class, paramName);
      vectorParameter = ((VectorGlobalParameter) tmpSim.
              get(GlobalParameterManager.class).getObject(paramName));
      vectorParameter.getQuantity().setDefinition("[0., 0., 0.]");
    }
    return vectorParameter;
  }

  //Coordinate Systems
  public static CartesianCoordinateSystem useLabBasisForLocalBasis(
          CartesianCoordinateSystem labBasedCsys,
          CartesianCoordinateSystem reBasedCsys){
      // useLabBasisForLocalBasis simply takes the labBasedCsys basis vectors
      //   and puts the x-basis vector in the same value direction but in the
      //   body coordinate basis set
      reBasedCsys.setBasis0(labBasedCsys.getBasis0());
      reBasedCsys.setBasis1(labBasedCsys.getBasis1());

      reBasedCsys.getOrigin().setCoordinate(
              labBasedCsys.getOrigin().getUnits0(),
              labBasedCsys.getOrigin().getUnits0(), 
              labBasedCsys.getOrigin().getUnits0(),
              labBasedCsys.getOriginVector());

      return reBasedCsys;
  }
  
  public static CartesianCoordinateSystem equalLevelCoordToNestedCoord(
      Simulation tmpSim, CartesianCoordinateSystem currentBasis,
      CartesianCoordinateSystem toEquallyNestedBasis){
    // equalLevelCoordToNestedCoord moves the coordinate basis currentBasis
    // into the toEquallyNestedBasis basis set such that it does not experience
    // rotation relative to the observer standing in the currentBasis 
    // parent-based frame.
    
    // need local Csys rotation matrix inverse
    double[][] rotBodyToLab = 
        get3x3Inverse(getCoordBasisSet(toEquallyNestedBasis));
    double[][] identMat = {{1.,0.,0.},{0.,1.,0.},{0.,0.,1.}};
  
    // lab based rotation matrix
    double[][] xyzBasis = getCoordBasisSet(currentBasis);
    
    // effect of 2 rotation matrices for new basis
    // stay exact same orientation regardless of body orientation
    double[][] prod = multiply3x3Matrices(rotBodyToLab, xyzBasis);
  
    // assume body and original system should rotate as one
     double[] newXBodyBasis = {prod[0][0],prod[1][0],prod[2][0]};
     double[] newYBodyBasis = {prod[0][1],prod[1][1],prod[2][1]};

     //note: all have same basis orientations
     //r_c/l+r_b/c=r_b/l
     //r_c/b=r_cl-r_b/l
     double[] r_cl = currentBasis.getOrigin().getVector().toDoubleArray();
     double[] r_bl = toEquallyNestedBasis.getOrigin().getVector().toDoubleArray();

      double dX=r_cl[0]-r_bl[0];
      double dY=r_cl[1]-r_bl[1];
      double dZ=r_cl[2]-r_bl[2];
      // the r_c/b vector in LAB BASIS SET

      double[] r_cb={dX,dY,dZ};
      // check body basis orientation
      double[][] bodyBasis = getCoordBasisSet(toEquallyNestedBasis);
      double mX=1.0; double mY=1.0; double mZ=1.0;
      if(bodyBasis[0][0]<0) mX=-1.0;
      if(bodyBasis[1][1]<0) mY=-1.0;
      if(bodyBasis[2][2]<0) mZ=-1.0;

      r_cb[0]=mX*r_cb[0];
      r_cb[1]=mY*r_cb[1];
      r_cb[2]=mZ*r_cb[2];

      CartesianCoordinateSystem newCoord=
              getNestedCoordinate(toEquallyNestedBasis,
                  currentBasis.getPresentationName());
      modifyLocalCoordinate(
          tmpSim, newCoord, newXBodyBasis, newYBodyBasis, r_cb);

      return newCoord;

  }
  
  
  
  
  public static CartesianCoordinateSystem labBasedCoordToBodyCoord(
      Simulation tmpSim, CartesianCoordinateSystem labBasedCsys,
      CartesianCoordinateSystem bodyCsys){
      // labBasedCoordToBodyCoord moves a lab based coordinate basis set into
      //   the named bodyCsys basis set such that it does not experience
      //   rotation relative to the observer standing in the lab based frame

      // need local Csys rotation matrix inverse
      double[][] rotBodyToLab = get3x3Inverse(getCoordBasisSet(bodyCsys));
      double[][] identMat = {{1.,0.,0.},{0.,1.,0.},{0.,0.,1.}};

      // lab based rotation matrix
      double[][] xyzBasis = getCoordBasisSet(labBasedCsys);

      // effect of 2 rotation matrices for new basis
      // stay exact same orientation regardless of body orientation
      double[][] prod = multiply3x3Matrices(rotBodyToLab,xyzBasis);

      // assume body and original system should rotate as one
      double[] newXBodyBasis = {prod[0][0],prod[1][0],prod[2][0]};
      double[] newYBodyBasis = {prod[0][1],prod[1][1],prod[2][1]};

      //note: all have same basis orientations
      //r_c/l+r_b/c=r_b/l
      //r_c/b=r_cl-r_b/l
      double[] r_cl = labBasedCsys.getOrigin().getVector().toDoubleArray();
      double[] r_bl = bodyCsys.getOrigin().getVector().toDoubleArray();

      double dX=r_cl[0]-r_bl[0];
      double dY=r_cl[1]-r_bl[1];
      double dZ=r_cl[2]-r_bl[2];
      // the r_c/b vector in LAB BASIS SET

      double[] r_cb={dX,dY,dZ};
      // check body basis orientation
      double[][] bodyBasis = getCoordBasisSet(bodyCsys);
      double mX=1.0; double mY=1.0; double mZ=1.0;
      if(bodyBasis[0][0]<0) mX=-1.0;
      if(bodyBasis[1][1]<0) mY=-1.0;
      if(bodyBasis[2][2]<0) mZ=-1.0;

      r_cb[0]=mX*r_cb[0];
      r_cb[1]=mY*r_cb[1];
      r_cb[2]=mZ*r_cb[2];

      CartesianCoordinateSystem newCoord=
              getNestedCoordinate(bodyCsys, labBasedCsys.getPresentationName());
      modifyLocalCoordinate(tmpSim, newCoord,newXBodyBasis,newYBodyBasis,r_cb);

      return newCoord;

  }
  
  public static double[][] getBasisSet(CartesianCoordinateSystem tmpCsys){
      double[] xBasis = tmpCsys.getBasis0().toDoubleArray();
      double[] yBasis = tmpCsys.getBasis1().toDoubleArray();
      double[] zBasis = tmpCsys.getBasis2().toDoubleArray();

      double[][] retMat = new double[3][3];
      retMat[0][0]=xBasis[0];retMat[0][1]=yBasis[0];retMat[0][2]=zBasis[0];
      retMat[1][0]=xBasis[1];retMat[1][1]=yBasis[1];retMat[1][2]=zBasis[1];
      retMat[2][0]=xBasis[2];retMat[2][1]=yBasis[2];retMat[2][2]=zBasis[2];
      return retMat;
  }
    
  public static CartesianCoordinateSystem getNestedCoordinate(
          CoordinateSystem majCsys,String csysName){
      CartesianCoordinateSystem tmpCsys;
      try{
          tmpCsys= getLocalCartCoord(majCsys,csysName);
      }catch(NeoException e){
          tmpCsys = 
            majCsys.getLocalCoordinateSystemManager()
                    .createLocalCoordinateSystem(
                            CartesianCoordinateSystem.class, "Cartesian");
          tmpCsys.setPresentationName(csysName);
      }
      return tmpCsys;
  }
  
  public static CylindricalCoordinateSystem getNestedCylindricalCoordinate(
          CoordinateSystem majCsys,String csysName){
      CylindricalCoordinateSystem tmpCsys;
      try{
          tmpCsys= getLocalCylCoord(majCsys,csysName);
      }catch(NeoException e){
          tmpCsys = 
            majCsys.getLocalCoordinateSystemManager()                    
                    .createLocalCoordinateSystem(
                            CylindricalCoordinateSystem.class, csysName);
          tmpCsys.setPresentationName(csysName);
      }
      return tmpCsys;
  }
  
  
  public static CartesianCoordinateSystem getLabBasedCoordinate(
          Simulation tmpSim, String csysName){
      return getLabBasedCartCoord(tmpSim,csysName);
  }
  
  
  public static CartesianCoordinateSystem getLabBasedRotorCoordinate(
          Simulation tmpSim, String rotorName){
      Collection<CoordinateSystem> allLocals =
              getLabCsys(tmpSim).getLocalCoordinateSystemManager().getObjects();
      for(CoordinateSystem tmpCoord:allLocals){
          if(tmpCoord instanceof CartesianCoordinateSystem){
              String tmpName = tmpCoord.getPresentationName();
              if(tmpName.startsWith("RTR_"+rotorName)){
                  return (CartesianCoordinateSystem) tmpCoord;
              }                
          }
      }
      tmpSim.println("LabBasedRotorCoord: Returning null. Expect to fail!");
      return null;
  }
  public static void rotateLabBasedCoordinateSystem(Simulation tmpSim,
          double newAngle,double[] aboutAxis,CoordinateSystem thisCsys){
      Units prefUVec = getUnitsVec(tmpSim);
      if(Math.abs(newAngle) > SMALL_EPS ){
          getLabCsys(tmpSim).getLocalCoordinateSystemManager().
                  rotateLocalCoordinateSystems(new NeoObjectVector(
                          new Object[] {thisCsys}),
                      new DoubleVector(aboutAxis),
                      new NeoObjectVector(
                              new Object[] {prefUVec, prefUVec, prefUVec})
                      , newAngle*Math.PI/180., getLabCsys(tmpSim));
      }
  }
  public static void rotateCoordinateSystem(
      Simulation tmpSim, double newAngleInDegrees, double[] aboutAxis,
      CoordinateSystem inThisCsys, CoordinateSystem rotatedCsys){
    Units prefUVec = getUnitsVec(tmpSim);
    if(Math.abs(newAngleInDegrees) > SMALL_EPS){
      rotatedCsys.getLocalCoordinateSystemManager()
          .rotateLocalCoordinateSystems(
              new NeoObjectVector(new Object[] {rotatedCsys}),
                  new DoubleVector(aboutAxis),
                  new NeoObjectVector(
                      new Object[] {prefUVec, prefUVec, prefUVec}),
                  newAngleInDegrees * Math.PI / 180.0, inThisCsys);
    }
  }

  // FIELD FUNCTIONS
  public static UserFieldFunction getUserFF(Simulation tmpSim,
          String displayName){
    UserFieldFunction retFF;
    try{
      retFF = (UserFieldFunction) tmpSim.getFieldFunctionManager().
              getObject(displayName);
    }catch(NeoException e){
      retFF = tmpSim.getFieldFunctionManager().createFieldFunction();
      retFF.setPresentationName(displayName);
    }
    return retFF;
  }
  
  public static UserFieldFunction getUserVectorFF(Simulation tmpSim,
          String displayName){
    UserFieldFunction retFF = getUserFF(tmpSim, displayName);
    retFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.VECTOR);
    return retFF;
  }
  
  public static UserFieldFunction getUserScalarFF(Simulation tmpSim,
          String displayName){
    UserFieldFunction retFF = getUserFF(tmpSim, displayName);
    retFF.getTypeOption().setSelected(FieldFunctionTypeOption.Type.SCALAR);
    return retFF;
  }
  //important field functions!
  public static PressureCoefficientFunction setPressureCoefficientFF(
          Simulation tmpSim, double refRho,double refPress,double refVel){
      PressureCoefficientFunction pCF = 
        ((PressureCoefficientFunction) tmpSim.getFieldFunctionManager()
                .getFunction("PressureCoefficient"));
      pCF.getReferenceDensity().setValue(refRho);
      pCF.getReferencePressure().setValue(refPress);
      pCF.getReferenceVelocity().setValue(refVel);
      return pCF;
  }
  public static SkinFrictionCoefficientFunction setSkinFrictionCoefficientFF(
          Simulation tmpSim,double refRho,double refVel){
      SkinFrictionCoefficientFunction sFCF = 
          ((SkinFrictionCoefficientFunction) tmpSim.getFieldFunctionManager().
                  getFunction("SkinFrictionCoefficient"));
      sFCF.getReferenceDensity().setValue(refRho);
      sFCF.getReferenceVelocity().setValue(refVel);
      return sFCF;
  }

  //Representations
  public static ProxyRepresentation getSimProxy(Simulation tmpSim,
          String proxyName){
      ProxyRepresentation myProxy;
      try{
          myProxy = (ProxyRepresentation) tmpSim.getRepresentationManager().
                  getObject(proxyName);
      }catch(NeoException e){
          myProxy =  tmpSim.getRepresentationManager().
                  createUserRepresentation();
          myProxy.setPresentationName(proxyName);
      }
      return myProxy;
  }

  //Pure math
  public static void simPrint1x3Array(Simulation tmpSim, double[] vec){
    tmpSim.println("" + vec[0] + "," + vec[1] + "," + vec[2]);
  }  
  
  public static void simPrint3x3Matrix(Simulation tmpSim, double[][] mat){
      tmpSim.println(" " + mat[0][0] + ", " + mat[0][1] + ", " + mat[0][2]);
      tmpSim.println(" " + mat[1][0] + ", " + mat[1][1] + ", " + mat[1][2]);
      tmpSim.println(" " + mat[2][0] + ", " + mat[2][1] + ", " + mat[2][2]);
  }
  public static double get3x3Determinant(double[][] mat){
      double det;
      det=mat[0][0]*mat[1][1]*mat[2][2]+mat[0][1]*mat[1][2]*mat[2][0]+mat[0][2]*mat[1][0]*mat[2][1]
         -mat[2][0]*mat[1][1]*mat[0][2]-mat[2][1]*mat[1][2]*mat[0][0]-mat[2][2]*mat[1][0]*mat[0][1];
      return det;
  }
  public static double[] transformVector(double[][] mat,double[] vect){
      double[] transformed = {mat[0][0]*vect[0]+mat[0][1]*vect[1]+mat[0][2]*vect[2],
                              mat[1][0]*vect[0]+mat[1][1]*vect[1]+mat[1][2]*vect[2],
                              mat[2][0]*vect[0]+mat[2][1]*vect[1]+mat[2][2]*vect[2]};
      return transformed;
  }
  public static double[][] get3x3Inverse(double[][] mat){
      double det = get3x3Determinant(mat);
      //Cofactor
      //
      double a11 = (mat[1][1]*mat[2][2]-mat[2][1]*mat[1][2]);
      double a12 =-(mat[1][0]*mat[2][2]-mat[2][0]*mat[1][2]);
      double a13 = (mat[1][0]*mat[2][1]-mat[2][0]*mat[1][1]);
      //
      double a21 =-(mat[0][1]*mat[2][2]-mat[2][1]*mat[1][2]);
      double a22 = (mat[0][0]*mat[2][2]-mat[2][0]*mat[0][2]);
      double a23 =-(mat[0][0]*mat[2][1]-mat[2][0]*mat[0][1]);
      //
      //
      double a31 = (mat[0][1]*mat[1][2]-mat[1][1]*mat[0][2]);
      double a32 =-(mat[0][0]*mat[1][2]-mat[1][0]*mat[0][2]);
      double a33 = (mat[0][0]*mat[1][1]-mat[1][0]*mat[0][1]);

      //Inverse
      double[][] invMat ={{a11/det,a21/det,a31/det},
                          {a12/det,a22/det,a32/det},
                          {a13/det,a23/det,a33/det}};
      return invMat;
  }
  
  public static double[][] get3x3Transpose(double[][] mat){
    
    double[][] retMat = new double[3][3];
    retMat[0][0] = mat[0][0]; retMat[0][1] = mat[1][0]; retMat[0][2] = mat[2][0]; 
    retMat[1][0] = mat[0][1]; retMat[1][1] = mat[1][1]; retMat[1][2] = mat[2][1]; 
    retMat[2][0] = mat[0][2]; retMat[2][1] = mat[1][2]; retMat[2][2] = mat[2][2];
    return retMat;
    
  }
  
  
  public static double[][] multiply3x3Matrices(double[][] a, double[][] b){
      double[][] c = new double[3][3];
      for (int i=0; i<3; i++) {
          for(int k=0;k<3;k++){
              double sum = 0;
              for(int j=0;j<3;j++){
                  sum=sum+a[i][j]*b[j][k];
              }
              c[i][k]=sum;
          }
      }
      return c;
  }
   public static double[][] getR_X(double angle){
   double [][] retMat = new double[3][3];
   retMat[0][0] = 1; retMat[0][1] =                0; retMat[0][2] =               0;
   retMat[1][0] = 0; retMat[1][1] =  Math.cos(angle); retMat[1][2] = Math.sin(angle);
   retMat[2][0] = 0; retMat[2][1] = -Math.sin(angle); retMat[2][2] = Math.cos(angle);
   return retMat;
 }
 
 public static double[][] getR_Y(double angle){
   double [][] retMat = new double[3][3];
   retMat[0][0] = Math.cos(angle); retMat[0][1] = 0; retMat[0][2] = -Math.sin(angle);
   retMat[1][0] =               0; retMat[1][1] = 1; retMat[1][2] =                0;
   retMat[2][0] = Math.sin(angle); retMat[2][1] = 0; retMat[2][2] =  Math.cos(angle);
   return retMat;
 }
 
  public static double[][] getR_Z(double angle){
   double [][] retMat = new double[3][3];
   retMat[0][0] =  Math.cos(angle); retMat[0][1] =  Math.sin(angle); retMat[0][2] = 0;
   retMat[1][0] = -Math.sin(angle); retMat[1][1] =  Math.cos(angle); retMat[1][2] = 0;
   retMat[2][0] =                0; retMat[2][1] =                0; retMat[2][2] = 1;
   return retMat;
 }
  // Statistics
  public static double getMeanReportItVal(Report tmpRep, String itPostFix, int nSamples){
      String repName=tmpRep.getPresentationName();
      String monName=repName+itPostFix;
      return MonitorTool.getLastMonitorSamplesMean(tmpRep.getSimulation(),monName,nSamples);
  }
  public static double getMeanReportStddevItVal(Report tmpRep, String itPostFix, int nSamples,double meanVal){
      String repName=tmpRep.getPresentationName();
      String monName=repName+itPostFix;
      return MonitorTool.getLastMonitorSamplesStDev(tmpRep.getSimulation(),monName,nSamples,meanVal);
  }

  //FUN WITH COORDINATE SYSTEMS
  public static void modifyLocalCoordinate(Simulation tmpSim,
          CartesianCoordinateSystem  nestedCsys,
          double[] newXAxis , double[] newYAxis, double[] axisOrigin){
      Units prefUVec = getUnitsVec(tmpSim);
      //get old X/Y basis and Origin
      double[] oldXBasis=nestedCsys.getBasis0().toDoubleArray();
      double[] oldYBasis=nestedCsys.getBasis1().toDoubleArray();
      double[] oldOrigin=nestedCsys.getOrigin().getVector().toDoubleArray();

      if((Math.abs(newXAxis[0]-oldXBasis[0]) > SMALL_EPS)||
         (Math.abs(newXAxis[1]-oldXBasis[1]) > SMALL_EPS)||
         (Math.abs(newXAxis[2]-oldXBasis[2]) > SMALL_EPS)){
          nestedCsys.setBasis0(new DoubleVector(newXAxis));
      }
      if((Math.abs(newYAxis[0]-oldYBasis[0]) > SMALL_EPS)||
         (Math.abs(newYAxis[1]-oldYBasis[1]) > SMALL_EPS)||
         (Math.abs(newYAxis[2]-oldYBasis[2]) > SMALL_EPS)){
          nestedCsys.setBasis1(new DoubleVector(newYAxis));
      }

      if((Math.abs(oldOrigin[0]-axisOrigin[0]) > SMALL_EPS)||
         (Math.abs(oldOrigin[1]-axisOrigin[1]) > SMALL_EPS)||
         (Math.abs(oldOrigin[2]-axisOrigin[2]) > SMALL_EPS)){
          nestedCsys.getOrigin().setCoordinate(
                  prefUVec, prefUVec, prefUVec, new DoubleVector(axisOrigin));
      }
  }
  private static double[][] getCoordBasisSet(CartesianCoordinateSystem tmpCsys){
      double[] xBasis = tmpCsys.getBasis0().toDoubleArray();
      double[] yBasis = tmpCsys.getBasis1().toDoubleArray();
      double[] zBasis = tmpCsys.getBasis2().toDoubleArray();
      double[][] retMat = new double[3][3];
      retMat[0][0]=xBasis[0];retMat[0][1]=yBasis[0];retMat[0][2]=zBasis[0];
      retMat[1][0]=xBasis[1];retMat[1][1]=yBasis[1];retMat[1][2]=zBasis[1];
      retMat[2][0]=xBasis[2];retMat[2][1]=yBasis[2];retMat[2][2]=zBasis[2];
      return retMat;
  }
  private static CartesianCoordinateSystem getLocalCartCoord(
          CoordinateSystem majCsys,String csysName){
      CartesianCoordinateSystem tmpCsys;
      try{
          tmpCsys= (CartesianCoordinateSystem) majCsys
                  .getLocalCoordinateSystemManager().getObject(csysName);
      }catch(NeoException e){
          tmpCsys = 
            majCsys.getLocalCoordinateSystemManager()
                    .createLocalCoordinateSystem(
                            CartesianCoordinateSystem.class, "Cartesian");
          tmpCsys.setPresentationName(csysName);
      }
      return tmpCsys;
  }
  private static CylindricalCoordinateSystem getLocalCylCoord(
          CoordinateSystem majCsys,String csysName){
      CylindricalCoordinateSystem tmpCsys;
      try{
          tmpCsys= (CylindricalCoordinateSystem) majCsys
                  .getLocalCoordinateSystemManager().getObject(csysName);
      }catch(NeoException e){
          tmpCsys = 
            majCsys.getLocalCoordinateSystemManager()
                    .createLocalCoordinateSystem(
                            CylindricalCoordinateSystem.class, csysName);
          tmpCsys.setPresentationName(csysName);
      }
      return tmpCsys;
  }
  private static CartesianCoordinateSystem getLabBasedCartCoord(
          Simulation tmpSim, String csysName){
      CartesianCoordinateSystem tmpCsys;
      try{
          tmpCsys= (CartesianCoordinateSystem) getLabCsys(tmpSim).
                  getCoordinateSystemManager().getCoordinateSystem(csysName);
      }catch(NeoException e){
          tmpCsys = 
            getLabCsys(tmpSim).getCoordinateSystemManager()
                    .createCoordinateSystem(
                            CartesianCoordinateSystem.class, "Cartesian");
          tmpCsys.setPresentationName(csysName);
      }
      return tmpCsys;
  }

}
