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
package Naming;

import java.util.*;
import star.common.*;


public class NamingConvention {
    
    // Tags
    String wtTagID = "WT";
    String osTagID = "OS";
    String vcTagID = "VC";
    String aerodynamicSurfaceTagID = "AerodynamicSurface";
  
    // Geometric Identification - "basic" boundary types
    String inletStr    = "000";
    String outletStr   = "001";
    String switchStr   = "002";
    String freeStr     = "003";
    String symmStr     = "004";
    String wallStr     = "005";
    String oversetStr  = "006";
    String radInStr    = "007";
    String radOutStr   = "008";
    String slidingStr  = "009";
    //
    String inletName   = inletStr   + " Inlet";
    String outletName  = outletStr  + " Outlet";
    String switchName  = switchStr  + " Switch";
    String freeName    = freeStr    + " Freestream";
    String symmName    = symmStr    + " Symm";
    String wallName    = wallStr    + " Wall";
    String oversetName = oversetStr + " Overset";
    String slidingName = slidingStr + " Sliding Int";
    //
    // Special GeometryPart Name PreFixes
    String oversetPreFix = "OS";        // (O)ver(S)et
    String vcPreFix = "VC";             // (V)olume (C)ontrol
    String statVCPreFix = "VCS";        // (V)olume (C)control (stat)ionary 
    //
    // Boundary Layer Probes
    String blpPreFix = "BLP";
    //
    // Geometry Configuration
    // Note: Airfoils start with "01XX"
    //       and are really only present in wind tunnel analyses
    ArrayList<String> airfoilStrList;
    //       At any point, common components of hardware that develop best
    //       practice settings can be subcategoried under the list, up to 999
    //       standard componetns I think.
    //
    // Large Component Key:
    //  -Main Wing       - 1XXX
    //  -Flaps/Ailerons  - 2XXX
    //  -Random Hardware - 3XXX
    //  -Pylons          - 4XXX
    //  -Empennage       - 5XXX
    //      -Elevators   - 54XX
    //      -Rudders     - 56XX
    //  -RBM Rotors      - 8XXX
    //      -BladeID     - 81XX
    //
    String mainWingID   = "1"; String mainWingName =mainWingID +"000 Main Wing";
    //
    String flapsID      = "2"; String flapsName    =flapsID    +"000 Flaps"    ;
    //
    String hardwareID   = "3"; String hardwareName =hardwareID +"000 Hardware" ;
    //
    String pylonID      = "4"; String pylonName    =pylonID    +"000 Pylons"   ;
    
    //
    String empennageID  = "5"; String empennageName=empennageID+"000 Empennage";
    //note any 51x is considered an htail
    //note any 57x is considered a vtail
    String elevatorID   ="54"; String elevatorName =elevatorID + "00 Elevator" ;
    String rudderID     ="56"; String rudderName   =rudderID   + "00 Rudder"   ;
    
    //
    String fuselageID   = "6"; String fuselageName =fuselageID +"000 Fuselage" ;

    //
    String rotorID      = "8" ; String rotorName      = rotorID      + "000 Rotor";
    String rotorBladeID = "81"; String rotorBladeName = rotorBladeID + "00 Blades"; 
    //Monitors
    String itPostFix = " - It";
    String unsPostFix= " - Uns";
    
    //Coordinate Systems
    String bodyCsysName  = "Body Csys";
    String inletCsysName = "Velocity Inlet";
    String windCsysName  = "Wind Axis";

    String proxyRepName= "Proxy";
    
    
    public NamingConvention(){
      //Airfoil prefix generation
      this.airfoilStrList=new ArrayList();
      for(int i=1;i<=9;i++){
          airfoilStrList.add("0"+i);
      }
    }
    
    public String getWindTunnelTagID(){
      return wtTagID;
    }
    public String getOversetDomainTagID(){
     return osTagID;
    }
    public String getVolumeControlTagID(){
      return vcTagID;
    }
    
    //Airfoil String Names
    public ArrayList<String> getAirfoilStrList(){
        return airfoilStrList;
    }
    
    //Boundary Convention String Names
    public String getInletStr(){
        return inletStr;
    }
    public String getSwitchStr(){
        return switchStr;
    }
    public String getOutletStr(){
        return outletStr;
    }
    public String getFreeStr(){
        return freeStr;
    }
    public String getSymmStr(){
        return symmStr;
    }
    public String getWallStr(){
        return wallStr;
    }
    public String getOversetStr(){
        return oversetStr;
    }
    public String getSlidingStr(){
        return slidingStr;
    }

    // Full config
    public ArrayList<String> getAllFullConfigurationPreFixes(){
        ArrayList<String> retList=new ArrayList();
        retList.add(getMainWingID());
        retList.add(getFlapsID());
        retList.add(getHardwareID());
        retList.add(getPylonID());
        retList.add(getFuselageID());
        retList.add(getEmpennageID());
        retList.add(getElevatorID());
        retList.add(getRudderID());
        
        return retList;
    }
    
    public String getMainWingID(){
        return mainWingID;
    }
    public String getFlapsID(){
        return flapsID;
    }
    public String getHardwareID(){
        return hardwareID;
    }
    public String getPylonID(){
        return pylonID;
    }
    public String getFuselageID(){
        return fuselageID;
    }
    public String getEmpennageID(){
        return empennageID;
    }
    public String getElevatorID(){
        return elevatorID;
    }
    public String getRudderID(){
        return rudderID;
    }
    public String getRotorID(){
        return rotorID;
    }
    public String getRotorBladeID(){
        return rotorBladeID;
    }
    
    public String getBndyName(String preFix){
        // Domain Boundaries
        if(preFix.equals(inletStr)){
            return inletName;
        }else if(preFix.equals(outletStr)){
            return outletName;
        }else if(preFix.equals(switchStr)){
            return switchName;
        }else if(preFix.equals(freeStr)){
            return freeName;
        }else if(preFix.equals(symmStr)){
            return symmName;
        }else if(preFix.equals(wallStr)){
            return wallName;
        }else if(preFix.equals(oversetStr)){
            return oversetName;
        }else if(preFix.equals(slidingStr)){
            return slidingName;
        }else if(preFix.equals(rotorID)){
            return rotorName;
        }else if(preFix.equals(mainWingID)){
            return mainWingName;
        }else if(preFix.equals(flapsID)){
            return flapsName;
        }else if(preFix.equals(elevatorID)){
            return elevatorName;
        }else if(preFix.equals(rudderID)){
            return rudderName;
        }else if(preFix.equals(hardwareID)){
            return hardwareName;
        }else if(preFix.equals(pylonID)){
            return pylonName;
        }else if(preFix.equals(empennageID)){
            return empennageName;
        }else if(preFix.equals(fuselageID)){
            return fuselageName;
        }
        //
        // Airfoil Boundaries
        if(airfoilStrList.contains(preFix)){
            return preFix+"00 Airfoil";
        }
        //
        // Rotor Boundaries
        if(preFix.equals(rotorID)){
            return rotorName;
        }
        if(preFix.equals(rotorBladeID)){
            return rotorBladeName;
        }
        //
        return "Unknown Type";
    }
    
    public String getBodyCsysName(){
        return bodyCsysName;
    }
    public String getInletCsysName(){
        return inletCsysName;
    }
    public String getWindCsysName(){
        return windCsysName;
    }
    public String getVCPreFix(){
        return vcPreFix;
    }
    public String getStatVCPreFix(){
        return statVCPreFix;
    }
    public String getOversetPreFix(){
        return oversetPreFix;
    }
    public String getBLPPreFix(){
      return blpPreFix;
    }
    
    //Monitor Convention String Names
    public String getItPostFix(){
        return itPostFix;
    }
    public String getUnsPostFix(){
        return unsPostFix;
    }
    
    //Representations
    public String getProxyName(){
        return proxyRepName;
    }
}
