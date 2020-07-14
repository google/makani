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
import java.util.ArrayList;
import star.base.neo.*;
import star.common.*;

public class SystemTool {

public static void touchDirectory(String directoryName){
  File checkDir = new File(directoryName);
  // If the directory does not exist, create it.
  if (!checkDir.exists()) {
    boolean result = false;
    try{
      checkDir.mkdir();
      result = true;
    } 
    catch(SecurityException se){
    }        
    if(result) {    
    }
  }
}

public static String getAbsoluteFolderPath(String folderPath){
  if(!folderPath.endsWith(""+File.separator)){
    folderPath = folderPath + File.separator;
  }
  // Correct for relative folder Paths.
  while(folderPath.contains("..")){
    // i.e. remove "../"
    int lastDotDotIndx = folderPath.lastIndexOf("..");
    String lastFolder = folderPath.substring(lastDotDotIndx+3); 
    // Drop a directory level for the fully qualified path.
    // removes final File.sep
    folderPath = folderPath.substring(0, lastDotDotIndx-1);
    int lastFileSepIndx = folderPath.lastIndexOf(File.separator);
    folderPath = folderPath.substring(0, lastFileSepIndx+1)+lastFolder;
  }
  return folderPath;
}

public static void touchDirectoryChain(Simulation tmpSim, String folderPath){
  ArrayList<String> allDirectories = new ArrayList();

  folderPath = getAbsoluteFolderPath(folderPath);
  // Remove final file separator
  folderPath = folderPath.substring(0, folderPath.length()-1);

  // Get last directory and start building an array of paths.
  int fileSeparatorIndx = folderPath.lastIndexOf(File.separator);
  while(fileSeparatorIndx > 0){
    String nextFolder = folderPath.substring(fileSeparatorIndx+1);
    allDirectories.add(nextFolder);
    folderPath = folderPath.substring(0, fileSeparatorIndx);
    fileSeparatorIndx = folderPath.lastIndexOf(File.separator);
  }

  String parentFolder = "" + File.separator
      + allDirectories.get(allDirectories.size() - 1);
  SystemTool.touchDirectory(parentFolder);
  for(int i = allDirectories.size() - 2; i >= 0; i-- ){
    String addFolder = allDirectories.get(i);
    String childFolder = parentFolder + File.separator + addFolder;
    tmpSim.println("touch folder: " + childFolder);
    SystemTool.touchDirectory(childFolder);
    parentFolder = childFolder; 
  }
}
}//END CLASS SYSTEMTOOL  
