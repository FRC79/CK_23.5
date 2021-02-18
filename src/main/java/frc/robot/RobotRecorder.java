/*
This class will have an array of hashmaps that store given info about the robot and play it back on request
an example of how to use this can be found (link here) 
this is also used in the CK_23.5 repo found (link here)
TODO:
use the sendable chooser to let drivers choose what file to read from?
    if not just make the filename an option in constants( current solution )
*/
package frc.robot;

// import constants from constants.java
import frc.robot.Constants.RobotRecorderConstants;

// use an arraylist of hashmaps for storing the data about the robot
import java.util.ArrayList;
import java.util.HashMap;

// for saving and retrieving files 
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;

public class RobotRecorder {

    // import constants from constants.java
    static final double  UPDATE_FREQ    = RobotRecorderConstants.RECORDING_FREQUENCY;
    static final double  RECORD_LENGTH  = RobotRecorderConstants.RECORDING_DURATION; 
    static final String  FILE_EXT       = RobotRecorderConstants.SAVE_FILE_EXTENSION;
    static final String  FILE_PATH      = RobotRecorderConstants.SAVE_FILE_PATH;
    static final String  FILE_NAME      = RobotRecorderConstants.SAVE_FILE_NAME;
    static final boolean PRINT_DEBUG    = RobotRecorderConstants.PRINT_DEBUG_INFO;
    static final boolean VERBOSE_DEBUG  = RobotRecorderConstants.VERBOSE_DEBUG_PRINT;
    
    // time recording started (to stop recording once the auton timer is over ) 
    private double startTime; 
    
    // use an arraylist of HashMaps for storing and reading data about the robot 
    private ArrayList<HashMap<String, Double>> recordArray;

    /* HashMap that holds info about the robot in a single moment.
     gets saved and cleared every update() */
    private HashMap<String, Double> curState = new HashMap<String, Double>();

    // current ID for the robot's State in the arraylist, for playback 
    private int curUpdateIndex; 
    
    // last time update() ran 
    private double lastUpdate;

    // the modes of operation for the robotRecorder 
    enum Mode{
        PLAY,
        RECORD,
        NORMAL
    }
    private Mode curMode = Mode.NORMAL;
	
	
    // methods for saing and retrieving recordArray to/from files
	
    private void saveRecordArray(String fileName){
        File outFile =  new File(FILE_PATH+fileName+FILE_EXT); // make a new file
	try { // attempt to save the array to the file
        boolean fileSuccess = outFile.createNewFile();
		FileOutputStream fs = new FileOutputStream(outFile);
		ObjectOutputStream os = new ObjectOutputStream(fs);
		os.writeObject(recordArray);
		os.close();
        infoPrint("file made: "+fileSuccess, false);
	} catch (Exception e) { // in the case the file cannot be written to
		e.printStackTrace();
	}
    }
    
    private void loadRecordArray(String fileName){
        try {
		FileInputStream fs = new FileInputStream(FILE_PATH+fileName+FILE_EXT); // try to find file by name
		ObjectInputStream os = new ObjectInputStream(fs);
				
		recordArray = (ArrayList<HashMap<String, Double>>) os.readObject(); // cast the data to an array of HashMaps and then assign it to recordArray
				
		os.close();
		fs.close();
	} 
	catch (IOException e) {
		e.printStackTrace();
	} 
	catch (ClassNotFoundException e) {
		e.printStackTrace();
	}
    }
    
    // methods for starting and stopping the recorder's different opporations   
	
    public void startPlayback(){
        curMode = Mode.PLAY;
        curUpdateIndex = 0;
	infoPrint("playback starting", false);
        // grab recordArray from a file
        loadRecordArray(FILE_NAME);
    }
    
    public void stopPlaying(){
        curMode = Mode.NORMAL;
	infoPrint("playback over", false);
    }

    public void startRecording(){
        recordArray = new ArrayList<HashMap<String, Double>>();
        startTime = System.currentTimeMillis();
        curMode = Mode.RECORD;
	infoPrint("recording starting", false);
    }
    
    public void stopRecording(){
        curMode = Mode.NORMAL;
	infoPrint("recording over", false);
        // save recordArray to a file
        saveRecordArray(FILE_NAME);
    }

    /**
     * @param Key a unique string value that is a name for the value stored in the recording
     * @param Value the value sto be stored at this point in the record, must be a double
     * will store a value under a unique key at the current point in the robot record, only works when the robot is in recording mode.
     */
    public void setRobotData(String Key, double Value){
        if(curMode == Mode.RECORD){
            curState.put(Key,Value); // at the current state, put this key value pair in the hashMap
        }
    }
    
    /**
     * @param Key the unique string name of a value to retrieve at this point in the record
     * if no such key is found or not in playback mode, returns null
     * only works 
     */
    public double getRobotData(String Key){
        if(curMode == Mode.PLAY){
            return curState.get(Key);
        }
        return (Double) null;
    }
	
    /**
    * @param text the text that should be printed
    * @param verbose is this text verbose debug info
    * a method that should be used for printing that considers if something should be printed or not depending on the settings
    */
    private void infoPrint(String text, boolean verbose){   
	if(!PRINT_DEBUG){ return; } // if printing is turned off then stop right here
	if(verbose){
		if(VERBOSE_DEBUG){
			System.out.println(text); // print verbose messages of allowed
		}
	}else{
		System.out.println(text); // print normal messages
	}
	
    }

    public void update(){
        infoPrint("update()", true);
        if( (System.currentTimeMillis() - lastUpdate) >= UPDATE_FREQ){ // after the given time frequency
            if(curMode == Mode.PLAY){ // when playing back info
		infoPrint("playback", true);
                if(curUpdateIndex > recordArray.size()){ // stop when out of instructions to follow

                    stopPlaying();
                    return;
                }
                curUpdateIndex++;   // update index for save array on next go around
                curState = recordArray.get(curUpdateIndex); // update curState
            }else if(curMode == Mode.RECORD){ // when recording info
                //infoPrint(String.valueOf(System.currentTimeMillis()*1e-12-startTime*1e-12), false);
                if(true){//System.currentTimeMillis()*1e-12-startTime*1e-12 > AUTO_LENGTH*1000){ // stop recording when auton recording ends
                    infoPrint(String.valueOf(System.currentTimeMillis()*1e-12), false);
                    infoPrint(String.valueOf(startTime*1e-12), false);
                    infoPrint(String.valueOf(RECORD_LENGTH*1000), false);
                    infoPrint(String.valueOf(System.currentTimeMillis()*1e-12-startTime*1e-12), false);
                    stopRecording();
                    return;
                }
                recordArray.add(curState); // save current state to record array
                curState.clear(); // clear state for next go around
            }
            lastUpdate = System.currentTimeMillis(); // set lastUpdate to reset the timer 
        }
    }
}