import socket
import time 

""" Terminal calls:
python ur5dashboard.py --arm right
python ur5dashboard.py --arm left
"""

class UR5DashboardClient:
    def __init__(self, host, port=29999):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.host = host
        self.port = port
        
    def connect(self):
        """
        Should print "Connected: Universal Robots Dashboard Server" upon success.
        """
        connected = False
        attempts = 0
        while attempts < 5:
            try:
                self.s.connect((self.host,self.port))
                connected = True
                data = self.s.recv(1024)
                decodedData = data.decode('utf-8')
                decodedData = decodedData.replace('\n', '')
                print(decodedData)
                return
            except Exception as e:
                attempts += 1
                print("DASHBOARD SERVER: Attempt {} to connect to the dashboard server failed with error: {} \n\n, retrying in 2 seconds".format(attempts,e))
                time.sleep(2.0)
                pass
        raise Exception("Cannot connect to dashboard server? Maybe arms aren't on")

    def close(self):
        self.s.close()

    def send(self, cmd):
        """
        Sends data over the socket
        """
        self.s.sendall(cmd.encode() + b'\n')
    
    def receive(self):
        """
        Receives data over the socket
        """
        s = self.s
        first = s.recv(1024)
        decodedFirst = first.decode('utf-8')
        decodedFirst = decodedFirst.replace('\n', '')
        if decodedFirst == "Connected: Universal Robots Dashboard Server":
            second = s.recv(1024)
            decodedSecond = second.decode('utf-8')
            decodedSecond = decodedSecond.replace('\n', '')
            return decodedSecond
        return decodedFirst
    
    def loadProgram(self, program):
        """
        Parameters:
        ---------------
        program: the file name (.urp) as a string that is to be loaded
        example: program = "test.urp"

        Loads a desired program to the UR5 arm 
        """
        load_file = "load " + program
        self.send(load_file)
        result = self.receive()
        if result.find("Loading program: ") == -1:
            raise RuntimeError(result)
        else:
            print(result)
    
    def play(self):
        """
        Starts the program loaded 
        """
        play = "play"
        self.send(play)
        result = self.receive()
        if result != "Starting program":
            raise RuntimeError(result)
        else:
            print(result)
    
    def stop(self):
        """
        Stops the current program
        """
        stop = "stop"
        self.send(stop)
        result = self.receive()
        if result != "Stopped":
            raise RuntimeError(result)
        else:
            print(result)
    
    def pause(self):
        """
        Pauses the program
        """
        pause = "pause"
        self.send(pause)
        result = self.receive()
        if result != "Pausing program":
            raise RuntimeError(result)
        else:
            print(result)
    
    def quit(self):
        """
        Quits the program
        """
        quit = "quit"
        self.send(quit)
        result = self.receive()
        print(result)

    def shutdown(self):
        """ 
        Shuts down the robot and controller
        """
        shutdown = "shutdown"
        self.send(shutdown)
        result = self.receive()
        print(result)

    def running(self):
        """
        Returns True if a program is running, false otherwise
        """
        running = "running"
        self.send(running)
        result = self.receive()
        if result.find("true") != -1:
            return True
        return False
    
    def robotmode(self):
        """
        Robot mode enquiry
        OPTIONS: NO_CONTROLLER, DISCONNECTED, CONFIRM_SAFETY,
            BOOTING, POWER_OFF, POWER_ON, IDLE, BACKDRIVE, RUNNING
        """
        robotmode = "robotmode"
        self.send(robotmode)
        result = self.receive()
        return result

    def getLoadedProgram(self):
        """
        Returns which program is loaded
        """
        getLoadedProgram = "get loaded program"
        self.send(getLoadedProgram)
        result = self.receive()
        return result
        
    def popup(self, message):
        """
        Creates a popup message
        """
        popup = "popup " + message
        self.send(popup)
        result = self.receive()
        print(result)

    def closePopup(self):
        """ 
        Closes the popup
        """
        closePopup = "close popup"
        self.send(closePopup)
        result = self.receive()
        print(result)
    
    def addToLog(self, message):
        """
        Adds log-message to Log history
        """
        addToLog = "addToLog " + message
        self.send(addToLog)
        result = self.receive()
        print(result)

    def isProgramSaved(self):
        """
        Returns save state of active program and the path to the loaded program file
        """
        isProgramSaved = "isProgramSaved"
        self.send(isProgramSaved)
        result = self.receive()
        if result.find("true") != -1:
            return True
        return False

    def programState(self):
        """ 
        Returns the state of the active program and path to loaded program file, or STOPPED if no program is loaded

        Options: STOPPED, PLAYING, PAUSED
        """
        programState = "programState"
        self.send(programState)
        result = self.receive()
        return result

    def polyscopeVersion(self):
        """
        Returns version of the Polyscope software
        """
        polyscopeVersion = "PolyscopeVersion"
        self.send(polyscopeVersion)
        result = self.receive()
        return result

    def setOperationalMode(self, mode):
        """
        Parameters:
        --------------
        mode: A string that is either "manual" or "automatic"; otherwise, a runtime error is raised.

        Sets the operational mode.
        """
        if mode != "manual" or mode != "automatic":
            raise RuntimeError("Invalid mode. Must enter 'manual' or 'automatic'")
        setOperationalMode = "set operational mode " + mode
        self.send(setOperationalMode)
        result = self.receive()
        if result.find("Failed") != -1:
            raise RuntimeError(result)
        else:
            print(result)

    def getOperationalMode(self):
        """
        Returns the operational mode as MANUAL or AUTOMATIC if the password has been set for Mode in settings. 
        Returns NONE if the password has not been set.
        """
        getOperationalMode = "get operational mode"
        self.send(getOperationalMode)
        result = self.receive()
        return result

    def clearOperationalMode(self):
        """
        If this function is called the operational mode can again be changed from PolyScope, and the user password is enabled.
        """
        clearOperationalMode = "clear operational mode"
        self.send(clearOperationalMode)
        result = self.receive()
        print(result)

    def powerOn(self):
        """
        Powers on the robot arm
        """
        powerOn = "power on"
        self.send(powerOn)
        result = self.receive()
        print(result)

    def powerOff(self):
        """
        Powers off the robot arm
        """
        powerOff = "power off"
        self.send(powerOff)
        result = self.receive()
        print(result)

    def brakeRelease(self):
        """ 
        Releases the brakes
        """
        brakeRelease = "brake release"
        self.send(brakeRelease)
        result = self.receive()
        print(result)

    def safetyStatus(self):
        """
        Returns the safety status 
        OPTIONS: NORMAL, REDUCED, PROTECTIVE_STOP, RECOVERY, SAFEGUARD_STOP,
            SYSTEM_EMERGENCY_STOP, ROBOT_EMERGENCY_STOP, VIOLATION, FAULT,
            AUTOMATIC_MODE_SAFEGUARD_STOP, SYSTEM_THREE_POSITION_ENABLING STOP
        """
        safetystatus =  "safetystatus"
        self.send(safetystatus)
        result = self.receive()
        return result

    def unlockProtectiveStop(self):
        """ 
        Unlocks the protective stop
        """
        unlockProtectiveStop = "unlock protective stop"
        self.send(unlockProtectiveStop)
        result = self.receive()
        if result != "Protective stop releasing" and result != "Safetystatus: PROTECTIVE_STOP":
            raise RuntimeError("Unlock protective stop failure: " + result)
        else:
            print(result)

    def closeSafetyPopup(self):
        """ 
        Closes a safety popup
        """
        closeSafetyPopup = "close safety popup"
        self.send(closeSafetyPopup)
        result = self.receive()
        print(result)

    def loadInstallation(self, installation):
        """
        Parameters:
        ---------------
        installation: the file name (.installation) as a string that is to be loaded
        example: program = "test.installation"

        Loads a desired installation file to the UR5 arm but does not return until the load has completed or fails.

        This command fails if the associated installation requires confirmation of safety, returning "Failed to load installation"
        """
        load_file = "load " + installation
        self.send(load_file)
        result = self.receive()
        if result.find("Loading installation:") == -1:
            raise RuntimeError(result)
        else:
            print(result)
    
    def restartSafety(self):
        """
        Used when a robot gets a safety fault or violation to restart the safety. 
        After safety has been rebooted the robot will be in Power Off.
        """
        restartSafety = "restart safety"
        self.send(restartSafety)
        result = self.receive()
        print(result)

    def isInRemoteControl(self):
        """
        Returns the remote control status of the robot.
        True if in remote control, False if in local control
        """
        isInRemoteControl = "is in remote control"
        self.send(isInRemoteControl)
        result = self.receive()
        if result.find("true") != -1:
            return True
        return False

    def getSerialNumber(self):
        """
        Returns the serial number of the robot
        """
        getSerial = "get serial number"
        self.send(getSerial)
        result = self.receive()
        print(result)

    def getRobotModel(self):
        """
        Returns the robot model
        """
        getRobotModel = "get robot model"
        self.send(getRobotModel)
        result = self.receive()
        print(result)

    def generateFlightReport(self, type):
        """
        Parameters:
        --------------
        type: A string of the type of report. The options are controller, software, and system.

        Upon success, a report ID is printed. Upon failure, an error message is printed.
        Command can take few minutes to complete.
        """
        if type != "controller" or type != "software" or type != "system":
            raise RuntimeError("Invalid report type. Must be 'controller', 'software', or 'system'.")
        generateFlightReport = "generate flight report " + type
        self.send(generateFlightReport)
        result = self.receive()
        print("Result of generateFlightReport: " + result)

    def generateSupportFile(self,directory):
        """
        Parameters:
        --------------
        directory: A string of a path to an already existing directory location inside the programs directory. 
            In particular path can point to special usbdisk subfolders inside programs folder.

        On success: "Completed successfully:" <result file name> is printed.
        On failure: An error message with possible cause of error is printed.
        Command can take up to 10 minutes to complete
        """
        generateSupportFile = "generate supprt file " + directory
        self.send(generateSupportFile)
        result = self.receive()
        print(result)

if __name__ == "__main__":
    # take in strings as argument to set HOST and command
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument('--addr', action='store', type=str, required=True)
    args = ap.parse_args()

    HOST = args.addr 
    PORT = 29999

    a = UR5DashboardClient(HOST,PORT)
    host = a.host
    port = a.port
    a.connect()
    print(a.safetyStatus())
    print(a.robotmode())
