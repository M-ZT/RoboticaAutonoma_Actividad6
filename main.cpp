#define ARNL

#include <QCoreApplication>
#include <locale>

#include <Aria.h>
#include <ArNetworking.h>
#include <Arnl.h>
#include <ArPathPlanningInterface.h>
#include <ArLocalizationTask.h>
#include <ArSystemStatus.h>

#include "control.h"
void logOptions(const char *progname)
{
  ArLog::log(ArLog::Normal, "Usage: %s [options]\n", progname);
  ArLog::log(ArLog::Normal, "[options] are any program options listed below, or any ARNL configuration");
  ArLog::log(ArLog::Normal, "parameters as -name <value>, see params/arnl.p for list.");
  ArLog::log(ArLog::Normal, "For example, -map <map file>.");
  Aria::logOptions();
}

class RobotMonitor
{
protected:
  ArRobot *robot;
  ArServerHandlerPopup *popupServer;
  ArTypes::Byte4 motorsDisabledPopupID;
  ArServerHandlerPopupInfo motorsDisabledPopupInfo;
  ArFunctor2C<RobotMonitor, ArTypes::Byte4, int> handleMotorsDisabledPopupResponseCB;
  ArFunctorC<RobotMonitor> robotMonitorCB;

public:
  RobotMonitor(ArRobot *r, ArServerHandlerPopup *ps) :
    robot(r),
    popupServer(ps),
    motorsDisabledPopupID(0),
    motorsDisabledPopupInfo( NULL,
      "Robot Motors Are Disabled",
      "The robot's motors are disabled",
      ArServerHandlerPopup::WARNING,
      1,  // default button ID
      0,  // escape button ID
      -1, // timeout
      NULL, // timeout message
      "Enable Motors", "Enabling Motors...",
      "Ignore", "Ignore"
    ),
    handleMotorsDisabledPopupResponseCB(this, &RobotMonitor::handleMotorsDisabledResponse),
    robotMonitorCB(this, &RobotMonitor::robotMonitorTask)
  {
    robot->addUserTask("arnlServerRobotMonitor", 30, &robotMonitorCB);
  }

  ~RobotMonitor()
  {
    robot->remUserTask(&robotMonitorCB);
  }

protected:
  void handleMotorsDisabledResponse(ArTypes::Byte4 popupID, int button)
  {
    if(button == 0)
    {
      ArLog::log(ArLog::Normal, "Enabling motors...");
      robot->enableMotors();
    }
    popupServer->closePopup(motorsDisabledPopupID, "Closing motor disable popup.");
    motorsDisabledPopupID = 0;
  }


  // This function is called as a robot task (every 100ms) to check on the robot
  // state and perform feedback and interact with user as needed.
  void robotMonitorTask()
  {

    // a way for user to re-enable motors if disabled -- show a popup dialog in
    // MobileEyes.
    if(motorsDisabledPopupID == 0 && robot && !robot->areMotorsEnabled() && robot->isConnected())
    {
      motorsDisabledPopupID = popupServer->createPopup(&motorsDisabledPopupInfo, &handleMotorsDisabledPopupResponseCB);
    }

    // Set LX wheel light pattern based on robot activity. You could add more
    // conditions/light patterns here if you want.
    if(robot->isEStopPressed())
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x02\0\0\0", 4); // pattern #2, flash red
    else if(!robot->areMotorsEnabled())
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x03\0\0\0", 4); // pattern #3, flash yellow
    else if(fabs(robot->getVel()) < 5)
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x0A\0\0\0", 4);  // pattern #10, slow blue flash
    else
      robot->comDataN(ArCommands::WHEEL_LIGHT, "\x09\0\0\0", 4);  // pattern 9, blue sweep.

  }
};


/** Main function */
int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);
    setlocale(LC_NUMERIC,"C");
    // Initialize Aria and Arnl global information
    Aria::init();
    Arnl::init();

    // You can change default ArLog options in this call, but the settings in the parameter file
    // (arnl.p) which is loaded below (Aria::getConfig()->parseFile())  will override the options.
    //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);

    // Used to parse the command line arguments.
    ArArgumentParser parser(&argc, argv);

    // Load default arguments for this computer (from /etc/Aria.args, environment
    // variables, and other places)
    parser.loadDefaultArguments();

    // Tell the laser connector to always connect the first laser since
    // this program always requires a laser.
    parser.addDefaultArgument("-connectLaser");

    // The robot object
    ArRobot robot;

    // This object is used to connect to the robot, which can be configured via
    // command line arguments.
    ArRobotConnector robotConnector(&parser, &robot);

    // Connect to the robot
    if (!robotConnector.connectRobot())
    {
      ArLog::log(ArLog::Normal, "Error: Could not connect to robot... exiting");
      Aria::exit(3);
    }



    // Set up where we'll look for files. Arnl::init() set Aria's default
    // directory to Arnl's default directory; addDirectories() appends this
    // "examples" directory.
    char fileDir[1024];
    ArUtil::addDirectories(fileDir, sizeof(fileDir), Aria::getDirectory(),
               "examples");


    // To direct log messages to a file, or to change the log level, use these  calls:
    //ArLog::init(ArLog::File, ArLog::Normal, "log.txt", true, true);
    //ArLog::init(ArLog::File, ArLog::Verbose);

    // Add a section to the configuration to change ArLog parameters
    ArLog::addToConfig(Aria::getConfig());

    // Our networking server
    ArServerBase server;


    // Set up our simpleOpener, used to set up the networking server
    ArServerSimpleOpener simpleOpener(&parser);

    // the laser connector
    ArLaserConnector laserConnector(&parser, &robot, &robotConnector);

//    // Used to connect to a "central server" which can be used as a proxy
//    // for multiple robot servers, and as a way for them to also communicate with
//    // each other.  (objects implementing some of these inter-robot communication
//    // features are created below).
//    // NOTE: If the central server is running on the same host as robot server(s),
//    // then you must use the -serverPort argument to instruct these robot-control
//    // server(s) to use different ports than the default 7272, since the central
//    // server will use that port.
//    ArClientSwitchManager clientSwitch(&server, &parser);

    // Load default arguments for this computer (from /etc/Aria.args, environment
    // variables, and other places)
    parser.loadDefaultArguments();

    // Parse arguments
    if (!Aria::parseArgs() || !parser.checkHelpAndWarnUnparsed())
    {
      logOptions(argv[0]);
      Aria::exit(1);
    }


    // This causes Aria::exit(9) to be called if the robot unexpectedly
    // disconnects
    ArGlobalFunctor1<int> shutdownFunctor(&Aria::exit, 9);
    robot.addDisconnectOnErrorCB(&shutdownFunctor);


    // Create an ArSonarDevice object (ArRangeDevice subclass) and
    // connect it to the robot.
    ArSonarDevice sonarDev;
    robot.addRangeDevice(&sonarDev);

    // This object will allow robot's movement parameters to be changed through
    // a Robot Configuration section in the ArConfig global configuration facility.
    ArRobotConfig robotConfig(&robot);

    // Start the robot thread.
    robot.runAsync(true);

    // connect the laser(s) if it was requested, this adds them to the
    // robot too, and starts them running in their own threads
    ArLog::log(ArLog::Normal, "Connecting to laser(s) configured in parameters...");
    if (!laserConnector.connectLasers())
    {
      ArLog::log(ArLog::Normal, "Error: Could not connect to laser(s). Exiting.");
      Aria::exit(2);
    }
    ArLog::log(ArLog::Normal, "Done connecting to laser(s).");

    // find the laser we should use for localization and/or mapping,
    // which will be the first laser
    robot.lock();
    ArLaser *firstLaser = robot.findLaser(1);
    if (firstLaser == NULL || !firstLaser->isConnected())
    {
      ArLog::log(ArLog::Normal, "Did not have laser 1 or it is not connected, cannot start localization and/or mapping... exiting");
      Aria::exit(2);
    }
    robot.unlock();


      /* Create and set up map object */

    // Set up the map object, this will look for files in the examples
    // directory (unless the file name starts with a /, \, or .
    // You can take out the 'fileDir' argument to look in the program's current directory
    // instead.
    // When a configuration file is loaded into ArConfig later, if it specifies a
    // map file, then that file will be loaded as the map.
    ArMap map(fileDir);
    // set it up to ignore empty file names (otherwise if a configuration omits
    // the map file, the whole configuration change will fail)
    map.setIgnoreEmptyFileName(true);
    // ignore the case, so that if someone is using MobileEyes or
    // MobilePlanner from Windows and changes the case on a map name,
    // it will still work.
    map.setIgnoreCase(true);


      /* Create localization and path planning threads */


    ArPathPlanningTask pathTask(&robot, &sonarDev, &map);

    ArLog::log(ArLog::Normal, "Creating laser localization task");
    // Laser Monte-Carlo Localization
    ArLocalizationTask locTask(&robot, firstLaser, &map);

    // Set some options  and callbacks on each laser that the laser connector
    // connected to.
    std::map<int, ArLaser *>::iterator laserIt;
    for (laserIt = robot.getLaserMap()->begin();
         laserIt != robot.getLaserMap()->end();
         laserIt++)
    {
      int laserNum = (*laserIt).first;
      ArLaser *laser = (*laserIt).second;

      // Skip lasers that aren't connected
      if(!laser->isConnected())
        continue;

      // add the disconnectOnError CB to shut things down if the laser
      // connection is lost
      laser->addDisconnectOnErrorCB(&shutdownFunctor);
      // set the number of cumulative readings the laser will take
      laser->setCumulativeBufferSize(200);
      // add the lasers to the path planning task
      pathTask.addRangeDevice(laser, ArPathPlanningTask::BOTH);
      // set the cumulative clean offset (so that they don't all fire at once)
      laser->setCumulativeCleanOffset(laserNum * 100);
      // reset the cumulative clean time (to make the new offset take effect)
      laser->resetLastCumulativeCleanTime();

      // Add the packet count to the Aria info strings (It will be included in
      // MobileEyes custom details so you can monitor whether the laser data is
      // being received correctly)
      std::string laserPacketCountName;
      laserPacketCountName = laser->getName();
      laserPacketCountName += " Packet Count";
      Aria::getInfoGroup()->addStringInt(
          laserPacketCountName.c_str(), 10,
          new ArRetFunctorC<int, ArLaser>(laser,
                       &ArLaser::getReadingCount));
    }

      /* Start the server */

    // Open the networking server
    if (!simpleOpener.open(&server, fileDir, 240))
    {
      ArLog::log(ArLog::Normal, "Error: Could not open server.");
      exit(2);
    }

    // Action to slow down robot when localization score drops but not lost.
    ArActionSlowDownWhenNotCertain actionSlowDown(&locTask);
    pathTask.getPathPlanActionGroup()->addAction(&actionSlowDown, 140);

    // Action to stop the robot when localization is "lost" (score too low)
    ArActionLost actionLostPath(&locTask, &pathTask);
    pathTask.getPathPlanActionGroup()->addAction(&actionLostPath, 150);

    // Arnl uses this object when it must replan its path because its
    // path is completely blocked.  It will use an older history of sensor
    // readings to replan this new path.  This should not be used with SONARNL
    // since sonar readings are not accurate enough and may prevent the robot
    // from planning through space that is actually clear.
    ArGlobalReplanningRangeDevice replanDev(&pathTask);

    // Service to provide drawings of data in the map display :
    ArServerInfoDrawings drawings(&server);
    drawings.addRobotsRangeDevices(&robot);
    drawings.addRangeDevice(&replanDev);

    /* Draw a box around the local path planning area use this
      (You can enable this particular drawing from custom commands
      which is set up down below in ArServerInfoPath) */
    ArDrawingData drawingDataP("polyLine", ArColor(200,200,200), 1, 75);
    ArFunctor2C<ArPathPlanningTask, ArServerClient *, ArNetPacket *>
      drawingFunctorP(&pathTask, &ArPathPlanningTask::drawSearchRectangle);
    drawings.addDrawing(&drawingDataP, "Local Plan Area", &drawingFunctorP);

    /* Draw a box showing the safety clearance used by the path planning task to */
    drawings.addDrawing(
      new ArDrawingData("polySegments", ArColor(166, 166, 166), 1, 60, 100, "DefaultOn"),
      "Path Planning Clearances",
      new ArFunctor2C<ArPathPlanningTask, ArServerClient*, ArNetPacket*>(&pathTask, &ArPathPlanningTask::drawRobotBounds)
    );

    /* Show the sample points used by MCL */
    ArDrawingData drawingDataL("polyDots", ArColor(0,255,0), 100, 75);
    ArFunctor2C<ArLocalizationTask, ArServerClient *, ArNetPacket *>
      drawingFunctorL(&locTask, &ArLocalizationTask::drawRangePoints);
    drawings.addDrawing(&drawingDataL, "Localization Points", &drawingFunctorL);


    // "Custom" commands. You can add your own custom commands here, they will
    // be available in MobileEyes' custom commands (enable in the toolbar or
    // access through Robot Tools)
    ArServerHandlerCommands commands(&server);

    // These provide various kinds of information to the client:
    ArServerInfoRobot serverInfoRobot(&server, &robot);
    ArServerInfoSensor serverInfoSensor(&server, &robot);
    ArServerInfoPath serverInfoPath(&server, &robot, &pathTask);
    serverInfoPath.addSearchRectangleDrawing(&drawings);
    serverInfoPath.addControlCommands(&commands);

    // Provides localization info and allows the client (MobileEyes) to relocalize at a given
    // pose:
    ArServerInfoLocalization serverInfoLocalization(&server, &robot, &locTask);
    ArServerHandlerLocalization serverLocHandler(&server, &robot, &locTask);

    // If you're using MobileSim, ArServerHandlerLocalization sends it a command
    // to move the robot's true pose if you manually do a localization through
    // MobileEyes.  To disable that behavior, use this constructor call instead:
    // ArServerHandlerLocalization serverLocHandler(&server, &robot, true, false);
    // The fifth argument determines whether to send the command to MobileSim.

    // Provide the map to the client (and related controls):
    ArServerHandlerMap serverMap(&server, &map);

    // These objects add some simple (custom) commands to 'commands' for testing and debugging:
    ArServerSimpleComUC uCCommands(&commands, &robot);                   // Send any command to the microcontroller
    ArServerSimpleComMovementLogging loggingCommands(&commands, &robot); // configure logging
    ArServerSimpleComLogRobotConfig configCommands(&commands, &robot);   // trigger logging of the robot config parameters
  //  ArServerSimpleServerCommands serverCommands(&commands, &server);     // monitor networking behavior (track packets sent etc.)


    // service that allows the client to monitor the communication link status
    // between the robot and the client.
    //
    ArServerHandlerCommMonitor handlerCommMonitor(&server);



    // service that allows client to change configuration parameters in ArConfig
    ArServerHandlerConfig handlerConfig(&server, Aria::getConfig(),
                        Arnl::getTypicalDefaultParamFileName(),
                        Aria::getDirectory());


    // This service causes the client to show simple dialog boxes
    ArServerHandlerPopup popupServer(&server);

    // Monitor the robot for current state such as if the motors are disabled,
    // indicate this state by various means and show a popup dialog on clients
    // if user confirmation is needed.
    // (Namely, some robots  disable motors automatically on various error conditions,
    // and we want the user to manually re-enable them after resolving the
    // problem)
    RobotMonitor robotMonitor(&robot, &popupServer);


    /* Set up the possible modes for remote control from a client such as
     * MobileEyes:
     */

    // Mode To go to a goal or other specific point:
    ArServerModeGoto modeGoto(&server, &robot, &pathTask, &map,
                  locTask.getRobotHome(),
                  locTask.getRobotHomeCallback());

    // Add a special command to Custom Commands that tours a list of goals rather
    // than all:
    modeGoto.addTourGoalsInListSimpleCommand(&commands);

    // Mode To stop and remain stopped:
    ArServerModeStop modeStop(&server, &robot);

    // Cause the sonar to turn off automatically
    // when the robot is stopped, and turn it back on when commands to move
    // are sent. (Note, if using SONARNL to localize, then don't do this
    // since localization may get lost)
    ArSonarAutoDisabler sonarAutoDisabler(&robot);

    // Teleoperation modes To drive by keyboard, joystick, etc:
    ArServerModeRatioDrive modeRatioDrive(&server, &robot);

    // Prevent normal teleoperation driving if localization is lost using
    // a high-priority action, which enables itself when the particular mode is
    // active.
    // (You have to enter unsafe drive mode to drive when lost.)
    ArActionLost actionLostRatioDrive(&locTask, &pathTask, &modeRatioDrive);
    modeRatioDrive.getActionGroup()->addAction(&actionLostRatioDrive, 110);

    // Add drive mode section to the configuration, and also some custom (simple) commands:
    modeRatioDrive.addToConfig(Aria::getConfig(), "Teleop settings");
    modeRatioDrive.addControlCommands(&commands);

    // Wander mode (also prevent wandering if lost):
    ArServerModeWander modeWander(&server, &robot);
    ArActionLost actionLostWander(&locTask, &pathTask, &modeWander);
    modeWander.getActionGroup()->addAction(&actionLostWander, 110);


    // This provides a small table of interesting information for the client
    // to display to the operator. You can add your own callbacks to show any
    // data you want.
    ArServerInfoStrings stringInfo(&server);
    Aria::getInfoGroup()->addAddStringCallback(stringInfo.getAddStringFunctor());

    // The following statements add fields to a set of informational data called
    // the InfoGroup. These are served to MobileEyes for displayi (turn on by enabling Details
    // and Custom Details in the View menu of MobileEyes.)

    Aria::getInfoGroup()->addStringInt(
        "Motor Packet Count", 10,
        new ArConstRetFunctorC<int, ArRobot>(&robot,
                             &ArRobot::getMotorPacCount));

    Aria::getInfoGroup()->addStringDouble(
        "Laser Localization Score", 8,
        new ArRetFunctorC<double, ArLocalizationTask>(
            &locTask, &ArLocalizationTask::getLocalizationScore),
        "%.03f");
    Aria::getInfoGroup()->addStringInt(
        "Laser Loc Num Samples", 8,
        new ArRetFunctorC<int, ArLocalizationTask>(
            &locTask, &ArLocalizationTask::getCurrentNumSamples),
        "%4d");

    // Display system CPU and wireless network status
    ArSystemStatus::startPeriodicUpdate(1000); // update every 1 second
    Aria::getInfoGroup()->addStringDouble("CPU Use", 10, ArSystemStatus::getCPUPercentFunctor(), "% 4.0f%%");
    Aria::getInfoGroup()->addStringInt("Wireless Link Quality", 9, ArSystemStatus::getWirelessLinkQualityFunctor(), "%d");
    Aria::getInfoGroup()->addStringInt("Wireless Link Noise", 9, ArSystemStatus::getWirelessLinkNoiseFunctor(), "%d");
    Aria::getInfoGroup()->addStringInt("Wireless Signal", 9, ArSystemStatus::getWirelessLinkSignalFunctor(), "%d");

    // Make Stop mode the default (If current mode deactivates without entering
    // a new mode, then Stop Mode will be selected)
    modeStop.addAsDefaultMode();
      // TODO move up near where stop mode is created?
    /*
    // If we are on a simulator, move the robot back to its starting position,
    // and reset its odometry.
    // This will allow localizeRobotAtHomeBlocking() below will (probably) work (it
    // tries current odometry (which will be 0,0,0) and all the map
    // home points.
    // (Ignored by a real robot)
    //robot.com(ArCommands::SIM_RESET);
    */

    // El problema del párrafo siguiente es que si se cierra
    // MobileSim no se borra el fichero con la pose y el locTask comienza a
    // buscar en la última pose grabada en la anterior ejecución, que suele ser
    // distinta de Home
    // Borrar a mano <directorio-del-ejecutable>/robotPose

    // create a pose storage class, this will let the program keep track
    // of where the robot is between runs...  after we try and restore
    // from this file it will start saving the robot's pose into the
    // file

//    ArPoseStorage poseStorage(&robot);
    /// if we could restore the pose from then set the sim there (this
    /// won't do anything to the real robot)... if we couldn't restore
    /// the pose then just reset the position of the robot (which again
    /// won't do anything to the real robot)
//    if (poseStorage.restorePose("robotPose"))
//      serverLocHandler.setSimPose(robot.getPose());
    //else
      robot.com(ArCommands::SIM_RESET);

      /* Load configuration values, map, and begin! */


    // When parsing the configuration file, also look at the program's command line options
    // from the command-line argument parser as well as the configuration file.
    // (So you can use any argument on the command line, namely -map.)
    Aria::getConfig()->useArgumentParser(&parser);

    // Read in parameter files.
    ArLog::log(ArLog::Normal, "Loading config file %s into ArConfig...", Arnl::getTypicalParamFileName());
    if (!Aria::getConfig()->parseFile(Arnl::getTypicalParamFileName()))
    {
      ArLog::log(ArLog::Normal, "Could not load ARNL configuration file. Set ARNL environment variable to use non-default installation director.y");
      Aria::exit(5);
    }

    // Warn about unknown params.
    if (!simpleOpener.checkAndLog() || !parser.checkHelpAndWarnUnparsed())
    {
      logOptions(argv[0]);
      Aria::exit(6);
    }

    // Warn if there is no map
    if (map.getFileName() == NULL || strlen(map.getFileName()) <= 0)
    {
      ArLog::log(ArLog::Normal, "");
      ArLog::log(ArLog::Normal, "### No map file is set up, you can make a map with the following procedure");
      ArLog::log(ArLog::Normal, "   0) You can find this information in README.txt or docs/Mapping.txt");
      ArLog::log(ArLog::Normal, "   1) Connect to this server with MobileEyes");
      ArLog::log(ArLog::Normal, "   2) Go to Tools->Map Creation->Start Scan");
      ArLog::log(ArLog::Normal, "   3) Give the map a name and hit okay");
      ArLog::log(ArLog::Normal, "   4) Drive the robot around your space (see docs/Mapping.txt");
      ArLog::log(ArLog::Normal, "   5) Go to Tools->Map Creation->Stop Scan");
      ArLog::log(ArLog::Normal, "   6) Start up Mapper3");
      ArLog::log(ArLog::Normal, "   7) Go to File->Open on Robot");
      ArLog::log(ArLog::Normal, "   8) Select the .2d you created");
      ArLog::log(ArLog::Normal, "   9) Create a .map");
      ArLog::log(ArLog::Normal, "  10) Go to File->Save on Robot");
      ArLog::log(ArLog::Normal, "  11) In MobileEyes, go to Tools->Robot Config");
      ArLog::log(ArLog::Normal, "  12) Choose the Files section");
      ArLog::log(ArLog::Normal, "  13) Enter the path and name of your new .map file for the value of the Map parameter.");
      ArLog::log(ArLog::Normal, "  14) Press OK and your new map should become the map used");
      ArLog::log(ArLog::Normal, "");
    }

    // Print a log message notifying user of the directory for map files
    ArLog::log(ArLog::Normal, "");
    ArLog::log(ArLog::Normal,
           "Directory for maps and file serving: %s", fileDir);

    ArLog::log(ArLog::Normal, "See the ARNL README.txt for more information");
    ArLog::log(ArLog::Normal, "");

    // Do an initial localization of the robot. ARNL and SONARNL try all the home points
    // in the map, as well as the robot's current odometric position, as possible
    // places the robot is likely to be at startup.   If successful, it will
    // also save the position it found to be the best localized position as the
    // "Home" position, which can be obtained from the localization task (and is
    // used by the "Go to home" network request).

    locTask.localizeRobotAtHomeBlocking();

//    // Let the client switch manager (for multirobot) spin off into its own thread
//    // TODO move to multirobot example?
//    clientSwitch.runAsync();

    // Start the networking server's thread
    server.runAsync();


    // Add a key handler so that you can exit by pressing
    // escape. Note that this key handler, however, prevents this program from
    // running in the background (e.g. as a system daemon or run from
    // the shell with "&") -- it will lock up trying to read the keys;
    // remove this if you wish to be able to run this program in the background.
    ArKeyHandler *keyHandler;
    if ((keyHandler = Aria::getKeyHandler()) == NULL)
    {
      keyHandler = new ArKeyHandler;
      Aria::setKeyHandler(keyHandler);
      robot.lock();
      robot.attachKeyHandler(keyHandler);
      robot.unlock();
      puts("Server running. To exit, press escape.");
    }

    // Instance of Control class defined above
    robot.lock();
    Control control(&robot, &pathTask, &locTask);
    robot.unlock();

    // Enable the motors and wait until the robot exits (disconnection, etc.) or this program is
    // canceled.
    robot.enableMotors();
    robot.waitForRunExit();
    Aria::exit(0);

    return a.exec();
}

