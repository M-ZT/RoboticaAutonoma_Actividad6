#include "control.h"

#include <time.h>

Control::Control(ArRobot *robot, ArPathPlanningTask *pathTask,
                 ArLocalizationTask *locTask) :
    myTaskCB(this, &Control::execute),
    myGoalDoneCB(this, &Control::goalDone),
    myGoalFailedCB(this, &Control::goalFailed),
    myGoalInterruptCB(this, &Control::goalInterrupt),
    pathPlanStateChangedCB(this, &Control::pathPlanStateChanged),
    locFailedCB(this, &Control::locFailed)
{
    myRobot = robot;
    myPathTask = pathTask;
    myLocTask = locTask;

    // trigger the first start off
    myGoalDone = true;

    // This is an example of a user task callback. This is called every ArRobot
    // task cycle (every 100ms). The function is defined below.
    myRobot->addUserTask("controlTask", 50, &myTaskCB);

    // Estas funciones están en ArPathPlanningInterface.h
    myPathTask->addGoalDoneCB(&myGoalDoneCB);
    myPathTask->addGoalFailedCB(&myGoalFailedCB);
    myPathTask->addGoalInterruptedCB(&myGoalInterruptCB);
    myPathTask->addStateChangeCB(&pathPlanStateChangedCB);

    myLocTask->addFailedLocalizationCB(&locFailedCB);

    srand(time(NULL)); // usar srand(1) si se quiere la misma serie
                       // de aleatorios siempre

    // para guardar datos
    file.setFileName("goalsLog.txt");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
        out.setDevice(&file);

    timeCounter = 0;
    oldPose = myRobot->getPose();

    // 1000 dseg. max. para ejecución de Command Functions, se puede aumentar si se quiere
    maxTimeCommandFunctions = 1000;

    // Primera pose de prueba a 1 m. a la derecha de Home
    newPose.setPose(myRobot->getPose().getX() + 4500, myRobot->getPose().getY(), myRobot->getPose().getTh());
}

void Control::execute(void)
{
    if (!myGoalDone)
        return;

    // Para ejecución de Command Functions sin llamar a sleep()
    // ya que execute es una userTask, se llama cada 100 ms y
    // un sleep() entraría en conflicto
    if(timeCounter > 0) {
        timeCounter--;
        // Ejemplo de retroceder 500 mm para escapar de un supuesto obstáculo frontal
        // puede parar el robot si tras el escape se está en una posición
        // inválida para ArPathPlanningTask.
        // Para salir de esa posición usar las flechas de movimiento en MobileEyes
        if(oldPose.findDistanceTo(myRobot->getPose()) > 500) {

           // 5 décimas para parar
           timeCounter = timeCounter > 5 ? 5 : timeCounter ;

           myRobot->lock();
           myRobot->stop();
           myRobot->unlock();
        }
        else {
            myRobot->lock();
            myRobot->setVel(-100);
            myRobot->unlock();
        }
    }
    if(timeCounter == 0) {
        myGoalDone = false;

        // Habilitamos Actions anulando de Command Functions
        myRobot->clearDirectMotion();

        myPathTask->pathPlanToPose(newPose, false); // false, el heading no se tiene en cuenta
    }
    else
        ArLog::log(ArLog::Normal, "Control: waiting: %d dseg.", timeCounter);

}

Control::~Control()
{
    file.close();
}

void Control::goalDone(ArPose pose)
{
    QString sPose = QString("Goal(%1, %2, %3) done").
            arg(pose.getX()).arg(pose.getY()).arg(pose.getTh());
    ArLog::log(ArLog::Normal, "Control: goalDone: %s", sPose.toLatin1().data());

    // Grabado datos
    out << sPose << "\n";
    out.flush();

    oldPose = pose;
    myGoalDone = true;
    generateRandomPose();
}

void Control::goalFailed(ArPose pose)
{
    QString sPose = QString("Goal(%1, %2, %3) failed").
            arg(pose.getX()).arg(pose.getY()).arg(pose.getTh());
    ArLog::log(ArLog::Normal, "Control: goalFailed: %s", sPose.toLatin1().data());

    // Grabado datos
    out << sPose << "\n";
    out.flush();

    // esta función está como ejemplo de cómo llamar a CommandFunctions
    // en execute() desconectando las acciones, p. ej. llamamos a escape()

    // escape();

    myGoalDone = true;
    generateRandomPose();
}

void Control::goalInterrupt(ArPose pose)
{
    QString sPose = QString("Goal(%1, %2, %3) failed").
            arg(pose.getX()).arg(pose.getY()).arg(pose.getTh());
    ArLog::log(ArLog::Normal, "Control: goalInterrupt: %s", sPose.toLatin1().data());
    // TODO
}

void Control::generateRandomPose()
{
    // Solo para pruebas
    ArPose poseMin = myPathTask->getAriaMap()->getLineMinPose();
    ArPose poseMax = myPathTask->getAriaMap()->getLineMaxPose();
    int ancho = int(poseMax.getX() - poseMin.getX());
    int alto = int(poseMax.getY() - poseMin.getY());
    newPose.setX(poseMin.getX() + rand() % ancho);
    newPose.setY(poseMin.getY() + rand() % alto);
    newPose.setTh(rand() % 360);
}

// This function is called whenever the path planning task changes its state
// (for example, from idle to planning a path, to following a planned path).
void Control::pathPlanStateChanged()
{
    char s[256];
    myPathTask->getFailureString(s, 256);
    ArLog::log(ArLog::Normal, "Control: Path planning state: %s", s);
    // TODO
}

void Control::locFailed(int n) // Número de veces que ha fallado la localización
{
    // Si se llega aquí no hay nada que hacer, sino empezar de nuevo el experimento
    ArLog::log(ArLog::Normal, "Control: Localization failed: %d", n);
}

void Control::escape() {
    oldPose = myRobot->getPose();

    // Deshabilitamos Actions en favor de CommandFunctions
    myRobot->setDirectMotionPrecedenceTime(0);

    timeCounter = maxTimeCommandFunctions;
}
