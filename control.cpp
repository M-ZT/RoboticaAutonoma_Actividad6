
#include "control.h"
#include <vector>

// Grid parameters
const int CELL_SIZE = 500; // Size of each cell in mm
std::vector<std::vector<bool>> grid; // Grid to track visited cells
int gridWidth, gridHeight;

Control::Control(ArRobot *robot, ArPathPlanningTask *pathTask,
                 ArLocalizationTask *locTask) :
    myTaskCB(this, &Control::execute),
    myGoalDoneCB(this, &Control::goalDone),
    myGoalFailedCB(this, &Control::goalFailed),
    myGoalInterruptCB(this, &Control::goalInterrupt),
    pathPlanStateChangedCB(this, &pathPlanStateChanged),
    locFailedCB(this, &locFailed)
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

    srand(time(NULL)); // usar srand(1) si se quiere la misma serie de aleatorios siempre

    // para guardar datos
    file.setFileName("goalsLog.txt");
    if (file.open(QIODevice::WriteOnly | QIODevice::Text))
        out.setDevice(&file);

    timeCounter = 0;
    oldPose = myRobot->getPose();

    // 1000 dseg. max. para ejecución de Command Functions, se puede aumentar si se quiere
    maxTimeCommandFunctions = 1000;

    // Inicializamos la rejilla para el seguimiento de celdas
    initializeGrid();

    // Establecemos el primer objetivo en el grid
    setNextGoal();
}

void Control::initializeGrid() {
    // Obtenemos las dimensiones del mapa a partir de la clase de planificación
    ArPose poseMin = myPathTask->getAriaMap()->getLineMinPose();
    ArPose poseMax = myPathTask->getAriaMap()->getLineMaxPose();

    // Calculamos el número de celdas en las direcciones X e Y
    gridWidth = (poseMax.getX() - poseMin.getX()) / CELL_SIZE;
    gridHeight = (poseMax.getY() - poseMin.getY()) / CELL_SIZE;

    // Creamos una matriz bidimensional para rastrear celdas visitadas
    grid.resize(gridHeight, std::vector<bool>(gridWidth, false));
}

void Control::setNextGoal() {
    // Iteramos sobre la rejilla para encontrar la siguiente celda no visitada
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            if (!grid[y][x]) {
                // Establecemos la posición objetivo como el centro de la celda
                ArPose nextPose(x * CELL_SIZE, y * CELL_SIZE);
                newPose = nextPose;
                return;
            }
        }
    }
    // Si todas las celdas están visitadas o son inaccesibles, lo registramos
    ArLog::log(ArLog::Normal, "Control: All cells visited or inaccessible.");
}

void Control::execute(void) {
    if (!myGoalDone)
        return;

    // Para ejecución de Command Functions sin llamar a sleep()
    // ya que execute es una userTask, se llama cada 100 ms y
    // un sleep() entraría en conflicto
    if (timeCounter > 0) {
        timeCounter--;
        if (oldPose.findDistanceTo(myRobot->getPose()) > 500) {
            // Detenemos el robot si se ha alejado lo suficiente
            timeCounter = std::min(timeCounter, 5);
            myRobot->lock();
            myRobot->stop();
            myRobot->unlock();
        } else {
            // Continuamos moviéndonos hacia atrás si es necesario
            myRobot->lock();
            myRobot->setVel(-100);
            myRobot->unlock();
        }
        return;
    }

    // Reiniciamos el contador y planificamos hacia el próximo objetivo
    timeCounter = 0;
    myGoalDone = false;
    myRobot->clearDirectMotion();
    myPathTask->pathPlanToPose(newPose, false);
}

Control::~Control() {
    file.close();
}

void Control::goalDone(ArPose pose) {
    QString sPose = QString("Goal(%1, %2, %3) done").arg(pose.getX()).arg(pose.getY()).arg(pose.getTh());
    ArLog::log(ArLog::Normal, "Control: goalDone: %s", sPose.toLatin1().data());

    // Grabamos los datos del objetivo alcanzado en el archivo
    out << sPose << "\n";
    out.flush();

    // Marcamos la celda correspondiente como visitada en la rejilla
    int x = pose.getX() / CELL_SIZE;
    int y = pose.getY() / CELL_SIZE;
    grid[y][x] = true;

    // Actualizamos la pose y establecemos el siguiente objetivo
    oldPose = pose;
    myGoalDone = true;
    setNextGoal();
}

void Control::goalFailed(ArPose pose) {
    QString sPose = QString("Goal(%1, %2, %3) failed").arg(pose.getX()).arg(pose.getY()).arg(pose.getTh());
    ArLog::log(ArLog::Normal, "Control: goalFailed: %s", sPose.toLatin1().data());

    // Grabamos los datos del fallo en el archivo
    out << sPose << "\n";
    out.flush();

    // Marcamos la celda correspondiente como inaccesible en la rejilla
    int x = pose.getX() / CELL_SIZE;
    int y = pose.getY() / CELL_SIZE;
    grid[y][x] = false;

    // Intentamos establecer un nuevo objetivo
    myGoalDone = true;
    setNextGoal();
}

void Control::goalInterrupt(ArPose pose) {
    // Registramos interrupciones inesperadas, como una parada manual
    ArLog::log(ArLog::Normal, "Control: goalInterrupt at (%f, %f)", pose.getX(), pose.getY());
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
void Control::pathPlanStateChanged() {
    // Registramos cambios en el estado de la planificación de trayectorias
    char s[256];
    myPathTask->getFailureString(s, 256);
    ArLog::log(ArLog::Normal, "Control: Path planning state: %s", s);
}

void Control::locFailed(int n) {
    // Si la localización falla, reiniciamos el experimento
    ArLog::log(ArLog::Normal, "Control: Localization failed: %d", n);
}
/*
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



*/