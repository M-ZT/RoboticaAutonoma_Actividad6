#ifndef CONTROL_H
#define CONTROL_H

#include <Aria.h>
#include <ArNetworking.h>
#include <Arnl.h>
#include <ArLocalizationTask.h>

#include <QFile>
#include <QTextStream>

class Control
{
public:
    Control(ArRobot *robot, ArPathPlanningTask *myPathTask,
            ArLocalizationTask *locTask);
    virtual ~Control();
    void execute(void);
    void goalDone(ArPose pose);
    void goalFailed(ArPose pose);
    void goalInterrupt(ArPose pose);
    void pathPlanStateChanged();

    void locFailed(int n);

private:
    void generateRandomPose();
    void escape();

protected:
    ArRobot *myRobot;
    ArPathPlanningTask *myPathTask;
    ArLocalizationTask *myLocTask;

    bool myGoalDone;
    ArPose oldPose, newPose;
    ArFunctorC<Control> myTaskCB;
    ArFunctor1C<Control, ArPose> myGoalDoneCB;
    ArFunctor1C<Control, ArPose> myGoalFailedCB;
    ArFunctor1C<Control, ArPose> myGoalInterruptCB;
    ArFunctorC<Control> pathPlanStateChangedCB;

    ArFunctor1C<Control, int> locFailedCB;

    QFile file;
    QTextStream out;

    int timeCounter;
    int maxTimeCommandFunctions;

};

#endif // CONTROL_H
