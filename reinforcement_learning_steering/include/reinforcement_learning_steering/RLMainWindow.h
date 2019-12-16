#ifndef REINFORCEMENT_LEARNING_STEERING_RL_STATISTICS_H_
#define REINFORCEMENT_LEARNING_STEERING_RL_STATISTICS_H_

#include <memory>
#include <algorithm>

#include <QMainWindow>
#include <QTimer>
#include <QSharedPointer>
#include <QPrinter>

#include <qcustomplot.h>

#include <ros/ros.h>
#include "reinforcement_learning_steering/RLSimulation.h"
#include <reinforcement_learning_steering/Util.h>

namespace Ui
{
class MainWindow;
}

namespace reinforcement_learning_steering
{
class RLMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    RLMainWindow(const std::string &robot_namespace, QWidget *parent = nullptr);
    virtual ~RLMainWindow();

private:
    void InitChart(const std::string & robot_namespace);
    void InitRos(const std::string &robot_namespace);
    void RlCallback(const reinforcement_learning_steering::RLSimulation & rl_state);
    void setLabeles(const reinforcement_learning_steering::RLSimulation & rl_state);
    void AddEpisodeToChart(const reinforcement_learning_steering::RLSimulation & rl_state);
    std::pair<double, double> getYAxisRanges();
    
    size_t current_size_;
    QTimer * dataTimer;
    Ui::MainWindow *ui;
    std::set<reinforcement_learning_steering::RLSimulation, util::RlEpisodeComperator> rl_msgs_;
    std::unique_ptr<ros::NodeHandle> nh_;
    ros::Subscriber sub_;

public slots:
    void realtimePlot();
};

} // namespace reinforcement_learning_steering

#endif
