#include "reinforcement_learning_steering/RLMainWindow.h"
#include "ui_mainwindow.h"
#include <qcustomplot.h>

namespace reinforcement_learning_steering
{

RLMainWindow ::RLMainWindow(const std::string &robot_namespace, QWidget *parent)
    : QMainWindow(parent),
      current_size_(0),
      dataTimer(new QTimer()),
      ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->label_nm_v_->setText(QString::fromStdString(robot_namespace));
    InitChart(robot_namespace);
    InitRos(robot_namespace);
}

RLMainWindow::~RLMainWindow()
{
    delete ui;
}

void RLMainWindow::InitChart(const std::string &robot_namespace)
{
    auto &custom_plot = this->ui->CustomPlot;
    custom_plot->xAxis->setLabel("episode");
    custom_plot->yAxis->setLabel("total reward");
    custom_plot->addGraph();
    custom_plot->graph(0)->setPen(QPen(QColor(40, 110, 255)));
    connect(dataTimer, SIGNAL(timeout()), this, SLOT(realtimePlot()));
    dataTimer->start(5);
}
void RLMainWindow::InitRos(const std::string &robot_namespace)
{
    std::string topic_name = robot_namespace + "/rl";
    if (!ros::isInitialized())
    {
        std::string node_name = robot_namespace + "/rl_statistics";
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, node_name,
                  ros::init_options::NoSigintHandler);
    }

    this->nh_ = std::make_unique<ros::NodeHandle>();
    this->sub_ = this->nh_->subscribe(topic_name, 10, &RLMainWindow::RlCallback, this);
}

void RLMainWindow::setLabeles(const reinforcement_learning_steering::RLSimulation &rl_state)
{
    this->ui->label_ce_v_->setText(QString::number(rl_state.episode_iteration));
    this->ui->label_lr_v_->setText(QString::number(rl_state.current_episode.last_reward));
    this->ui->label_tr_v_->setText(QString::number(rl_state.current_episode.total_reward));
    this->ui->label_it_v_->setText(QString::number(rl_state.current_episode.iteration));
}

void RLMainWindow::realtimePlot()
{
    if (current_size_ < this->rl_msgs_.size())
    {
        ui->CustomPlot->graph(0)->addData(this->rl_msgs_.rbegin()->episode_iteration,
                                          this->rl_msgs_.rbegin()->current_episode.total_reward);

        auto ranges = getYAxisRanges();
        ui->CustomPlot->xAxis->setRange(0, this->rl_msgs_.size() + 1);
        ui->CustomPlot->yAxis->setRange(ranges.first, ranges.second);
        ui->CustomPlot->replot();
        this->current_size_++;
    }
}

std::pair<double, double> RLMainWindow::getYAxisRanges()
{
    auto range_it = std::minmax_element(this->rl_msgs_.begin(), this->rl_msgs_.end(), [](const auto &rl_r_msg, const auto &rl_l_msg) {
        return rl_r_msg.current_episode.total_reward < rl_l_msg.current_episode.total_reward;
    });

    return std::pair<double, double>(range_it.first->current_episode.total_reward, range_it.second->current_episode.total_reward);
}

void RLMainWindow::RlCallback(const reinforcement_learning_steering::RLSimulation &rl_state)
{
    setLabeles(rl_state);
    if (rl_state.terminate)
    {
        this->rl_msgs_.insert(rl_state);
    }
}

} // namespace reinforcement_learning_steering