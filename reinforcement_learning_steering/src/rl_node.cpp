#include <reinforcement_learning_steering/RLMainWindow.h>
#include <QApplication>
int main(int argc, char *argv[])
{
    if(argc>1)
    {
        ros::init(argc, argv, "rl_node");
        ros::AsyncSpinner spinner(2);
        spinner.start();
        QApplication a(argc, argv);
        reinforcement_learning_steering::RLMainWindow w{argv[1]};
        w.show();
        return a.exec();

    }
    else
    {
        ROS_ERROR_STREAM("Please enter robot namespace as argv[1].");
    }
    
}