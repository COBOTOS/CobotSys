#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "JointsDataManager.h"
#include <LocalStorageRepositoryInterface.h>
#include <RobotDriverInterface.h>
#include <map>
#include <string>
#include <vector>
#if 0

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private Q_SLOTS:
    void on_tableWidget_activated(const QModelIndex &index);

    void on_pushButton_clicked();

    void on_pushButton_connect_clicked();

    void on_pushButton_disconnect_clicked();

    void on_pushButton_openfile_clicked();

    void on_pushButton_save_clicked();

    void on_pushButton_delete_clicked();

    void on_pushButton_modify_clicked();
private:
    void refresh();
    int findRow(int key);//都是私有的
    void addData();
private:
    Ui::MainWindow *ui;
    int _row;
    int _colum;
    int _count;
    std::vector<double> _joints;
    QTimer * _timer;
    boost::shared_ptr<LION_LSR_NS_API::LocalStorageRepositoryInterface> _lsr;
    boost::shared_ptr<WOLF_ROBOTDRIVER_NS_API::RobotDriverInterface> _robotDriver;
    boost::shared_ptr<JointsDataManager> _jointDataManager;
};

#endif // MAINWINDOW_H
#endif