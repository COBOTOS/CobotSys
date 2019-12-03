#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include "boost/shared_ptr.hpp"
#include <boost/make_shared.hpp>
#include "TaskMonitor.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow {
Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    ~MainWindow();

private slots:

    void on_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    boost::shared_ptr<TaskMonitor> _taskMonitor;
};

#endif // MAINWINDOW_H
