#ifndef MAIN_WIDGET_H
#define MAIN_WIDGET_H

// QT
#include <QMainWindow>
#include "QDataItemTree.h"

namespace Ui {
class QMainWidget;
}

class QMainWidget : public QMainWindow
{
    Q_OBJECT

public:
    explicit QMainWidget(QWidget *parent = 0);
    ~QMainWidget();

public slots:

    void load();

private:

    Ui::QMainWidget *ui;

    QDataItemTree* tree_view_model_;

    int n_loaded_;
};

#endif // MAIN_WIDGET_H
