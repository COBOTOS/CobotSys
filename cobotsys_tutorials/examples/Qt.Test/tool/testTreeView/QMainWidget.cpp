#include "QMainWidget.h"
#include "ui_QMainWidget.h"

// Qt
#include <QTreeView>
#include <QInputDialog>

QMainWidget::QMainWidget(QWidget *parent) :
    QMainWindow(parent),n_loaded_(0),
    ui(new Ui::QMainWidget)
{
    // Set up Ui
    ui->setupUi(this);

    // Data tree
    tree_view_model_ = new QDataItemTree("Root");
    ui->treeView->setModel(tree_view_model_);

    // Connect load
    connect(ui->addElements_pushButton,SIGNAL(clicked()),this,SLOT(load()));
}

QMainWidget::~QMainWidget()
{
    delete ui;
}

void QMainWidget::load()
{
    // How many elements to add
    int n_elements = QInputDialog::getInt(this,"Elements","How many elemtns?",10);

    // How many childs per element
    int n_childs_per_element = QInputDialog::getInt(this,"Elements","How many elemtns?",10);

    // Add batch to tree
    QDataItem* batch = tree_view_model_->addBatch(n_loaded_);
    for (int i = 0; i < n_elements; i++)
    {
        QDataItem* element = tree_view_model_->addBatchElement(i,batch);
        for (int j = 0; j <= n_childs_per_element; j++)
        {
            tree_view_model_->addBatchElementChild(j,element);

        }
    }
    n_loaded_++;
    ui->treeView->setModel(tree_view_model_);

}
