#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QImage>
#include <QPixmap>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_buttonStart_clicked()
{
    ui->buttonStart->setEnabled(false);
    ui->valueSampleSize->setEnabled(true);
    ui->backButton1->setEnabled(true);
    ui->buttonSamplePoints->setEnabled(true);
}

void MainWindow::on_buttonSamplePoints_clicked()
{
    ui->valueSampleSize->setEnabled(false);
    ui->backButton1->setEnabled(false);
    ui->buttonSamplePoints->setEnabled(false);
    ui->backButton2->setEnabled(true);
    ui->buttonPartitionVoronoi->setEnabled(true);
}

void MainWindow::on_buttonPartitionVoronoi_clicked()
{
    ui->backButton2->setEnabled(false);
    ui->buttonPartitionVoronoi->setEnabled(false);
    ui->valueMaxDiviation->setEnabled(true);
    ui->buttonUndoPoint->setEnabled(true);
    ui->buttonAddPoint->setEnabled(true);
    ui->backButton3->setEnabled(true);
    ui->buttonAddAll->setEnabled(true);
}

void MainWindow::on_buttonUndoPoint_clicked()
{

}

void MainWindow::on_buttonAddPoint_clicked()
{

}

void MainWindow::on_buttonAddAll_clicked()
{
    ui->valueMaxDiviation->setEnabled(false);
    ui->buttonUndoPoint->setEnabled(false);
    ui->buttonAddPoint->setEnabled(false);
    ui->backButton3->setEnabled(false);
    ui->buttonAddAll->setEnabled(false);
    ui->backButton4->setEnabled(true);
    ui->buttonUniformizeIntensity->setEnabled(true);
}

void MainWindow::on_buttonUniformizeIntensity_clicked()
{
    ui->backButton4->setEnabled(false);
    ui->buttonUniformizeIntensity->setEnabled(false);
    ui->valueMinArea->setEnabled(true);
    ui->backButton5->setEnabled(true);
    ui->buttonMergeCells->setEnabled(true);
}

void MainWindow::on_buttonMergeCells_clicked()
{
    ui->valueMinArea->setEnabled(false);
    ui->backButton5->setEnabled(false);
    ui->buttonMergeCells->setEnabled(false);
    ui->valueMinWidth->setEnabled(true);
    ui->backButton6->setEnabled(true);
    ui->buttonCreateBorders->setEnabled(true);
}

void MainWindow::on_buttonCreateBorders_clicked()
{
    ui->valueMinWidth->setEnabled(false);
    ui->backButton6->setEnabled(false);
    ui->buttonCreateBorders->setEnabled(false);
    ui->valueBudget->setEnabled(true);
    ui->backButton7->setEnabled(true);
    ui->buttonSetDensities->setEnabled(true);
}

void MainWindow::on_buttonSetDensities_clicked()
{
    ui->valueBudget->setEnabled(false);
    ui->backButton7->setEnabled(false);
    ui->buttonSetDensities->setEnabled(false);
    ui->buttonStop->setEnabled(true);
}

void MainWindow::on_buttonStop_clicked()
{

}

void MainWindow::on_backButton1_clicked()
{
    ui->valueSampleSize->setEnabled(false);
    ui->backButton1->setEnabled(false);
    ui->buttonSamplePoints->setEnabled(false);
    ui->buttonStart->setEnabled(true);
}

void MainWindow::on_backButton2_clicked()
{
    ui->backButton2->setEnabled(false);
    ui->buttonPartitionVoronoi->setEnabled(false);
    ui->valueSampleSize->setEnabled(true);
    ui->backButton1->setEnabled(true);
    ui->buttonSamplePoints->setEnabled(true);
}

void MainWindow::on_backButton3_clicked()
{
    ui->valueMaxDiviation->setEnabled(false);
    ui->buttonUndoPoint->setEnabled(false);
    ui->buttonAddPoint->setEnabled(false);
    ui->backButton3->setEnabled(false);
    ui->buttonAddAll->setEnabled(false);
    ui->backButton2->setEnabled(true);
    ui->buttonPartitionVoronoi->setEnabled(true);
}

void MainWindow::on_backButton4_clicked()
{
    ui->backButton4->setEnabled(false);
    ui->buttonUniformizeIntensity->setEnabled(false);
    ui->valueMaxDiviation->setEnabled(true);
    ui->buttonUndoPoint->setEnabled(true);
    ui->buttonAddPoint->setEnabled(true);
    ui->backButton3->setEnabled(true);
    ui->buttonAddAll->setEnabled(true);
}

void MainWindow::on_backButton5_clicked()
{
    ui->valueMinArea->setEnabled(false);
    ui->backButton5->setEnabled(false);
    ui->buttonMergeCells->setEnabled(false);
    ui->backButton4->setEnabled(true);
    ui->buttonUniformizeIntensity->setEnabled(true);
}

void MainWindow::on_backButton6_clicked()
{
    ui->valueMinWidth->setEnabled(false);
    ui->backButton6->setEnabled(false);
    ui->buttonCreateBorders->setEnabled(false);
    ui->valueMinArea->setEnabled(true);
    ui->backButton5->setEnabled(true);
    ui->buttonMergeCells->setEnabled(true);
}

void MainWindow::on_backButton7_clicked()
{
    ui->valueBudget->setEnabled(false);
    ui->backButton7->setEnabled(false);
    ui->buttonSetDensities->setEnabled(false);
    ui->valueMinWidth->setEnabled(true);
    ui->backButton6->setEnabled(true);
    ui->buttonCreateBorders->setEnabled(true);
}
