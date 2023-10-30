#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>


QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_buttonStart_clicked();
    void on_buttonSamplePoints_clicked();
    void on_buttonPartitionVoronoi_clicked();
    void on_buttonUndoPoint_clicked();
    void on_buttonAddPoint_clicked();
    void on_buttonAddAll_clicked();
    void on_buttonUniformizeIntensity_clicked();
    void on_buttonMergeCells_clicked();
    void on_buttonCreateBorders_clicked();
    void on_buttonSetDensities_clicked();
    void on_buttonStop_clicked();
    void on_backButton1_clicked();
    void on_backButton2_clicked();
    void on_backButton3_clicked();
    void on_backButton4_clicked();
    void on_backButton5_clicked();
    void on_backButton6_clicked();
    void on_backButton7_clicked();

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
