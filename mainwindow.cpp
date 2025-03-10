#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QFileDialog>
#include <chrono>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#if VTK_MAJOR_VERSION > 8
#include <vtkGenericOpenGLRenderWindow.h>
#endif

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    setFileMenuActions();
    // Set up the QVTK window
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    _viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    ui->qvtkWidget->setRenderWindow(_viewer->getRenderWindow());
    _viewer->setupInteractor(ui->qvtkWidget->interactor(), ui->qvtkWidget->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
#endif
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setFileMenuActions()
{
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionExit, &QAction::triggered, this, &QApplication::quit);
}

void MainWindow::refreshView()
{
#if VTK_MAJOR_VERSION > 8
    ui->qvtkWidget->renderWindow()->Render();
#else
    ui->qvtkWidget->update();
#endif
}

void MainWindow::openFile()
{
    QWidget* temp { new QWidget() };
    QString cloudFile {
        QFileDialog::getOpenFileName(temp, tr("Open Image"), "/home/user/fx/pclfx/data",
            tr("Point Cloud File (*.pcd "
               "*.ply "
               "*.obj)"))
    };

    loadCloud(cloudFile.toStdString());
    visualize();
    refreshView();
    delete temp;
}

int MainWindow::loadCloud(const std::string& filename)
{
    // PointCloudT::Ptr cloud { new PointCloudT };
    // _cloud = std::move(cloud);
    auto format { get_format(filename) };
    switch (format) {
    case FILEFORMATS::pcd:
        if (pcl::io::loadPCDFile(filename, *_cloud) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    case FILEFORMATS::ply:
        if (pcl::io::loadPLYFile(filename, *_cloud) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    case FILEFORMATS::obj:
        if (pcl::io::loadOBJFile(filename, *_cloud) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    default:
        PCL_ERROR("Unsupported file format, %s \n", filename);
        return (-1);
    }

    //* load the file
    std::cout << "Loaded " << _cloud->width * _cloud->height << " data points from "
              << filename << " with the following fields: " << std::endl;

    return 0;
}

void MainWindow::visualize()
{
    // Visualize the point cloud
    _viewer->removeAllPointClouds();
    _viewer->addPointCloud(_cloud);
    _viewer->spin();

    // // Wait until the window closes
    while (!_viewer->wasStopped()) {
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }
}
