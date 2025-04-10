#include "mainwindow.h"
#include <QFileDialog>
#include <QLineEdit>
#include <QShortcut>
#include <iostream>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#if VTK_MAJOR_VERSION > 8
#include <QMessageBox>
#include <vtkGenericOpenGLRenderWindow.h>
#endif

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _filter(new Filter(_pc))
    , _ransac(new Ransac(_pc))
{
    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");
    _tabWidget = new QTabWidget(ui->centralWidget);
    _tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    _tabWidget->setGeometry(QRect(10, 10, 1940, 1100));

    initParams();
    setFileMenuActions();
    setButtonsActions();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete _filter;
    delete _ransac;
    delete _tabWidget;
    delete _vtkWidget;
}

void MainWindow::setFileMenuActions()
{
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionExit, &QAction::triggered, this, &QApplication::quit);
}

void MainWindow::initParams()
{
    if (_filter) {
        _filter->stdDev(ui->leStdDev->text().toDouble());
        _filter->kMean(ui->leKMean->text().toUInt());
        _filter->vg_x(ui->leLeafSizex->text().toDouble());
        _filter->vg_x(ui->leLeafSizex->text().toDouble());
        _filter->vg_y(ui->leLeafSizey->text().toDouble());
        _filter->vg_z(ui->leLeafSizez->text().toDouble());
    }
}

void MainWindow::setButtonsActions()
{
    connect(ui->leStdDev, &QLineEdit::returnPressed, this, [this]() {
        _filter->stdDev(ui->leStdDev->text().toDouble());
    });
    connect(ui->leKMean, &QLineEdit::returnPressed, this, [this]() {
        _filter->kMean(ui->leKMean->text().toUInt());
    });
    connect(ui->pbRemoveOutliers, &QPushButton::released, this, [this]() {
        _filter->outlierRemoval();
    });
    auto warnMessage = []() {
        QMessageBox msgBox;
        msgBox.setText("Set Parameters and push Remove Outliers button first.");
        msgBox.exec();
    };
    connect(ui->pbShowInliers, &QPushButton::released, this, [this, &warnMessage]() {
        auto inliers{_filter->sorInliers()};
        (!inliers->cloud->empty()) ? this->visualizeInNewTab(inliers) : warnMessage();
    });
    connect(ui->pbShowOutliers, &QPushButton::released, this, [this, &warnMessage]() {
        auto outliers{_filter->sorOutliers()};
        (!outliers->cloud->empty()) ? this->visualizeInNewTab(outliers) : warnMessage();
    });

    connect(ui->leLeafSizex, &QLineEdit::returnPressed, this, [this]() {
        _filter->vg_x(ui->leLeafSizex->text().toDouble());
    });
    connect(ui->leLeafSizey, &QLineEdit::returnPressed, this, [this]() {
        _filter->vg_y(ui->leLeafSizey->text().toDouble());
    });
    connect(ui->leLeafSizez, &QLineEdit::returnPressed, this, [this]() {
        _filter->vg_z(ui->leLeafSizez->text().toDouble());
    });

    connect(ui->pbVoxelGrid, &QPushButton::released, this, [this]() {
        visualizeInNewTab(_filter->voxelGridFilter());
    });
    connect(ui->pbRansac, &QPushButton::released, this, [this]() {
        _ransac->ransacSegmentation();
        visualizeInNewTab<_PointCloud<pcl::PointXYZRGB>>(_ransac->getResult());
    });
}

void MainWindow::openFile()
{
    std::unique_ptr<QWidget> temp { std::make_unique<QWidget>() };
    QString cloudFile{QFileDialog::getOpenFileName(temp.get(),
                                                   tr("Open Image"),
                                                   "/home/fx/rf/pclfx/data/tutorials",
                                                   tr("Point Cloud File (*.pcd "
                                                      "*.ply "
                                                      "*.obj)"))};

    if (cloudFile.isEmpty())
        return;

    loadCloud(cloudFile.toStdString());
    visualizeInNewTab(_pc);
}

int MainWindow::loadCloud(std::string_view filename)
{
    auto format{get_format(filename.data())};
    switch (format) {
    case FILEFORMATS::pcd:
        if (pcl::io::loadPCDFile(filename.data(), *(_pcXYZ->cloud)) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    case FILEFORMATS::ply:
        if (pcl::io::loadPLYFile(filename.data(), *(_pcXYZ->cloud)) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    case FILEFORMATS::obj:
        if (pcl::io::loadOBJFile(filename.data(), *(_pcXYZ->cloud)) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    default:
        PCL_ERROR("Unsupported file format, %s \n", filename);
        return (-1);
    }
    _pc->cloud->points.reserve(_pcXYZ->cloud->points.size());
    // xyz2xyzrgb(_pcXYZ, _pc, 70, 220, 10);
    _pc->cloud = _pcXYZ->cloud;
    _pc->id = _pcXYZ->id;
    // Get only the filename and set it as the PointCloud id
    auto it { std::find(filename.crbegin(), filename.crend(), '/') };
    auto last_slash { std::distance(filename.crbegin(), it) };
    _pc->id = filename.substr(filename.length() - last_slash, filename.length());

    for (auto point : *_pc->cloud) {
        point.x = 1.0f;
        point.y = 1.0f;
        point.z = 1.0f;
    }
    // Print some Info
    std::cout << "Loaded " << _pc->cloud->width * _pc->cloud->height << " data points from "
              << filename << " with the following fields: " << std::endl
              << "Width: " << _pc->cloud->width << "\tHeight: " << _pc->cloud->height << "\nHeader: "
              << _pc->cloud->header << "\nIs " << (_pc->cloud->is_dense ? "" : " not") << " Dense" << std::endl;

    return 0;
}

template<typename PC>
void MainWindow::visualizeInNewTab(const std::shared_ptr<PC> pc)
{
    _vtkWidget = newTab(pc->id);
    visualize(pc);
    refreshView();
}

/**  Visualize the point cloud **/
template<typename PC>
void MainWindow::visualize(const std::shared_ptr<PC> pc)
{
    // Set up the QVTK window
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    _viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    _vtkWidget->setRenderWindow(_viewer->getRenderWindow());
    _viewer->setupInteractor(_vtkWidget->interactor(), _vtkWidget->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    _viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#endif
    _viewer->removeAllPointClouds();
    _viewer->addPointCloud(pc->cloud, pc->id);

    // Wait until the window closes
    while (!_viewer->wasStopped()) {
        _viewer->spinOnce(100);
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(100ms);
    }
}

void MainWindow::refreshView()
{
#if VTK_MAJOR_VERSION > 8
    _vtkWidget->renderWindow()->Render();
#else
    _vtkWidget->update();
#endif
}

PCLQVTKWidget *MainWindow::newTab(std::string_view tab_name)
{
    QWidget* new_tab { new QWidget() };
    new_tab->setObjectName(QString::fromUtf8("tab0"));
    PCLQVTKWidget* vtkWidget = new PCLQVTKWidget(new_tab);
    vtkWidget->setObjectName(QString::fromUtf8("vtkWidge"));
    vtkWidget->setGeometry(12, 12, 1920, 1080);
    _tabWidget->addTab(new_tab, QString::fromStdString(tab_name.data()));
    // auto shortcut = QShortcut (QKeySequence ("Ctrl+w"), _tabWidget);
    _tabWidget->setCurrentWidget(new_tab);

    return vtkWidget;
}
