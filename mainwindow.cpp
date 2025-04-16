#include "mainwindow.h"
#include <QFileDialog>
#include <QLineEdit>
#include <QShortcut>
#include <filesystem>
#include <iostream>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#if VTK_MAJOR_VERSION > 8
#include <QMessageBox>
#include <vtkGenericOpenGLRenderWindow.h>
#endif

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _filter{std::make_unique<Filter>(_pc)}
    , _ransac{std::make_unique<Ransac>(_pc)}
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
    if (ui)
        delete ui;
}

void MainWindow::initParams()
{
    if (_filter) {
        _filter->stdDev(ui->leStdDev->text().toDouble());
        _filter->kMean(ui->leKMean->text().toUInt());
        _filter->vg_x(ui->leLeafSizex->text().toDouble());
        _filter->vg_y(ui->leLeafSizey->text().toDouble());
        _filter->vg_z(ui->leLeafSizez->text().toDouble());
    }
}

void MainWindow::setFileMenuActions()
{
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionExit, &QAction::triggered, this, &QApplication::quit);
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
    connect(ui->pbShowInliers, &QPushButton::released, this, [this, warnMessage]() {
        auto inliers{_filter->sorInliers()};
        (!inliers->cloud->empty()) ? this->visualizeInNewTab(inliers) : warnMessage();
    });
    connect(ui->pbShowOutliers, &QPushButton::released, this, [this, warnMessage]() {
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
    std::string cloudFile{(QFileDialog::getOpenFileName(temp.get(),
                                                        tr("Open Image"),
                                                        "/home/fx/rf/pclfx/data/tutorials",
                                                        tr("Point Cloud File (*.pcd "
                                                           "*.ply "
                                                           "*.obj *.PCD *.PLY *.OBJ *.*)")))
                              .toStdString()};

    if (cloudFile.empty())
        return;

    if (loadCloud(cloudFile))
        visualizeInNewTab(_pc);
    else
        std::cout << "Unsupported or invalid file format\n";
}

bool MainWindow::loadCloud(std::string_view filename)
{
    if (!Utils::loadFile(filename, _pc)) //unsupported file format
        return false;

    // xyz2xyzrgb(_pcXYZ, _pc, 70, 220, 10);
    // pcl::copyPointCloud(*_pcXYZ->cloud, *_pc->cloud);
    // set filename as the point cloud id
    _pc->id = std::filesystem::path(filename).filename().string();

    // Print some Info
    std::cout << "Loaded " << _pc->cloud->width * _pc->cloud->height << " data points from "
              << filename << " with the following fields: " << std::endl
              << "Width: " << _pc->cloud->width << "\tHeight: " << _pc->cloud->height << "\nHeader: "
              << _pc->cloud->header << "\nIs " << (_pc->cloud->is_dense ? "" : " not") << " Dense" << std::endl;
    return true;
}

template<typename PC>
void MainWindow::visualizeInNewTab(const std::shared_ptr<PC> pc)
{
    newTab(pc->id);
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
    _vtkWidgets.back()->setRenderWindow(_viewer->getRenderWindow());
    _viewer->setupInteractor(_vtkWidgets.back()->interactor(), _vtkWidgets.back()->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    _viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#endif
    _viewer->removeAllPointClouds();
    _viewer->addPointCloud(pc->cloud, pc->id);
}

void MainWindow::refreshView()
{
#if VTK_MAJOR_VERSION > 8
    _vtkWidgets.back()->renderWindow()->Render();
#else
    _vtkWidgets.back()->update();
#endif
}

void MainWindow::newTab(std::string_view tab_name)
{
    QWidget *new_tab{new QWidget()};
    new_tab->setObjectName(QString::fromUtf8("tab0"));
    std::unique_ptr<PCLQVTKWidget> vtkWidget = std::make_unique<PCLQVTKWidget>(new_tab);
    vtkWidget->setObjectName(QString::fromUtf8("vtkWidge"));
    vtkWidget->setGeometry(12, 12, 1920, 1080);
    _tabWidget->addTab(new_tab, QString::fromStdString(tab_name.data()));
    // auto shortcut = QShortcut (QKeySequence ("Ctrl+w"), _tabWidget);
    _tabWidget->setCurrentWidget(new_tab);

    _vtkWidgets.push_back(std::move(vtkWidget));
}
