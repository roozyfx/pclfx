#include "mainwindow.h"
#include <QFileDialog>
#include <QLineEdit>
#include <QShortcut>
#include <chrono>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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
    _tabWidget = new QTabWidget(ui->centralWidget);
    _tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
    _tabWidget->setGeometry(QRect(10, 10, 1940, 1100));

    setFileMenuActions();
    setButtonsActions();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete _tabWidget;
    delete _vtkWidget;
}

void MainWindow::setFileMenuActions()
{
    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
    connect(ui->actionExit, &QAction::triggered, this, &QApplication::quit);
}

void MainWindow::setButtonsActions()
{
    _sor_standardDeviation = ui->leStdDev->text().toDouble();
    _sor_kMean = ui->leKMean->text().toUInt();
    connect(ui->leStdDev, &QLineEdit::returnPressed, this, &MainWindow::sorSetParams);
    connect(ui->leKMean, &QLineEdit::returnPressed, this,
        &MainWindow::sorSetParams);
    connect(ui->pbRemoveOutliers, &QPushButton::released, this, &MainWindow::outlierRemoval);

    connect(ui->leLeafSizex, &QLineEdit::returnPressed, this, &MainWindow::voxelGridFilter);
    connect(ui->leLeafSizey, &QLineEdit::returnPressed, this, &MainWindow::voxelGridFilter);
    connect(ui->leLeafSizez, &QLineEdit::returnPressed, this, &MainWindow::voxelGridFilter);
    connect(ui->pbVoxelGrid, &QPushButton::released, this, &MainWindow::voxelGridFilter);
    connect(ui->pbRansac, &QPushButton::released, this, &MainWindow::ransacSegmentation);
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

void MainWindow::xyz2xyzrgb(const std::shared_ptr<_PointCloud<PointXYZ>> in,
                            std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> out,
                            const uint8_t r,
                            const uint8_t g,
                            const uint8_t b)
{
    out->id = in->id;
    for (auto idx = 0; idx < in->cloud->size(); ++idx) {
        pcl::PointXYZRGB temp;
        temp.x = (*in->cloud)[idx].x;
        temp.y = (*in->cloud)[idx].y;
        temp.z = (*in->cloud)[idx].z;
        temp.r = r;
        temp.g = g;
        temp.b = b;
        out->cloud->push_back(temp);
    }
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

void MainWindow::sorSetParams()
{
    _sor_standardDeviation = ui->leStdDev->text().toDouble();
    _sor_kMean = ui->leKMean->text().toUInt();
    cout << "Std Dev: " << _sor_standardDeviation << endl;
    cout << "K Mean: " << _sor_kMean << endl;
}

void MainWindow::outlierRemoval()
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;

    sor.setInputCloud(_pc->cloud);

    sor.setMeanK(_sor_kMean);
    sor.setStddevMulThresh(_sor_standardDeviation);

    //@todo Why should this be shared_ptr? Shouldn't
    std::shared_ptr<PointCloud> pc_output_inliers{std::make_shared<PointCloud>()};
    sor.filter(*pc_output_inliers->cloud);
    pc_output_inliers->id = "inliers";
    connect(ui->pbShowInliers, &QPushButton::released, this, [this, pc_output_inliers]() {
        this->visualizeInNewTab(pc_output_inliers);
    });

    //@todo Why should this be shared_ptr? Shouldn't
    std::shared_ptr<PointCloud> pc_output_outliers{std::make_shared<PointCloud>()};
    sor.setNegative(true);
    sor.filter(*pc_output_outliers->cloud);
    pc_output_outliers->id = "outliers";
    connect(ui->pbShowOuliers, &QPushButton::released, this, [this, pc_output_outliers]() {
        this->visualizeInNewTab(pc_output_outliers);
    });
}

void MainWindow::voxelGridFilter()
{
    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud(_pc->cloud);
    auto x { ui->leLeafSizex->text().toFloat() };
    auto y { ui->leLeafSizex->text().toDouble() };
    auto z { ui->leLeafSizex->text().toDouble() };
    vg.setLeafSize(x, y, z);
    std::shared_ptr<PointCloud> pc_voxel_grid{std::make_shared<PointCloud>()};
    vg.filter(*pc_voxel_grid->cloud);
    pc_voxel_grid->id = "Voxel Grid";
    visualizeInNewTab(pc_voxel_grid);
}

//@todo Extend to extract multiple planes
void MainWindow::ransacSegmentation()
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);

    seg.setInputCloud(_pc->cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " " << coefficients->values[1]
              << " " << coefficients->values[2] << " " << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;

    pcl::ExtractIndices<PointT> extract;
    std::shared_ptr<PointCloud> extractedCloud{std::make_shared<PointCloud>()};

    extract.setInputCloud(_pc->cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // Extract only the selected points
    extract.filter(*extractedCloud->cloud);
    extractedCloud->id = "extractedCloud";

    std::shared_ptr<_PointCloud<pcl::PointXYZRGB>> extColor{
        std::make_shared<_PointCloud<pcl::PointXYZRGB>>()};

    xyz2xyzrgb(extractedCloud, extColor, 40, 200, 50);
    visualizeInNewTab<_PointCloud<pcl::PointXYZRGB>>(extColor);
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
    // _viewer->spin();

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
