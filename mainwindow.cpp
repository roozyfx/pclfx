#include "mainwindow.h"
#include <QFileDialog>
#include <QLineEdit>
#include <chrono>
#include <pcl/filters/statistical_outlier_removal.h>
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
    setButtonsActions();
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

void MainWindow::setButtonsActions()
{
    _sor_standardDeviation = ui->leStdDev->text().toDouble();
    _sor_kMean = ui->leKMean->text().toUInt();
    connect(ui->leStdDev, &QLineEdit::returnPressed, this, &MainWindow::sorSetParams);
    connect(ui->leKMean, &QLineEdit::returnPressed, this,
        &MainWindow::sorSetParams);
    connect(ui->pbRemoveOutliers, &QPushButton::released, this, &MainWindow::outlierRemoval);

    connect(ui->pbNewTab, &QPushButton::released, this, &MainWindow::newTab);
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
    visualize(_pc);
    refreshView();
    delete temp;
}

int MainWindow::loadCloud(const std::string& filename)
{
    auto format { get_format(filename) };
    switch (format) {
    case FILEFORMATS::pcd:
        if (pcl::io::loadPCDFile(filename, *(_pc->cloud)) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    case FILEFORMATS::ply:
        if (pcl::io::loadPLYFile(filename, *(_pc->cloud)) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    case FILEFORMATS::obj:
        if (pcl::io::loadOBJFile(filename, *(_pc->cloud)) == -1) {
            PCL_ERROR("Couldn't read file %s \n", filename);
            return (-1);
        }
        break;
    default:
        PCL_ERROR("Unsupported file format, %s \n", filename);
        return (-1);
    }
    // Get only the filename and set it as the PointCloud id
    auto it { std::find(filename.crbegin(), filename.crend(), '/') };
    auto last_slash { std::distance(filename.crbegin(), it) };
    _pc->id = filename.substr(filename.length() - last_slash, filename.length());

    // Print some Info
    std::cout << "Loaded " << _pc->cloud->width * _pc->cloud->height << " data points from "
              << filename << " with the following fields: " << std::endl
              << "Width: " << _pc->cloud->width << "\tHeight: " << _pc->cloud->height << "\nHeader: "
              << _pc->cloud->header << "\nIs " << (_pc->cloud->is_dense ? "" : " not") << " Dense" << std::endl;

    return 0;
}

/**  Visualize the point cloud **/
void MainWindow::visualize(std::shared_ptr<PointCloud> pc)
{
    _viewer->removeAllPointClouds();
    _viewer->addPointCloud(pc->cloud, pc->id);
    _viewer->spin();

    // Wait until the window closes
    while (!_viewer->wasStopped()) {
        // using namespace std::chrono_literals;
        // std::this_thread::sleep_for(100ms);
    }
}

void MainWindow::visualize(pcl::visualization::PCLVisualizer::Ptr viewer, std::shared_ptr<PointCloud> pc)
{
    viewer->removeAllPointClouds();
    viewer->addPointCloud(pc->cloud, pc->id);
    viewer->spin();

    // Wait until the window closes
    while (!viewer->wasStopped()) {
        // using namespace std::chrono_literals;
        // std::this_thread::sleep_for(100ms);
    }
}

void MainWindow::setupViewer(pcl::visualization::PCLVisualizer::Ptr&& viewer,
    std::unique_ptr<PCLQVTKWidget>&& vtkWidget)
{
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#endif
}

void MainWindow::refreshView()
{
#if VTK_MAJOR_VERSION > 8
    ui->qvtkWidget->renderWindow()->Render();
#else
    ui->qvtkWidget->update();
#endif
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
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

    sor.setInputCloud(_pc->cloud);

    sor.setMeanK(_sor_kMean);
    sor.setStddevMulThresh(_sor_standardDeviation);

    sor.filter(*_pc_out->cloud);
    // std::cerr << "Cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;
    // pcl::PCDWriter writer;
    // // Create the filtering object
    // writer.write<PointT>("temp_inliers.pcd", *_pc_out->cloud, false);
    // connect(ui->pbShowInliers, &QPushButton::released, this,
    //     [this]() {
    //         this->loadCloud("temp_inliers.pcd");
    //         this->visualize(_pc_out);
    //         this->refreshView();
    //     });

    // sor.setNegative(true);
    // sor.filter(*_pc_out->cloud);
    // writer.write<PointT>("temp_outliers.pcd", *_pc_out->cloud, false);
    // connect(ui->pbShowOuliers, &QPushButton::released, this,
    //     [this]() {
    //         this->loadCloud("temp_outliers.pcd");
    //         this->visualize(_pc_out);
    //         this->refreshView();
    //     });
}

void MainWindow::newTab()
{
    QWidget* newTab { new QWidget() };
    newTab->setObjectName(QString::fromUtf8("temp_0"));
    newTab->setAccessibleName(QString::fromUtf8("Fx"));
    PCLQVTKWidget* vtkWidget = new PCLQVTKWidget(newTab);
    // std::unique_ptr<PCLQVTKWidget> vtkWidget = std::make_unique<PCLQVTKWidget>(newTab);
    vtkWidget->setObjectName(QString::fromUtf8("vtkWidget"));
    vtkWidget->setGeometry(ui->tab->geometry());
    ui->tabWidget->addTab(newTab, QString::fromUtf8("Fx_new"));

    pcl::visualization::PCLVisualizer::Ptr viewer;
    // Temporary
    // Set up the QVTK window
    // setupViewer(std::move(viewer), std::move(vtkWidget));
#if VTK_MAJOR_VERSION > 8
    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renderWindow, "viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#endif
    ui->tabWidget->setCurrentWidget(newTab);
    visualize(viewer, _pc_out);
}
