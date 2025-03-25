#include "mainwindow.h"
#include <QFileDialog>
#include <QLineEdit>
#include <QShortcut>
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

    connect(ui->pbNewTab, &QPushButton::released, this,
        [this]() { emit sigNewTab(""); });
    connect(this, &MainWindow::sigNewTab, this, &MainWindow::newTab);
}

void MainWindow::openFile()
{
    std::unique_ptr<QWidget> temp { std::make_unique<QWidget>() };
    QString cloudFile {
        QFileDialog::getOpenFileName(temp.get(), tr("Open Image"), "/home/user/fx/pclfx/data/tutorials",
            tr("Point Cloud File (*.pcd "
               "*.ply "
               "*.obj)"))
    };

    if (cloudFile.isEmpty())
        return;

    loadCloud(cloudFile.toStdString());
    visualizeInNewTab(_pc);
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

    std::shared_ptr<PointCloud> pc_output_inliers { std::make_shared<PointCloud>() };
    sor.filter(*pc_output_inliers->cloud);
    pc_output_inliers->id = "inliers";
    connect(ui->pbShowInliers, &QPushButton::released, this, [this, pc_output_inliers]() {
        this->visualizeInNewTab(pc_output_inliers);
    });

    std::shared_ptr<PointCloud> pc_output_outliers { std::make_shared<PointCloud>() };
    sor.setNegative(true);
    sor.filter(*pc_output_outliers->cloud);
    pc_output_outliers->id = "outliers";
    connect(ui->pbShowOuliers, &QPushButton::released, this, [this, pc_output_outliers]() {
        this->visualizeInNewTab(pc_output_outliers);
    });
}

void MainWindow::visualizeInNewTab(const std::shared_ptr<PointCloud> pc)
{
    _vtkWidget = newTab(pc->id);
    visualize(pc);
    refreshView();
}

/**  Visualize the point cloud **/
void MainWindow::visualize(const std::shared_ptr<PointCloud> pc)
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
    _viewer->spin();

    // Wait until the window closes
    while (!_viewer->wasStopped()) {
        // using namespace std::chrono_literals;
        // std::this_thread::sleep_for(100ms);
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
PCLQVTKWidget* MainWindow::newTab(const std::string& tab_name)
{
    QWidget* new_tab { new QWidget() };
    new_tab->setObjectName(QString::fromUtf8("tab0"));
    PCLQVTKWidget* vtkWidget = new PCLQVTKWidget(new_tab);
    vtkWidget->setObjectName(QString::fromUtf8("vtkWidge"));
    vtkWidget->setGeometry(12, 12, 1920, 1080);
    _tabWidget->addTab(new_tab, QString::fromStdString(tab_name));
    // auto shortcut = QShortcut (QKeySequence ("Ctrl+w"), _tabWidget);
    _tabWidget->setCurrentWidget(new_tab);

    return vtkWidget;
}
