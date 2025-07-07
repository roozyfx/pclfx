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
#include <iterator>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , _filter{std::make_unique<Filter>(_pc)}
    , _ransac{std::make_unique<Ransac>(_pc)}
{
    ui->setupUi(this);
    this->setWindowTitle("PCL viewer");

    InitParams();
    SetFileMenuActions();
    SetButtonsActions();
    SetUpTabWidget();
}

MainWindow::~MainWindow()
{
    if (ui)
        delete ui;
}

void MainWindow::InitParams()
{
    if (_filter) {
        _filter->StdDev(ui->le_std_dev->text().toDouble());
        _filter->KMean(ui->le_k_mean->text().toUInt());
        _filter->Vg_x(ui->le_leaf_size_x->text().toDouble());
        _filter->Vg_y(ui->le_leaf_size_y->text().toDouble());
        _filter->Vg_z(ui->le_leaf_size_z->text().toDouble());
    }
}

void MainWindow::SetFileMenuActions()
{
    connect(ui->action_open, &QAction::triggered, this, &MainWindow::OpenFile);
    connect(ui->action_exit, &QAction::triggered, this, &QApplication::quit);
}

void MainWindow::SetButtonsActions()
{
    connect(ui->le_std_dev, &QLineEdit::returnPressed, this, [this]() {
        _filter->StdDev(ui->le_std_dev->text().toDouble());
    });
    connect(ui->le_k_mean, &QLineEdit::returnPressed, this, [this]() {
        _filter->KMean(ui->le_k_mean->text().toUInt());
    });
    connect(ui->pb_remove_outliers, &QPushButton::released, this, [this]() {
        _filter->OutlierRemoval();
    });
    auto WarnMessage = []() {
        QMessageBox msg_box;
        msg_box.setText("Set Parameters and push Remove Outliers button first.");
        msg_box.exec();
    };
    connect(ui->pb_show_inliers, &QPushButton::released, this, [this, WarnMessage]() {
        auto inliers{_filter->SorInliers()};
        (!inliers->cloud->empty()) ? this->VisualizeInNewTab(inliers) : WarnMessage();
    });
    connect(ui->pb_show_outliers, &QPushButton::released, this, [this, WarnMessage]() {
        auto outliers{_filter->SorOutliers()};
        (!outliers->cloud->empty()) ? this->VisualizeInNewTab(outliers) : WarnMessage();
    });

    connect(ui->le_leaf_size_x, &QLineEdit::returnPressed, this, [this]() {
        _filter->Vg_x(ui->le_leaf_size_x->text().toDouble());
    });
    connect(ui->le_leaf_size_y, &QLineEdit::returnPressed, this, [this]() {
        _filter->Vg_y(ui->le_leaf_size_y->text().toDouble());
    });
    connect(ui->le_leaf_size_z, &QLineEdit::returnPressed, this, [this]() {
        _filter->Vg_z(ui->le_leaf_size_z->text().toDouble());
    });

    connect(ui->pb_voxel_grid, &QPushButton::released, this, [this]() {
        VisualizeInNewTab(_filter->VoxelGridFilter());
    });
    connect(ui->pb_ransac, &QPushButton::released, this, [this]() {
        _ransac->RansacSegmentation();
        VisualizeInNewTab<_PointCloud<pcl::PointXYZRGB>>(_ransac->GetResult());
    });
}

void MainWindow::SetUpTabWidget()
{
    _tab_widget = new QTabWidget(ui->central_widget);
    _tab_widget->setObjectName(QString::fromUtf8("tabWidget"));
    _tab_widget->setGeometry(QRect(10, 10, 1940, 1100));
    _tab_widget->setMovable(true);
    _tab_widget->setTabsClosable(true);
    connect(_tab_widget, &QTabWidget::tabCloseRequested, this, &MainWindow::CloseTab);
}

void MainWindow::OpenFile()
{
    std::string cloud_file{(QFileDialog::getOpenFileName(this,
                                                         tr("Open Image"),
                                                         "../../../data/tutorials",
                                                         tr("Point Cloud File (*.pcd "
                                                            "*.ply "
                                                            "*.obj *.PCD *.PLY *.OBJ *.*)")))
                               .toStdString()};

    if (cloud_file.empty())
        return;

    if (LoadCloud(cloud_file))
        VisualizeInNewTab(_pc_color);
    else
        std::cout << "Unsupported or invalid file format\n";
}

bool MainWindow::LoadCloud(std::string_view file_name)
{
    if (!Utils::LoadFile(file_name, _pc)) //unsupported file format
        return false;

    pcl::copyPointCloud(*_pc->cloud, *_pc_color->cloud);
    Utils::Colorize(_pc, _pc_color, 70, 220, 10);
    // set filename as the point cloud id
    _pc->id = std::filesystem::path(file_name).filename().string();
    _pc_color->id = "[" + _pc->id + "]";

    // Print some Info
    std::cout << "Loaded " << _pc->cloud->width * _pc->cloud->height << " data points from "
              << file_name << " with the following fields: " << std::endl
              << "Width: " << _pc->cloud->width << "\tHeight: " << _pc->cloud->height
              << "\nHeader: " << _pc->cloud->header << "\nIs "
              << (_pc->cloud->is_dense ? "" : " not") << " Dense" << std::endl;
    return true;
}

template<typename PC>
void MainWindow::VisualizeInNewTab(const std::shared_ptr<PC> pc)
{
    NewTab(pc->id);
    Visualize(pc);
    RefreshView();
}

/**  Visualize the point cloud **/
template<typename PC>
void MainWindow::Visualize(const std::shared_ptr<PC> pc)
{
    // Set up the QVTK window
#if VTK_MAJOR_VERSION > 8
    auto renderer{vtkSmartPointer<vtkRenderer>::New()};
    auto render_window{vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New()};
    render_window->AddRenderer(renderer);
    _viewer.reset(new pcl::visualization::PCLVisualizer(renderer, render_window, "viewer", false));
    _vtk_widgets.back()->setRenderWindow(_viewer->getRenderWindow());
    _viewer->setupInteractor(_vtk_widgets.back()->interactor(), _vtk_widgets.back()->renderWindow());
#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    vtkWidget->setRenderWindow(viewer->getRenderWindow());
    _viewer->setupInteractor(vtkWidget->interactor(), vtkWidget->renderWindow());
#endif
    _viewer->removeAllPointClouds();
    _viewer->addPointCloud(pc->cloud, pc->id);
}

void MainWindow::RefreshView()
{
#if VTK_MAJOR_VERSION > 8
    _vtk_widgets.back()->renderWindow()->Render();
#else
    _vtk_widgets.back()->update();
#endif
}

void MainWindow::NewTab(std::string_view tab_name)
{
    QWidget *new_tab{new QWidget()};
    new_tab->setObjectName(QString(tab_name.data()));
    std::unique_ptr<PCLQVTKWidget> vtk_widget = std::make_unique<PCLQVTKWidget>(new_tab);
    vtk_widget->setObjectName(QString(tab_name.data()));
    vtk_widget->setGeometry(12, 12, 1920, 1080);
    _tab_widget->addTab(new_tab, QString(tab_name.data()));
    _tab_widget->setCurrentWidget(new_tab);

    _vtk_widgets.push_back(std::move(vtk_widget));
    cout << "Added to the list: " << _vtk_widgets.back()->objectName().toStdString() << endl;
}

void MainWindow::CloseTab()
{
    auto idx{_tab_widget->currentIndex()};
    _tab_widget->removeTab(idx);
    // Also remove the corresponding element from our P.C. container
    auto it{_vtk_widgets.begin()};
    std::advance(it, idx);
    cout << "Removing from the list: " << (*it)->objectName().toStdString() << endl;
    _vtk_widgets.erase(it);
}
