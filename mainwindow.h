#pragma once
#include <QMainWindow>
#include "./ui_mainwindow.h"
#include "common.hpp"
#include "filter.h"
#include "ransac.h"
#include <iostream>
#include <memory>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/qvtk_compatibility.h>

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    pcl::visualization::PCLVisualizer::Ptr _viewer;
    std::shared_ptr<_PointCloud<PointXYZ>> _pcXYZ{std::make_shared<_PointCloud<PointXYZ>>()};
    std::shared_ptr<PointCloud> _pc{std::make_shared<PointCloud>()};

private:
    Filter *_filter;
    Ransac *_ransac;
    void setFileMenuActions();
    void setButtonsActions();
    void openFile();

    void sorSetParams();
    void outlierRemoval();
    void voxelGridFilter();

    Ui::MainWindow *ui;
    void initParams();
    int loadCloud(std::string_view);

    template<typename PC>
    void visualizeInNewTab(const std::shared_ptr<PC> pc);
    template<typename PC>
    void visualize(const std::shared_ptr<PC> pc);
    void refreshView();
    PCLQVTKWidget *newTab(std::string_view tab_name);

    QTabWidget *_tabWidget;
    PCLQVTKWidget* _vtkWidget;

    enum class FILEFORMATS { pcd, ply, obj, other };
    FILEFORMATS get_format(const std::string& file)
    {
        std::string format { file.substr(file.size() - 3) };
        std::cout << "File format: " << format << endl;
        if (format.compare("pcd") == 0 || format.compare("PCD") == 0)
            return FILEFORMATS::pcd;
        if (format.compare("obj") == 0 || format.compare("OBJ") == 0)
            return FILEFORMATS::obj;
        if (format.compare("ply") == 0 || format.compare("PLY") == 0)
            return FILEFORMATS::ply;
        return FILEFORMATS::other;
    }
};
