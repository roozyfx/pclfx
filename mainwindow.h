#pragma once

#include "./ui_mainwindow.h"
#include <QMainWindow>
#include <iostream>
#include <memory>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

// template <typename PCT = PointCloudT>
struct PointCloud {
    PointCloudT::Ptr cloud { new PointCloudT };
    std::string id { "cloud" };
};

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void refreshView();

    pcl::visualization::PCLVisualizer::Ptr _viewer;
    // std::shared_ptr<PointCloud> _pc { std::make_shared<PointCloud>() };
    std::shared_ptr<PointCloud> _pc { std::make_shared<PointCloud>() };
    std::shared_ptr<PointCloud> _pc_out { std::make_shared<PointCloud>() };
    // PointCloudT::Ptr _cloud_output { new PointCloudT };

private:
    void setFileMenuActions();
    void setButtonsActions();
    void openFile();

    void sorSetParams();
    void outlierRemoval();

    void newTab();

    Ui::MainWindow* ui;
    int loadCloud(const std::string&);
    void visualize(std::shared_ptr<PointCloud> pc);
    void visualize(pcl::visualization::PCLVisualizer::Ptr viewer,
        std::shared_ptr<PointCloud> pc);
    void setupViewer(pcl::visualization::PCLVisualizer::Ptr&& viewer,
        std::unique_ptr<PCLQVTKWidget>&& vtkWidget);

    enum class FILEFORMATS { pcd,
        ply,
        obj,
        other };
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

    // Move to another file
    float _sor_standardDeviation;
    uint _sor_kMean;
};
