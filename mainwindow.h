#pragma once

#include <iostream>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <QMainWindow>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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
    void
    refreshView();

    pcl::visualization::PCLVisualizer::Ptr _viewer;
    // PointCloudT::Ptr _cloud;
    // pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud { new pcl::PointCloud<pcl::PointXYZ> };
    PointCloudT::Ptr _cloud { new PointCloudT };

private:
    void setFileMenuActions();
    void openFile();

    Ui::MainWindow* ui;
    int loadCloud(const std::string&);
    void visualize();
    // pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud;
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
};
