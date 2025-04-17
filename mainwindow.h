#pragma once
#include <QMainWindow>
#include "./ui_mainwindow.h"
#include "common.hpp"
#include "filter.h"
#include "ransac.h"
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
    std::shared_ptr<PointCloud> _pc{std::make_shared<PointCloud>()};
    std::shared_ptr<PointCloudColor> _pcColor{std::make_shared<PointCloudColor>()};

private:
    Ui::MainWindow *ui;
    std::unique_ptr<Filter> _filter;
    std::unique_ptr<Ransac> _ransac;
    void setFileMenuActions();
    void setButtonsActions();
    void openFile();

    void initParams();
    bool loadCloud(std::string_view);

    template<typename PC>
    void visualizeInNewTab(const std::shared_ptr<PC> pc);
    template<typename PC>
    void visualize(const std::shared_ptr<PC> pc);
    void refreshView();
    void newTab(std::string_view tab_name);

    QTabWidget *_tabWidget;
    std::vector<std::unique_ptr<PCLQVTKWidget>> _vtkWidgets;
};
