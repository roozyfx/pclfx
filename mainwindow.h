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
    std::shared_ptr<PointCloudColor> _pc_color{std::make_shared<PointCloudColor>()};

    void resizeEvent(QResizeEvent *event) override;

private:
    Ui::MainWindow *ui;
    std::unique_ptr<Filter> _filter;
    std::unique_ptr<Ransac> _ransac;
    void SetFileMenuActions();
    void SetButtonsActions();
    void SetUpTabWidget();
    void OpenFile();

    void InitParams();
    bool LoadCloud(std::string_view file_name);

    void CloseTab();

    template<typename PC>
    void VisualizeInNewTab(const std::shared_ptr<PC> pc);
    template<typename PC>
    void Visualize(const std::shared_ptr<PC> pc);
    void RefreshView();
    void NewTab(std::string_view tab_name);
    void UpdateWindowSize();

    std::unique_ptr<QTabWidget> _tab_widget;
    int _tab_width{1940}, _tab_height{1100};
    int _render_margin{10};
    std::list<std::unique_ptr<PCLQVTKWidget>> _vtk_widgets;
};
