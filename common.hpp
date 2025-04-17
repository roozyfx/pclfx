#pragma once
// Point Cloud Library
#include <filesystem>
#include <memory>
#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <thread>
#include <vector>

typedef pcl::PointXYZ Point;
typedef pcl::PointXYZRGB PointColor;

template<typename PT>
struct _PointCloud
{
    pcl::PointCloud<PT>::Ptr cloud{std::make_shared<pcl::PointCloud<PT>>()};
    std::string id{"cloud"};
};
typedef _PointCloud<Point> PointCloud;
typedef _PointCloud<PointColor> PointCloudColor;

//@todo: make a better solution for coloring, e.g. user can change via UI
namespace Utils {

inline void _xyz2xyzrgb(std::shared_ptr<const PointCloud> in,
                        std::shared_ptr<PointCloudColor> out,
                        const size_t begin,
                        const size_t end,
                        const uint8_t r,
                        const uint8_t g,
                        const uint8_t b)
{
    for (size_t idx = begin; idx < end; ++idx) {
        pcl::PointXYZRGB temp;
        temp.x = (*in->cloud)[idx].x;
        temp.y = (*in->cloud)[idx].y;
        temp.z = (*in->cloud)[idx].z;
        temp.r = r;
        temp.g = g;
        temp.b = b;
        out->cloud->at(idx) = temp;
    }
}

inline void colorize(std::shared_ptr<const PointCloud> in,
                     std::shared_ptr<PointCloudColor> out,
                     const uint8_t r,
                     const uint8_t g,
                     const uint8_t b)
{
    auto num_threads{std::thread::hardware_concurrency()};
    num_threads = std::max(1u, (num_threads > 4) ? num_threads - 2 : num_threads);
    //important! threads are going to access each element and not in-order
    out->cloud->resize(in->cloud->size());
    out->id = in->id;
    // Don't create threads for smaller Point Clouds
    auto pc_small_size{10000};
    if (in->cloud->size() < pc_small_size || num_threads == 1) {
        _xyz2xyzrgb(in, out, 0, in->cloud->size(), r, g, b);
        return;
    }
    auto chunk_size{in->cloud->size() / num_threads};
    chunk_size = chunk_size == 0 ? 1 : chunk_size;
    std::vector<std::thread> threads;
    for (auto i = 0; i < in->cloud->size(); i += chunk_size) {
        auto end = (i + chunk_size) >= in->cloud->size() ? in->cloud->size() : i + chunk_size;
        std::thread t(_xyz2xyzrgb, std::ref(in), std::ref(out), i, end, r, g, b);
        threads.push_back(std::move(t));
    }
    for (auto &th : threads) {
        th.join();
    }
}

template<typename PCT>
class IFileLoader
{
protected:
    std::shared_ptr<PCT> _pc;
    std::string_view _file;

public:
    IFileLoader(std::string_view file, std::shared_ptr<PCT> pc)
        : _file{file}
        , _pc{pc}
    {}

    virtual ~IFileLoader() = default;
    virtual void loadFile() = 0;
};

template<typename PCT>
class UnsupportedFormat : public IFileLoader<PCT>
{
public:
    UnsupportedFormat(std::string_view file, std::shared_ptr<PCT> pc)
        : IFileLoader<PCT>(file, pc)
    {}

    void loadFile() final { PCL_ERROR("Unsupported file format, %s \n", IFileLoader<PCT>::_file); }
};

template<typename PCT>
class PCDLoader : public IFileLoader<PCT>
{
public:
    PCDLoader(std::string_view file, std::shared_ptr<PointCloud> pc)
        : IFileLoader<PCT>(file, pc)
    {}
    void loadFile() final
    {
        if (pcl::io::loadPCDFile(IFileLoader<PCT>::_file.data(), *(IFileLoader<PCT>::_pc->cloud))
            == -1) {
            PCL_ERROR("Couldn't read file %s \n", IFileLoader<PCT>::_file);
        }
    }
};

template<typename PCT>
class PLYLoader : public IFileLoader<PCT>
{
public:
    PLYLoader(std::string_view file, std::shared_ptr<PointCloud> pc)
        : IFileLoader<PCT>(file, pc)
    {}
    void loadFile() final
    {
        if (pcl::io::loadPLYFile(IFileLoader<PCT>::_file.data(), *(IFileLoader<PCT>::_pc->cloud))
            == -1) {
            PCL_ERROR("Couldn't read file %s \n", IFileLoader<PCT>::_file);
        }
    }
};

template<typename PCT>
class OBJLoader : public IFileLoader<PCT>
{
public:
    OBJLoader(std::string_view file, std::shared_ptr<PointCloud> pc)
        : IFileLoader<PCT>(file, pc)
    {}
    void loadFile() final
    {
        if (pcl::io::loadOBJFile(IFileLoader<PCT>::_file.data(), *(IFileLoader<PCT>::_pc->cloud))
            == -1) {
            PCL_ERROR("Couldn't read file %s \n", IFileLoader<PCT>::_file);
        }
    }
};

enum class FILEFORMATS { pcd, ply, obj, other };

inline FILEFORMATS get_format(std::string_view file)
{
    std::string extension{std::filesystem::path(file).extension().string()};
    // Convert to lowercase
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    std::cout << "File format: " << extension << std::endl;
    if (extension.compare(".pcd") == 0)
        return FILEFORMATS::pcd;
    if (extension.compare(".obj") == 0)
        return FILEFORMATS::obj;
    if (extension.compare(".ply") == 0)
        return FILEFORMATS::ply;
    return FILEFORMATS::other;
}

template<typename PCT>
inline bool loadFile(const std::string_view filename, std::shared_ptr<PCT> pc)
{
    std::unique_ptr<IFileLoader<PCT>> loader;
    auto format{get_format(filename)};
    switch (format) {
    case FILEFORMATS::pcd:
        loader = std::make_unique<PCDLoader<PCT>>(filename, pc);
        break;
    case FILEFORMATS::ply:
        loader = std::make_unique<PLYLoader<PCT>>(filename, pc);
        break;
    case FILEFORMATS::obj:
        loader = std::make_unique<OBJLoader<PCT>>(filename, pc);
        break;
    default:
        loader = std::make_unique<UnsupportedFormat<PCT>>(filename, pc);
        return false;
    }
    loader->loadFile();

    return true;
}

} //namespace Utils
