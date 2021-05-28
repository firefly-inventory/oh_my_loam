#include <pcl/io/pcd_io.h>

#include <chrono>
#include <filesystem>
#include <functional>

#include "common/common.h"
#include "oh_my_loam/oh_my_loam.h"

using namespace common;
using namespace oh_my_loam;
namespace fs = std::filesystem;

void PointCloudHandler(const PointCloudConstPtr &cloud, OhMyLoam *slam);

std::string ToCsv(Pose3d &d) {
    std::ostringstream oss;
    double w = d.r_quat().w(), a = d.r_quat().x(), b = d.r_quat().y(), c = d.r_quat().z();
    double x = d.t_vec().x(), y = d.t_vec().y(), z = d.t_vec().z();
    oss << std::setprecision(3) << w << "," << a << "," << b << "," << c << "," << x << "," << y << "," << z << "\n";
    return oss.str();
}

// https://www.delftstack.com/howto/cpp/read-csv-file-in-cpp/
std::string readFileIntoString(const fs::path &path) {
    auto ss = std::ostringstream{};
    ifstream input_file(path.string());
    if (!input_file.is_open()) {
        AFATAL << "Could not open the file - '" << path.string() << "'" << endl;
        exit(EXIT_FAILURE);
    }
    ss << input_file.rdbuf();
    input_file.close();
    return ss.str();
}


static fs::path posePath;

void writeToFile(const size_t &frameId, const std::shared_ptr<OhmyloamVisFrame> &frame) {
    // Append to pose map.
    ofstream pose_output(posePath, std::ios_base::app);
    if (!pose_output.is_open()) {
        AFATAL << "Could not open the file - '" << posePath << "'" << endl;
        exit(EXIT_FAILURE);
    }
    AUSER << posePath;
    pose_output << ToCsv(frame->pose_map);
    pose_output.close();
}


int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "\033[1m\033[31mConfiguration file and input path should be specified!\033[m" << std::endl;
        return -1;
    }
    // config
    YAMLConfig::Instance()->Init(argv[1]);
    bool is_log_to_file = YAMLConfig::Instance()->Get<bool>("log_to_file");
    auto log_path = YAMLConfig::Instance()->Get<std::string>("log_path");
    auto lidar = YAMLConfig::Instance()->Get<std::string>("lidar");
    // logging
    InitG3Logging(is_log_to_file, "oh_my_loam_" + lidar, log_path);
    AUSER << "LOAM start..., lidar = " << lidar;
    // SLAM system
    OhMyLoam slam;
    if (!slam.Init()) {
        AFATAL << "Failed to initialize slam system.";
    }
    // get input point cloud file names
    ACHECK(fs::exists(argv[2])) << "Directory not exist: " << argv[2];
    std::vector<fs::path> cloud_paths;
    for (auto &it : fs::directory_iterator(argv[2])) {
        if (fs::is_regular_file(it.path())) {
            cloud_paths.push_back(it.path());
        }
    }

    ACHECK(fs::exists(argv[3])) << "Output directory not exist: " << argv[3];
    posePath = fs::path(std::strcat(argv[3], "-trajectory.poses"));

    AWARN_IF(cloud_paths.empty()) << "No point cloud file in directory: " << argv[2];
    AINFO_IF(!cloud_paths.empty()) << "There are " << cloud_paths.size() << " point clouds in total";
    std::sort(cloud_paths.begin(), cloud_paths.end());
    // load point cloud and process
    for (auto &path : cloud_paths) {
        AUSER << path.string();

        PointCloudPtr cloud(new PointCloud);
        auto file_contents = readFileIntoString(path.string());

        std::istringstream sstream(file_contents);
        std::string record;
        char delimiter = ',';

        while (std::getline(sstream, record)) {
            std::istringstream line(record);
            std::vector<float> entry;
            while (std::getline(line, record, delimiter)) {
                entry.push_back(std::stof(record));
            }
            Point point(entry.at(0), entry.at(1), entry.at(2));
            cloud->push_back(point);
        }
        PointCloudHandler(cloud, &slam);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10 Hz
    }
    return 0;
}

void PointCloudHandler(const PointCloudConstPtr &cloud, OhMyLoam *const slam) {
    auto millisecs = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::system_clock::now().time_since_epoch());
    double timestamp = millisecs.count() / 1000.0;
    static size_t frame_id = 0;
    AINFO << "Ohmyloam: frame_id = " << ++frame_id
          << ", timestamp = " << FMT_TIMESTAMP(timestamp)
          << ", point_number = " << cloud->size();
    auto frame = slam->Run(timestamp, cloud);
    writeToFile(frame_id, frame);
}