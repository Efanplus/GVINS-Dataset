#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <gnss_comm/rinex_helper.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gflags/gflags.h>

DEFINE_string(bag_file, "", "bag file path");
DEFINE_string(output_file, "", "output file path");



using namespace gnss_comm;

std::vector<std::vector<ObsPtr>> parse_gnss_meas(const std::string &bag_filepath)
{
    std::vector<std::vector<ObsPtr>> result;
    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/ublox_driver/range_meas");
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    for(rosbag::MessageInstance const m : view)
    {
        GnssMeasMsgConstPtr obs_msg = m.instantiate<GnssMeasMsg>();
        if (obs_msg == NULL)
            continue;
        std::vector<ObsPtr> obs = msg2meas(obs_msg);
        result.push_back(obs);
    }
    return result;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag2rinex");
    std::cout << "the nums of the bag2rinex's commend: " << argc << std::endl;
    for (int i = 0; i < argc; ++i) {
      std::cout << "the " << i << "th commend: " << argv[i] << std::endl;
    }
    google::SetVersionString("1.0.0");
    google::SetUsageMessage("bag2rinex");
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "bag_file: " << FLAGS_bag_file << std::endl;
    std::cout << "output_file: " << FLAGS_output_file << std::endl;
    std::vector<std::vector<ObsPtr>> all_gnss_meas = parse_gnss_meas(FLAGS_bag_file);
    obs2rinex(FLAGS_output_file, all_gnss_meas);
    std::cout << "Done\n";
    return 0;
}