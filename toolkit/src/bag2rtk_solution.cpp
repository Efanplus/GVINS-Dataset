#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <memory>
#include <gnss_comm/gnss_utility.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include <gflags/gflags.h>

DEFINE_string(bag_file, "", "bag file path");
DEFINE_string(output_file, "", "output file path");

using namespace gnss_comm;

struct RTKState
{
    unsigned long gnss_ts_ns;
    Eigen::Vector3d t_ecef;
    Eigen::Vector3d geodetic_pos;
    Eigen::Vector3d v_enu;
    uint8_t fix_type;
    bool valid_fix;
    bool diff_soln;
    uint8_t carr_soln;
};
typedef std::shared_ptr<RTKState> RTKStatePtr;

void extract_rtk_states(const std::string &bag_filepath, std::vector<RTKStatePtr> &all_states)
{
    all_states.clear();

    rosbag::Bag bag;
    bag.open(bag_filepath, rosbag::bagmode::Read);
    std::vector<std::string> topics;
    topics.push_back("/ublox_driver/receiver_pvt");

    rosbag::View view(bag, rosbag::TopicQuery(topics));
    for(rosbag::MessageInstance const m : view)
    {
        GnssPVTSolnMsgConstPtr pvt_msg = m.instantiate<GnssPVTSolnMsg>();
        if (pvt_msg != NULL)
        {
            PVTSolutionPtr pvt = msg2pvt(pvt_msg);
            RTKStatePtr rtk_state(new RTKState());
            rtk_state->gnss_ts_ns = static_cast<unsigned long>(time2sec(pvt->time)*1e9);
            Eigen::Vector3d pvt_lla(pvt->lat, pvt->lon, pvt->hgt);
            rtk_state->geodetic_pos = pvt_lla;
            rtk_state->t_ecef = geo2ecef(pvt_lla);
            rtk_state->v_enu.x() =  pvt->vel_e;
            rtk_state->v_enu.y() =  pvt->vel_n;
            rtk_state->v_enu.z() = -pvt->vel_d;
            rtk_state->fix_type = pvt->fix_type;
            rtk_state->valid_fix = pvt->valid_fix;
            rtk_state->diff_soln = pvt->diff_soln;
            rtk_state->carr_soln = pvt->carr_soln;
            all_states.push_back(rtk_state);
        }
    }
}

void save_states(const std::vector<RTKStatePtr> &seq, const std::string &filepath)
{
    FILE *fp = fopen(filepath.c_str(), "w");
    for (uint32_t i = 0; i < seq.size(); ++i)
    {
        const RTKStatePtr &state = seq[i];
        /* global_timestamp, ecef_px, ecef_py, ecef_pz, enu_vx, enu_vy, enu_vz,
         * solution_fix_type, is_fix_valid, is_differential_correction_applied, 
         * carrier_phase_range_solution_status
         * Check gnss_comm/msg/GnssPVTSolnMsg.msg for details.
         */ 
        fprintf(fp, "%lu", state->gnss_ts_ns);
        for (uint32_t j = 0; j < 3; ++j)
            fprintf(fp, ", %.5f", state->geodetic_pos(j));
        for (uint32_t j = 0; j < 3; ++j)
            fprintf(fp, ", %.5f", state->t_ecef(j));
        for (uint32_t j = 0; j < 3; ++j)
            fprintf(fp, ", %.5f", state->v_enu(j));
        fprintf(fp, ", %d, %d, %d, %d\n", static_cast<int>(state->fix_type), 
            static_cast<int>(state->valid_fix), static_cast<int>(state->diff_soln), 
            static_cast<int>(state->carr_soln));
    }
    fclose(fp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bag2rtk_solution");
    std::cout << "the nums of the bag2rtk_solution's commend: " << argc << std::endl;
    for (int i = 0; i < argc; ++i) {
      std::cout << "the " << i << "th commend: " << argv[i] << std::endl;
    }
    google::SetVersionString("1.0.0");
    google::SetUsageMessage("bag2rtk_solution");
    google::ParseCommandLineFlags(&argc, &argv, true);
    std::cout << "bag_file: " << FLAGS_bag_file << std::endl;
    std::cout << "output_file: " << FLAGS_output_file << std::endl;
    // load RTK states
    std::vector<RTKStatePtr> all_states;
    extract_rtk_states(FLAGS_bag_file, all_states);
    // write to file
    save_states(all_states, FLAGS_output_file);

    std::cout << "Done.\n";
    return 0;
}