#include <ndt_registration/ndt_matcher_d2d_sc.h>
#include <ndt_registration/icp_matcher_p2p.h>
#include <ndt_generic/motion_model_3d.h>
#include <ndt_generic/io.h>
#include <ndt_map/pointcloud_utils.h>
#include <boost/program_options.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <ndt_generic/gnuplot-iostream.h>
#include <ndt_evaluation/registration_score_eval.h>

using namespace std;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
  cout << "--------------------------------------------------" << endl;
  cout << "Creates a set of gnuplot files to visualize " << endl;
  cout << "different scores of matching functions" << std::endl;
  cout << "--------------------------------------------------" << endl;

  po::options_description desc("Allowed options");
  std::string gt_file;
  std::string odom_file;
  std::string base_name_pcd;
  Eigen::Vector3d transl;
  Eigen::Vector3d euler;
  double resolution;
  int idx1, idx2;
  int dimidx1, dimidx2;
  int offset_size;
  double incr_dist;
  double incr_ang;
  std::string out_file;
  // Simply to make it transparant to the fuser node.
  perception_oru::MotionModel2d::Params motion_params;
  int iter_step;
  int iters;
  std::string global_ndtmap_file;
  int node_idx;
  double mx,my;

  desc.add_options()
      ("help", "produce help message")
      ("gt_file", po::value<std::string>(&gt_file), "vehicle pose files in world frame")
      ("odom_file", po::value<std::string>(&odom_file)->default_value(std::string("")), "estimated sensor poses (from egomotion) to be used in SC in world frame")
      ("base_name_pcd", po::value<string>(&base_name_pcd)->default_value(std::string("")), "prefix for the .pcd files")
      ("out_file", po::value<string>(&out_file)->default_value(std::string("scores")), "prefix for the output files")
      ("idx1", po::value<int>(&idx1)->default_value(0), "'fixed' index of pose/pointcloud to be used")
      ("idx2", po::value<int>(&idx2)->default_value(1), "'moving' index of pose/pointcloud to be used")
      ("dimidx1", po::value<int>(&dimidx1)->default_value(0), "dimension index_x (what to plot on the X axis in gnuplot - 0,1,2,3,4,5 -> x,y,z,roll,pitch,yaw)")
      ("dimidx2", po::value<int>(&dimidx2)->default_value(1), "dimension index_y (what to plot on the Y axis in gnuplot - 0,1,2,3,4,5 -> x,y,z,roll,pitch,yaw)")

      ("x", po::value<double>(&transl[0])->default_value(0.), "sensor pose - translation vector x")
      ("y", po::value<double>(&transl[1])->default_value(0.), "sensor pose - translation vector y")
      ("z", po::value<double>(&transl[2])->default_value(0.), "sensor pose - translation vector z")
      ("ex", po::value<double>(&euler[0])->default_value(0.), "sensor pose - euler angle vector x")
      ("ey", po::value<double>(&euler[1])->default_value(0.), "sensor pose - euler angle vector y")
      ("ez", po::value<double>(&euler[2])->default_value(0.), "sensor pose - euler angle vector z")
      ("resolution", po::value<double>(&resolution)->default_value(1.), "resolution of the map")
      ("offset_size", po::value<int>(&offset_size)->default_value(100), "2*number of evaluated points per dimension")
      ("incr_dist", po::value<double>(&incr_dist)->default_value(0.01), "incremental steps for distance directions (x,y,z)")
      ("incr_ang", po::value<double>(&incr_ang)->default_value(0.002), "incremental steps for angular directions (R,P,Y)")

      ("Dd", po::value<double>(&motion_params.Dd)->default_value(1.), "forward uncertainty on distance traveled")
      ("Dt", po::value<double>(&motion_params.Dt)->default_value(1.), "forward uncertainty on rotation")
      ("Cd", po::value<double>(&motion_params.Cd)->default_value(1.), "side uncertainty on distance traveled")
      ("Ct", po::value<double>(&motion_params.Ct)->default_value(1.), "side uncertainty on rotation")
      ("Td", po::value<double>(&motion_params.Td)->default_value(1.), "rotation uncertainty on distance traveled")
      ("Tt", po::value<double>(&motion_params.Tt)->default_value(1.), "rotation uncertainty on rotation")
      ("use_score_d2d_sc", "if the d2d sc score should be used in the objective plot")
      ("save_eps", "if .eps should be generated")
      ("save_pcd", "if .pcd point cloud should be generated gt, odom, d2d, d2d_sc...")
      ("save_global_Ts", "if global transforms on registrations odom, etc. should be generate, for evaluation with ate.py, rpe.py")
      ("iter_all_poses", "iterate over all available poses")
      ("iter_step", po::value<int>(&iter_step)->default_value(1), "iter step size")
      ("iters", po::value<int>(&iters)->default_value(1), "if additional iters should be evaluated (idx1 + iter, idx2 + iter)")
      ("gnuplot", "if gnuplot should be invoked")
      ("draw_icp", "if icp results should be plotted")
      ("use_d2d_in_grid", "plot the d2d estimate in (0,0)")
      ("use_d2d_sc_in_grid", "plot the d2d_sc estimate in (0,0)")
      ("global_ndtmap_file", po::value<std::string>(&global_ndtmap_file)->default_value(""), "global map name to be registred towards")
      ("node_idx", po::value<int>(&node_idx)->default_value(0), "node index to be used in a graph map")
      ("mx", po::value<double>(&transl[0])->default_value(0.), "global map offset in x")
      ("my", po::value<double>(&transl[1])->default_value(0.), "global map offset in y")

      ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    cout << desc << "\n";
    return 1;
  }
  if (odom_file.empty()) {
    cout << "odom_file not specified" << endl;
    return 1;
  }

  bool use_score_d2d_sc = vm.count("use_score_d2d_sc");
  bool save_eps = vm.count("save_eps");
  bool save_pcd = vm.count("save_pcd");
  bool save_global_Ts = vm.count("save_global_Ts");
  bool iter_all_poses = vm.count("iter_all_poses");
  bool use_gnuplot = vm.count("gnuplot");
  bool draw_icp = vm.count("skip_icp");
  bool use_d2d_in_grid = vm.count("use_d2d_in_grid");
  bool use_d2d_sc_in_grid = vm.count("use_d2d_sc_in_grid");
  bool to_global_map = !global_ndtmap_file.empty();

  if (use_d2d_sc_in_grid) {
    use_score_d2d_sc = true;
  }


  Eigen::Affine3d sensor_pose = ndt_generic::vectorsToAffine3d(transl,euler);
  std::cout << "Sensor pose used : " << ndt_generic::affine3dToStringRPY(sensor_pose) << std::endl;

  perception_oru::MotionModel3d::Params motion_params3d(motion_params);

  std::cout << "Motion params used : " << motion_params3d << std::endl;

  RegistrationScoreEval se(gt_file, odom_file, base_name_pcd, sensor_pose, motion_params);
  se.resolution = resolution;
  se.offset_size = offset_size;
  se.incr_dist = incr_dist;
  se.incr_ang = incr_ang;
  se.use_d2d_in_grid = use_d2d_in_grid;
  se.use_d2d_sc_in_grid = use_d2d_sc_in_grid;

  if (to_global_map) {
    // Should be using the graph map representation?
    Eigen::Affine3d map_pose;
    map_pose.setIdentity();
    map_pose.translation()[0] = mx;
    map_pose.translation()[1] = my;
    se.setGlobalMap(global_ndtmap_file, map_pose);
  }

  if (iter_all_poses) {
    iters = se.nbPoses();
  }

  if (iters > se.nbPoses()) {
    std::cerr << "[iters too large] # poses : " << se.nbPoses() << std::endl;
    exit(-1);
  }

  out_file +=  ".res" + ndt_generic::toString(resolution) + "Dd" + ndt_generic::toString(motion_params.Dd) + "Dt" + ndt_generic::toString(motion_params.Dt);

  int i = 0;
  while (i < iters) {
    int _idx1 = idx1 + i;
    int _idx2 = idx2 + i;
    std::cout << "computing scores" << std::endl;
    se.computeScoreSets(_idx1, _idx2, dimidx1, dimidx2);

    std::cout << "saving : " << out_file << std::endl;
    se.save2D(out_file + ".dat", dimidx1, dimidx2);
    se.savePoseEstInOdomFrame(out_file + ".T");

    if (save_pcd) {
      std::string out_file_pcd = out_file + ndt_generic::toString(_idx1) + "_" + ndt_generic::toString(_idx2);
      se.savePCD(out_file_pcd);
    }

    if (use_gnuplot) {
      // Show it using gnuplot
      Gnuplot gp;//(std::fopen("output.gnuplot", "wb"));

      if (save_eps) {
        std::string out_file_eps = out_file + ndt_generic::toString(_idx1) + "_" + ndt_generic::toString(_idx2) + "x" + ndt_generic::toString(dimidx1) + "y" + ndt_generic::toString(dimidx2) + "sc" + ndt_generic::toString(use_score_d2d_sc) + ".eps";
        std::cout << "saving : " << out_file_eps << std::endl;
        gp << "set terminal postscript eps size 3.5,2.62 enhanced color font 'Helvetica,12' lw 1\n";
        gp << "set output '" << out_file_eps << "'\n";
      }
      std::string title = "Objective score value ";
      if (use_score_d2d_sc) {
        title += "with SC";
      }
      else {
        title += "without SC";
      }
      title += " [" + ndt_generic::toString(dimidx1) + "," + ndt_generic::toString(dimidx2) + "]";
      title += " res: " + ndt_generic::toString(resolution);
      if (use_score_d2d_sc)  {
        title += " Dd: " + ndt_generic::toString(motion_params.Dd);
      }
      gp << "set title \"" << title << "\"\n";
      gp << "set key top right\n";
      gp << "set pm3d map\n";

      if (!draw_icp) {
        gp << "splot '-' with pm3d notitle, '-' with points pt 1 title 'GT',  '-' with points pt 4 title 'odom', '-' with points pt 2 title 'd2d', '-' with points pt 3 title 'd2d sc', '-' with points pt 2 title 'filter'\n";
        gp.send2d( se.getScoreSegments(dimidx1, dimidx2, use_score_d2d_sc));
        gp.send1d( se.getRelPoseGT(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseOdom(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseD2D(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseD2D_SC(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseFilter(dimidx1, dimidx2));
      }
      else {
        gp << "splot '-' with pm3d notitle, '-' with points pt 1 title 'GT', '-' with points pt 2 title 'd2d', '-' with points pt 3 title 'd2d sc', '-' with points pt 4 title 'icp', '-' with points pt 2 title 'filter', '-' with points pt 4 title 'icp filter'\n";
        gp.send2d( se.getScoreSegments(dimidx1, dimidx2, use_score_d2d_sc));
        gp.send1d( se.getRelPoseGT(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseD2D(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseD2D_SC(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseICP(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseFilter(dimidx1, dimidx2));
        gp.send1d( se.getRelPoseICPFilter(dimidx1, dimidx2));
      }
    }
    i += iter_step;
  }

  if (save_global_Ts && iters > 1) {
    // Saves all transforms
    std::string out_file_Ts = out_file + ".Ts";
    se.saveTsToEvalFiles(out_file_Ts);
    se.saveComputationTime(out_file + ".time");

  }
  std::cout << "done." << std::endl;
}
