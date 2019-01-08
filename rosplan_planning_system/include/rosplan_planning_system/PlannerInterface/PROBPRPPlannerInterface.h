#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>
#include <utility>
#include <map>
#include <vector>

#ifndef KCL_PROBPRP_planner_interface
#define KCL_PROBPRP_planner_interface

/**
 * This file contains an interface to the planner.
 */
namespace KCL_rosplan
{
class PROBPRPPlannerInterface: public PlannerInterface
{
private:
  /* runs external commands */
  std::string runCommand(std::string cmd);
  void loadParams();
  void parsePlan();
  bool isFileAcessible(const char *fileName);
  std::string getGraphFile(std::string viz_type);
  std::string problem_file, domain_file, domainfo_file, planner_path, script_path,
    translator_file, validator_file, policy_file, execution_files_path,
    planner_command, translator_command, validator_command, preprograph_command,
    prefVizType, activeStateName;
  std::map<int, std::string> stateIDname, statePolicy;
  std::map<int, int>  stateOutcomeSize;
  std::map<std::pair <int, std::string>, int> stateOutcome;
  std::map<int, std::vector<std::string>> stateOutcomeList;
  ros::Publisher graphfile_pub;
protected:
  bool runPlanner();
public:
  PROBPRPPlannerInterface(ros::NodeHandle& nh);
  virtual ~PROBPRPPlannerInterface();
};

}  // namespace KCL_rosplan

#endif
