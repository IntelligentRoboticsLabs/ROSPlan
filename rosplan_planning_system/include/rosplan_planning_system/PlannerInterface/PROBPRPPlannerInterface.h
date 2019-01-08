#include "PlannerInterface.h"
#include <fstream>
#include <sstream>
#include <string>
#include <ctime>
#include <streambuf>
#include <utility>
#include <map>
#include <vector>
#include <rosplan_planning_msgs/KeyValueIntInt.h>
#include <rosplan_planning_msgs/KeyValueIntIntMap.h>
#include <rosplan_planning_msgs/KeyValueIntStr.h>
#include <rosplan_planning_msgs/KeyValueIntStrMap.h>
#include <rosplan_planning_msgs/StateOutcome.h>
#include <rosplan_planning_msgs/StateOutcomeMap.h>
#include <rosplan_planning_msgs/StateOutcomeList.h>
#include <rosplan_planning_msgs/StateOutcomeListMap.h>


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
  void pubPlanMsgs();
  rosplan_planning_msgs::KeyValueIntStrMap buildIntStrMapMsg(std::map<int,std::string> map);
  rosplan_planning_msgs::KeyValueIntIntMap buildIntIntMapMsg(std::map<int,int> map);
  rosplan_planning_msgs::StateOutcomeMap buildStateOutcomeMapMsg(std::map<std::pair <int, std::string>, int> map);
  rosplan_planning_msgs::StateOutcomeListMap buildStateOutcomeListMapMsg(std::map<int, std::vector<std::string>> map);
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
  ros::Publisher graphfile_pub, state_id_pub, state_policy_pub, state_outcome_size_pub,
    state_outcome_pub, state_outcome_list_pub;
protected:
  bool runPlanner();
public:
  PROBPRPPlannerInterface(ros::NodeHandle& nh);
  virtual ~PROBPRPPlannerInterface();
};

}  // namespace KCL_rosplan

#endif
