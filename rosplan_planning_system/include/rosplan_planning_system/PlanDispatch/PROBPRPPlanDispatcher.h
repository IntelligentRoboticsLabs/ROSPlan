/*
 * This file describes the class used to dispatch a Simple (sequential) PDDL plan.
 */
#include "PlanDispatcher.h"

#include "rosplan_dispatch_msgs/CompletePlan.h"
#include <rosplan_dispatch_msgs/PROBPRPActionFeedback.h>
#include "rosplan_knowledge_msgs/KnowledgeQueryService.h"
#include "rosplan_knowledge_msgs/GetDomainOperatorDetailsService.h"
#include "rosplan_knowledge_msgs/DomainOperator.h"

#include <rosplan_planning_msgs/KeyValueIntInt.h>
#include <rosplan_planning_msgs/KeyValueIntIntMap.h>
#include <rosplan_planning_msgs/KeyValueIntStr.h>
#include <rosplan_planning_msgs/KeyValueIntStrMap.h>
#include <rosplan_planning_msgs/StateOutcome.h>
#include <rosplan_planning_msgs/StateOutcomeMap.h>
#include <rosplan_planning_msgs/StateOutcomeList.h>
#include <rosplan_planning_msgs/StateOutcomeListMap.h>

#include <map>

#ifndef KCL_simple_dispatcher
#define KCL_simple_dispatcher

namespace KCL_rosplan
{

	class PROBPRPPlanDispatcher: public PlanDispatcher
	{
	private:

		// current plan and time plan was recevied
		rosplan_dispatch_msgs::CompletePlan current_plan;
		double mission_start_time;

		/* check preconditions are true */
		bool checkPreconditions(rosplan_dispatch_msgs::ActionDispatch msg);
		ros::ServiceClient queryKnowledgeClient;
		ros::ServiceClient queryDomainClient;

		/* current action to dispatch */
		int current_action, activeStateID, goalState;
;
    std::map<int, std::string> stateIDname, statePolicy;
    std::map<int, int>  stateOutcomeSize;
    std::map<std::pair <int, std::string>, int> stateOutcome;
    std::map<int, std::vector<std::string>> stateOutcomeList;


	public:

		/* constructor */
		PROBPRPPlanDispatcher(ros::NodeHandle& nh);
		~PROBPRPPlanDispatcher();

    void stateIDCallback(const rosplan_planning_msgs::KeyValueIntStrMap::ConstPtr& id);
    void policyCallback(const rosplan_planning_msgs::KeyValueIntStrMap::ConstPtr& policy);
    void stateOutSizeCallback(const rosplan_planning_msgs::KeyValueIntIntMap::ConstPtr& size);
    void stateOutCallback(const rosplan_planning_msgs::StateOutcomeMap::ConstPtr& outcome);
    void stateOutListCallback(const rosplan_planning_msgs::StateOutcomeListMap::ConstPtr& outcome_list);
    void planCallback(const rosplan_dispatch_msgs::CompletePlan plan);

		void reset();

		bool dispatchPlanService(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
		bool dispatchPlan(double missionStartTime, double planStartTime);

    void feedbackCallback(const rosplan_dispatch_msgs::ActionFeedback::ConstPtr& msg){};
		void feedbackPROBPRPCallback(const rosplan_dispatch_msgs::PROBPRPActionFeedback::ConstPtr& msg);

	};
}

#endif
