#include "rosplan_action_interface/RPActionInterface.h"

/* The implementation of RPMoveBase.h */
namespace KCL_rosplan {


  bool RPActionInterface::checkConditions(const std::vector<rosplan_knowledge_msgs::DomainFormula>& df,
                                          const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg, bool positive)
  {
    // ROS_INFO("check conditions");
    bool ret = true;
    std::vector<rosplan_knowledge_msgs::DomainFormula>::const_iterator pit;
    for (pit = df.begin(); pit != df.end(); pit++)
    {
      rosplan_knowledge_msgs::KnowledgeItem req_ki;
      req_ki.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
      req_ki.attribute_name = pit->name;
      req_ki.is_negative = !positive;

      // ROS_INFO("[%s]", pit->name.c_str());

      int c = 0;
      std::vector<diagnostic_msgs::KeyValue>::const_iterator it_op;
      for (it_op = pit->typed_parameters.begin(); it_op != pit->typed_parameters.end(); ++it_op)
      {

        req_ki.instance_type = it_op->value;
        std::vector<diagnostic_msgs::KeyValue>::const_iterator it_msg;
        for (it_msg = msg->parameters.begin(); it_msg != msg->parameters.end(); ++it_msg)
        {

          if (it_msg->key == it_op->key)
          {
            diagnostic_msgs::KeyValue kv;

            rosplan_knowledge_msgs::DomainFormula f = predicates[pit->name];
            std::vector<diagnostic_msgs::KeyValue>::const_iterator it_pred;
            for (it_pred = f.typed_parameters.begin(); it_pred != f.typed_parameters.end(); ++it_pred)
            {
              if (it_op->value == it_pred->value)
                kv.key = predicate_args_[req_ki.attribute_name][c].first;
            }
            kv.value = it_msg->value;
            req_ki.values.push_back(kv);
          }
        }
        c++;
      }
      rosplan_knowledge_msgs::KnowledgeQueryService query;
      query.request.knowledge.push_back(req_ki);
      if (query_knowledge_client.call(query))
        ret = ret && query.response.all_true;
    }
    return ret;
  }

  bool RPActionInterface::checkAtStartConditions(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
  {
    return checkConditions(op.at_start_simple_condition, msg) && checkConditions(op.at_start_neg_condition, msg, false);
  }

  bool RPActionInterface::checkAtEndConditions(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
  {
    return checkConditions(op.at_end_simple_condition, msg) && checkConditions(op.at_end_neg_condition, msg, false);
  }
  bool RPActionInterface::checkOverAllConditions(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg)
  {
    return checkConditions(op.over_all_simple_condition, msg) && checkConditions(op.over_all_neg_condition, msg, false);
  }

	/* run action interface */
	void RPActionInterface::runActionInterface() {

		ros::NodeHandle nh("~");

		// set action name
		nh.getParam("pddl_action_name", params.name);

		// knowledge base services
		std::string kb = "rosplan_knowledge_base";
		nh.getParam("knowledge_base", kb);

		// fetch action params
		std::stringstream ss;
		ss << "/" << kb << "/domain/operator_details";
		ros::service::waitForService(ss.str(),ros::Duration(20));
		ros::ServiceClient client = nh.serviceClient<rosplan_knowledge_msgs::GetDomainOperatorDetailsService>(ss.str());
		rosplan_knowledge_msgs::GetDomainOperatorDetailsService srv;
		srv.request.name = params.name;
		if(client.call(srv)) {
			params = srv.response.op.formula;
			op = srv.response.op;
		} else {
			ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", params.name.c_str());
			return;
		}

    ss.str("");
		ss << "/" << kb << "/domain/predicates";
    ros::ServiceClient domain_predicates_client =
        nh.serviceClient<rosplan_knowledge_msgs::GetDomainAttributeService>(ss.str());
    rosplan_knowledge_msgs::GetDomainAttributeService pred_srv;
    if (domain_predicates_client.call(pred_srv))
    {
      for (int i = 0; i < pred_srv.response.items.size(); i++)
      {
        std::vector<std::pair<std::string, std::string> > args;
        for (int j = 0; j < pred_srv.response.items[i].typed_parameters.size(); j++)
        {
          std::pair<std::string, std::string> kv(pred_srv.response.items[i].typed_parameters[j].key,
                                                 pred_srv.response.items[i].typed_parameters[j].value);
          args.push_back(kv);
        }
        predicate_args_[pred_srv.response.items[i].name] = args;
      }
    }
    else
    {
      ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for operator details, %s", params.name.c_str());
      return;
    }

		// collect predicates from operator description
		std::vector<std::string> predicateNames;

		// effects
		std::vector<rosplan_knowledge_msgs::DomainFormula>::iterator pit = op.at_start_add_effects.begin();
		for(; pit!=op.at_start_add_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_start_del_effects.begin();
		for(; pit!=op.at_start_del_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_add_effects.begin();
		for(; pit!=op.at_end_add_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_del_effects.begin();
		for(; pit!=op.at_end_del_effects.end(); pit++)
			predicateNames.push_back(pit->name);

		// simple conditions
		pit = op.at_start_simple_condition.begin();
		for(; pit!=op.at_start_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.over_all_simple_condition.begin();
		for(; pit!=op.over_all_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_simple_condition.begin();
		for(; pit!=op.at_end_simple_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		// negative conditions
		pit = op.at_start_neg_condition.begin();
		for(; pit!=op.at_start_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.over_all_neg_condition.begin();
		for(; pit!=op.over_all_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		pit = op.at_end_neg_condition.begin();
		for(; pit!=op.at_end_neg_condition.end(); pit++)
			predicateNames.push_back(pit->name);

		// fetch and store predicate details
		ss.str("");
		ss << "/" << kb << "/domain/predicate_details";
		ros::service::waitForService(ss.str(),ros::Duration(20));
		ros::ServiceClient predClient = nh.serviceClient<rosplan_knowledge_msgs::GetDomainPredicateDetailsService>(ss.str());
		std::vector<std::string>::iterator nit = predicateNames.begin();
		for(; nit!=predicateNames.end(); nit++) {
			if (predicates.find(*nit) != predicates.end()) continue;
			if (*nit == "=" || *nit == ">" || *nit == "<" || *nit == ">=" || *nit == "<=") continue;
			rosplan_knowledge_msgs::GetDomainPredicateDetailsService predSrv;
			predSrv.request.name = *nit;
			if(predClient.call(predSrv)) {
				predicates.insert(std::pair<std::string, rosplan_knowledge_msgs::DomainFormula>(*nit, predSrv.response.predicate));
			} else {
				ROS_ERROR("KCL: (RPActionInterface) could not call Knowledge Base for predicate details, %s", params.name.c_str());
				return;
			}
		}

		// create PDDL info publisher
		ss.str("");
		ss << "/" << kb << "/pddl_action_parameters";
		pddl_action_parameters_pub = nh.advertise<rosplan_knowledge_msgs::DomainFormula>(ss.str(), 10, true);

		// create the action feedback publisher
		std::string aft = "/rosplan_plan_dispatcher/action_feedback";
		nh.getParam("action_feedback_topic", aft);
		action_feedback_pub = nh.advertise<rosplan_dispatch_msgs::ActionFeedback>(aft, 10, true);

		// knowledge interface
		ss.str("");
		ss << "/" << kb << "/update_array";
		update_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeUpdateServiceArray>(ss.str());

    ss.str("");
		ss << "/" << kb << "/query_state";
    query_knowledge_client = nh.serviceClient<rosplan_knowledge_msgs::KnowledgeQueryService>(ss.str());
		// listen for action dispatch
		std::string adt = "/rosplan_plan_dispatcher/action_dispatch";
		nh.getParam("action_dispatch_topic", adt);
		ros::Subscriber ds = nh.subscribe(adt, 1000, &KCL_rosplan::RPActionInterface::dispatchCallback, this);

		// loop
		ros::Rate loopRate(50);
		int counter = 0;
		ROS_INFO("KCL: (%s) Ready to receive", params.name.c_str());

		while(ros::ok()) {

			counter++;
			if(counter==20) {
				pddl_action_parameters_pub.publish(params);
				counter = 0;
			}

			loopRate.sleep();
			ros::spinOnce();
		}
	}

	/* run action interface */
	void RPActionInterface::dispatchCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr& msg) {

		// check action name
		if(0!=msg->name.compare(params.name)) return;

		ROS_INFO("KCL: (%s) action received", params.name.c_str());

		// check PDDL parameters
		std::vector<bool> found(params.typed_parameters.size(), false);
		std::map<std::string, std::string> boundParameters;
		for(size_t j=0; j<params.typed_parameters.size(); j++) {
			for(size_t i=0; i<msg->parameters.size(); i++) {
				if(params.typed_parameters[j].key == msg->parameters[i].key) {
					boundParameters[msg->parameters[i].key] = msg->parameters[i].value;
          found[j] = true;
          break;
				}
			}
			if(!found[j]) {
				ROS_INFO("KCL: (%s) aborting action dispatch; malformed parameters, missing %s", params.name.c_str(), params.typed_parameters[j].key.c_str());
				return;
			}
		}
		// send feedback (enabled)
		rosplan_dispatch_msgs::ActionFeedback fb;
		fb.action_id = msg->action_id;
		fb.status = "action enabled";
		action_feedback_pub.publish(fb);
    action_success = concreteCallback(msg);
    if (action_success)
		{
			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;

			// simple START del effects
			for(int i=0; i<op.at_start_del_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_start_del_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_start_del_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_start_del_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_start_del_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
			}

			// simple START add effects
			for(int i=0; i<op.at_start_add_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_start_add_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_start_add_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_start_add_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_start_add_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
			}

			if(updatePredSrv.request.knowledge.size()>0 && !update_knowledge_client.call(updatePredSrv))
				ROS_INFO("KCL: (%s) failed to update PDDL model in knowledge base", params.name.c_str());
		}

		// action_success = concreteCallback(msg); This call the action in duplicate. It is not neccessary, action_success is set in feedback callback.
		if(action_success) {
			// update knowledge base
			rosplan_knowledge_msgs::KnowledgeUpdateServiceArray updatePredSrv;

			// simple END del effects
			for(int i=0; i<op.at_end_del_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
        item.attribute_name = op.at_end_del_effects[i].name;
				item.values.clear();				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_end_del_effects[i].typed_parameters.size(); j++) {          pair.key = predicates[op.at_end_del_effects[i].name].typed_parameters[j].key;
          pair.value = boundParameters[op.at_end_del_effects[i].typed_parameters[j].key];
          item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::REMOVE_KNOWLEDGE);
			}
			// simple END add effects
			for(int i=0; i<op.at_end_add_effects.size(); i++) {
				rosplan_knowledge_msgs::KnowledgeItem item;
				item.knowledge_type = rosplan_knowledge_msgs::KnowledgeItem::FACT;
				item.attribute_name = op.at_end_add_effects[i].name;
				item.values.clear();
				diagnostic_msgs::KeyValue pair;
				for(size_t j=0; j<op.at_end_add_effects[i].typed_parameters.size(); j++) {
					pair.key = predicates[op.at_end_add_effects[i].name].typed_parameters[j].key;
					pair.value = boundParameters[op.at_end_add_effects[i].typed_parameters[j].key];
					item.values.push_back(pair);
				}
				updatePredSrv.request.knowledge.push_back(item);
				updatePredSrv.request.update_type.push_back(rosplan_knowledge_msgs::KnowledgeUpdateService::Request::ADD_KNOWLEDGE);
			}
			if(updatePredSrv.request.knowledge.size()>0 && !update_knowledge_client.call(updatePredSrv))
				ROS_INFO("KCL: (%s) failed to update PDDL model in knowledge base", params.name.c_str());

			// publish feedback (achieved)
			fb.status = "action achieved";
			action_feedback_pub.publish(fb);
		} else {

			// publish feedback (failed)
			fb.status = "action failed";
			action_feedback_pub.publish(fb);
		}
	}

} // close namespace
