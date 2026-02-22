#include "behaviortree_cpp/condition_node.h"
#include "ceres/ceres.h"
#include "rclcpp/rclcpp.hpp"

class CompareCeresSummaries : public BT::ConditionNode
{
public:
    CompareCeresSummaries(const std::string& name, const BT::NodeConfig& config)
        : BT::ConditionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<ceres::Solver::Summary>("summary_a"),
            BT::InputPort<ceres::Solver::Summary>("summary_b")
        };
    }

    BT::NodeStatus tick() override
    {
        ceres::Solver::Summary sum_a, sum_b;

        if (!getInput<ceres::Solver::Summary>("summary_a", sum_a) ||
            !getInput<ceres::Solver::Summary>("summary_b", sum_b))
        {
            RCLCPP_ERROR(rclcpp::get_logger("CeresSummaryComparer"), "Missing Ceres summaries on blackboard!");
            return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "--- Ceres Comparison Results ---");

        // 1. Compare Final Costs
        double cost_diff = sum_a.final_cost - sum_b.final_cost;
        RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "Cost A: %.6f | Cost B: %.6f", sum_a.final_cost, sum_b.final_cost);
        
        if (std::abs(cost_diff) < 1e-6) {
            RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "Result: Costs are effectively equal.");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "Result: %s performed better by %.6f", 
                       (cost_diff < 0 ? "Summary A" : "Summary B"), std::abs(cost_diff));
        }

        // 2. Compare Convergence/Termination
        RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "Terminations: A [%s] | B [%s]", 
                    sum_a.message.c_str(), sum_b.message.c_str());

        if (sum_a.termination_type == sum_b.termination_type) {
            RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "Both solvers terminated with the same status.");
        }

        // 3. Iteration Count
        RCLCPP_INFO(rclcpp::get_logger("CeresSummaryComparer"), "Iterations: A [%d] | B [%d]", 
                    (int)sum_a.iterations.size(), (int)sum_b.iterations.size());

        return BT::NodeStatus::SUCCESS;
    }
};

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<CompareCeresSummaries>("CompareCeresSummaries");
}
