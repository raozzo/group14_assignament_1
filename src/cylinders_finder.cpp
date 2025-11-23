#include "group14_assignment_1/cylinders_finder.hpp"

#include <cmath>
#include <Eigen/Dense>

CylindersFinder::CylindersFinder(const rclcpp::NodeOptions &options)
    : Node("cylinders_finder_node", options)
{
    RCLCPP_INFO(this->get_logger(), "Cylinders finder node has been started.");

    /*
    // TF2 listener initialization
    RCLCPP_INFO(this->get_logger(), "Initializing TF2 listener");
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscription to LIDAR data
    RCLCPP_INFO(this->get_logger(), "Subscribing to /scan topic");
    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        rclcpp::QoS(10),
        std::bind(&CylindersFinder::process_scan, this, std::placeholders::_1));
    */

    // Service server configuration to respond to tables requests
    RCLCPP_INFO(this->get_logger(), "Creating 'look_for_tables' service...");
    service_ = this->create_service<group14_interfaces::srv::LookForTables>(
        "look_for_tables",
        std::bind(
            &CylindersFinder::look_for_tables_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));
}

void CylindersFinder::process_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    int scan_dim = static_cast<int>(scan->ranges.size());
    if (scan_dim == 0)
        return;

    /* try a conversion to map reference frame */
    std::optional<geometry_msgs::msg::TransformStamped> tf_to_map = transform_(scan->header, "map");
    if (tf_to_map.has_value())
    {
        /* ranges clustering */
        std::vector<std::vector<group14::RangePoint>> clusters;
        cluster_ranges_(scan, clusters);

        /* try to fit a circle to every cluster */
        for (std::vector<group14::RangePoint> &cluster : clusters)
            look_for_tables_(cluster, tf_to_map.value());
    }
    else
    {
        return; // if the scan cannot be referenced to the map frame, ignore it
    }
}

void CylindersFinder::look_for_tables_callback(
    const std::shared_ptr<group14_interfaces::srv::LookForTables::Request> request,
    std::shared_ptr<group14_interfaces::srv::LookForTables::Response> response)
{
    (void)request;
    for (Table &table : tables_)
    {
        if (table.num_detections > NUM_DETECTIONS_THRESHOLD)
        {
            group14_interfaces::msg::Table table_msg;
            table_msg.center = table.circle.center;
            table_msg.radius = table.circle.r;
            response.get()->tables.push_back(table_msg);
        }
    }
}

/* CYLINDERS FINDER - PRIVATE METHODS */

void CylindersFinder::cluster_ranges_(
    const sensor_msgs::msg::LaserScan::SharedPtr &scan,
    std::vector<std::vector<group14::RangePoint>> &clusters)
{
    clusters = {};

    std_msgs::msg::Header header = scan->header;
    int scan_dim = static_cast<int>(scan->ranges.size());
    float range_min = scan->range_min;
    float range_max = scan->range_max;

    // For each range measurement
    for (int i = 0; i < scan_dim; ++i)
    {
        // Remove unreliable measures
        float range = scan->ranges[i];
        if (range < range_min || range > range_max || std::isinf(range) || std::isnan(range))
            continue;

        // Build a RangePoint representing the measure
        float angle = scan->angle_min + i * scan->angle_increment;
        group14::RangePoint rp(range, angle, i, rangeToPointStamped_(range, angle, header));

        // Assign the current rp to the correct cluster
        if (clusters.empty())
        {
            clusters.emplace_back(std::vector<group14::RangePoint>{rp});
        }
        else
        {
            group14::RangePoint prev_rp = clusters.back().back();
            double euclidean_dist = std::hypot(rp.point.point.x - prev_rp.point.point.x,
                                               rp.point.point.y - prev_rp.point.point.y);
            double D_max = compute_dynamic_D_max_(prev_rp.range, (angle - prev_rp.angle),
                                                  INCIDENCE_ANGLE_THRESHOLD, SCAN_NOISE_FLOOR);

            if (euclidean_dist > D_max)
                clusters.emplace_back(std::vector<group14::RangePoint>{rp});
            else
                clusters.back().emplace_back(rp);
        }
    }

    // Merge the 1st and the last clusters if their extremities are closer than D_max
    group14::RangePoint rp_1 = clusters.back().back();
    group14::RangePoint rp_2 = clusters.front().front();
    double euclidean_dist = std::hypot(rp_2.point.point.x - rp_1.point.point.x,
                                       rp_2.point.point.y - rp_1.point.point.y);
    double D_max = compute_dynamic_D_max_(rp_1.range, (rp_2.angle + 2 * group14::PI - rp_1.angle),
                                          INCIDENCE_ANGLE_THRESHOLD, SCAN_NOISE_FLOOR);
    if (euclidean_dist > D_max)
        merge_cyclic_clusters_(clusters);

    // Remove clusters having less than 5 points
    clusters.erase(
        std::remove_if(clusters.begin(), clusters.end(),
                       [](std::vector<group14::RangePoint> &x)
                       { return x.size() < 5; }),
        clusters.end());
}

void CylindersFinder::merge_cyclic_clusters_(std::vector<std::vector<group14::RangePoint>> &clusters)
{
    std::vector<group14::RangePoint> &front = clusters.front();
    std::vector<group14::RangePoint> &back = clusters.back();
    front.reserve(front.size() + back.size());
    front.insert(front.begin(), back.begin(), back.end());

    clusters.pop_back();
}

geometry_msgs::msg::PointStamped CylindersFinder::rangeToPointStamped_(
    float range, float angle, std_msgs::msg::Header header)
{
    geometry_msgs::msg::PointStamped p;
    p.header = header;
    p.point.x = range * std::cos(angle);
    p.point.y = range * std::sin(angle);

    return p;
}

double CylindersFinder::compute_dynamic_D_max_(
    float prev_valid_range, float angle_increment,
    float INCIDENCE_ANGLE_THRESHOLD, float NOISE_FLOOR)
{
    return (prev_valid_range * sin(angle_increment)) / sin(INCIDENCE_ANGLE_THRESHOLD - angle_increment) + NOISE_FLOOR;
}

void CylindersFinder::look_for_tables_(const std::vector<group14::RangePoint> &cluster,
                                       geometry_msgs::msg::TransformStamped &tf)
{
    // Try to fit a circle to the cluster points with Kasa mathod
    std::optional<group14::Circle> circle = fit_circle_Kasa_(cluster);

    // If the cluster well represents a circular arc
    if (circle.has_value() && validate_circle_fit_(cluster, circle.value()))
    {
        // Build a new circle object (i.e. candidate table) with respect to the map frame
        geometry_msgs::msg::PointStamped center_wrt_map;
        tf2::doTransform(circle.value().center, center_wrt_map, tf);
        group14::Circle candidate_table(center_wrt_map, circle.value().r);

        // Compare this candidate table with already seen tables
        bool match_found = false;
        for (Table &t : tables_)
        {
            if (Table::is_same_table(t, candidate_table))
            {
                // Compute the distance between the robot [(0, 0) in the laserscan frame] and the circle center
                double distance = std::hypot(circle.value().center.point.x, circle.value().center.point.y);

                // Update the table estimate
                t.update(candidate_table, distance, static_cast<int>(cluster.size()));

                match_found = true;
                break;
            }
        }

        if (!match_found)
        {
            // New table discovered
            tables_.emplace_back(candidate_table);
        }
    }
}

std::optional<group14::Circle> CylindersFinder::fit_circle_Kasa_(const std::vector<group14::RangePoint> &cluster)
{
    int n_points = static_cast<int>(cluster.size());

    // for each point (x, y):
    // (x - x_c)^2 + (y - y_c)^2 = R^2
    // (x^2 - 2xx_c + x_c^2) + (y^2 - 2yy_c + y_c^2) = R^2
    // x*(2x_c) + y*(2y_c) + 1*(R^2 - x_c^2 - y_c^2) = x^2 + y^2
    // A*X = b

    Eigen::MatrixXd A(n_points, 3);
    Eigen::VectorXd b(n_points);
    for (int i = 0; i < n_points; ++i)
    {
        double x = cluster[i].point.point.x;
        double y = cluster[i].point.point.y;
        A(i, 0) = x;
        A(i, 1) = y;
        A(i, 2) = 1.0;
        b(i) = x * x + y * y;
    }

    // Least squares solution of the A*X = b system
    Eigen::Vector3d X = A.colPivHouseholderQr().solve(b);

    // Circle params decoding
    float x_center = X(0) / 2.0;
    float y_center = X(1) / 2.0;
    float radius = std::sqrt(X(2) + x_center * x_center + y_center * y_center); // X(2) = R^2 - x_c^2 - y_c^2

    // Radius validation
    if (std::isnan(radius))
        return std::nullopt;

    return group14::Circle(x_center, y_center, radius, cluster.front().point.header);
}

bool CylindersFinder::validate_circle_fit_(const std::vector<group14::RangePoint> &cluster,
                                           const group14::Circle &circle)
{
    // Radius check
    if (circle.r < MIN_RADIUS || circle.r > MAX_RADIUS)
        return false;

    // MSE check
    int n_points = static_cast<int>(cluster.size());
    double MSE = 0;
    for (const group14::RangePoint &p : cluster)
        MSE += std::pow(std::hypot(p.point.point.x - circle.center.point.x, p.point.point.y - circle.center.point.y) - circle.r, 2);
    MSE /= n_points;

    if (MSE > MSE_THRESHOLD)
        return false;

    return true;
}

std::optional<geometry_msgs::msg::TransformStamped> CylindersFinder::transform_(
    std_msgs::msg::Header header,
    const std::string dst_frame_id)
{
    try
    {
        geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
            dst_frame_id,
            header.frame_id,
            header.stamp,
            rclcpp::Duration::from_seconds(0.1));
        return tf;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform not available: %s", ex.what());
        return std::nullopt;
    }
}

/* TABLE CLASS */

Table::Table(group14::Circle circle_, double initial_weight, int num_detections_)
    : circle(circle_), cumulative_weight(initial_weight), num_detections(num_detections_) {}

void Table::update(const group14::Circle &new_circle, double distance, int cluster_size)
{
    // Compute a weight for the current measure (proportional to the number of points in the cluster,
    // inversely proportional to the distance at which the scan has been taken)
    double weight = cluster_size / (distance + 0.01);

    // Update the cumulative weight
    cumulative_weight += weight;

    // Update circle parameters estimate
    float x = circle.center.point.x; // old estimate of x
    float y = circle.center.point.y; // old estimate of y
    float r = circle.r;              // old estimate of r
    circle.center.point.x = x + (weight / cumulative_weight) * (new_circle.center.point.x - x);
    circle.center.point.y = y + (weight / cumulative_weight) * (new_circle.center.point.y - y);
    circle.r = r + (weight / cumulative_weight) * (new_circle.r - r);

    num_detections++;
}

bool Table::is_same_table(const Table &t, const group14::Circle &c)
{
    return std::hypot(t.circle.center.point.x - c.center.point.x, t.circle.center.point.y - c.center.point.y) < DISTANCE_THRESHOLD;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CylindersFinder>());
    rclcpp::shutdown();
    return 0;
}