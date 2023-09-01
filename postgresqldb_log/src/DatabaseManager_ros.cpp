#include <ros/ros.h>
#include <rosgraph_msgs/Log.h>
#include <postgresqldb_log/DatabaseManager.h> // #include <std_srvs/Trigger.h> // #include <std_srvs/SetBool.h> // #include <std_msgs/Int64.h>

#include "DatabaseManager.h"

#include <iostream>
#include <functional>
#include <string>



unsigned int in_counter;
unsigned int out_counter;
unsigned int qsize;
unsigned int drop_counter;

static pthread_mutex_t in_counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t out_counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t drop_counter_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t qsize_mutex = PTHREAD_MUTEX_INITIALIZER;

// string[] row_data      |  req.row_data
// string[] update_values |  req.update_values
// string search_value  |  req.search_value
// int32 primary_key      |  req.primary_key
// -------------------------------------------
// int32 status          |  res.status

int DatabaseManagerRos::create_cb(
    postgresqldb_log::DatabaseManager::Request &req,
    postgresqldb_log::DatabaseManager::Response &res)
{
    if (dbManager.createTable(req.table_name, table_columns)) {
        res.status = 1; // Success
        ROS_INFO("Record inserted successfully.");
    } else {
        res.status = 0; // Failure
        ROS_ERROR("Failed to insert record.");
    }
}



int DatabaseManagerRos::insert_cb(
    postgresqldb_log::DatabaseManager::Request &req,
    postgresqldb_log::DatabaseManager::Response &res) 
{
    std::vector<std::string> row_data;
    for (const std::string& column : req.row_data) { row_data.push_back(column); }
    if (dbManager.insertRecord(req.table_name, table_columns, row_data)) {
        res.status = 1; // Success
        ROS_INFO("Record inserted successfully.");
    } else {
        res.status = 0; // Failure
        ROS_ERROR("Failed to insert record.");
    }
}



int DatabaseManagerRos::update_cb(
    postgresqldb_log::DatabaseManager::Request &req,
    postgresqldb_log::DatabaseManager::Response &res)
{
    std::vector<std::string> row_data;
    for (const std::string& columnrd : req.row_data) {row_data.push_back(columnrd); }
    std::string search_value = req.search_value;
    std::vector<std::string> update_values;
    for (const std::string& columnuv : req.update_values) {row_data.push_back(columnuv); }
    if (dbManager.updateRecord(table_name, table_columns, 
        search_column, search_value, update_columns, update_values)) {
        res.status = 1; // Success
        ROS_INFO("Record inserted successfully.");
    } else {
        res.status = 0; // Failure
        ROS_ERROR("Failed to insert record.");
    }
}



int DatabaseManagerRos::delete_cb(
    postgresqldb_log::DatabaseManager::Request &req,
    postgresqldb_log::DatabaseManager::Response &res) 
{
    int primary_key = req.primary_key;
    if (dbManager.deleteRecord(table_name, table_columns, primary_key)) {
        res.status = 1; // Success
        ROS_INFO("Record inserted successfully.");
    } else {
        res.status = 0; // Failure
        ROS_ERROR("Failed to insert record.");
    }
}



void DatabaseManagerRos::RecordLogs(const rosgraph_msgs::Log::ConstPtr & log_msg)
{
        if (ShouldSendToCloudWatchLogs(log_msg->level)) {
            auto message = FormatLogs(log_msg);
            // time_ms = milliseconds.count();
            // std::chrono::milliseconds time_ms = 
            time_ms = 
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
            long long time_ms_count = time_ms.count();
      // write or insert message, time_ms into postgresql db.
    }

  // If we'd get access to the message queue this could be more useful
  // https://code.ros.org/trac/ros/ticket/744
  pthread_mutex_lock(&in_counter_mutex);
  ++in_counter;
  pthread_mutex_unlock(&in_counter_mutex);
  pthread_mutex_lock(&out_counter_mutex);
  ++out_counter;
  pthread_mutex_unlock(&out_counter_mutex);
}



void DatabaseManagerRos::print_count(const ros::TimerEvent &te)
{
  unsigned int l_in_counter, l_out_counter, l_drop_counter, l_qsize;

  pthread_mutex_lock(&in_counter_mutex);
  l_in_counter = in_counter; in_counter = 0;
  pthread_mutex_unlock(&in_counter_mutex);

  pthread_mutex_lock(&out_counter_mutex);
  l_out_counter = out_counter; out_counter = 0;
  pthread_mutex_unlock(&out_counter_mutex);

  pthread_mutex_lock(&drop_counter_mutex);
  l_drop_counter = drop_counter; drop_counter = 0;
  pthread_mutex_unlock(&drop_counter_mutex);

  pthread_mutex_lock(&qsize_mutex);
  l_qsize = qsize; qsize = 0;
  pthread_mutex_unlock(&qsize_mutex);

  printf("%u:%u:%u:%u\n", l_in_counter, l_out_counter, l_drop_counter, l_qsize);
  fflush(stdout);
}



bool DatabaseManagerRos::ShouldSendToCloudWatchLogs(const int8_t log_severity_level)
{
  return log_severity_level >= this->min_log_severity_;
}



const std::string DatabaseManagerRos::FormatLogs(const rosgraph_msgs::Log::ConstPtr & log_msg)
{
  std::stringstream ss;
  ss << log_msg->header.stamp << " ";
  switch (log_msg->level) {
    case rosgraph_msgs::Log::FATAL:
      ss << "FATAL ";
      break;
    case rosgraph_msgs::Log::ERROR:
      ss << "ERROR ";
      break;
    case rosgraph_msgs::Log::WARN:
      ss << "WARN ";
      break;
    case rosgraph_msgs::Log::DEBUG:
      ss << "DEBUG ";
      break;
    case rosgraph_msgs::Log::INFO:
      ss << "INFO ";
      break;
    default:
      ss << log_msg->level << " ";
  }
  ss << "[node name: " << log_msg->name << "] ";
  if (publish_topic_names_) {
    ss << "[topics: ";
    auto it = log_msg->topics.begin();
    auto end = log_msg->topics.end();
    for (; it != end; ++it) {
      const std::string & topic = *it;
      if (it != log_msg->topics.begin()) {
        ss << ", ";
      }
      ss << topic;
    }
    ss  << "] ";
  }
  ss << log_msg->msg << "\n";
  return ss.str();
}



void DatabaseManagerRos::ReadSubscriberList(
  const bool subscribe_to_rosout,
  std::vector<std::string> topics,
  const boost::function<void(const rosgraph_msgs::Log::ConstPtr &)>& callback,
  ros::NodeHandle & nh,
  std::vector<ros::Subscriber> & subscriptions)
{
  
  for (const std::string& topic : topics) {
    ros::Subscriber sub = nh.subscribe(topic, kNodeSubQueueSize, callback);
    ROS_INFO("Subscribing to topic: %s",  topic);
    subscriptions.push_back(sub);
  }
  if (subscribe_to_rosout) {
    ros::Subscriber sub = nh.subscribe(kNodeRosoutAggregatedTopicName, kNodeSubQueueSize, callback);
    ROS_INFO("Subscribing to rosout_agg : %s", kNodeRosoutAggregatedTopicName);
    subscriptions.push_back(sub);
  }

}



// void DatabaseManager::TriggerLogPublisher(const ros::TimerEvent & /*unused*/) {
//  this->log_service_->publishBatchedData();
// }







int main (int argc, char **argv)
{
    std::string kNodeName = "postgresqldb_log_tf";
    bool subscribe_to_rosout = true;
    bool publish_topic_names_ = true;
    std::vector<std::string> topics = {"tf", "odom", "scan"};
    double publish_frequency = 5; //hz
    in_counter = out_counter = drop_counter = qsize = 0;

    ros::init(argc, argv, kNodeName);
    ros::NodeHandle nh;

    std::string dbname;
    std::string user;
    std::string pswd;
    std::string hostaddr;
    std::string port;

    std::string table_name;
    std::vector<std::string> table_columns;
    std::string search_column;
    std::string update_columns;

    // Get parameters from ROS parameter server
    if (!nh.getParam("dbname", dbname) ||
        !nh.getParam("user", user) ||
        !nh.getParam("pswd", pswd) ||
        !nh.getParam("hostaddr", hostaddr) ||
        !nh.getParam("port", port) ||     
        !nh.getParam("table_name", table_name) ||
        !nh.getParam("table_columns", table_columns) ||
        !nh.getParam("table_update/search_column", search_column) ||
        !nh.getParam("table_update/update_columns", update_columns)) {
        ROS_ERROR("Failed to retrieve parameters from the ROS parameter server.");
        return 1;
    }

    // Print the obtained parameters
    ROS_INFO("Table Name: %s", table_name.c_str());
    ROS_INFO("Table Columns: ");
    for (const std::string& column : table_columns) { ROS_INFO("  %s", column.c_str()); }
    ROS_INFO("Search Column: %s", search_column.c_str());
    ROS_INFO("Update Column: %s", update_column.c_str());

    // Publish on a timer if we are not publishing on a size limit.
    // ros::Timer timer = nh.createTimer(ros::Duration(publish_frequency), [&dbManager](const ros::TimerEvent& event) { dbManager.TriggerLogPublisher(event); });

    // callback function
    boost::function<void(const rosgraph_msgs::Log::ConstPtr &)> callback;
    callback = [&DatabaseManagerRos](const rosgraph_msgs::Log::ConstPtr & log_msg) -> void {
        DatabaseManagerRos::RecordLogs(log_msg);
    };

    // subscribe to additional topics, if any
    std::vector<ros::Subscriber> subscriptions;
    DatabaseManagerRos::ReadSubscriberList(subscribe_to_rosout, topics, callback, nh, subscriptions);
    ROS_INFO("Initialized : %s",  kNodeName.c_str());

    // Initialize your DatabaseManager and perform database operations using these parameters
    DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
    if (!dbManager.connect()) {
        std::cerr << "Error connecting to the database" << std::endl;
    }
    std::cout << "Connected to the database" << std::endl;

    // ros::Subscriber sub = nh.subscribe("/number", 10, callback_number);
    ros::Publisher pub = nh.advertise<std_msgs::Int64>("/dbmngr_status", 10);
    ros::ServiceServer create_service = nh.advertiseService("/dbmngr_create", create_cb);
    ros::ServiceServer insert_service = nh.advertiseService("/dbmngr_insert", insert_cb);
    ros::ServiceServer delete_service = nh.advertiseService("/dbmngr_delete", delete_cb);
    ros::ServiceServer update_service = nh.advertiseService("/dbmngr_update", update_cb);

    // ros::Timer count_print_timer = nh.createTimer(ros::Duration(5, 0), print_count);
    std::function<void(const ros::TimerEvent&)> printCountFunc = std::bind(&DatabaseManager::print_count, &dbManager, std::placeholders::_1);
    ros::Timer count_print_timer = nh.createTimer(ros::Duration(5.0), printCountFunc);
 
    ros::spin();

    return 0;
}