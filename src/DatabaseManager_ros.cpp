
#include <ros/package.h>
#include <postgresqldb_log/DatabaseManager_ros.h>
#include <postgresqldb_log/DatabaseManager.h>

#include <iostream>
#include <yaml-cpp/yaml.h>

/***************************************************************************
 *  Created: Sun Sep 3 15:05:37 2023
 *  Copyright  2023  Azeez Adebayo
 ****************************************************************************/

// USAGE:
// source /app/birfen_ws/devel/setup.bash; roslaunch postgresqldb_log postgresqldb_log.launch

// SERVICE.srv
// string mode            |  req.mode
// string[] row_data      |  req.row_data
// string[] update_values |  req.update_values
// string search_value    |  req.search_value
// int32 primary_key      |  req.primary_key
// -------------------------------------------
// bool status            |  res.status



bool DatabaseManagerRos::create_cb(postgresqldb_log::Postgresqldb::Request& req,
      postgresqldb_log::Postgresqldb::Response& res) {
   DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
   if (dbManager.connect()) {
      if (req.mode == "task"){
         if (dbManager.createTable(task_table_name, task_table_columns)) {
            res.status = true; // Success
            ROS_INFO("task table created successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("Failed to create task table.");
         }
      } else if (req.mode == "robot"){
         if (dbManager.createTable(robot_table_name, robot_table_columns)) {
            res.status = true; // Success
            ROS_INFO("robot table created successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to create robot table.");
         }
      }
   }
   dbManager.disconnect();
   return true;
}




bool DatabaseManagerRos::insert_cb(postgresqldb_log::Postgresqldb::Request& req,
      postgresqldb_log::Postgresqldb::Response& res) {
   DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
   if (dbManager.connect()) {
      std::vector<std::string> row_data(req.row_data.begin(), req.row_data.end());
      if (req.mode == "task"){
         if (dbManager.insertRecord(task_table_name, task_table_columns, row_data)) {
            res.status = true; // Success
            ROS_INFO("task record inserted successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to insert task record.");
         }
      } else if (req.mode == "robot"){
         if (dbManager.insertRecord(robot_table_name, robot_table_columns, row_data)) {
            res.status = true; // Success
            ROS_INFO("robot record inserted successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to insert robot record.");
         }
      }
   }
   dbManager.disconnect();
   return true;
}




bool DatabaseManagerRos::delete_cb(postgresqldb_log::Postgresqldb::Request& req,
      postgresqldb_log::Postgresqldb::Response& res) {
   DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
   if (dbManager.connect()) {
      int primary_key = req.primary_key;
      if (req.mode == "task"){
         if (dbManager.deleteRecord(task_table_name, task_table_columns, primary_key)) {
            res.status = true; // Success
            ROS_INFO("task record deleted successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to delete task record.");
         }
      } else if (req.mode == "robot"){
         if (dbManager.deleteRecord(robot_table_name, robot_table_columns, primary_key)) {
            res.status = true; // Success
            ROS_INFO("robot record deleted successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to delete robot record.");
         }
      }
   }
   dbManager.disconnect();
   return true;
}




bool DatabaseManagerRos::update_cb(postgresqldb_log::Postgresqldb::Request& req,
      postgresqldb_log::Postgresqldb::Response& res) {
   DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
   if (dbManager.connect()) {
      std::vector<std::string> row_data;
      for (const std::string& columnrd : req.row_data) {row_data.push_back(columnrd);}
      std::string search_value = req.search_value;
      std::vector<std::string> update_values;
      for (const std::string& columnuv : req.update_values) {update_values.push_back(columnuv); }
      if (req.mode == "task"){
         if (dbManager.updateRecord(task_table_name, task_table_columns, 
            task_search_column, search_value, task_update_columns, update_values)) {
            res.status = true; // Success
            ROS_INFO("task record updated successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to update task record.");
         }
      } else if (req.mode == "robot"){
         if (dbManager.updateRecord(robot_table_name, robot_table_columns, 
            robot_search_column, search_value, robot_update_columns, update_values)) {
            res.status = true; // Success
            ROS_INFO("robot record updated successfully.");
         } else {
            res.status = false; // Failure
            ROS_ERROR("failed to update robot record.");
         }
      }
   }
   dbManager.disconnect();
   return true;
}




bool DatabaseManagerRos::select_cb(
   postgresqldb_log::Postgresqldb::Request& req,
   postgresqldb_log::Postgresqldb::Response& res) {
   // std::cout<<":-select_cb-:"<<dbname<<" "<<user<<" "<<pswd<<" "<<hostaddr<<" "<<port<<std::endl;
   std::map<std::string, std::vector<std::vector<std::string>>> record_ ;
   DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
   if (dbManager.connect()) {

      record_ = dbManager.selectRecords(task_table_name, task_table_columns);
      for (const auto& entry : record_) { // Iterate through the record
         // Print Table name and column names
         std::cout << "Table Name: " << entry.first << std::endl;
         std::cout << "Columns: ";
         std::cout << "id ";  // Add "id" column to the printed columns
         for (const std::string& column : task_table_columns) { std::cout << column << " "; }   
         std::cout << " " << std::endl; 
         // Iterate through the vectors associated with each table name
         for (const std::vector<std::string>& row : entry.second) {
            // new row indicator
            std::cout << "Row:" << std::endl;
            // Iterate through the elements in each row and match them with columns
            for (size_t i = 0; i < row.size(); ++i) {
                  std::cout << "  " << row[i] << std::endl;
                  // PUT A PUBLISHER HERE OR WRITE TO FILE OR ...
                  std_msgs::String msg; // Build the message content
                  std::stringstream ss;  // Create a message holder
                  ss << "  " << row[i]; // ss << "  " << task_table_columns[i] << ": " << row[i];
                  msg.data = ss.str();
                  task_table_pub.publish(msg); // Publish the message
            }
         }
      }
      
      record_ = dbManager.selectRecords(robot_table_name, robot_table_columns);
      for (const auto& entry : record_) { // Iterate through the record
         // Print Table name and column names
         std::cout << "Table Name: " << entry.first << std::endl;
         std::cout << "Columns: ";
         std::cout << "id ";  // Add "id" column to the printed columns
         for (const std::string& column : robot_table_columns) { std::cout << column << " "; }   
         std::cout << " " << std::endl; 
         // Iterate through the vectors associated with each table name
         for (const std::vector<std::string>& row : entry.second) {
            // new row indicator
            std::cout << "Row:" << std::endl;
            // Iterate through the elements in each row and match them with columns
            for (size_t i = 0; i < row.size(); ++i) {
                  std::cout << "  " << row[i] << std::endl;
                  // PUT A PUBLISHER HERE OR WRITE TO FILE OR ...
                  std_msgs::String msg; // Build the message content
                  std::stringstream ss;  // Create a message holder
                  ss << "  " << row[i]; // ss << "  " << robot_table_columns[i] << ": " << row[i];
                  msg.data = ss.str();
                  robot_table_pub.publish(msg); // Publish the message
            }
         }
      }
   res.status = true; // Success
   }
   dbManager.disconnect();
   return true;
}










// source /app/birfen_ws/devel/setup.bash; roslaunch postgresqldb_log postgresqldb_log.launch
int main (int argc, char **argv)
{
   DatabaseManagerRos manager;

   std::string kNodeName = "postgresqldb_log";
   ros::init(argc, argv, kNodeName);
   ros::NodeHandle nh;

   // Get parameters from ROS parameter server
   std::string param_name;
   std::vector<std::string> temp;

   // Set the values of the member variables with the values obtained from ROS parameters
   // Database properties
   if (nh.searchParam("dbname", param_name)) {
      nh.getParam(param_name, manager.dbname);
   } else { ROS_WARN("Parameter 'dbname' not defined");return 1; }

   if (nh.searchParam("user", param_name)) {
      nh.getParam(param_name, manager.user);
   } else { ROS_WARN("Parameter 'user' not defined"); return 1;}

   if (nh.searchParam("pswd", param_name)) {
      nh.getParam(param_name, manager.pswd);
   } else { ROS_WARN("Parameter 'pswd' not defined");return 1; }

   if (nh.searchParam("hostaddr", param_name)) {
      nh.getParam(param_name, manager.hostaddr);
   } else { ROS_WARN("Parameter 'hostaddr' not defined");return 1; }

   if (nh.searchParam("port", param_name)) {
      nh.getParam(param_name, manager.port);
   } else { ROS_WARN("Parameter 'port' not defined");return 1; }

   // extra param
   if (nh.searchParam("publish_frequency", param_name)) {
      nh.getParam(param_name, manager.publish_frequency);
   } else { ROS_WARN("Parameter 'publish_frequency' not defined");return 1; }

   // task properties
   if (nh.searchParam("task_table_name", param_name)) {
      nh.getParam(param_name, manager.task_table_name);
   } else { ROS_WARN("Parameter 'task_table_name' not defined");return 1; }

   if (nh.searchParam("task_table_columns", param_name)) {
      nh.getParam(param_name, temp);
      for (const std::string& column : temp) { manager.task_table_columns.push_back(column); }
   } else { ROS_WARN("Parameter 'task_table_columns' not defined");return 1; }

   if (nh.searchParam("task_table_update/task_search_column", param_name)) {
      nh.getParam(param_name, manager.task_search_column);
   } else { ROS_WARN("Parameter 'task_search_column' not defined");return 1; }
   
   if (nh.searchParam("task_table_update/task_update_columns", param_name)) {
      nh.getParam(param_name, temp);
      for (const std::string& column : temp) { manager.task_update_columns.push_back(column); }
   } else { ROS_WARN("Parameter 'task_update_columns' not defined");return 1; }

   // robot properties
   if (nh.searchParam("robot_table_name", param_name)) {
      nh.getParam(param_name, manager.robot_table_name);
   } else { ROS_WARN("Parameter 'robot_table_name' not defined");return 1; }

   if (nh.searchParam("robot_table_columns", param_name)) {
      nh.getParam(param_name, manager.robot_table_columns);
   } else { ROS_WARN("Parameter 'robot_table_columns' not defined");return 1; }

   if (nh.searchParam("robot_table_update/robot_search_column", param_name)) {
      nh.getParam(param_name, manager.robot_search_column);
   } else { ROS_WARN("Parameter 'robot_search_column' not defined");return 1; }
   
   if (nh.searchParam("robot_table_update/robot_update_columns", param_name)) {
      nh.getParam(param_name, manager.robot_update_columns);
   } else { ROS_WARN("Parameter 'robot_update_columns' not defined");return 1; }

   // Test DbManager and perform database operations using these parameters
   // DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);
   // if (!dbManager.connect()) {std::cerr << "Error connecting to db" << std::endl; }

   // Advertise the service servers
   manager.create_service = nh.advertiseService(
      "dbmngr_create", 
      &DatabaseManagerRos::create_cb, 
      &manager
   );
   manager.insert_service = nh.advertiseService(
      "dbmngr_insert", 
      &DatabaseManagerRos::insert_cb, 
      &manager
   );
   manager.delete_service = nh.advertiseService(
      "dbmngr_delete", 
      &DatabaseManagerRos::delete_cb, 
      &manager
   );
   manager.update_service = nh.advertiseService(
      "dbmngr_update", 
      &DatabaseManagerRos::update_cb, 
      &manager
   );
   manager.select_service = nh.advertiseService(
      "dbmngr_select", 
      &DatabaseManagerRos::select_cb, 
      &manager
   );
   
   manager.task_table_pub = nh.advertise<std_msgs::String>("/dbmngr_tasktable", 10);
   manager.robot_table_pub = nh.advertise<std_msgs::String>("/dbmngr_robottable", 10);
   // ros::Subscriber sub = nh.subscribe("/number", 10, callback_number);

   // Publish on a timer if we are not publishing on a size limit.
   postgresqldb_log::Postgresqldb::Request req;
   postgresqldb_log::Postgresqldb::Response res;
   // req.mode = "robot"; //---> robot || task
   // manager.create_cb(req, res);
   // req.mode = "task"; //---> robot || task
   // manager.create_cb(req, res);
   // req.row_data =  {"value1", "value2", "value3"};
   // manager.insert_cb(req, res);
   ros::Timer timer = nh.createTimer(ros::Duration(manager.publish_frequency), 
      [&manager, &req, &res](const ros::TimerEvent& /*unused*/) {
         manager.select_cb(req, res); // Call with req and res
      }
   );
   
   ROS_INFO("Initialized Node: %s",  kNodeName.c_str());
   std::this_thread::sleep_for(std::chrono::seconds(3)); // Sleep for 3 seconds

   ros::spin();

   return 0;
}








































































































































































































// ~/birfen_ws/src$ chmod +x test_db.cpp
// ~/birfen_ws/src$ g++ -o db test_db.cpp -lpqxx -lpq && chmod +x db && ./db

// apt-cache search postgresql
// sudo apt-get install libpqxx-dev   
// sudo apt-get install libpqxx-6.4

// pg_config --version
// sudo gedit /etc/postgresql/14/main/postgresql.conf
// sudo gedit /etc/postgresql/14/main/pg_hba.conf

















// const std::string DatabaseManagerRos::FormatLogs(const rosgraph_msgs::Log::ConstPtr & log_msg)
// {
//   std::stringstream ss;
//   ss << log_msg->header.stamp << " ";
//   switch (log_msg->level) {
//     case rosgraph_msgs::Log::FATAL:
//       ss << "FATAL ";
//       break;
//     case rosgraph_msgs::Log::ERROR:
//       ss << "ERROR ";
//       break;
//     case rosgraph_msgs::Log::WARN:
//       ss << "WARN ";
//       break;
//     case rosgraph_msgs::Log::DEBUG:
//       ss << "DEBUG ";
//       break;
//     case rosgraph_msgs::Log::INFO:
//       ss << "INFO ";
//       break;
//     default:
//       ss << log_msg->level << " ";
//   }
//   ss << "[node name: " << log_msg->name << "] ";
//   if (publish_topic_names_) {
//     ss << "[topics: ";
//     auto it = log_msg->topics.begin();
//     auto end = log_msg->topics.end();
//     for (; it != end; ++it) {
//       const std::string & topic = *it;
//       if (it != log_msg->topics.begin()) {
//         ss << ", ";
//       }
//       ss << topic;
//     }
//     ss  << "] ";
//   }
//   ss << log_msg->msg << "\n";
//   return ss.str();
// }















   // try {
      // std::string yaml_path = ros::package::getPath("postgresqldb_log") ; //+ "/config/dbmngr_params.yaml" 
      // YAML::Node config = YAML::LoadFile("/app/birfen_ws/src/postgresqldb_log/config/dbmngr_params.yaml");
      // std::string dbname = config["dbname"].as<std::string>();
      // std::string user = config["user"].as<std::string>();
      // std::string pswd = config["pswd"].as<std::string>();
      // std::string hostaddr = config["hostaddr"].as<std::string>();
      // std::string port = config["port"].as<std::string>(); //.as<int>();
      // // Task properties
      // std::string task_table_name = config["task_table_name"].as<std::string>();
      // std::vector<std::string> task_table_columns = config["task_table_columns"].as<std::vector<std::string>>();
      // std::string task_search_column = config["task_table_update"]["task_search_column"].as<std::string>();
      // std::vector<std::string> task_update_columns = config["task_table_update"]["task_update_columns"].as<std::vector<std::string>>();
      // // Robot properties
      // std::string robot_table_name = config["robot_table_name"].as<std::string>();
      // std::vector<std::string> robot_table_columns = config["robot_table_columns"].as<std::vector<std::string>>();
      // std::string robot_search_column = config["robot_table_update"]["robot_search_column"].as<std::string>();
      // std::vector<std::string> robot_update_columns = config["robot_table_update"]["robot_update_columns"].as<std::vector<std::string>>();
   // } catch (const std::exception& e) {
   //    std::cerr << "Error: " << e.what() << std::endl;
   // }










// Connecting To Database
/* 
#include <iostream>
#include <pqxx/pqxx> 

using namespace std;
using namespace pqxx;

int main(int argc, char* argv[]) {
   try {
      connection C("dbname = postgres user = postgres password = root \
      hostaddr = 127.0.0.1 port = 5432");
      if (C.is_open()) {
         cout << "Opened database successfully: " << C.dbname() << endl;
      } else {
         cout << "Can't open database" << endl;
         return 1;
      }
      C.disconnect ();
   } catch (const std::exception &e) {
      cout << "wrong database param i guess " << endl;
      cerr << e.what() << std::endl;
      return 1;
   }
}
 */
// $g++ test.cpp -lpqxx -lpq
// $./a.out
// Opened database successfully: testdb








// Create a Table
/* 
#include <iostream>
#include <pqxx/pqxx> 

using namespace std;
using namespace pqxx;

int main(int argc, char* argv[]) {
   char * sql;
   
   try {
      connection C("dbname = testdb user = postgres password = cohondob \
      hostaddr = 127.0.0.1 port = 5432");
      if (C.is_open()) {
         cout << "Opened database successfully: " << C.dbname() << endl;
      } else {
         cout << "Can't open database" << endl;
         return 1;
      }

      // Create SQL statement 
      sql = "CREATE TABLE COMPANY("  \
      "ID INT PRIMARY KEY     NOT NULL," \
      "NAME           TEXT    NOT NULL," \
      "AGE            INT     NOT NULL," \
      "ADDRESS        CHAR(50)," \
      "SALARY         REAL );";

      // Create a transactional object. 
      work W(C);
      
      // Execute SQL query 
      W.exec( sql );
      W.commit();
      cout << "Table created successfully" << endl;
      C.disconnect();
   } catch (const std::exception &e) {
      cerr << e.what() << std::endl;
      return 1;
   }

   return 0;
}
*/
// Opened database successfully: testdb
// Table created successfully







// INSERT Operation
/* 
#include <iostream>
#include <pqxx/pqxx> 

using namespace std;
using namespace pqxx;

int main(int argc, char* argv[]) {
   char * sql;
   
   try {
      connection C("dbname = testdb user = postgres password = cohondob \
      hostaddr = 127.0.0.1 port = 5432");
      if (C.is_open()) {
         cout << "Opened database successfully: " << C.dbname() << endl;
      } else {
         cout << "Can't open database" << endl;
         return 1;
      }

      // Create SQL statement 
      sql = "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) "  \
         "VALUES (1, 'Paul', 32, 'California', 20000.00 ); " \
         "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY) "  \
         "VALUES (2, 'Allen', 25, 'Texas', 15000.00 ); "     \
         "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY)" \
         "VALUES (3, 'Teddy', 23, 'Norway', 20000.00 );" \
         "INSERT INTO COMPANY (ID,NAME,AGE,ADDRESS,SALARY)" \
         "VALUES (4, 'Mark', 25, 'Rich-Mond ', 65000.00 );";

      // Create a transactional object. 
      work W(C);
      
      // Execute SQL query 
      W.exec( sql );
      W.commit();
      cout << "Records created successfully" << endl;
      C.disconnect ();
   } catch (const std::exception &e) {
      cerr << e.what() << std::endl;
      return 1;
   }

   return 0;
}
 */
// Opened database successfully: testdb
// Records created successfully







// SELECT Operation
/* 
#include <iostream>
#include <pqxx/pqxx> 

using namespace std;
using namespace pqxx;

int main(int argc, char* argv[]) {
   char * sql;
   
   try {
      connection C("dbname = testdb user = postgres password = cohondob \
      hostaddr = 127.0.0.1 port = 5432");
      if (C.is_open()) {
         cout << "Opened database successfully: " << C.dbname() << endl;
      } else {
         cout << "Can't open database" << endl;
         return 1;
      }

      // Create SQL statement 
      sql = "SELECT * from COMPANY";

      // Create a non-transactional object. 
      nontransaction N(C);
      
      // Execute SQL query 
      result R( N.exec( sql ));
      
      // List down all the records 
      for (result::const_iterator c = R.begin(); c != R.end(); ++c) {
         cout << "ID = " << c[0].as<int>() << endl;
         cout << "Name = " << c[1].as<string>() << endl;
         cout << "Age = " << c[2].as<int>() << endl;
         cout << "Address = " << c[3].as<string>() << endl;
         cout << "Salary = " << c[4].as<float>() << endl;
      }
      cout << "Operation done successfully" << endl;
      C.disconnect ();
   } catch (const std::exception &e) {
      cerr << e.what() << std::endl;
      return 1;
   }

   return 0;
}
 */
// Opened database successfully: testdb
// ID = 1
// Name = Paul
// Age = 32
// Address = California
// Salary = 20000
// ID = 2
// Name = Allen
// Age = 25
// Address = Texas
// Salary = 15000
// ID = 3
// Name = Teddy
// Age = 23
// Address = Norway
// Salary = 20000
// ID = 4
// Name = Mark
// Age = 25
// Address = Rich-Mond
// Salary = 65000
// Operation done successfully






// UPDATE Operation
/* 
#include <iostream>
#include <pqxx/pqxx> 

using namespace std;
using namespace pqxx;

int main(int argc, char* argv[]) {
   char * sql;
   
   try {
      connection C("dbname = testdb user = postgres password = cohondob \
      hostaddr = 127.0.0.1 port = 5432");
      if (C.is_open()) {
         cout << "Opened database successfully: " << C.dbname() << endl;
      } else {
         cout << "Can't open database" << endl;
         return 1;
      }
      
      // Create a transactional object. 
      work W(C);
      // Create  SQL UPDATE statement 
      sql = "UPDATE COMPANY set SALARY = 25000.00 where ID=1";
      // Execute SQL query 
      W.exec( sql );
      W.commit();
      cout << "Records updated successfully" << endl;
      
      // Create SQL SELECT statement 
      sql = "SELECT * from COMPANY";

      // Create a non-transactional object. 
      nontransaction N(C);
      
      // Execute SQL query 
      result R( N.exec( sql ));
      
      // List down all the records 
      for (result::const_iterator c = R.begin(); c != R.end(); ++c) {
         cout << "ID = " << c[0].as<int>() << endl;
         cout << "Name = " << c[1].as<string>() << endl;
         cout << "Age = " << c[2].as<int>() << endl;
         cout << "Address = " << c[3].as<string>() << endl;
         cout << "Salary = " << c[4].as<float>() << endl;
      }
      cout << "Operation done successfully" << endl;
      C.disconnect ();
   } catch (const std::exception &e) {
      cerr << e.what() << std::endl;
      return 1;
   }

   return 0;
}

 */
// Opened database successfully: testdb
// Records updated successfully
// ID = 2
// Name = Allen
// Age = 25
// Address = Texas
// Salary = 15000
// ID = 3
// Name = Teddy
// Age = 23
// Address = Norway
// Salary = 20000
// ID = 4
// Name = Mark
// Age = 25
// Address = Rich-Mond
// Salary = 65000
// ID = 1
// Name = Paul
// Age = 32
// Address = California
// Salary = 25000
// Operation done successfully










// DELETE Operation
/* 
#include <iostream>
#include <pqxx/pqxx> 

using namespace std;
using namespace pqxx;

int main(int argc, char* argv[]) {
   char * sql;
   
   try {
      connection C("dbname = testdb user = postgres password = cohondob \
      hostaddr = 127.0.0.1 port = 5432");
      if (C.is_open()) {
         cout << "Opened database successfully: " << C.dbname() << endl;
      } else {
         cout << "Can't open database" << endl;
         return 1;
      }
      
      // Create a transactional object. 
      work W(C);
      // Create  SQL DELETE statement 
      sql = "DELETE from COMPANY where ID = 2";
      // Execute SQL query 
      W.exec( sql );
      W.commit();
      cout << "Records deleted successfully" << endl;
      
      // Create SQL SELECT statement 
      sql = "SELECT * from COMPANY";

      // Create a non-transactional object. 
      nontransaction N(C);
      
      // Execute SQL query 
      result R( N.exec( sql ));
      
      // List down all the records 
      for (result::const_iterator c = R.begin(); c != R.end(); ++c) {
         cout << "ID = " << c[0].as<int>() << endl;
         cout << "Name = " << c[1].as<string>() << endl;
         cout << "Age = " << c[2].as<int>() << endl;
         cout << "Address = " << c[3].as<string>() << endl;
         cout << "Salary = " << c[4].as<float>() << endl;
      }
      cout << "Operation done successfully" << endl;
      C.disconnect ();
   } catch (const std::exception &e) {
      cerr << e.what() << std::endl;
      return 1;
   }

   return 0;
}
 */

// Opened database successfully: testdb
// Records deleted successfully
// ID = 3
// Name = Teddy
// Age = 23
// Address = Norway
// Salary = 20000
// ID = 4
// Name = Mark
// Age = 25
// Address = Rich-Mond
// Salary = 65000
// ID = 1
// Name = Paul
// Age = 32
// Address = California
// Salary = 25000
// Operation done successfully
