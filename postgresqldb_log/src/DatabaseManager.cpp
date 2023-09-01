/***************************************************************************
 *  Created: Mon Dec 13 15:05:37 2010
 *  Copyright  2010  Azeez Adebayo
 ****************************************************************************/
#include "DatabaseManager.h"
#include <iostream>
#include <functional>
#include <string>


DatabaseManager::DatabaseManager(const std::string& dbname, const std::string& user,
                                 const std::string& password, const std::string& hostaddr,
                                 const std::string& port)
    : connection_("dbname=" + dbname + " user=" + user + " password=" + password +
                  " hostaddr=" + hostaddr + " port=" + port) {}


bool DatabaseManager::connect() {
    if (connection_.is_open()) {
        return true;
    }
    return false;
}


void DatabaseManager::disconnect() {
    connection_.disconnect();
}


bool DatabaseManager::createTable(const std::string& table_name, const std::vector<std::string>& table_columns) {
    try {
        pqxx::work W(connection_);

        // Construct the SQL query dynamically based on table_name and table_columns
        std::string sql = "CREATE TABLE " + table_name + " (ID INT PRIMARY KEY NOT NULL, ";
        for (size_t i = 0; i < table_columns.size(); ++i) {
            sql += table_columns[i] + " TEXT";
            if (i < table_columns.size() - 1) {
                sql += ", ";
            }
        }
        sql += ");";
        W.exec(sql.c_str()); // Convert std::string to const char*
        W.commit();
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}


bool DatabaseManager::insertRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     const std::vector<std::string>& row_data) {
    try {
        pqxx::work W(connection_);
        if (table_columns.size() != row_data.size()) {
            return false; // Number of columns and data elements should match
        }
        // table_columns: ["task_id","robot_id","signal_time","signal_type","signal_description"] 
        std::string sql = "INSERT INTO " + table_name + " (ID,";
        for (size_t i = 0; i < table_columns.size(); ++i) {
            sql += table_columns[i];
            if (i < table_columns.size() - 1) {
                sql += ",";
            }
        }
        sql += ") VALUES (DEFAULT,";
        for (size_t i = 0; i < row_data.size(); ++i) {
            sql += "'" + row_data[i] + "'";
            if (i < row_data.size() - 1) {
                sql += ",";
            }
        }
        sql += ");";
        W.exec(sql.c_str()); // Convert std::string to const char*
        W.commit();
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}


bool DatabaseManager::updateRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     const std::string& search_column, const std::string& search_value,
                     const std::vector<std::string>& update_columns, const std::vector<std::string>& update_values) {
    try {
        pqxx::work W(connection_);
        if (update_columns.size() != update_values.size()) {
            return false; // Number of columns and values for update should match
        }
        
        std::string sql = "UPDATE " + table_name + " SET ";
        for (size_t i = 0; i < update_columns.size(); ++i) {
            sql += update_columns[i] + " = '" + update_values[i] + "'";
            if (i < update_columns.size() - 1) {
                sql += ",";
            }
        }
        sql += " WHERE " + search_column + " = '" + search_value + "';"; // time = time

        W.exec(sql.c_str()); // Convert std::string to const char*
        W.commit();
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}


bool DatabaseManager::deleteRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                    int primary_key) {
    try {
        pqxx::work W(connection_);
        std::string sql = "DELETE FROM " + table_name + " WHERE id = " + std::to_string(primary_key) + ";";
        W.exec(sql.c_str()); // Convert std::string to const char*
        W.commit();
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}


bool DatabaseManager::selectRecords(const std::string& table_name, const std::vector<std::string>& table_columns) {
    try {
        pqxx::nontransaction W(connection_);

        // Construct the SQL query dynamically based on the table name and columns
        std::string sql = "SELECT ";
        // Add table columns to the query
        for (size_t i = 0; i < table_columns.size(); ++i) {
            sql += table_columns[i];
            if (i < table_columns.size() - 1) {
                sql += ", ";
            }
        }
        // Specify the table name
        sql += " FROM " + table_name;
        pqxx::result R(W.exec(sql.c_str()));  // Convert std::string to const char*

        for (const auto& row : R) {
            // Print the contents of each column in the row
            for (const std::string& column : table_columns) {
                std::cout << column << " = ";
                // Adjust the data type as needed based on your schema
                std::cout << row.at(column).as<std::string>() << " . ";
                std::cout << std::endl;
            }
            std::cout << "----------------------------------" << std::endl;
        }
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}


int main(int argc, char **argv) {

    std::string dbname="postgres", user="postgres", pswd="root", hostaddr="127.0.0.1", port="5432";

    DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);

    std::string table_name = "table_birfen"; // Replace with your desired table name
    std::vector<std::string> table_columns = {"column1", "column2", "column3"}; // Replace with your desired column names
    std::vector<std::string> row_data = {"value1", "value2", "value3"}; // Replace with your desired row data
    std::string search_column = "search_column"; // Replace with your desired search column
    std::string search_value = "search_value"; // Replace with your desired search value
    std::vector<std::string> update_columns = {"column_to_update"}; // Replace with your desired update columns
    std::vector<std::string> update_values = {"new_value"}; // Replace with your desired update values
    int primary_key = 1;

    // Example create, insert, update, and delete operations:
    if (dbManager.connect()) {
        std::cout << "Connected to the database" << std::endl;
        
        if (dbManager.createTable(table_name, table_columns)) {
            std::cout << "Table created successfully." << std::endl;
        } else {
            std::cerr << "Failed to create table." << std::endl;
            return 1;
        }

        if (dbManager.insertRecord(table_name, table_columns, row_data)) {
            std::cout << "Record inserted successfully." << std::endl;
        } else {
            std::cerr << "Failed to insert record." << std::endl;
        }

        if (dbManager.updateRecord(table_name, table_columns, search_column, search_value, update_columns, update_values)) {
            std::cout << "Record updated successfully." << std::endl;
        } else {
            std::cerr << "Failed to update record." << std::endl;
        }

        if (dbManager.deleteRecord(table_name, table_columns, primary_key)) {
            std::cout << "Record deleted successfully." << std::endl;
        } else {
            std::cerr << "Failed to delete record." << std::endl;
        }

        dbManager.disconnect();

    } else {
        std::cerr << "Error connecting to the database" << std::endl;
    }

    return 0;
}









    // createTable(const std::string& table_name, const std::vector<std::string>& table_columns) 
    // insertRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
    //                     const std::vector<std::string>& row_data)
    // updateRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
    //                     const std::string& search_column, const std::string& search_value,
    //                     const std::vector<std::string>& update_columns, const std::vector<std::string>& update_values)
    // deleteRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
    //                    int primary_key) 
















//   Aws::CloudWatchLogs::Model::InputLogEvent convertInputToBatched(
//           const std::string &input,
//           const std::chrono::milliseconds &milliseconds) override {

//     Aws::CloudWatchLogs::Model::InputLogEvent log_event;

//     log_event.SetMessage(input.c_str());
//     log_event.SetTimestamp(milliseconds.count());

//     return log_event;
//   }






  // int c;
  // while ((c = getopt(argc, argv, "t:m:n:c:ak:l:g:")) != -1) {
  //   if ((c == '?') || (c == ':')) {
  //     printf("Usage: %s -t topic -m postgresqldb -n nodename -c collection -k vectorial-threshold -l angular-threshold -g time-threshold -a\n", argv[0]);
  //     exit(-1);
  //   } else if (c == 't') {
  //     topic = optarg;
  //   } else if (c == 'm') {
  //     postgresqldb = optarg;
  //   } else if (c == 'n') {
  //     nodename = optarg;
  //   } else if (c == 'c') {
  //     collection = optarg;
  //   } else if (c == 'a') {
  //     bAlwaysLog = false;
  //   } else if (c == 'k') {
  //     sscanf(optarg, "%f", &fVectorialDistanceThreshold);
  //   } else if (c == 'l') {
  //     sscanf(optarg, "%f", &fAngularDistanceThreshold);
  //   } else if (c == 'g') {
  //     sscanf(optarg, "%f", &fTimeDistanceThreshold);
  //   }
  // }
  
  // if (topic == "") {
  //   printf("No topic given.\n");
  //   exit(-2);
  // } else if (nodename == "") {
  //   printf("No node name given.\n");
  //   exit(-2);
  // }




    // for (const std::string& topic : topics) {
    //     // Process each topic
    //     if (topic == "tf") {ros::Subscriber sub = n.subscribe<tf::tfMessage>(topic, 1000, msg_callback);}
    //     if (topic == "odom") {ros::Subscriber sub = n.subscribe<tf::tfMessage>(topic, 1000, msg_callback);}
    //     if (topic == "scan") {ros::Subscriber sub = n.subscribe<tf::tfMessage>(topic, 1000, msg_callback);}
    //     std::cout << "Topic: " << topic << std::endl;
    // }
