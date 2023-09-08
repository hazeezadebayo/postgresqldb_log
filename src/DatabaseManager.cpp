
#include <iostream>
#include <functional>
#include <string>
#include <postgresqldb_log/DatabaseManager.h> 

/***************************************************************************
 *  Created: Sun Sep 3 15:05:37 2023
 *  Copyright  2023  Azeez Adebayo
 ****************************************************************************/


DatabaseManager::DatabaseManager(const std::string& dbname, const std::string& user,
                                 const std::string& pswd, const std::string& hostaddr,
                                 const std::string& port)
    : connection_("dbname=" + dbname + " user=" + user + " password=" + pswd +
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
        //std::cout << "---------x---------o---------c------1" << std::endl;
        pqxx::work W(connection_);
        //std::cout << "----------------------------------2" << std::endl; 
        std::string sql = "CREATE TABLE IF NOT EXISTS " + table_name + " (ID SERIAL PRIMARY KEY, ";
        // Add timestamp column with default current timestamp
        sql += "insert_time TIMESTAMP DEFAULT current_timestamp, ";      
        for (size_t i = 0; i < table_columns.size(); ++i) {
            sql += table_columns[i] + " TEXT";
            if (i < table_columns.size() - 1) {
                sql += ", ";
            }
        }
        sql += ");";
        //std::cout << "----------------------------------3" << std::endl;
        W.exec(sql.c_str()); // Convert std::string to const char*
        //std::cout << "----------------------------------4" << std::endl;
        W.commit();
        //std::cout << "----------------------------------5" << std::endl;
        return true;
    } catch (const std::exception& e) {
        //std::cout << "----------------------------------6" << std::endl;
        return false;
    }
}


bool DatabaseManager::insertRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     const std::vector<std::string>& row_data) {
    try {
        //std::cout << "----------------------------------1" << std::endl;
        pqxx::work W(connection_);
        //std::cout << "----------------------------------2" << std::endl;
        if (table_columns.size() != row_data.size()) {
            return false; // Number of columns and data elements should match
        }
        //std::cout << "----------------------------------3" << std::endl;
        // table_columns: ["task_id","robot_id","signal_time","signal_type","signal_description"] 
        std::string sql = "INSERT INTO " + table_name + " (";
        for (size_t i = 0; i < table_columns.size(); ++i) {
            sql += table_columns[i];
            if (i < table_columns.size() - 1) {
                sql += ",";
            }
        }
        sql += ") VALUES (";
        for (size_t i = 0; i < row_data.size(); ++i) {
            sql += "'" + row_data[i] + "'";
            if (i < row_data.size() - 1) {
                sql += ",";
            }
        }
        sql += ");";
        //std::cout << "----------------------------------4: " << sql.c_str() << std::endl;
        W.exec(sql.c_str()); // Convert std::string to const char*
        //std::cout << "----------------------------------5: "  << std::endl;
        W.commit();
        //std::cout << "----------------------------------6: "  << std::endl;
        return true;
    } catch (const std::exception& e) {
        //std::cout << "----------------------------------7: "  << std::endl;
        return false;
    }
}


bool DatabaseManager::updateRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     const std::string& search_column, const std::string& search_value,
                     const std::vector<std::string>& update_columns, const std::vector<std::string>& update_values) {
    try {
        //std::cout << "----------------------------------1" << std::endl;
        pqxx::work W(connection_);
        //std::cout << "----------------------------------2" << std::endl;
        if (update_columns.size() != update_values.size()) {
            return false; // Number of columns and values for update should match
        }
        //std::cout << "----------------------------------3" << std::endl;
        std::string sql = "UPDATE " + table_name + " SET ";
        for (size_t i = 0; i < update_columns.size(); ++i) {
            sql += update_columns[i] + " = '" + update_values[i] + "'";
            if (i < update_columns.size() - 1) {
                sql += ",";
            }
        }
        sql += " WHERE " + search_column + " = '" + search_value + "';"; // time = time
        //std::cout << "----------------------------------4" << std::endl;
        W.exec(sql.c_str()); // Convert std::string to const char*
        //std::cout << "----------------------------------5" << std::endl;
        W.commit();
        //std::cout << "----------------------------------6" << std::endl;
        return true;
    } catch (const std::exception& e) {
        //std::cout << "----------------------------------7" << std::endl;
        return false;
    }
}


bool DatabaseManager::deleteRecord(const std::string& table_name, 
        const std::vector<std::string>& table_columns,
        int primary_key) {
    try {
        //std::cout << "----------------------------------1" << std::endl;
        pqxx::work W(connection_);
        //std::cout << "----------------------------------2" << std::endl;
        std::string sql = "DELETE FROM " + table_name + " WHERE id = " + std::to_string(primary_key) + ";";
        //std::cout << "----------------------------------3" << std::endl;
        W.exec(sql.c_str()); // Convert std::string to const char*
        //std::cout << "----------------------------------4" << std::endl;
        // After deletion, renumber the remaining entries
        sql = "UPDATE " + table_name + " SET id = id - 1 WHERE id > " + std::to_string(primary_key) + ";";
        //std::cout << "----------------------------------5" << std::endl;
        W.exec(sql.c_str()); // Update IDs of remaining entries
        //std::cout << "----------------------------------6" << std::endl;
        W.commit(); // Commit the transaction
        return true;
    } catch (const std::exception& e) {
        // Handle exceptions if needed
        std::cout << "An error occurred when deleting record: " << e.what() << std::endl;
        return false;
    }
}



std::map<std::string, std::vector<std::vector<std::string>>> DatabaseManager::selectRecords(const std::string& table_name, const std::vector<std::string>& table_columns) {
    std::map<std::string, std::vector<std::vector<std::string>>> records;  // Create a dictionary to store the records
    try {
        pqxx::nontransaction W(connection_);
        // Check if the table exists
        std::string check_sql = "SELECT EXISTS (SELECT 1 FROM information_schema.tables WHERE table_name = '" + table_name + "');";
        pqxx::result check_result(W.exec(check_sql.c_str()));
        if (check_result[0][0].as<bool>()) {
            // Table exists, proceed with selecting records
            std::string sql = "SELECT id, ";  // Include the 'id' column
            // Add table columns to the query
            for (size_t i = 0; i < table_columns.size(); ++i) {
                sql += table_columns[i];
                if (i < table_columns.size() - 1) {
                    sql += ", ";
                }
            }
            // Specify the table name
            sql += " FROM " + table_name;  
            // Add a WHERE clause to filter by current date
            sql += " WHERE DATE(insert_time) = CURRENT_DATE;";  // comment out if you want to return entire table 
            pqxx::result R(W.exec(sql.c_str()));  // Convert std::string to const char*
            // Iterate over rows
            for (const auto& row : R) {
                // Create a vector to store the values for each row
                std::vector<std::string> row_values;
                // Include 'id' value
                row_values.push_back(row.at("id").as<std::string>());
                // Iterate over columns (excluding 'id')
                for (const std::string& column : table_columns) {
                    // Adjust the data type as needed based on your schema
                    row_values.push_back(row.at(column).as<std::string>());
                }
                // Store the row values in the map, using the table name as the key
                records[table_name].push_back(row_values);
            }
        } else {
            // Table does not exist
            // std::cout << "Table '" << table_name << "' does not exist." << std::endl;
        }
    } catch (const std::exception& e) {
        // Handle exceptions if needed
        std::cout << "An error occurred when retrieving records: " << e.what() << std::endl;
    }
    return records;  // Return the dictionary
}








int test_main(int argc, char **argv) {

    std::string dbname="postgres", user="postgres", pswd="root", hostaddr="192.168.1.81", port="5432";

    DatabaseManager dbManager(dbname, user, pswd, hostaddr, port);

    std::string table_name = "table_birfen"; // Replace with your desired table name
    std::vector<std::string> table_columns = {"column1", "column2", "column3"}; // Replace with your desired column names
    std::vector<std::string> row_data = {"value1", "value2", "value3"}; // Replace with your desired row data
    std::string search_column = "column2"; // Replace with your desired search column
    std::string search_value = "value2"; // Replace with your desired search value
    std::vector<std::string> update_columns = {"column1","column3"}; // Replace with your desired update columns
    std::vector<std::string> update_values = {"new_value1","new_value2"}; // Replace with your desired update values
    int primary_key = 1;

    // Example create, insert, update, and delete operations:
    if (dbManager.connect()) {
        std::cout << "Connected to the database" << std::endl;
        // tested
        if (dbManager.createTable(table_name, table_columns)) {
            std::cout << "Table created successfully." << std::endl;
        } else {
            std::cerr << "Failed to create table." << std::endl;
            return 1;
        }
        // tested
        if (dbManager.insertRecord(table_name, table_columns, row_data)) {
            std::cout << "Record inserted successfully." << std::endl;
        } else {
            std::cerr << "Failed to insert record." << std::endl;
        }
        // tested
        if (dbManager.updateRecord(table_name, table_columns, search_column, search_value, update_columns, update_values)) {
            std::cout << "Record updated successfully." << std::endl;
        } else {
            std::cerr << "Failed to update record." << std::endl;
        }
        // tested
        std::map<std::string, std::vector<std::vector<std::string>>> record_ = dbManager.selectRecords(table_name, table_columns);
        // Iterate through the map
        for (const auto& entry : record_) {
            std::cout << "Table Name: " << entry.first << std::endl;
            // Print column names
            std::cout << "Columns: ";
            for (const std::string& column : table_columns) {
                std::cout << column << " ";
            }
            std::cout << std::endl;
            // Iterate through the vectors associated with each table name
            for (const std::vector<std::string>& row : entry.second) {
                std::cout << "Row:" << std::endl;
                // Iterate through the elements in each row and match them with columns
                for (size_t i = 0; i < row.size(); ++i) {
                    std::cout << "  " << table_columns[i] << ": " << row[i] << std::endl;
                }
            }
        }
        // tested
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
