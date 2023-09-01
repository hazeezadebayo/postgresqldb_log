#ifndef DATABASE_MANAGER_H
#define DATABASE_MANAGER_H

#include <string>
#include <vector>
#include <pqxx/pqxx>

class DatabaseManager {
public:
   DatabaseManager(const std::string& dbname, const std::string& user,
                  const std::string& password, const std::string& hostaddr,
                  const std::string& port);

   bool connect();
   void disconnect();
   bool createTable(const std::string& table_name, const std::vector<std::string>& table_columns);
   bool insertRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     const std::vector<std::string>& row_data);
   bool updateRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     const std::string& search_column, const std::string& search_value,
                     const std::vector<std::string>& update_columns, const std::vector<std::string>& update_values);
   bool deleteRecord(const std::string& table_name, const std::vector<std::string>& table_columns,
                     int primary_key);
   bool selectRecords(const std::string& table_name, const std::vector<std::string>& table_columns);

private:
   pqxx::connection connection_;
   std::string dbname;
   std::string user;
   std::string pswd;
   std::string hostaddr;
   std::string port;

   std::string table_name;
   std::vector<std::string> table_columns;
   std::string search_column;
   std::string update_column;


};

#endif  // DATABASE_MANAGER_H


















// ~/birfen_ws/src$ chmod +x db_test.cpp
// ~/birfen_ws/src$ g++ -o db db_test.cpp -lpqxx -lpq
// ~/birfen_ws/src$ chmod +x db
// ~/birfen_ws/src$ ./db

// apt-cache search postgresql
// sudo apt-get install libpqxx-dev   
// sudo apt-get install libpqxx-6.4

// pg_config --version
// sudo gedit /etc/postgresql/14/main/postgresql.conf
// sudo gedit /etc/postgresql/14/main/pg_hba.conf

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
