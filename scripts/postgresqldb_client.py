#!/usr/bin/env python3

import rospy
from postgresqldb_log.srv import Postgresqldb

""" 
// SERVICE.srv
// string mode            |  req.mode
// string[] row_data      |  req.row_data
// string[] update_values |  req.update_values
// string search_value    |  req.search_value
// int32 primary_key      |  req.primary_key
// -------------------------------------------
// bool status            |  res.status
 """

def create_table(mode): # table_name, table_columns
    rospy.wait_for_service('dbmngr_create')
    try:
        service_proxy = rospy.ServiceProxy('dbmngr_create', Postgresqldb)
        request = Postgresqldb()
        request.mode = mode
        request.row_data = []
        request.update_values = []
        request.search_value = ""
        request.primary_key = 0
        # ORDER IS IMPORTANT: 'mode', 'row_data', 'update_values', 'search_value', 'primary_key'
        response = service_proxy(mode = request.mode,
                                 row_data = request.row_data,
                                 update_values = request.update_values,
                                 search_value = request.search_value,
                                 primary_key = request.primary_key,
                                 )
        if response.status:
            rospy.loginfo("table created successfully.")
        else:
            rospy.logerr("failed to create table.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to 'dbmngr_create' failed: %s", e)


def insert_record(mode,row_data): # table_name, table_columns, row_data
    rospy.wait_for_service('dbmngr_insert')
    try:
        service_proxy = rospy.ServiceProxy('dbmngr_insert', Postgresqldb)
        request = Postgresqldb()
        request.mode = mode
        request.row_data = row_data
        request.update_values = []
        request.search_value = ""
        request.primary_key = 0
        response = service_proxy(mode = request.mode,
                                 row_data = request.row_data,
                                 update_values = request.update_values,
                                 search_value = request.search_value,
                                 primary_key = request.primary_key,
                                 )
        if response.status:
            rospy.loginfo("record inserted successfully.")
        else:
            rospy.logerr("failed to insert record.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to 'dbmngr_insert' failed: %s", e)


def update_record(mode, search_value, update_values): # table_name, table_columns, search_column, search_value, update_columns, update_values
    rospy.wait_for_service('dbmngr_update')
    try:
        service_proxy = rospy.ServiceProxy('dbmngr_update', Postgresqldb)
        request = Postgresqldb()
        request.mode = mode
        request.row_data = []
        request.update_values = update_values
        request.search_value = search_value
        request.primary_key = 0
        response = service_proxy(mode = request.mode,
                                 row_data = request.row_data,
                                 update_values = request.update_values,
                                 search_value = request.search_value,
                                 primary_key = request.primary_key,
                                 )
        if response.status:
            rospy.loginfo("table updated successfully.")
        else:
            rospy.logerr("failed to update task table.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to 'dbmngr_update' failed: %s", e)


def delete_record(mode, primary_key): # table_name, table_columns, primary_key
    rospy.wait_for_service('dbmngr_delete')
    try:
        service_proxy = rospy.ServiceProxy('dbmngr_delete', Postgresqldb)
        request = Postgresqldb()
        request.mode = mode
        request.row_data = []
        request.update_values = []
        request.search_value = ""
        request.primary_key = primary_key
        response = service_proxy(mode = request.mode,
                                 row_data = request.row_data,
                                 update_values = request.update_values,
                                 search_value = request.search_value,
                                 primary_key = request.primary_key,
                                 )
        if response.status:
            rospy.loginfo("record deleted successfully.")
        else:
            rospy.logerr("failed to delete task record.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to 'dbmngr_delete' failed: %s", e)


def select_record(): # table_name, table_columns, primary_key
    rospy.wait_for_service('dbmngr_select')
    try:
        service_proxy = rospy.ServiceProxy('dbmngr_select', Postgresqldb)
        request = Postgresqldb()
        request.mode = ""
        request.row_data = []
        request.update_values = []
        request.search_value = ""
        request.primary_key = 0
        response = service_proxy(mode = request.mode,
                                 row_data = request.row_data,
                                 update_values = request.update_values,
                                 search_value = request.search_value,
                                 primary_key = request.primary_key,
                                 )
        if response.status:
            rospy.loginfo("record selected successfully.")
        else:
            rospy.logerr("failed to select all task record.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call to 'dbmngr_select' failed: %s", e)







if __name__ == '__main__':
 
    rospy.init_node("postgresqldb_log_service_client")

    # ---------- Define these in the .yaml file ----------------
    # table_name = "table_birfen_task";                       # // Replace with your desired table name
    # table_columns = ["column1", "column2", "column3"]; # // Replace with your desired column names
    # search_column = "column2";                         # // Replace with your desired search column
    # update_columns = ["column1","column3"];            # // Replace with your desired update columns
    # ----------------------------------------------------------

    # -------- sample input for the service calls --------------
    mode = "task"
    row_data = ["value1", "value2", "value3"];   # // Replace with your desired row data
    search_value = "value2";                     # // Replace with your desired search value
    update_values = ["new_value1","new_value2"]; # // Replace with your desired update values
    primary_key = 1;                             # // Replace with your desired primary_key for deletion
    # ----------------------------------------------------------
    # Example service calls for create, insert, update, delete, and select.
    create_table(mode)
    insert_record(mode,row_data) 
    update_record(mode, search_value, update_values)
    select_record()
    delete_record(mode, primary_key)









# TERMINAL 1:
# docker-compose up --build
# or
# docker run -it env1_ros_noetic /bin/bash  
# or
# docker run -it --net=host --ipc=host --pid=host --device /dev/dri/ -e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/root/.Xauthority:ro env1_ros_noetic

# TERMINAL 2:
# docker exec -it env1_ros_noetic_1 bash
# cd birfen_ws
# rm -rf build
# catkin_make clean
# catkin_make --only-pkg-with-deps postgresqldb_log  ===> catkin_make --only-pkg-with-deps <package-name>

# docker exec -it env1_ros_noetic_1 bash
# cd birfen_ws && source devel/setup.bash && python3 src/postgresqldb_log/scripts/postgresqldb_client.py































# from std_msgs.msg import Int64, Bool
# from threading import Thread

# class TestPublisher(object):

#     def __init__(self, topic, delay, rate, count=20):
#         super(TestPublisher, self).__init__()
#         self.publisher = rospy.Publisher(topic, Int64, queue_size=min(10, count))
#         self.count = count
#         self.delay = delay
#         self.rate = rate
#         self.thread = Thread(target=self.publish, name=topic)


#     def publish(self):
#         rospy.sleep(self.delay)
#         for i in range(self.count):
#             self.publisher.publish(i)
#             self.rate.sleep()

# if __name__ == '__main__':
 
    # rospy.init_node("postgresqldb_log_service_client")

    # Call other functions for update, delete, and select...

    # topic, delay, rate, count
    # to_publish = [ ('test_0', rospy.Duration(10), rospy.Rate(1), 20), ('test_1', rospy.Duration(20), rospy.Rate(1), 20) ] 
    # publishers = map(lambda tup: TestPublisher(*tup), to_publish)
    # map(lambda pub: pub.thread.start(), publishers)
    # map(lambda pub: pub.thread.join(), publishers)

