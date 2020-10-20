#!/usr/bin/python3
from functools import partial
import mysql.connector
import datetime
import json

def load_waypoints(filename):
    with open(filename, 'rb') as f:
        all_waypoints = json.load(f)

    # setup table stuff
    mydb = mysql.connector.connect(
      host="127.0.0.1",
      user="larkin_h",
      password="Meepp973",
      database="llh_saruser",
      auth_plugin="caching_sha2_password"
    )
    mycursor = mydb.cursor()
#    if mydb.is_connected():
#        value = input("connected to database, upload waypoints? y/n: ")
#        if value == 'y': # WAS TESTING HERE! SEEING IF IS_CONNECTED WILL HELP WITH HISTORICAL GPS DATA 


    waypoint_dict = all_waypoints['robot_list']['robot_0'] # only use one robot for now

    waypoints_json = []
    for i,pt in enumerate(waypoint_dict.keys()):
        waypoints_json.append({'stamp' : waypoint_dict[pt][0], 'timestamp' : -1, 'lat' :    waypoint_dict[pt][2], 'long' : waypoint_dict[pt][3]}) # leaving out altitude for now!
    waypoints_json = sorted(waypoints_json, key = lambda i: i['stamp'])
    wp_table = 'app3_waypointsdata'
    wp_data = {'deviceid' : 'drone_0',
               'taskid' : 'search_0',
               'waypointsdata' : json.dumps(waypoints_json),
               'created_at':datetime.datetime.now(),
               'updated_at':datetime.datetime.now()}

    sql_waypoint_update = 'UPDATE ' + wp_table + ' SET {}'.format(', '.join('{}=%s'.format(k) for k in wp_data)) + ' WHERE deviceid = %s'
    mycursor.execute(sql_waypoint_update, list(wp_data.values()) + ['drone_0'])
    mydb.commit()
    print("waypoints updated")

    mydb.close()
    mycursor.close()
    #rospy.loginfo("shuttin' down, bye")
    print("shuttin' down")


if __name__ == '__main__':
    load_waypoints('waypoints.json')
