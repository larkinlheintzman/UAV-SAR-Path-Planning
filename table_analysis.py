import json
import os
import numpy as np
import math
from plotly import graph_objects as go
from plotly.subplots import make_subplots

def analyze_runs(data_path, fields_of_interest):
    # translate saved data into averages
    # data_path = "C:\\Users\\Larkin\\planning_llh_bgc\\results\\grayson_n2_s2_basic"
    print('-------------' + data_path.split('\\')[-1] + '-------------')
    filelist = os.listdir(data_path)

    # get format first
    with open(data_path + "\\" + filelist[0]) as f:
        temp_dict = json.load(f)
        avg_dict = dict.fromkeys(temp_dict.keys(), [])

    for fn in filelist:
        with open(data_path + "\\" + fn, 'r') as f:
            run_data = json.load(f)
            for ky in list(run_data.keys()):
                avg_dict[ky] = avg_dict[ky] + [run_data[ky]]

    # calculate averages from mc runs
    for ky in list(keysoi.keys()):
        vals = [v for v in avg_dict[ky] if not math.isnan(v)]
        if ky != 'search_advantage':
            print("{} = {:0.4} \\pm {:0.4}".format(ky, sum(vals)/len(vals),np.std(vals)))
            pass
        else:
            print("{} = {:0.4} \\pm {:0.4}\n".format(ky, sum(vals)/len(vals),np.std(vals)))
            pass
        keysoi[ky] = [sum(vals)/len(vals), np.std(vals)] # save mean and std
    # print('-------------\n')
    return keysoi

if __name__ == "__main__":
    # translate saved data into averages
    files = [["C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n1_s2_basic",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n1_s2_rc",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n1_s2_unopt",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n1_s2_ring",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n1_s2_sweep",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n2_s2_basic",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n2_s2_rc",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n2_s2_unopt",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n2_s2_ring",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n2_s2_sweep",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n3_s2_basic",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n3_s2_rc",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n3_s2_unopt",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n3_s2_ring",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n3_s2_sweep",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n4_s2_basic",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n4_s2_rc",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n4_s2_unopt",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n4_s2_ring",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n4_s2_sweep",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n5_s2_basic",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n5_s2_rc",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n5_s2_unopt",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n5_s2_ring",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n5_s2_sweep",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n6_s2_basic",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n6_s2_rc",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n6_s2_unopt",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n6_s2_ring",
              "C:\\Users\\Larkin\\planning_llh_bgc\\results\\hmpark_n6_s2_sweep"]]


    keysoi = {'min_cost_old' : -1}

    for i, file_set in enumerate(files):
        for fn in file_set:
            data_dict = analyze_runs(fn, keysoi)