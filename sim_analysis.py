import json
import os
import numpy as np
import math
from plotly import graph_objects as go
from plotly.subplots import make_subplots

def analyze_runs(data_path, fields_of_interest):
    # translate saved data into averages
    # data_path = "C:\\Users\\Larkin\\planning_llh_bgc\\results\\grayson_n2_s2_basic"
    # print('-------------' + data_path.split('\\')[-1] + '-------------')
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
            # print("{} : mean = {:0.4e}, std = {:0.4e}".format(ky, sum(vals)/len(vals),np.std(vals)))
            pass
        else:
            # print("{} : mean = {:0.4e} + 0.50, std = {:0.4e}".format(ky, sum(vals)/len(vals) - 0.5,np.std(vals)))
            pass
        keysoi[ky] = [sum(vals)/len(vals), np.std(vals)] # save mean and std
    # print('-------------\n')
    return keysoi

if __name__ == "__main__":
    # translate saved data into averages
    base_folder = "C:\\Users\\Larkin\\planning_llh_bgc\\results\\kentland-revision\\"
    files = [
             [base_folder + "kentland_n1_s2_sweep",
              base_folder + "kentland_n2_s2_sweep",
              base_folder + "kentland_n3_s2_sweep",
              base_folder + "kentland_n4_s2_sweep",
              base_folder + "kentland_n5_s2_sweep",
              base_folder + "kentland_n6_s2_sweep"],

             [base_folder + "kentland_n1_s2_rc",
              base_folder + "kentland_n2_s2_rc",
              base_folder + "kentland_n3_s2_rc",
              base_folder + "kentland_n4_s2_rc",
              base_folder + "kentland_n5_s2_rc",
              base_folder + "kentland_n6_s2_rc"],

             [base_folder + "kentland_n1_s2_basic",
              base_folder + "kentland_n2_s2_basic",
              base_folder + "kentland_n3_s2_basic",
              base_folder + "kentland_n4_s2_basic",
              base_folder + "kentland_n5_s2_basic",
              base_folder + "kentland_n6_s2_basic"],
            ]

    keysoi = {'min_cost_old' : -1, 'min_risk_old' : -1, 'min_len_old' : -1, 'search_advantage' : -1, 'time_to_find' : -1, 'find_percentage' : -1}
    plot_data = []
    color_list = ['royalblue', 'brown', 'mediumseagreen', 'orangered', 'teal']
    er_scale = 2
    x_ticks = [1,2,3,4,5,6]
    y_order = []
    marker_list = ['diamond', 'x']
    # marker_list = ['square', 'circle']

    fig = make_subplots(specs = [[{"secondary_y": True}]])
    y_data = []
    max_len = 0

    for i, file_set in enumerate(files):
        locale = file_set[0].split('\\')[-1].split('_')[0].capitalize()
        traces = dict.fromkeys(keysoi.keys(),[])
        errors = dict.fromkeys(keysoi.keys(),[])
        traces.update({'locale' : locale})
        for j, fn in enumerate(file_set):
            data_dict = analyze_runs(fn, keysoi)
            for ky in list(data_dict.keys()):
                traces[ky] = traces[ky] + [data_dict[ky][0]]
                errors[ky] = errors[ky] + [data_dict[ky][1]]
        trace_name = fn.split('_')[-1]
        if trace_name == 'basic':
            trace_name = 'opt.'

        y_data.append([trace_name,np.array(traces['min_risk_old']), np.array(traces['min_len_old']), np.array(traces['min_cost_old'])])
        # y_data.append([trace_name,np.array(traces['min_risk_old']),np.array(traces['min_len_old']),np.array(traces['min_len_old'])])
        if max_len < np.max(np.array(traces['min_len_old'])):
            max_len = np.max(np.array(traces['min_len_old']))
    # max_len = 1
    for i, ydat in enumerate(y_data):
        yplt = ydat[3]
        fig.add_trace(go.Scatter(x = x_ticks, y = yplt - ydat[2], mode = "lines+markers", name=ydat[0], marker=dict(size = 10), marker_symbol = marker_list[0],
                                 line = dict(color = color_list[i], width = 3),
                                 error_y=dict(type='data', array = er_scale*np.array(errors['min_risk_old']), visible=True)), secondary_y = False)
        yplt = ydat[2]
        fig.add_trace(go.Scatter(x = x_ticks, y = yplt, mode = "lines+markers", name=ydat[0], marker=dict(size = 10), marker_symbol = marker_list[1],
                                 line = dict(color = color_list[i], width = 3, dash = 'dash'),
                                 showlegend=False, error_y=dict(type='data', array = er_scale*np.array(errors['min_len_old']), visible=True)), secondary_y = False)

        # fig.add_trace(go.Scatter(x = x_ticks, y = traces['min_risk_old'], mode = "lines+markers", name="{}".format(trace_name), marker=dict(size = 10), marker_symbol = marker_list[0], line = dict(color = color_list[i], width = 3),
        #     error_y=dict(type='data', array = er_scale*np.array(errors['min_risk_old']), visible=True)), secondary_y = False)
        #
        # fig.add_trace(go.Scatter(x = x_ticks, y = traces['min_len_old'], mode = "lines+markers", name="({}.)".format(trace_name), marker=dict(size = 10), marker_symbol = marker_list[1], line = dict(color = color_list[i], width = 3, dash = 'dash'),
        #     error_y=dict(type='data', array = er_scale*np.array(errors['min_len_old']), visible=True), showlegend=False), secondary_y = True)

    fig.update_layout(title='Metrics', autosize=True, width=1500, height=600,
                        margin=dict(l=100, r=100, b=65, t=90),
                        legend=dict(yanchor="top", y=0.99, xanchor="left", x=0.01),
                        font=dict(family="Courier New, monospace",size=24, color="Black"),
                        xaxis = dict(tickmode = 'array', tickvals = x_ticks),
                        yaxis = dict(showexponent = 'all', exponentformat = 'e'))

    fig.update_xaxes(title_text = "UAV Team Size (N)")

    fig.update_yaxes(title_text='Obj. Cost', secondary_y = False)
    fig.update_yaxes(title_text='Len. Cost', secondary_y = True)
    # fig.update_yaxes(title_text='Risk Cost', secondary_y = False)
    # fig.update_yaxes(title_text='Len. Cost', secondary_y = True)
    fig.show()