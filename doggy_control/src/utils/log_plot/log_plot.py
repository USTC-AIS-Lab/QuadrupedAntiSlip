import sys
import pandas as pd
import numpy as np
from datetime import datetime
import matplotlib.pyplot as plt

class LogPlotter:
    def __init__(self):
        self.start_time = ""
        self.data = pd.DataFrame()
        self.data_columns = []
        self.data_header_str = "data_header:"
        self.data_str = "data_log:"

    def load_log_file(self, path):
        with open(path, "r") as file:
            prev_line = None # skip the last line
            for line in file:
                if prev_line is not None:
                    data_line = prev_line.strip()
                    word_split = data_line.split(" ")
                    time = prev_line[1:24]
                    if(word_split[4] == self.data_header_str):
                        self.start_time = time
                        time_start = datetime.strptime(self.start_time, "%Y-%m-%d %H:%M:%S.%f")
                        self.data_columns = word_split[5].split(",")
                        self.data_columns.insert(0, "time")
                        self.data = pd.DataFrame(columns=self.data_columns, dtype=np.float64)
                        print("data header: ")
                        print(self.data_columns)
                    elif(word_split[4] == self.data_str):
                        time_now = datetime.strptime(time, "%Y-%m-%d %H:%M:%S.%f")
                        time_elapsed_sec = (time_now - time_start).total_seconds()
                        new_data_line_str = word_split[5].split(",")
                        new_data_line = [float(data) for data in new_data_line_str]
                        new_data_line.insert(0, time_elapsed_sec)
                        if(len(new_data_line) == len(self.data_columns)):
                            self.data.loc[len(self.data)] = new_data_line
                        # self.data = self.data.concat(pd.Series(new_data_line, index=self.data.columns[:len(new_data_line)]), ignore_index=True)
                prev_line = line  # 更新 prev_line

    def plot_data(self, data_name_list):
        print("reading param list: ", data_name_list)
        data_name_list_all = []
        for data_name in data_name_list:
            if(data_name[-1]).isdigit():
                data_name_list_all.append(data_name)
            else:
                for data_head_name in self.data_columns:
                    if data_name in data_head_name and data_head_name[len(data_name)+1:].isdigit():
                        data_name_list_all.append(data_head_name)
        for data_name in data_name_list_all:
            plt.plot(self.data['time'].to_numpy(), self.data[data_name].to_numpy(), label=data_name, marker='.')
        plt.xlabel('time(sec)')
        plt.legend()
        plt.grid()
        plt.show()

if __name__ == "__main__":
    log_plotter = LogPlotter()
    file_name = "20230831-214035.txt"
    folder_path = "../../../logs/"
    para_list = ['com_pos_3','com_lin_vel_3']
    if(len(sys.argv) > 1):
        file_name = sys.argv[1]
    if(len(sys.argv) > 2):
        para_list = sys.argv[2:]
    path = folder_path + file_name
    log_plotter.load_log_file(path)
    log_plotter.plot_data(para_list)
