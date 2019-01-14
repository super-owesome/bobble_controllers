import logging, os, glob, yaml
import rosbag_pandas
import numpy as np
import pandas as pd


def parse_config_file(config_file):
    print('Loading configs from file : ' + config_file)
    config_file_dict = {}
    with open(config_file, 'r') as stream:
        try:
            config_file_dict = yaml.load(stream)
        except (IOError, yaml.YAMLError) as exc:
            print('Failed to parse config file : ' + config_file)
            print(exc)
    return config_file_dict


def parse_single_run(bag_file, csv_convert=False):
    yaml_info = rosbag_pandas.get_bag_info(bag_file)
    df = rosbag_pandas.bag_to_dataframe(bag_file)
    df['time'] = (df.index - df.index[0]) / np.timedelta64(1, 's')
    df.index = df['time']
    if csv_convert:
        csv_file_name = os.path.splitext(yaml_info["path"])[0] + '.csv'
        df.to_csv(csv_file_name)
    return df


def parse_all_runs_in_dir(runs_dir, csv_convert=False):
    bag_file_pattern = os.path.join(runs_dir, "*.bag")
    df = {}
    for sim_run in glob.glob(bag_file_pattern):
        run_name = os.path.splitext(os.path.basename(sim_run))[0]
        df[run_name] = parse_single_run(sim_run, csv_convert)
    return df


def parse_all_csv_runs_in_dir(runs_dir):
    csv_file_pattern = os.path.join(runs_dir, "*.csv")
    df = {}
    for sim_run in glob.glob(csv_file_pattern):
        run_name = os.path.splitext(os.path.basename(sim_run))[0]
        df[run_name] = pd.read_csv(sim_run)
    return df

