# Establish the analysis environment used with Jupyter Notebook
import os, sys, subprocess
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
from analysis_tools.parsing import *
from analysis_tools.plots import *

# Set some global plotting configs. We want these for all plots
sns.set_context("poster", font_scale=1.1)
#sns.set_palette("Dark2")

# Set paths to relevant data directories and config files
cgx_data_home = os.path.abspath(os.path.join('data', 'move_cg_x'))
cgx_plot_config = os.path.join(cgx_data_home, 'plots.yaml')
cgz_data_home = os.path.abspath(os.path.join('data', 'move_cg_z'))
cgz_plot_config = os.path.join(cgz_data_home, 'plots.yaml')

# Load configs and data
pc_x = parse_config_file(cgx_plot_config)
df_x = parse_all_runs_in_dir(cgx_data_home)
pc_z = parse_config_file(cgz_plot_config)
df_z = parse_all_runs_in_dir(cgz_data_home)
