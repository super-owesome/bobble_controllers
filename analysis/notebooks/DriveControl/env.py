# Establish the analysis environment used with Jupyter Notebook
import os, sys
os.sys.path.append('/home/mike/.local/lib/python2.7/site-packages/')
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import numpy as np
import scipy
import scipy.fftpack
import math
from analysis_tools.parsing import *
from analysis_tools.plots import *
from analysis_tools.filters import *

# Set some global plotting configs. We want these for all plots
sns.set_context("poster", font_scale=1.1)
#sns.set_palette("Dark2")

# Set paths to relevant data directories and config files
drive_fwd_bwd_data_home = os.path.abspath(os.path.join('data', 'drive_fwd_bwd'))
drive_square_data_home = os.path.abspath(os.path.join('data', 'drive_square'))

# Load configs and data
df = parse_all_runs_in_dir(drive_fwd_bwd_data_home)
df.update(parse_all_runs_in_dir(drive_square_data_home))

